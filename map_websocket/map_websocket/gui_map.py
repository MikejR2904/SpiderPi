import sys
import asyncio
import json
import socket
import math
import numpy as np
import scipy.stats
from PIL import Image
import matplotlib.pyplot as plt

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap, QImage, QPainter, QColor
from PyQt5.QtCore import Qt, QSize

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# Global variables for map data storage
spiderpi_last_command = None
turtlebot_last_command = None

# Constants for map visualization
map_bg_color = 255  # Background color for the map (white)

class Occupy(Node):

    def __init__(self):
        super().__init__('occupy')
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Subscription to turtlebot_map
        self.subscription_turtlebot = self.create_subscription(
            OccupancyGrid,
            '/turtlebot_map',
            self.listener_callback_turtlebot,
            10)  # qos_profile_sensor_data is not defined, assuming default QoS profile
        self.subscription_turtlebot  # prevent unused variable warning

        # Subscription to spiderpi_map
        self.subscription_spiderpi = self.create_subscription(
            OccupancyGrid,
            '/spiderpi_map',
            self.listener_callback_spiderpi,
            10)  # qos_profile_sensor_data is not defined, assuming default QoS profile
        self.subscription_spiderpi  # prevent unused variable warning

        # Subscription to turtlebot_command
        self.subscription_turtlebot_command = self.create_subscription(
            String,
            '/turtlebot_command',
            self.command_callback_turtlebot,
            10)
        self.subscription_turtlebot_command  # prevent unused variable warning

        # Subscription to spiderpi_command
        self.subscription_spiderpi_command = self.create_subscription(
            String,
            '/spiderpi_command',
            self.command_callback_spiderpi,
            10)
        self.subscription_spiderpi_command  # prevent unused variable warning

    def listener_callback_turtlebot(self, msg):
        self.process_map(msg, 'turtlebot')

    def listener_callback_spiderpi(self, msg):
        self.process_map(msg, 'spiderpi')

    def command_callback_turtlebot(self, msg):
        global turtlebot_last_command
        turtlebot_last_command = msg.data

    def command_callback_spiderpi(self, msg):
        global spiderpi_last_command
        spiderpi_last_command = msg.data

    def process_map(self, msg, robot_type):
        try:
            # create numpy array
            occdata = np.array(msg.data)
            # compute histogram to identify bins with -1, values between 0 and below 50,
            # and values between 50 and 100. The binned_statistic function will also
            # return the bin numbers so we can use that easily to create the image
            occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=[-1, 0, 50, 100])
            # get width and height of map
            iwidth = msg.info.width
            iheight = msg.info.height

            # find transform to obtain base_link coordinates in the map frame
            # lookup_transform(target_frame, source_frame, time)
            try:
                trans = self.tfBuffer.lookup_transform('map', 'base_link', self.get_clock().now())
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().info(f'No transformation found for {robot_type}')
                return
                
            cur_pos = trans.transform.translation
            cur_rot = trans.transform.rotation
            # convert quaternion to Euler angles
            roll, pitch, yaw = self.euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)

            # get map resolution
            map_res = msg.info.resolution
            # get map origin struct has fields of x, y, and z
            map_origin = msg.info.origin.position
            # get map grid positions for x, y position
            grid_x = round((cur_pos.x - map_origin.x) / map_res)
            grid_y = round(((cur_pos.y - map_origin.y) / map_res))

            # binnum go from 1 to 3 so we can use uint8
            # convert into 2D array using column order
            odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))
            # set current robot location to 0
            odata[grid_y][grid_x] = 0
            # create image from 2D array using PIL
            img = Image.fromarray(odata)
            # find center of image
            i_centerx = iwidth / 2
            i_centery = iheight / 2
            # find how much to shift the image to move grid_x and grid_y to center of image
            shift_x = round(grid_x - i_centerx)
            shift_y = round(grid_y - i_centery)

            # pad image to move robot position to the center
            left = 0
            right = 0
            top = 0
            bottom = 0
            if shift_x > 0:
                # pad right margin
                right = 2 * shift_x
            else:
                # pad left margin
                left = 2 * (-shift_x)
                
            if shift_y > 0:
                # pad bottom margin
                bottom = 2 * shift_y
            else:
                # pad top margin
                top = 2 * (-shift_y)
                
            # create new image
            new_width = iwidth + right + left
            new_height = iheight + top + bottom
            img_transformed = Image.new(img.mode, (new_width, new_height), map_bg_color)
            img_transformed.paste(img, (left, top))

            # rotate by 90 degrees so that the forward direction is at the top of the image
            rotated = img_transformed.rotate(np.degrees(yaw) - 90, expand=True, fillcolor=map_bg_color)

            # Show the image using grayscale map
            plt.imshow(rotated, cmap='gray', origin='lower')
            plt.draw_all()
            # Pause to make sure the plot gets created
            plt.pause(0.00000000001)

        except Exception as e:
            self.get_logger().error(f"Error processing map for {robot_type}: {str(e)}")

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z


class MainWindow(QMainWindow):
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.setup_ros()

    def init_ui(self):
        self.setWindowTitle('Occupancy Grid Viewer')
        self.setGeometry(100, 100, 800, 400)
        
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        
        self.spiderpi_label = QLabel(self)
        self.spiderpi_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.spiderpi_label)
        
        self.turtlebot_label = QLabel(self)
        self.turtlebot_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.turtlebot_label)
        
        self.spiderpi_command_label = QLabel(self)
        self.spiderpi_command_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.spiderpi_command_label)
        
        self.turtlebot_command_label = QLabel(self)
        self.turtlebot_command_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.turtlebot_command_label)
        
    def setup_ros(self):
        rclpy.init(args=None)
        
        self.occupy_node = Occupy()
        
        # Create a timer to update GUI with command data
        self.command_update_timer = self.createTimer()

    def createTimer(self):
        return self.occupy_node.create_timer(1.0, self.update_gui_commands)

    def update_gui_commands(self):
        global spiderpi_last_command, turtlebot_last_command
        
        if spiderpi_last_command:
            self.spiderpi_command_label.setText(f"Spiderpi Command: {spiderpi_last_command}")
        if turtlebot_last_command:
            self.turtlebot_command_label.setText(f"Turtlebot Command: {turtlebot_last_command}")

    def closeEvent(self, event):
        # Shutdown ROS when the application is closed
        self.occupy_node.destroy_node()
        rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
