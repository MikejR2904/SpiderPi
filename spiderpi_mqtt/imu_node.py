#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

import json

class ImuPublisherNode(Node):

    def __init__(self):
        super().__init__('imu_publisher_node')
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)  # Using sensor_msgs/Imu message type
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("192.168.137.228", 1883)  # Replace with your MQTT broker address and port
        self.get_logger().info("IMU Publisher Node initialized")
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker with result code {rc}")
        self.mqtt_client.subscribe("/imu/raw")
        self.get_logger().info("Subscribed to /imu/raw topic")

    def on_message(self, client, userdata, message):
    	payload = message.payload.decode('utf-8')  # Convert payload bytes to string
    	data = json.loads(payload)
    	
    	accel = data['accel']
    	gyro = data['gyro']
    	
    	# Example: Print the received data
    	print(f"Accelerometer: x={accel['x']}, y={accel['y']}, z={accel['z']}")
    	print(f"Gyroscope: x={gyro['x']}, y={gyro['y']}, z={gyro['z']}")
    	
    	self.publish_imu_data(accel, gyro)

    def publish_imu_data(self, accel, gyro):
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = float(accel['x'])
        imu_msg.linear_acceleration.y = float(accel['y'])
        imu_msg.linear_acceleration.z = float(accel['z'])

        imu_msg.angular_velocity.x = float(gyro['x'])
        imu_msg.angular_velocity.y = float(gyro['y'])
        imu_msg.angular_velocity.z = float(gyro['z'])
        
        # Calculate angles
        angle_x = int(math.degrees(math.atan2(float(accel["x"]), float(accel["z"]))))
        angle_y = int(math.degrees(math.atan2(float(accel["y"]), float(accel["z"]))))

        # Assuming MPU6050 doesn't provide orientation, setting it to identity quaternion
        imu_msg.orientation = Quaternion()
        imu_msg.orientation.w = 1.0
        
        threshold_angle = 5.0
        if abs(angle_x) < threshold_angle or abs(angle_y) < threshold_angle:
            self.publisher_.publish(imu_msg)
            # Print angles to terminal
            self.get_logger().info(f'angle_x: {angle_x}, angle_y: {angle_y}')
        else:
            self.get_logger().info("Robot is not horizontal. Mapping is disabled.")
        
    def destroy_node(self):
    	self.mqtt_client.loop_stop()
    	self.mqtt_client.disconnect()
    	super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
