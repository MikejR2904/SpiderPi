import rospy
from sensor_msgs.msg import LaserScan
import paho.mqtt.client as mqtt
import json

MQTT_BROKER = "192.168.137.102"  # Your MQTT broker address
MQTT_PORT = 1883
MQTT_TOPIC = "lidar/scan"

# Initialize MQTT client
client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)

def scan_callback(data):
    scan_data = {
        "ranges": data.ranges,
        "intensities": data.intensities,
        "angle_min": data.angle_min,
        "angle_max": data.angle_max,
        "angle_increment": data.angle_increment,
        "time_increment": data.time_increment,
        "scan_time": data.scan_time,
        "range_min": data.range_min,
        "range_max": data.range_max
    }
    client.publish(MQTT_TOPIC, json.dumps(scan_data))
    print(f'Scan data sent with {scan_data}')

def main():
    rospy.init_node('mqtt_lidar_publisher', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
