import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spiderpi_sensors',
            executable='imu_node',
            name='imu_node',            
            output='screen',
        ),
        Node(
            package='spiderpi_sensors',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),
    ])
