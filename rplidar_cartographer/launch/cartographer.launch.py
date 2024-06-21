import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Define launch arguments
    head_urdf = LaunchConfiguration('head_urdf', default='$(find rplidar_cartographer)/urdf/head_2d.urdf')
    configuration_directory = LaunchConfiguration('configuration_directory', default='$(find rplidar_cartographer)/configuration_files')
    configuration_basename = LaunchConfiguration('configuration_basename', default='rplidar_a1.lua')

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': TextSubstitution(textfile=head_urdf)}]
    )

    # RPLIDAR node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
            {'serial_baudrate': 115200},
            {'frame_id': 'laser'},
            {'inverted': False},
            {'angle_compensate': True}
        ]
    )
    
    imu_node = Node(
        package='spiderpi',
        executable='mpu6050_modified.py',
        name='mp6050_modified',
        output='screen',
    )

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ]
    )

    cartographer_occupancy_grid_node = Node(
    	package='cartographer_ros',
    	executable='cartographer_occupancy_grid_node',
    	name='cartographer_occupancy_grid_node',
    	output='screen',
    	arguments=['-resolution', '0.05']
    )
    
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_base_link_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'base_link', '100'],
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    ld.add_action(rplidar_node)
    ld.add_action(imu_node)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(tf2_node)
    
    return ld
