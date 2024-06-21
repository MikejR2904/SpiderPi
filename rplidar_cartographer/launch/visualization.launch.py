import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Define launch argument for RViz configuration file
    rviz_config_file = DeclareLaunchArgument(
        'rviz_config',
        default_value='$(find rplidar_cartographer)/rviz/rplidar.rviz',
        description='Path to the RViz configuration file'
    )

    # Launch RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(rviz_config_file)
    ld.add_action(rviz_node)

    return ld
