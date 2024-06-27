from setuptools import find_packages, setup

package_name = 'rplidar_cartographer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cartographer.launch.py', 'launch/visualization.launch.py', 'launch/occupancy_grid.launch.py']),
        ('share/' + package_name + '/config', ['config/rplidar_a1.lua']),
        ('share/' + package_name + '/rviz', ['rviz/rplidar.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/head_2d.urdf']),
    ],
    install_requires=['setuptools', 'rclcpp', 'cartographer_ros', 'geometry_msgs', 'rclpy',
		      'sensor_msgs', 'std_msgs', 'std_srvs', 'tf'],
    zip_safe=True,
    maintainer='idp',
    maintainer_email='e1129809@u.nus.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
