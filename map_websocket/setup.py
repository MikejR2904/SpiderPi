from setuptools import find_packages, setup

package_name = 'map_websocket'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 
    'rclpy',
    'websockets',
    'ament_python'],
    zip_safe=True,
    maintainer='idp',
    maintainer_email='e1129809@u.nus.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'websocket_publisher_node = map_websocket.websocket_publisher_node:main',
        	'websocket_client_node = map_websocket.websocket_client_node:main',
        ],
    },
)
