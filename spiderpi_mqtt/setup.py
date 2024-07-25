from setuptools import find_packages, setup

package_name = 'spiderpi_mqtt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='idp',
    maintainer_email='e1129809@u.nus.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'imu_node=spiderpi_mqtt.imu_node:main',
        	'lidar_node=spiderpi_mqtt.lidar_node:main',
        ],
    },
)
