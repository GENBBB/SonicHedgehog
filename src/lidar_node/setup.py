from setuptools import setup

package_name = 'lidar_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GENBBB',
    maintainer_email='gengensav@gmail.com',
    description='ROS 2 node for reading LiDAR data via serial port, filtering, stabilizing, and publishing LaserScan messages to /scan topic',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_node = lidar_node.lidar_node:main',
        ],
    },
)