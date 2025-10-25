from setuptools import setup

package_name = 'odom_node'

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
    description='ROS2 node to publish CobraFlex odometry',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_node = odom_node.odom_node:main',
        ],
    },
)