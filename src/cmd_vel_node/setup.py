from setuptools import setup

package_name = 'cmd_vel_node'

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
    description='ROS2 node to control CobraFlex via cmd_vel',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_node = cmd_vel_node.cmd_vel_node:main',
        ],
    },
)