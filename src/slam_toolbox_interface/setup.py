from setuptools import setup

package_name = 'slam_toolbox_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_toolbox_interface_launch.py']),
        ('share/' + package_name + '/configs', ['configs/slam_toolbox.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GENBBB',
    maintainer_email='gengensav@gmail.com',
    description='ROS2 Python interface node for SLAM Toolbox 2D',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slam_wrapper_node = slam_toolbox_interface.slam_wrapper_node:main',
        ],
    },
)