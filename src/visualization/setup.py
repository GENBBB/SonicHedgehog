from setuptools import setup

package_name = 'visualization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GENBBB',
    maintainer_email='gengensav@gmail.com',
    description='ROS2 visualization for robot pose, map, and LIDAR boundaries',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'visualization_node = visualization.visualization_node:main',
        ],
    },
)
