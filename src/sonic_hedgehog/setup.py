from setuptools import setup

package_name = 'sonic_hedgehog'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/teleop_test.launch.py'
        ]),
        ('share/' + package_name + '/configs', [
            'configs/slam_toolbox.yaml',
            'configs/controller_server.yaml',
            'configs/planner_server.yaml',
            'configs/bt_navigator.yaml',
            'configs/behavior_server.yaml',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/rviz_config.rviz',
        ]),
        ('share/' + package_name + '/maps', [
            'maps/map_serial.data',
            'maps/map_serial.posegraph',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GENBBB',
    maintainer_email='gengesav@gmail.com',
    description='Main meta-package for SonicHedgehog',
    license='Apache-2.0',
    entry_points={
    },
)
