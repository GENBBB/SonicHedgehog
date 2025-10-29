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
            'launch/sim_no_cmd.launch.py',
            'launch/real_no_cmd.launch.py',
            'launch/nav_slam_to_point.launch.py',
            'launch/real_cmd.launch.py',
            'launch/real_no_cmd_ld19.launch.py',
            'launch/real_cmd_ld19.launch.py',
        ]),
        ('share/' + package_name + '/configs', [
            'configs/slam_toolbox.yaml',
            'configs/controller_server.yaml',
            'configs/planner_server.yaml',
            'configs/bt_navigator.yaml',
            'configs/behavior_server.yaml',
            'configs/lidar.yaml',
            'configs/odom.yaml',
            'configs/cmd_vel.yaml',
            'configs/ekf_odom.yaml',
            'configs/ld_lidar.yaml'
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/rviz_config.rviz',
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
