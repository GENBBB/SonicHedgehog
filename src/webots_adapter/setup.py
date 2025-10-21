from setuptools import setup, find_packages

package_name = 'webots_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/webots_adapter_launch.py']),
    ],
    install_requires=['setuptools', 'transforms3d'],
    zip_safe=True,
    maintainer='GENBBB',
    maintainer_email='gengensav@gmail.com',
    description='ROS2 adapter for Webots simulation: telemetry and cmd_vel interface',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'webots_adapter_node = webots_adapter.adapter_node:main',
        ],
    },
)
