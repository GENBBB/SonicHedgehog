from setuptools import setup

package_name = 'task1'

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
    description='Task1 mission',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'task1 = task1.task1:main',
        ],
    },
)
