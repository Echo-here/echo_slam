from setuptools import setup
from glob import glob

package_name = 'echo_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
     data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/map', glob('map/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bitbyte08',
    maintainer_email='me@bitworkspace.kr',
    description='SLAM launch package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_odom_node=echo_slam.dummy_odom_node:main',
            'map_utils_node=echo_slam.map_utils_node:main',
            'mqtt_node=echo_slam.mqtt_node:main'
        ],
    },
)
