from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'masterpi_bringup'

setup(
    name=package_name,
    version='0.0.0',
    # CAMBIO AQUÍ: Añadimos la subcarpeta del SDK explícitamente usando puntos
    packages=[
        package_name,
        f'{package_name}.hiwonder_sdk'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        (os.path.join('share', package_name, 'config', 'hiwonder'),
            glob('config/hiwonder/*')),

        (os.path.join('share', package_name, 'config', 'calibration'),
            glob('config/calibration/*')),

        (os.path.join('share', package_name, 'actions', 'hiwonder'),
            glob('actions/hiwonder/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jp',
    maintainer_email='jp@todo.todo',
    description='ROS 2 bringup package for MasterPi robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = masterpi_bringup.motor_node:main',
            'servo_node = masterpi_bringup.servo_node:main',
            'battery_node = masterpi_bringup.battery_node:main',
            'camera_node = masterpi_bringup.camera_node:main',
            'odom_node = masterpi_bringup.odom_node:main',
            'sonar_node = masterpi_bringup.sonar_node:main',
            'aruco = masterpi_bringup.aruco:main',
            'line_follower_node = masterpi_bringup.line_follower_node:main',
        ],
    },
)