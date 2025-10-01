from setuptools import setup
from glob import glob
import os

package_name = 'pwm_ctrl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='PWM control over ROS2 topic (Jetson.GPIO)',
    license='',
    entry_points={
        'console_scripts': [
            'servo_pwm_node = pwm_ctrl.servo_pwm_node:main',
        ],
    },
)

