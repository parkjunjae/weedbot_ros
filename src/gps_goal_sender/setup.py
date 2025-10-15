from setuptools import setup, find_packages

package_name = 'gps_goal_sender'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['gps_goal_sender/config/datum.yaml']),
        ('share/' + package_name + '/launch', ['launch/gps_goal_sender.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Send Nav2 NavigateToPose goals from latitude/longitude',
    license='MIT',
    entry_points={
        'console_scripts': [
            'send_nav_goal = gps_goal_sender.send_nav_goal:main',
        ],
    },
)

