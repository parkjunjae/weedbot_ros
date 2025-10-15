from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    lat   = DeclareLaunchArgument('lat',  default_value='37.000100')
    lon   = DeclareLaunchArgument('lon',  default_value='127.000100')
    yaw   = DeclareLaunchArgument('yaw',  default_value='0.0')
    datum = DeclareLaunchArgument('datum', default_value=os.path.join(
        get_package_share_directory('gps_goal_sender'), 'config', 'datum.yaml'))
        

    proc = ExecuteProcess(
        cmd=['python3', '-m', 'gps_goal_sender.send_nav_goal',
             '--lat', LaunchConfiguration('lat'),
             '--lon', LaunchConfiguration('lon'),
             '--yaw', LaunchConfiguration('yaw'),
             '--datum', LaunchConfiguration('datum')],
        output='screen'
    )
    return LaunchDescription([lat, lon, yaw, datum, proc])

