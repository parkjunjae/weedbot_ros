from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction  # ← 추가

def generate_launch_description():
    return LaunchDescription([

        # ---- octomap_server: TF/드라이버 워밍업 후 시작(3초 지연) ----
        TimerAction(
            period=3.0,
            actions=[Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                output='screen',
                parameters=[{
                    'frame_id': 'map',          
                    'base_frame_id': 'base_link',
                    'resolution': 0.05,
                    'publish_2d_map': True,
                    'publish_2d_map_period_sec': 0.2,
                    'occupancy_min_z': 0.10,
                    'occupancy_max_z': 1.2,
                    'pointcloud_min_z': -0.10,
                    'pointcloud_max_z': 1.2,
                    'filter_speckles': True,
                    'sensor_model.max_range': 6.0,
                    'sensor_model.min_range': 0.3,
                    'sensor_model.hit_prob': 0.72,
                    'sensor_model.miss_prob': 0.40,
                    'sensor_model.clamping_thres_min': 0.20,
                    'sensor_model.clamping_thres_max': 0.90,
                    'queue_size': 100            
                }],
                remappings=[('cloud_in', '/cloud_registered_base')]
            )]
        ),

        # ---- Nav2 묶음: octomap가 투영맵을 1~2회 내보낸 뒤 시작(4초 지연) ----
        TimerAction(
            period=4.0,
            actions=[
                Node(package='nav2_planner', executable='planner_server',
                     name='planner_server', output='screen',
                     parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml']),

                Node(package='nav2_controller', executable='controller_server',
                     name='controller_server', output='screen',
                     parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml'],
                     remappings=[('/cmd_vel', '/cmd_vel_nav')]),

                Node(package='nav2_smoother', executable='smoother_server',
                     name='smoother_server', output='screen',
                     parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml']),

                Node(package='nav2_behaviors', executable='behavior_server',
                     name='behavior_server', output='screen',
                     parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml'],
                     remappings=[('/cmd_vel', '/cmd_vel_nav')]),

                Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                     name='velocity_smoother', output='screen',
                     parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml'],
                     remappings=[('/cmd_vel', '/cmd_vel_nav'),
                                 ('/cmd_vel_smoothed', '/cmd_vel')]),

                Node(package='nav2_bt_navigator', executable='bt_navigator',
                     name='bt_navigator', output='screen',
                     parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml']),

                Node(package='nav2_waypoint_follower', executable='waypoint_follower',
                     name='waypoint_follower', output='screen',
                     parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml']),

                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_navigation', output='screen',
                     parameters=[{
                         'autostart': True,
                         'bond_disable_heartbeat_timeout': True,
                         'node_names': [
                             'controller_server','planner_server',
                             'behavior_server','bt_navigator',
                             'waypoint_follower','velocity_smoother',
                             'smoother_server'
                         ]
                     }]),
            ]
        ),
    ])

