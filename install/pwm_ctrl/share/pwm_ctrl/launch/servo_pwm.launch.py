from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    enable_steer   = LaunchConfiguration('enable_steer')
    mode           = LaunchConfiguration('mode')
    fwd_near_us    = LaunchConfiguration('fwd_near_us')
    fwd_far_us     = LaunchConfiguration('fwd_far_us')
    rev_near_us    = LaunchConfiguration('rev_near_us')
    rev_far_us     = LaunchConfiguration('rev_far_us')
    invert_ch2     = LaunchConfiguration('invert_ch2')
    invert_ch4     = LaunchConfiguration('invert_ch4')
    swap_channels  = LaunchConfiguration('swap_channels')
    invert_turn    = LaunchConfiguration('invert_turn_dir')
    spin_boost     = LaunchConfiguration('spin_boost_norm')
    use_spin_sign  = LaunchConfiguration('use_spin_sign')
    arming_style   = LaunchConfiguration('arming_style')
    arming_sec     = LaunchConfiguration('arming_sec')
    debug_log      = LaunchConfiguration('debug_log')

    return LaunchDescription([
        # ---- 런치 인자 (필요시 커맨드라인에서 override) ----
        DeclareLaunchArgument('enable_steer',   default_value='true'),
        DeclareLaunchArgument('mode',           default_value='simple_split'),
        DeclareLaunchArgument('fwd_near_us',    default_value='1500'),
        DeclareLaunchArgument('fwd_far_us',     default_value='1000'),
        DeclareLaunchArgument('rev_near_us',    default_value='1500'),
        DeclareLaunchArgument('rev_far_us',     default_value='2000'),
        DeclareLaunchArgument('invert_ch2',     default_value='false'),
        DeclareLaunchArgument('invert_ch4',     default_value='false'),
        DeclareLaunchArgument('swap_channels',  default_value='false'),
        DeclareLaunchArgument('invert_turn_dir',default_value='false'),
        DeclareLaunchArgument('spin_boost_norm',default_value='0.35'),
        DeclareLaunchArgument('use_spin_sign',  default_value='false'),
        DeclareLaunchArgument('arming_style',   default_value='neutral'),
        DeclareLaunchArgument('arming_sec',     default_value='2.0'),
        DeclareLaunchArgument('debug_log',      default_value='true'),

        Node(
            package='pwm_ctrl',
            executable='servo_pwm_node',
            name='throttle_steer_node',
            output='screen',
            parameters=[{
                'enable_steer': enable_steer,
                'mode': mode,
                'fwd_near_us': fwd_near_us,
                'fwd_far_us': fwd_far_us,
                'rev_near_us': rev_near_us,
                'rev_far_us': rev_far_us,
                'invert_ch2': invert_ch2,
                'invert_ch4': invert_ch4,
                'swap_channels': swap_channels,
                'invert_turn_dir': invert_turn,
                'spin_boost_norm': spin_boost,
                'use_spin_sign': use_spin_sign,
                'arming_style': arming_style,
                'arming_sec': arming_sec,
                'debug_log': debug_log,
            }]
        ),
    ])

