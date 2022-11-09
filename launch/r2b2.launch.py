from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    actions = []

    actions.append(
        DeclareLaunchArgument('ros_ws', default_value='/ros2')
    )
    actions.append(
        DeclareLaunchArgument('robot_model_file',
            default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/urdf/r2b2.xacro']
        )
    )
    actions.append(
        DeclareLaunchArgument('run_nav', default_value='true')
    )
    actions.append(
        DeclareLaunchArgument('nav_params_file',
            default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/params/nav2_params.yaml']
        )
    )
    actions.append(
        DeclareLaunchArgument('map_file',
            default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/map/b2-downstairs4.yaml']
        )
    )
    actions.append(
        DeclareLaunchArgument('test_mode', default_value='false')
    )
    actions.append(
        DeclareLaunchArgument('sim_mode', default_value='false')
    )
    actions.append(LogInfo(msg=['Arg: ros_ws = ', LaunchConfiguration('ros_ws')]))
    actions.append(LogInfo(msg=['Arg: robot_model_file = ', LaunchConfiguration('robot_model_file')]))
    actions.append(LogInfo(msg=['Arg: run_nav = ', LaunchConfiguration('run_nav')]))
    actions.append(LogInfo(msg=['Arg: nav_params_file = ', LaunchConfiguration('nav_params_file')]))
    actions.append(LogInfo(msg=['Arg: map_file = ', LaunchConfiguration('map_file')]))
    actions.append(LogInfo(msg=['Arg: test_mode = ', LaunchConfiguration('test_mode')]))
    actions.append(LogInfo(msg=['Arg: sim_mode = ', LaunchConfiguration('sim_mode')]))

    actions.append(
        # Hardware robot nodes. Not used during simulations
        GroupAction(
            condition=LaunchConfigurationEquals('sim_mode', 'false'),
            actions=[
                Node(
                    name='roboclaw',
                    package='roboclaw_driver2',
                    executable='roboclaw_node',
                    parameters=[{
                        'loop_hz': 20,
                        'speed_cmd_topic': 'roboclaw/speed_command',
                        'stats_topic': 'roboclaw/stats',
                        'test_mode': LaunchConfiguration('test_mode'),
                        'dev_names': '/dev/ttyACM0,/dev/ttyACM1'
                    }]
                ),

                Node(
                    name='base_node',
                    package='r2b2_base',
                    executable='base_node',
                    parameters=[{
                        'loop_hz': 20,
                        'cmd_vel_topic': 'cmd_vel',
                        'odom_topic': 'odom',
                        'speed_command_topic': 'roboclaw/speed_command',
                        'roboclaw_front_stats_topic': 'roboclaw/stats',
                        'log_level': 'info',
                        'publish_odom_tf': True
                    }]
                ),

                Node(
                    name='rplidar',
                    condition=LaunchConfigurationEquals('test_mode', 'false'),
                    package='rplidar_ros',
                    executable='rplidar_composition',
                    parameters=[{
                        'serial_port': '/dev/ttyUSB0',
                        'serial_baudrate': 115200,
                        'frame_id': 'scanner_link',
                        'inverted': False,
                        'angle_compensate': True
                    }]
                ),

                IncludeLaunchDescription(
                    condition=LaunchConfigurationEquals('test_mode', 'false'),
                    launch_description_source=[FindPackageShare('realsense2_camera'), '/launch/rs_launch.py'],
                    launch_arguments={
                        'test_mode': 'true'
                    }.items()
                ),
            ]
        )
    )

    actions.append(
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro', ' ', LaunchConfiguration('robot_model_file')]),
                    value_type=str),
                # 'use_sim_time': LaunchConfiguration('sim_mode'),
            }]
        )
    )

    actions.append(
        Node(
            name='joint_state_publisher',
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{
                'use_gui': False,
                # 'use_sim_time': LaunchConfiguration('sim_mode'),
            }]
        )
    )

    actions.append(
        IncludeLaunchDescription(
            condition=LaunchConfigurationEquals('run_nav', 'true'),
            launch_description_source=[LaunchConfiguration('ros_ws'), '/src/r2b2/launch/nav2/bringup_launch.py'],
            launch_arguments={
                #   'namespace': '',
                #   'use_namespace': 'false'
                #   'slam': 'false'
                'map': LaunchConfiguration('map_file'),
                # 'use_sim_time': LaunchConfiguration('sim_mode'),
                # 'use_sim_time': 'true',
                'params_file': LaunchConfiguration('nav_params_file'),
                #   'autostart':
                # 'use_composition': 'False',
                #   'use_respawn': 'true'
            }.items()
        )
    )

    return LaunchDescription(actions)
