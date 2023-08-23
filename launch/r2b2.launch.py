from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    actions = []

    # ----------------
    # Launch Arguments
    # ----------------
    actions.append(DeclareLaunchArgument('test_mode', default_value='False'))
    actions.append(DeclareLaunchArgument('sim_mode', default_value='False'))
    actions.append(DeclareLaunchArgument('run_nav', default_value='True'))
    actions.append(DeclareLaunchArgument('slam', default_value='False'))
    actions.append(DeclareLaunchArgument('run_core_nodes', default_value='True'))

    actions.append(DeclareLaunchArgument('ros_ws', default_value='/ros2'))
    actions.append(
        DeclareLaunchArgument('robot_model_file',
            default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/urdf/r2b2.xacro']
        )
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
        DeclareLaunchArgument('ekf_param_file',
            default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/params/ekf.yaml']
        )
    )

    actions.append(LogInfo(msg=['Arg: test_mode = ', LaunchConfiguration('test_mode')]))
    actions.append(LogInfo(msg=['Arg: sim_mode = ', LaunchConfiguration('sim_mode')]))
    actions.append(LogInfo(msg=['Arg: run_nav = ', LaunchConfiguration('run_nav')]))
    actions.append(LogInfo(msg=['Arg: slam = ', LaunchConfiguration('slam')]))
    actions.append(LogInfo(msg=['Arg: run_core_nodes = ', LaunchConfiguration('run_core_nodes')]))
    actions.append(LogInfo(msg=['Arg: ros_ws = ', LaunchConfiguration('ros_ws')]))
    actions.append(LogInfo(msg=['Arg: robot_model_file = ', LaunchConfiguration('robot_model_file')]))
    actions.append(LogInfo(msg=['Arg: nav_params_file = ', LaunchConfiguration('nav_params_file')]))
    actions.append(LogInfo(msg=['Arg: map_file = ', LaunchConfiguration('map_file')]))

    # -----------------------------
    # Hardware robot nodes
    # (Not used during simulations)
    # -----------------------------
    actions.append(
        GroupAction(
            condition=LaunchConfigurationEquals('sim_mode', 'False'),
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
                        'odom_topic': 'odometry/wheels',
                        'speed_command_topic': 'roboclaw/speed_command',
                        'roboclaw_front_stats_topic': 'roboclaw/stats',
                        'log_level': 'info',
                        'publish_odom_tf': False
                    }]
                ),

                Node(
                    name='rplidar',
                    condition=LaunchConfigurationEquals('test_mode', 'False'),
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
                    condition=LaunchConfigurationEquals('test_mode', 'False'),
                    launch_description_source=[FindPackageShare('realsense2_camera'), '/launch/rs_launch.py'],
                    launch_arguments={
                        'align_depth.enable': 'True'
                    }.items()
                ),
            ]
        )
    )

    # ------------------------
    # Robot Localization (EKF)
    # ------------------------
    actions.append(
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                LaunchConfiguration('ekf_param_file')
                # {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/odometry/filtered', '/base/odom/filtered')
            ]
        )
    )

    # ---------------------
    # Robot State Publisher
    # ---------------------
    actions.append(
        Node(
            condition=LaunchConfigurationEquals('run_core_nodes', 'True'),
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro', ' ', LaunchConfiguration('robot_model_file')]),
                    value_type=str),
            }]
        )
    )

    # ----
    # Nav2
    # ----
    include_nav2 = IncludeLaunchDescription(
        launch_description_source=[LaunchConfiguration('ros_ws'), '/src/r2b2/launch/nav2/bringup_launch.py'],
        launch_arguments={
            'ros_ws': LaunchConfiguration('ros_ws'),
            #   'namespace': '',
            #   'use_namespace': 'false'
            'slam': LaunchConfiguration('slam'),
            'map': LaunchConfiguration('map_file'),
            # 'use_sim_time': 'true',
            'params_file': LaunchConfiguration('nav_params_file'),
            #   'autostart':
            # 'use_composition': 'False',
            #   'use_respawn': 'true'
        }.items()
    )

    actions.append(
        TimerAction(
            condition=LaunchConfigurationEquals('run_nav', 'True'),
            period=1.0,
            actions=[include_nav2]
        )
    )

    return LaunchDescription(actions)
