from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler,
    EmitEvent
)
from launch.conditions import LaunchConfigurationEquals
from launch.events import Shutdown
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.substitutions import LaunchConfiguration, LocalSubstitution
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    actions = []

    arg_ros_ws = DeclareLaunchArgument('ros_ws', default_value='/ros2')
    actions.append(arg_ros_ws)

    arg_world_file = DeclareLaunchArgument('world_file',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/gazebo/worlds/downstairs.sdf'])
    actions.append(arg_world_file)

    arg_models_path = DeclareLaunchArgument('models_path',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/gazebo/models'])
    actions.append(arg_models_path)

    arg_gui_config_file = DeclareLaunchArgument('gui_config',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/gazebo/config/gui.config'])
    actions.append(arg_gui_config_file)

    arg_run_ros_nodes = DeclareLaunchArgument('run_ros_nodes', default_value='true')
    actions.append(arg_run_ros_nodes)

    arg_gz_bridge_config_file = DeclareLaunchArgument('gz_bridge_config_file',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/gazebo/config/gz_bridge.params.yaml']
    )
    actions.append(arg_gz_bridge_config_file)

    actions.append(LogInfo(msg=['Arg: ros_ws = ', LaunchConfiguration('ros_ws')]))
    actions.append(LogInfo(msg=['Arg: world_file = ', LaunchConfiguration('world_file')]))
    actions.append(LogInfo(msg=['Arg: models_path = ', LaunchConfiguration('models_path')]))
    actions.append(LogInfo(msg=['Arg: gui_config = ', LaunchConfiguration('gui_config')]))
    actions.append(LogInfo(msg=['Arg: run_ros_nodes = ', LaunchConfiguration('run_ros_nodes')]))
    actions.append(LogInfo(msg=['Arg: gz_bridge_config_file = ', LaunchConfiguration('gz_bridge_config_file')]))

    # ------------------------
    # Args from r2b2.launch.py
    # ------------------------
    arg_robot_model_file = DeclareLaunchArgument('robot_model_file',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/urdf/r2b2.xacro']
    )
    actions.append(arg_robot_model_file)

    arg_run_nav = DeclareLaunchArgument('run_nav', default_value='true')
    actions.append(arg_run_nav)

    arg_nav_params_file = DeclareLaunchArgument('nav_params_file',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/params/nav2_params.yaml']
    )
    actions.append(arg_nav_params_file)

    arg_map_file = DeclareLaunchArgument('map_file',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/map/b2-downstairs4.yaml']
    )
    actions.append(arg_map_file)

    actions.append(SetParameter("use_sim_time", True))

    # ----------
    # Gazebo Sim
    # ----------
    exec_gazebo = ExecuteProcess(
        cmd=[
            'gz sim --render-engine ogre -r ',
            '--gui-config ', LaunchConfiguration('gui_config'),
            ' ', LaunchConfiguration('world_file')],
        additional_env={'GZ_SIM_RESOURCE_PATH': LaunchConfiguration('models_path')},
        shell=True
    )
    actions.append(exec_gazebo)

    # ROS GZ Bridge
    node_gz_bridge = Node(
        name='ros_gz_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': LaunchConfiguration('gz_bridge_config_file')
        }]
    )
    actions.append(node_gz_bridge)

    # ----------------
    # R2B2 in sim mode
    # ----------------
    include_r2b2 = IncludeLaunchDescription(
        condition=LaunchConfigurationEquals('run_ros_nodes', 'true'),
        launch_description_source=[LaunchConfiguration('ros_ws'), '/src/r2b2/launch/r2b2.launch.py'],
        launch_arguments=[
            ('sim_mode', 'true'),
            ('robot_model_file', LaunchConfiguration('robot_model_file')),
            ('run_nav', LaunchConfiguration('run_nav')),
            ('nav_params_file', LaunchConfiguration('nav_params_file')),
            ('map_file', LaunchConfiguration('map_file'))
        ]
    )
    actions.append(include_r2b2)

    # ------------
    # Rviz and RQT
    # ------------
    include_display = IncludeLaunchDescription(
        condition=LaunchConfigurationEquals('run_ros_nodes', 'true'),
        launch_description_source=[LaunchConfiguration('ros_ws'), '/src/r2b2/launch/display.launch.yaml'],
        # launch_arguments=[
        #     ('use_sim_time', 'true')
        # ]
    )
    actions.append(include_display)

    handle_gazebo_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=exec_gazebo,
            on_exit=[
                LogInfo(msg='Gazebo exited, shutting down'),
                EmitEvent(event=Shutdown(reason='Gazeobo exited'))
            ]
        )
    )
    actions.append(handle_gazebo_exit)

    handle_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(msg=['Launch was asked to shutdown because: ', LocalSubstitution('event.reason')])]
        )
    )
    actions.append(handle_on_shutdown)

    return LaunchDescription(actions)
