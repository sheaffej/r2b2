from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler,
    EmitEvent, TimerAction
)
from launch.conditions import LaunchConfigurationEquals
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    actions = []

    actions.append(DeclareLaunchArgument('ros_ws', default_value='/ros2'))

    actions.append(DeclareLaunchArgument('world_file',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/gazebo/worlds/downstairs.sdf'])
    )

    actions.append(DeclareLaunchArgument('models_path',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/gazebo/models'])
    )

    actions.append(DeclareLaunchArgument('gui_config',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/gazebo/config/gui.config'])
    )

    actions.append(DeclareLaunchArgument('gz_bridge_config_file',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/gazebo/config/gz_bridge.params.yaml'])
    )

    actions.append(DeclareLaunchArgument('run_ros_nodes', default_value='true'))
    actions.append(DeclareLaunchArgument('run_display', default_value='true'))

    actions.append(LogInfo(msg=['Arg: ros_ws = ', LaunchConfiguration('ros_ws')]))
    actions.append(LogInfo(msg=['Arg: world_file = ', LaunchConfiguration('world_file')]))
    actions.append(LogInfo(msg=['Arg: models_path = ', LaunchConfiguration('models_path')]))
    actions.append(LogInfo(msg=['Arg: gui_config = ', LaunchConfiguration('gui_config')]))
    actions.append(LogInfo(msg=['Arg: gz_bridge_config_file = ', LaunchConfiguration('gz_bridge_config_file')]))
    actions.append(LogInfo(msg=['Arg: run_ros_nodes = ', LaunchConfiguration('run_ros_nodes')]))
    actions.append(LogInfo(msg=['Arg: run_display = ', LaunchConfiguration('run_display')]))

    # ------------------------
    # Args from r2b2.launch.py
    # ------------------------
    actions.append(DeclareLaunchArgument('robot_model_file',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/urdf/r2b2.xacro'])
    )

    actions.append(DeclareLaunchArgument('run_nav', default_value='true'))

    actions.append(DeclareLaunchArgument('nav_params_file',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/params/nav2_params.yaml'])
    )

    actions.append(DeclareLaunchArgument('map_file',
        default_value=[LaunchConfiguration('ros_ws'), '/src/r2b2/config/map/b2-downstairs4.yaml'])
    )

    actions.append(SetParameter("use_sim_time", True))

    # -------------------
    # Convert xacro files
    # -------------------
    exec_xacro_r2b2 = ExecuteProcess(
        cmd=['xacro ', (LaunchConfiguration('ros_ws'), '/src/r2b2/config/urdf/r2b2.xacro'), ' > /tmp/r2b2.urdf'],
        shell=True
    )
    actions.append(exec_xacro_r2b2)

    exec_xacro_walls = ExecuteProcess(
        cmd=['xacro ', (LaunchConfiguration('ros_ws'), '/src/r2b2/gazebo/models/walls.xacro'), ' > /tmp/walls.sdf'],
        shell=True
    )
    actions.append(
        RegisterEventHandler(
            OnProcessStart(target_action=exec_xacro_r2b2, on_start=[exec_xacro_walls])
        )
    )

    # ----------
    # Run Gazebo
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

    # -------------
    # ROS GZ Bridge
    # -------------
    node_gz_bridge = Node(
        name='ros_gz_bridge',
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': LaunchConfiguration('gz_bridge_config_file')
        }]
    )

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

    # ------------
    # Rviz and RQT
    # ------------
    include_display = IncludeLaunchDescription(
        condition=LaunchConfigurationEquals('run_display', 'true'),
        launch_description_source=[LaunchConfiguration('ros_ws'), '/src/r2b2/launch/display.launch.yaml'],
    )

    actions.append(
        RegisterEventHandler(
            OnProcessStart(
                target_action=exec_gazebo,
                on_start=[
                    node_gz_bridge,
                    TimerAction(period=2.0, actions=[include_r2b2]),
                    TimerAction(period=2.0, actions=[include_display])
                ]
            )
        )
    )

    # ------------------
    # Handle Gazebo exit
    # ------------------
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=exec_gazebo,
                on_exit=[
                    LogInfo(msg='Gazebo exited, shutting down'),
                    EmitEvent(event=Shutdown(reason='Gazeobo exited'))
                ]
            )
        )
    )

    return LaunchDescription(actions)
