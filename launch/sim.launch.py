from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    actions = []

    actions.append(
        DeclareLaunchArgument('world_file', default_value='/ros2/src/r2b2/gazebo/worlds/downstairs.sdf')
    )
    actions.append(
        DeclareLaunchArgument('models_path', default_value='/ros2/src/r2b2/gazebo/models')
    )
    actions.append(
        DeclareLaunchArgument('gui_config', default_value='/ros2/src/r2b2/gazebo/config/gui.config')
    )
    actions.append(
        DeclareLaunchArgument('run_ros_nodes', default_value='true')
    )
    actions.append(
        DeclareLaunchArgument('gz_bridge_config_file',
            default_value='/ros2/src/r2b2/gazebo/config/gz_bridge.params.yaml'
        )
    )

    # Args from r2b2.launch.py
    actions.append(
        DeclareLaunchArgument('robot_model_file',
            default_value=PathJoinSubstitution([FindPackageShare('r2b2'), 'config/urdf/r2b2.xacro'])
        )
    )
    actions.append(
        DeclareLaunchArgument('run_nav', default_value='true')
    )
    actions.append(
        DeclareLaunchArgument('nav_params_file',
            default_value=PathJoinSubstitution([FindPackageShare('r2b2'), 'config/params/nav2_params.yaml'])
        )
    )
    actions.append(
        DeclareLaunchArgument('map_file',
            default_value=PathJoinSubstitution([FindPackageShare('r2b2'), 'config/map/b2-downstairs4.yaml'])
        )
    )

    actions.append(
        ExecuteProcess(
            cmd=[
                'gz sim --render-engine ogre ',
                '--gui-config ', LaunchConfiguration('gui_config'),
                ' ', LaunchConfiguration('world_file')],
            additional_env={'GZ_SIM_RESOURCE_PATH': LaunchConfiguration('models_path')},
            shell=True
        )
    )

    actions.append(
        Node(
            name='ros_gz_bridge',
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': LaunchConfiguration('gz_bridge_config_file')
            }]
        )
    )

    actions.append(
        IncludeLaunchDescription(
            condition=LaunchConfigurationEquals('run_ros_nodes', 'true'),
            launch_description_source=PathJoinSubstitution(
                [FindPackageShare('r2b2'), 'launch', 'r2b2.launch.py']
            ),
            launch_arguments=[
                ('sim_mode', 'true'),
                ('robot_model_file', LaunchConfiguration('robot_model_file')),
                ('run_nav', LaunchConfiguration('run_nav')),
                ('nav_params_file', LaunchConfiguration('nav_params_file')),
                ('map_file', LaunchConfiguration('map_file'))
            ]
        )
    )

    actions.append(
        IncludeLaunchDescription(
            condition=LaunchConfigurationEquals('run_ros_nodes', 'true'),
            launch_description_source=PathJoinSubstitution(
                [FindPackageShare('r2b2'), 'launch', 'display.launch.yaml']
            )
        )
    )

    return LaunchDescription(actions)
