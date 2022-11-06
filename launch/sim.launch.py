from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        DeclareLaunchArgument('gz_bridge_config_file',
            default_value='/ros2/src/r2b2/gazebo/config/gz_bridge.params.yaml'
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

    return LaunchDescription(actions)
