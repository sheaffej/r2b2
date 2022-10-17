from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package='roboclaw_driver2',
                executable='roboclaw_node',
                # name=roboclaw_node,
                parameters=[{
                    'name': 'roboclaw',
                    'speed_cmd_topic': 'roboclaw/speed_command',
                    'stats_topic': 'roboclaw/stats',
                    'test_mode': True,
                    # 'dev_names': '/dev/ttyACM0',
                    # 'baud': '115200',
                    # 'address': '128',
                    # 'loop_hz': 20,
                    # 'deadman_secs': '3'

                }],
                # arguments=['--ros-args', '--log-level', 'roboclaw_driver:=debug']
            ),

            Node(
                package='r2b2_base',
                executable='base_node',
                # name=base_node,
                parameters=[{
                    # "wheel_dist": 0.180,
                    # "wheel_radius": 0.032,
                    # "wheel_slip_factor": ,
                    # "ticks_per_rotation": ,
                    # "max_drive_secs": ,
                    # "max_qpps": ,
                    # "max_x_lin_vel": ,
                    # "max_z_ang_vel": ,
                    # "max_accel": ,
                    # "base_frame_id": ,
                    # "odom_frame_id": ,
                    "loop_hz": 1,
                    # {"deadman_secs": },
                    "cmd_vel_topic": "cmd_vel",
                    "odom_topic": "odom",
                    "speed_command_topic": "roboclaw/speed_command",
                    "roboclaw_front_stats_topic": "roboclaw/stats",
                    # "roboclaw_rear_stats_topic": "roboclaw/rear_stats",
                    "log_level": "info",
                    "publish_odom_tf": True
                }],
                # arguments=['--ros-args', '--log-level', 'base_node:=debug']
            )
        ]
    )
