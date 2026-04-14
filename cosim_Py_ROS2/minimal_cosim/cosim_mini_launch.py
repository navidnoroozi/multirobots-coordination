"""
cosim_mini_launch.py
====================
ROS2 launch file — starts both nodes in a single terminal command.

WHAT IS A LAUNCH FILE?
-----------------------
A launch file is a Python (or XML/YAML) script that tells ROS2 which nodes
to start, with what arguments, in what order.  It replaces running multiple
`ros2 run ...` commands in separate terminals.

Run with:
    ros2 launch formation_cosim_mini cosim_mini_launch.py

Or with verbose logging:
    ros2 launch formation_cosim_mini cosim_mini_launch.py log_level:=debug
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    generate_launch_description() is the required entry point for any ROS2
    launch file.  It must return a LaunchDescription object.
    """

    # ── Declare a launch argument so the user can override the log level ───
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level: debug | info | warn | error',
    )

    # ── Motion Controller Node ─────────────────────────────────────────────
    motion_controller_node = Node(
        package='formation_cosim_mini',
        executable='motion_controller',
        name='motion_controller',
        output='screen',                       # print logs to terminal
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        parameters=[
            # You can pass ROS parameters here — used in the full system
            # to set, e.g., max_linear_speed, wheel_separation, etc.
        ],
    )

    # ── Planner Node ───────────────────────────────────────────────────────
    planner_node = Node(
        package='formation_cosim_mini',
        executable='planner',
        name='planner',
        output='screen',
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        log_level_arg,
        LogInfo(msg='Starting Formation Co-Sim Platform 1 (Mini)...'),
        motion_controller_node,
        planner_node,
    ])
