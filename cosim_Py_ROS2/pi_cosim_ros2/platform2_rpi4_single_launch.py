"""
platform2_rpi4_single_launch.py
================================
Raspberry Pi 4 launch file: ONE robot per RPi4.

Use this launch file when the additional three RPi4s arrive.
Run it on each RPi4 with a different robot_id (1..4).

STAGE 1 — Simulation (no hardware, RPi4 only):
  Each RPi4 runs one robot_sim_node, simulating the diff-drive kinematics.
  Run this file with simulation:=true (default).

STAGE 2 — Real hardware (linorobot2 installed, ESP32 connected):
  Set simulation:=false.  This file then launches linorobot2_bringup instead,
  which connects to the actual motors, encoders, and IMU via the ESP32.
  The VM formation_planner_node requires ZERO changes.

STARTUP (one RPi4 per robot):
  RPi4 for robot 1:
    export ROS_DOMAIN_ID=42
    ros2 launch formation_cosim_ros2 platform2_rpi4_single_launch.py \
        robot_id:=1 project_dir:=/home/ubuntu/formation_project

  RPi4 for robot 2:
    ros2 launch formation_cosim_ros2 platform2_rpi4_single_launch.py \
        robot_id:=2 project_dir:=/home/ubuntu/formation_project
  (etc.)

LAUNCH ARGUMENTS
----------------
  robot_id         : int   1-based robot index (1..4) — REQUIRED
  simulation       : bool  true=use robot_sim_node, false=use linorobot2 (default true)
  alpha            : float Motor lag for simulation mode (default 1.0)
  project_dir      : str   Path to consensus_config.py (simulation mode only)
  robot_base       : str   linorobot2 base type: '2wd'|'4wd' (hardware mode only)
  log_level        : str   ROS2 log level (default 'info')
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IfCondition,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument(
            'robot_id',
            description='1-based robot index (1, 2, 3, or 4) — REQUIRED',
        ),
        DeclareLaunchArgument('simulation',  default_value='true'),
        DeclareLaunchArgument('alpha',       default_value='1.0'),
        DeclareLaunchArgument('robot_base',  default_value='2wd'),
        DeclareLaunchArgument('log_level',   default_value='info'),
        DeclareLaunchArgument(
            'project_dir',
            default_value=os.getcwd(),
            description='Path to directory containing consensus_config.py',
        ),
    ]

    # PYTHONPATH is only needed for the simulation node
    set_pythonpath = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=[
            LaunchConfiguration('project_dir'),
            ':',
            EnvironmentVariable('PYTHONPATH', default_value=''),
        ],
    )

    # ── STAGE 1: simulation node ───────────────────────────────────────────
    # Launched when simulation:=true (default).
    # Simulates one robot with the event-driven unicycle kinematics.
    sim_node = Node(
        package='formation_cosim_ros2',
        executable='robot_sim',
        name=PythonExpression(["'robot_sim_' + '", LaunchConfiguration('robot_id'), "'"]),
        output='screen',
        condition=IfCondition(LaunchConfiguration('simulation')),
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'alpha':    LaunchConfiguration('alpha'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # ── STAGE 2: linorobot2 bringup ────────────────────────────────────────
    # Launched when simulation:=false.
    # Requires linorobot2 to be installed on this RPi4.
    #
    # linorobot2_bringup publishes:
    #   /cmd_vel          ← geometry_msgs/Twist  (receives from planner)
    #   /odom             → nav_msgs/Odometry    (EKF-fused, sends to planner)
    #   /odom/unfiltered  → nav_msgs/Odometry    (raw wheel odometry)
    #   /imu/data         → sensor_msgs/Imu
    #
    # The formation_planner_node uses /robot_N/cmd_vel and /robot_N/odom.
    # linorobot2 by default publishes to /cmd_vel and /odom (no namespace).
    # We remap these topics to the namespaced versions the planner expects.
    #
    # IMPORTANT: When you install linorobot2 and switch to hardware mode,
    # uncomment the IncludeLaunchDescription block below and comment out sim_node.
    # The formation_planner_node on the VM requires NO changes.
    #
    # ─────────────────────────────────────────────────────────────────────
    # HARDWARE ACTIVATION (uncomment when linorobot2 is installed):
    # ─────────────────────────────────────────────────────────────────────
    #
    # from launch.actions import IncludeLaunchDescription
    # from launch.launch_description_sources import PythonLaunchDescriptionSource
    # from ament_index_python.packages import get_package_share_directory
    #
    # linorobot2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('linorobot2_bringup'),
    #             'launch', 'bringup.launch.py'
    #         )
    #     ),
    #     launch_arguments={
    #         'robot_base': LaunchConfiguration('robot_base'),
    #         'joy': 'false',
    #     }.items(),
    #     condition=IfCondition(
    #         PythonExpression(["not ", LaunchConfiguration('simulation')])
    #     ),
    # )
    #
    # # Topic remapping: /cmd_vel → /robot_N/cmd_vel, /odom → /robot_N/odom
    # # Add to the IncludeLaunchDescription or use a separate remapping node.
    # ─────────────────────────────────────────────────────────────────────

    return LaunchDescription([
        *args,
        set_pythonpath,
        LogInfo(msg=['[Platform 2 RPi4 Single] robot_id=', LaunchConfiguration('robot_id')]),
        LogInfo(msg=['  simulation=', LaunchConfiguration('simulation')]),
        LogInfo(msg='  Ensure ROS_DOMAIN_ID matches the VM.'),
        sim_node,
        # linorobot2_launch,   # ← uncomment for hardware mode
    ])
