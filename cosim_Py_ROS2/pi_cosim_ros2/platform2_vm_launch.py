"""
platform2_vm_launch.py
=======================
Ubuntu VM launch file for Co-Simulation Platform 2.

Starts ONLY the formation_planner_node on the VM.
The fleet (robot simulations) runs on the RPi4.

WHAT RUNS WHERE
---------------
Ubuntu VM (this launch file):
  formation_planner_node  ← MPC + ZMQ + CSV logging

Raspberry Pi 4 (platform2_rpi4_fleet_launch.py):
  robot_sim_node × 4      ← one simulated robot each in /robot_1..4

STARTUP ORDER
-------------
  RPi4 terminal:
    ros2 launch formation_cosim_ros2 platform2_rpi4_fleet_launch.py \
        project_dir:=/path/to/project

  VM terminal A:  python coordinator_node.py
  VM terminal B–E: python controller_node.py --agent-id 1..4

  VM terminal F (this file):
    ros2 launch formation_cosim_ros2 platform2_vm_launch.py \
        project_dir:=/path/to/project

NETWORK REQUIREMENTS
--------------------
  • Both VM and RPi4 on the same WiFi subnet
  • Same ROS_DOMAIN_ID on both machines (export ROS_DOMAIN_ID=42)
  • RPi4 hostname or IP reachable from VM (DDS multicast handles discovery)

LAUNCH ARGUMENTS
----------------
  project_dir      : str  Path to directory containing consensus_config.py
  n_agents         : int  Number of robots (default 4)
  model            : str  'single_integrator' | 'double_integrator'
  outer_steps      : int  MPC outer iterations (default 18)
  safety_enabled   : bool (default true)
  obstacles_enabled: bool (default true)
  k_speed          : float Cartesian→unicycle speed scale (default 1.0)
  log_dir          : str  CSV output directory (default 'ros2_cosim_logs')
  log_level        : str  ROS2 log level (default 'info')
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument('n_agents',          default_value='4'),
        DeclareLaunchArgument('model',             default_value='single_integrator'),
        DeclareLaunchArgument('outer_steps',       default_value='18'),
        DeclareLaunchArgument('safety_enabled',    default_value='true'),
        DeclareLaunchArgument('obstacles_enabled', default_value='true'),
        DeclareLaunchArgument('k_speed',           default_value='1.0'),
        DeclareLaunchArgument('log_dir',           default_value='ros2_cosim_logs'),
        DeclareLaunchArgument('log_level',         default_value='info'),
        DeclareLaunchArgument(
            'project_dir',
            default_value=os.getcwd(),
            description='Path to directory containing consensus_config.py',
        ),
    ]

    set_pythonpath = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=[
            LaunchConfiguration('project_dir'),
            ':',
            EnvironmentVariable('PYTHONPATH', default_value=''),
        ],
    )

    # formation_planner_node — identical to Platform 1.
    # k_heading is auto-computed as 1/cfg.dt (deadbeat) inside the node.
    planner_node = Node(
        package='formation_cosim_ros2',
        executable='formation_planner',
        name='formation_planner',
        output='screen',
        parameters=[{
            'n_agents':          LaunchConfiguration('n_agents'),
            'model':             LaunchConfiguration('model'),
            'outer_steps':       LaunchConfiguration('outer_steps'),
            'safety_enabled':    LaunchConfiguration('safety_enabled'),
            'obstacles_enabled': LaunchConfiguration('obstacles_enabled'),
            'k_speed':           LaunchConfiguration('k_speed'),
            'log_dir':           LaunchConfiguration('log_dir'),
            'startup_delay_s':   3.0,   # extra time for cross-machine DDS discovery
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        *args,
        set_pythonpath,
        LogInfo(msg='[Platform 2 VM] Starting formation_planner_node...'),
        LogInfo(msg=['  project_dir: ', LaunchConfiguration('project_dir')]),
        LogInfo(msg=['  CSV logs  -> ', LaunchConfiguration('log_dir')]),
        LogInfo(msg='  Fleet running on RPi4 — ensure same ROS_DOMAIN_ID on both machines.'),
        planner_node,
    ])
