"""
platform2_rpi4_fleet_launch.py
===============================
Raspberry Pi 4 launch file for Co-Simulation Platform 2 (one RPi4).

Starts N robot_sim_node instances on a single RPi4, each in its own
namespace /robot_1 .. /robot_N.  This is the temporary solution until
the additional three RPi4s arrive; then each RPi4 runs one robot via
platform2_rpi4_single_launch.py.

WHAT RUNS WHERE
---------------
This RPi4 (this launch file):
  robot_sim_node × N   ← N simulated robots in /robot_1.../robot_N

Ubuntu VM (platform2_vm_launch.py):
  formation_planner_node, coordinator_node.py, controller_node.py × N

STARTUP ORDER
-------------
  RPi4 terminal (run FIRST so initial odom is ready before planner starts):
    export ROS_DOMAIN_ID=42
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch formation_cosim_ros2 platform2_rpi4_fleet_launch.py \
        project_dir:=/home/ubuntu/formation_project

  VM (run AFTER RPi4 nodes are up — wait for "[Ri] Initial odom published"):
    export ROS_DOMAIN_ID=42
    ros2 launch formation_cosim_ros2 platform2_vm_launch.py ...

LAUNCH ARGUMENTS
----------------
  n_agents         : int   Number of robots to simulate (default 4)
  alpha            : float Motor lag ∈ (0,1]: 1.0=perfect (default)
  project_dir      : str   Path to directory containing consensus_config.py
  log_level        : str   ROS2 log level (default 'info')
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument('n_agents',   default_value='4'),
        DeclareLaunchArgument('alpha',      default_value='1.0'),
        DeclareLaunchArgument('log_level',  default_value='info'),
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

    # ── Read n_agents and build one Node per robot ─────────────────────────
    # ROS2 launch files are pure Python, so we can build the node list
    # dynamically.  n_agents is resolved at parse time from the default value
    # or the command-line override.  We parse it here for the loop.
    #
    # Note: LaunchConfiguration values are not plain Python ints at this point
    # in generate_launch_description() — they are substitution objects resolved
    # at runtime.  For dynamic node lists we read the default directly; the
    # command-line override is handled by reading sys.argv below.
    import sys
    n_agents = 4  # default
    for arg in sys.argv:
        if arg.startswith('n_agents:='):
            try:
                n_agents = int(arg.split(':=')[1])
            except ValueError:
                pass

    robot_nodes = []
    for i in range(1, n_agents + 1):
        robot_nodes.append(
            Node(
                package='formation_cosim_ros2',
                executable='robot_sim',
                name=f'robot_sim_{i}',
                output='screen',
                parameters=[{
                    'robot_id': i,
                    'alpha':    LaunchConfiguration('alpha'),
                    # init_x/init_y default to NaN → auto-read from consensus_config
                }],
                arguments=[
                    '--ros-args',
                    '--log-level', LaunchConfiguration('log_level'),
                ],
            )
        )

    return LaunchDescription([
        *args,
        set_pythonpath,
        LogInfo(msg=f'[Platform 2 RPi4] Starting {n_agents} robot_sim nodes...'),
        LogInfo(msg=['  project_dir: ', LaunchConfiguration('project_dir')]),
        LogInfo(msg='  Ensure ROS_DOMAIN_ID matches the VM before launching.'),
        *robot_nodes,
    ])
