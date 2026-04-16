"""
formation_cosim_launch.py
=========================
ROS2 launch file for the full Formation Co-Simulation Platform 1.

DESIGN CHANGES (event-driven fleet)
-------------------------------------
The unicycle_fleet_node is now event-driven: one cmd_vel → one sim step.
Parameters removed:  sim_rate, kv, k_omega  (obsolete, caused instability).
Parameter added:     alpha  (motor lag coefficient, default 1.0 = perfect).

STARTUP ORDER
-------------
  Terminal A:  python coordinator_node.py
  Terminals B–E: python controller_node.py --agent-id 1..4

  Terminal F:
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    ros2 launch formation_cosim_ros2 formation_cosim_launch.py \
        project_dir:=/path/to/your/project

LAUNCH ARGUMENTS
----------------
  n_agents         : int   Number of robots (default 4)
  model            : str   'single_integrator' | 'double_integrator'
  outer_steps      : int   MPC outer iterations (default 18)
  safety_enabled   : bool  (default true)
  obstacles_enabled: bool  (default true)
  k_heading        : float Cartesian→unicycle heading gain   (default 4.0)
  k_speed          : float Cartesian→unicycle speed scale    (default 1.0)
  alpha            : float Motor lag α ∈ (0,1]: 1.0=perfect, 0.8=mild lag
  log_dir          : str   CSV output directory (default 'ros2_cosim_logs')
  log_level        : str   ROS2 log level (default 'info')
  project_dir      : str   Path to directory containing consensus_config.py
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument('n_agents',          default_value='4'),
        DeclareLaunchArgument('model',             default_value='single_integrator'),
        DeclareLaunchArgument('outer_steps',       default_value='300'),
        DeclareLaunchArgument('safety_enabled',    default_value='true'),
        DeclareLaunchArgument('obstacles_enabled', default_value='true'),
        # k_heading is not exposed here: the planner auto-computes 1/cfg.dt (deadbeat).
        DeclareLaunchArgument('k_speed',           default_value='1.0'),
        DeclareLaunchArgument('alpha',             default_value='1.0'),
        DeclareLaunchArgument('log_dir',           default_value='/home/navid/git_repos/multirobots-coordination/cosim_Py_ROS2/py_env'),
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

    # Fleet: only alpha and n_agents. dt and initial positions come from
    # consensus_config imported at node startup via PYTHONPATH.
    fleet_params = [{
        'n_agents':  LaunchConfiguration('n_agents'),
        'alpha':     LaunchConfiguration('alpha'),
        'diag_rate': 1.0,
    }]

    # Planner: dt_inner is always cfg.dt (no parameter).
    planner_params = [{
        'n_agents':          LaunchConfiguration('n_agents'),
        'model':             LaunchConfiguration('model'),
        'outer_steps':       LaunchConfiguration('outer_steps'),
        'safety_enabled':    LaunchConfiguration('safety_enabled'),
        'obstacles_enabled': LaunchConfiguration('obstacles_enabled'),
        # k_heading: auto-computed as 1/cfg.dt inside the planner node
        'k_speed':           LaunchConfiguration('k_speed'),
        'log_dir':           LaunchConfiguration('log_dir'),
        'startup_delay_s':   2.0,
    }]

    fleet_node = Node(
        package='formation_cosim_ros2',
        executable='unicycle_fleet',
        name='unicycle_fleet',
        output='screen',
        parameters=fleet_params,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    planner_node = Node(
        package='formation_cosim_ros2',
        executable='formation_planner',
        name='formation_planner',
        output='screen',
        parameters=planner_params,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        *args,
        set_pythonpath,
        LogInfo(msg='Starting Formation Co-Simulation Platform 1 (Full MPC)...'),
        LogInfo(msg=['CSV logs -> ', LaunchConfiguration('log_dir')]),
        LogInfo(msg=['PYTHONPATH prepend -> ', LaunchConfiguration('project_dir')]),
        fleet_node,
        planner_node,
    ])