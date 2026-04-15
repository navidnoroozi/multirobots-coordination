"""
formation_cosim_launch.py
=========================
ROS2 launch file for the full Formation Co-Simulation Platform 1.

WHAT THIS STARTS
----------------
  1. unicycle_fleet_node    — N simulated unicycle robots at 20 Hz
  2. formation_planner_node — hybrid MPC planning loop + ZMQ bridge + CSV logs

WHAT YOU START SEPARATELY (unchanged Python processes)
------------------------------------------------------
  Terminal A:  python coordinator_node.py
  Terminal B:  python controller_node.py --agent-id 1
  Terminal C:  python controller_node.py --agent-id 2
  Terminal D:  python controller_node.py --agent-id 3
  Terminal E:  python controller_node.py --agent-id 4

Then in Terminal F:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  export PYTHONPATH=/path/to/your/project:$PYTHONPATH
  ros2 launch formation_cosim_ros2 formation_cosim_launch.py

LAUNCH ARGUMENTS (all optional — defaults match NetConfig)
-----------------------------------------------------------
  n_agents         : int   Number of robots (default 4)
  model            : str   'single_integrator' | 'double_integrator' (default si)
  outer_steps      : int   MPC outer iterations (default 18)
  safety_enabled   : bool  Hybrid safety filter on/off (default true)
  obstacles_enabled: bool  Obstacle avoidance on/off (default true)
  k_heading        : float Proportional heading gain (default 4.0)
  k_speed          : float Speed feedforward scale (default 1.0)
  sim_rate         : float Unicycle integration rate in Hz (default 20.0)
  log_dir          : str   CSV output directory (default 'ros2_cosim_logs')
  log_level        : str   ROS2 log level: debug|info|warn (default info)
  project_dir      : str   Path to your MPC project (sets PYTHONPATH)

EXAMPLE — override obstacle avoidance off, 30 outer steps:
  ros2 launch formation_cosim_ros2 formation_cosim_launch.py \
      obstacles_enabled:=false outer_steps:=30

EXAMPLE — specify project directory explicitly:
  ros2 launch formation_cosim_ros2 formation_cosim_launch.py \
      project_dir:=/home/navid/my_formation_project
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Declare all tunable launch arguments ──────────────────────────────
    args = [
        DeclareLaunchArgument('n_agents',          default_value='4'),
        DeclareLaunchArgument('model',             default_value='single_integrator'),
        DeclareLaunchArgument('outer_steps',       default_value='18'),
        DeclareLaunchArgument('safety_enabled',    default_value='true'),
        DeclareLaunchArgument('obstacles_enabled', default_value='true'),
        DeclareLaunchArgument('k_heading',         default_value='4.0'),
        DeclareLaunchArgument('k_speed',           default_value='1.0'),
        DeclareLaunchArgument('sim_rate',          default_value='20.0'),
        DeclareLaunchArgument('log_dir',           default_value='ros2_cosim_logs'),
        DeclareLaunchArgument('log_level',         default_value='info'),

        # Path to the directory containing consensus_config.py, consensus_comm.py, etc.
        # Default: current working directory (works if you launch from your project dir).
        DeclareLaunchArgument(
            'project_dir',
            default_value=os.getcwd(),
            description='Absolute path to the MPC project containing consensus_config.py',
        ),
    ]

    # ── Prepend project_dir to PYTHONPATH ─────────────────────────────────
    # This makes consensus_config, consensus_comm, etc. importable by the
    # formation_planner_node without installing them as a package.
    set_pythonpath = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=PythonExpression([
            '"', LaunchConfiguration('project_dir'), '"',
            ' + ":" + (os.environ.get("PYTHONPATH") or "")',
        ]),
    )

    # ── Shared parameter list (passed to both nodes) ───────────────────────
    # ROS2 parameters are passed as a list of dicts to the Node action.
    fleet_params = [{
        'n_agents':  LaunchConfiguration('n_agents'),
        'sim_rate':  LaunchConfiguration('sim_rate'),
        'diag_rate': 1.0,
    }]

    planner_params = [{
        'n_agents':          LaunchConfiguration('n_agents'),
        'model':             LaunchConfiguration('model'),
        'outer_steps':       LaunchConfiguration('outer_steps'),
        'safety_enabled':    LaunchConfiguration('safety_enabled'),
        'obstacles_enabled': LaunchConfiguration('obstacles_enabled'),
        'k_heading':         LaunchConfiguration('k_heading'),
        'k_speed':           LaunchConfiguration('k_speed'),
        'log_dir':           LaunchConfiguration('log_dir'),
        'dt_inner':          1.0,       # must match cfg.dt in NetConfig
        'startup_delay_s':   2.0,
    }]

    # ── Node: unicycle fleet ───────────────────────────────────────────────
    fleet_node = Node(
        package='formation_cosim_ros2',
        executable='unicycle_fleet',
        name='unicycle_fleet',
        output='screen',
        parameters=fleet_params,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # ── Node: formation planner ────────────────────────────────────────────
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
        LogInfo(msg='Starting Formation Co-Simulation Platform 1 (Full MPC)…'),
        LogInfo(msg=['CSV logs will be written to: ', LaunchConfiguration('log_dir')]),
        fleet_node,
        planner_node,
    ])
