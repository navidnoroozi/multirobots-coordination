"""
setup.py — ROS2 ament_python build descriptor.

This replaces CMakeLists.txt for pure-Python ROS2 packages.
It tells colcon (the ROS2 build tool) how to install the package
and which Python functions serve as node entry points.
"""
from setuptools import setup

PACKAGE_NAME = 'formation_cosim_mini'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[PACKAGE_NAME],

    # ament resource index marker (required by ROS2 tooling — do not remove)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # Install the launch directory so `ros2 launch` can find it
        ('share/' + PACKAGE_NAME + '/launch', ['launch/cosim_mini_launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Navid',
    description='Minimal ROS2 co-simulation platform for formation control learning',
    license='MIT',

    # ─────────────────────────────────────────────────────────────────────────
    # entry_points: maps CLI command names → Python module:function
    # After `colcon build`, these become shell commands you can run directly:
    #   ros2 run formation_cosim_mini motion_controller
    #   ros2 run formation_cosim_mini planner
    # ─────────────────────────────────────────────────────────────────────────
    entry_points={
        'console_scripts': [
            'motion_controller = formation_cosim_mini.motion_controller_node:main',
            'planner           = formation_cosim_mini.planner_node:main',
        ],
    },
)
