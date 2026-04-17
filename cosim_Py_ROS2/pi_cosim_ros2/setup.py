from setuptools import setup

PACKAGE_NAME = 'formation_cosim_ros2'

setup(
    name=PACKAGE_NAME,
    version='2.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch', [
            # Platform 1 launch files
            'launch/formation_cosim_launch.py',
            # Platform 2 launch files
            'launch/platform2_vm_launch.py',
            'launch/platform2_rpi4_fleet_launch.py',
            'launch/platform2_rpi4_single_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Navid',
    description='ROS2 Co-Simulation Platforms 1 & 2 for hybrid MPC formation control',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Platform 1: N-robot fleet simulation (runs on VM)
            'unicycle_fleet    = formation_cosim_ros2.unicycle_fleet_node:main',
            # Platforms 1 & 2: MPC planning loop (always runs on VM)
            'formation_planner = formation_cosim_ros2.formation_planner_node:main',
            # Platform 2: single-robot simulation (runs on each RPi4)
            'robot_sim         = formation_cosim_ros2.robot_sim_node:main',
        ],
    },
)
