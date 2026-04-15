from setuptools import setup

PACKAGE_NAME = 'formation_cosim_ros2'

setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch', ['launch/formation_cosim_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Navid',
    description='Full ROS2 Co-Simulation Platform 1 for hybrid MPC formation control',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Node that simulates N unicycle robots at 20 Hz
            'unicycle_fleet    = formation_cosim_ros2.unicycle_fleet_node:main',
            # Node that runs the MPC+hybrid planning loop (replaces plant_node.py)
            'formation_planner = formation_cosim_ros2.formation_planner_node:main',
        ],
    },
)
