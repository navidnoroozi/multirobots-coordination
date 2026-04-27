# RPi4 & ROS2 Humble Cheat Sheet

## SSH into Pi
```bash
ssh pi@192.168.178.61
```
OR
```bash
ssh pi@robot1.local
```

## List of installed packages of ROS2
Before sourcing such as the package rqt-plot
```bash
dpkg -l | grep ros-humble
```
Or after sourcing ROS 2:
```bash
ros2 pkg list
```
To verify if a specific package is installed
```bash
ros2 pkg list | grep rqt_plot
```