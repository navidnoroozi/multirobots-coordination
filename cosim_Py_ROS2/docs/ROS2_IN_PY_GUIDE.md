# Step-by-Step Guide towards creating a ROS2 Package via Python

## Source the ROS2 enviroments
```bash
source ~/.bashrc
```
## Make a ROS2 Workspace and build it
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
colocon build
source ~/.bashrc
```
👉 Watch out https://www.youtube.com/watch?v=3GbrKQ7G2P0&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy&index=3 for more details on setting up the ROS2 and Packages Enviroments

## Make a ROS2 Package
```bash
ros2 pkg create formation_cosim_ros2 --build-type ament_python --dependencies rclpy
# -> formation_cosim_ros2:  your_pkg_name
cd ~/ros2_ws
colocon build --packages-select formation_cosim_ros2
source ~/.bashrc
```