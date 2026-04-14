# Co-Simulation Platform 1 (Mini) — Step-by-Step Guide
### Python Planning Layer ↔ ROS2 Motion Controller on Ubuntu VM

---

## Overview of What You Are Building

```
Ubuntu VM
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│   ┌────────────────────┐        ┌─────────────────────────────┐ │
│   │   planner_node.py  │        │ motion_controller_node.py   │ │
│   │  (Python planning) │        │  (ROS2 motion control)      │ │
│   │                    │        │                             │ │
│   │  • generates v, ω  │──────→ │  • receives v, ω            │ │
│   │    from a circular │ /cmd_vel│  • integrates unicycle      │ │
│   │    trajectory      │        │    kinematics (x, y, θ)     │ │
│   │                    │ /odom  │  • publishes odom @ 20 Hz   │ │
│   │  • receives pose   │←──────  │                             │ │
│   │    (x, y, θ) and  │        │  • publishes diagnostics     │ │
│   │    health signals  │←──────  │    @ 1 Hz                   │ │
│   │                    │ /diag  │                             │ │
│   └────────────────────┘        └─────────────────────────────┘ │
│                                                                 │
│               ROS2 DDS middleware (shared memory, same machine) │
└─────────────────────────────────────────────────────────────────┘
```

**Analogy with your MATLAB co-simulation:**

| MATLAB platform            | This ROS2 mini platform             |
|----------------------------|-------------------------------------|
| `coordinator_node.py`      | `planner_node.py`                   |
| MATLAB Engine API + ZMQ    | ROS2 DDS (topics)                   |
| Simulink unicycle model    | `motion_controller_node.py`         |
| `cosim_bridge.py` loop     | ROS2 timer callbacks                |
| CSV logger                 | `ros2 topic echo` / `rqt_plot`      |

---

## File Structure

```
ros2_cosim_platform1/                 ← your ROS2 workspace (create this)
└── src/
    └── formation_cosim_mini/         ← the ROS2 package
        ├── package.xml               ← package metadata + dependencies
        ├── setup.py                  ← build descriptor (entry points)
        ├── setup.cfg                 ← ament_python install paths
        ├── resource/
        │   └── formation_cosim_mini  ← ament marker file (empty, required)
        ├── launch/
        │   └── cosim_mini_launch.py  ← starts both nodes together
        └── formation_cosim_mini/
            ├── __init__.py
            ├── motion_controller_node.py
            └── planner_node.py
```

---

## Step 0 — Understand the Key ROS2 Concepts (Read This First)

### 0.1 The ROS2 Computation Graph

ROS2 organises software as a **graph of nodes communicating over topics**.

- **Node**: One Python process (one .py file with `rclpy.init()` + `rclpy.spin()`).
  Think of it as one process in your `multiprocessing.Process()` pool.

- **Topic**: A named, typed message bus. Any node can publish to it or subscribe
  to it. No direct function calls between nodes — only messages.

- **Message**: A typed data structure (like a dataclass). Defined by ROS2
  (e.g., `geometry_msgs/Twist`) or custom. Serialised automatically by DDS.

- **Publisher**: Sends messages TO a topic. Like `zmq_socket.send()`.

- **Subscriber**: Receives messages FROM a topic. Like `zmq_socket.recv()`,
  but callback-based — you register a function that ROS2 calls for you.

- **Timer**: Fires a callback at a fixed rate. Like `threading.Timer`, but
  managed by the ROS2 executor.

- **Executor**: The event loop (`rclpy.spin()`). Dispatches callbacks as
  messages arrive or timers fire. Equivalent to your `asyncio.run()`.

### 0.2 Topics in This Platform

| Topic          | Direction          | Message Type             | Rate   |
|----------------|--------------------|--------------------------|--------|
| `/cmd_vel`     | planner → motor    | `geometry_msgs/Twist`    | 5 Hz   |
| `/odom`        | motor → planner    | `nav_msgs/Odometry`      | 20 Hz  |
| `/diagnostics` | motor → planner    | `diagnostic_msgs/...`    | 1 Hz   |

### 0.3 The Twist Message (cmd_vel)

```
geometry_msgs/Twist:
  linear:
    x: <forward speed v [m/s]>   ← the one you care about
    y: 0.0                        ← always 0 for diff-drive
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: <yaw rate ω [rad/s]>      ← the one you care about
```

### 0.4 The Odometry Message

```
nav_msgs/Odometry:
  header:
    stamp: <ROS2 timestamp>
    frame_id: 'odom'              ← the fixed world frame
  child_frame_id: 'base_link'    ← the robot body frame
  pose:
    pose:
      position: {x, y, z}        ← robot position in world frame
      orientation: {x,y,z,w}    ← heading as quaternion
  twist:
    twist:
      linear: {x, y, z}          ← velocity in body frame
      angular: {x, y, z}
```

---

## Step 1 — Create the ROS2 Workspace

A **workspace** is just a directory with a `src/` subdirectory where you put
ROS2 packages. `colcon` (the build tool) will add `build/`, `install/`,
`log/` directories when you build.

```bash
# On your Ubuntu VM
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

---

## Step 2 — Copy the Package

Copy the `formation_cosim_mini/` directory into `~/ros2_ws/src/`:

```bash
# If you download the files from your dev machine:
cp -r /path/to/formation_cosim_mini ~/ros2_ws/src/

# Verify the structure
ls ~/ros2_ws/src/formation_cosim_mini/
# Should show: package.xml  setup.py  setup.cfg  resource/  launch/  formation_cosim_mini/
```

---

## Step 3 — Source ROS2 and Install Dependencies

Every time you open a new terminal you must **source** ROS2 to make its tools
available. This sets up PATH, PYTHONPATH, and ROS-specific environment variables.

```bash
source /opt/ros/humble/setup.bash
```

**Tip:** Add this to your `~/.bashrc` so it happens automatically:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install any missing Python message packages:
```bash
sudo apt install ros-humble-nav-msgs ros-humble-diagnostic-msgs -y
```

---

## Step 4 — Build the Package with colcon

`colcon` is the ROS2 build tool. It reads `package.xml` to understand
dependencies and `setup.py` to install Python entry points.

```bash
cd ~/ros2_ws

# Build only our package (faster than building everything)
colcon build --packages-select formation_cosim_mini

# Expected output:
# Starting >>> formation_cosim_mini
# Finished <<< formation_cosim_mini [2.34s]
# Summary: 1 package finished [2.50s]
```

**What `colcon build` actually does (for an ament_python package):**
1. Reads `package.xml` — validates dependencies are installed.
2. Runs `pip install -e .` inside the package — installs the Python module.
3. Registers the `entry_points` from `setup.py` as shell scripts in
   `install/formation_cosim_mini/lib/formation_cosim_mini/`.
4. Creates `install/setup.bash` — the workspace overlay.

---

## Step 5 — Source the Workspace Overlay

After building, you must source the **workspace overlay** to make the newly
built package available to `ros2 run` and `ros2 launch`.

```bash
source ~/ros2_ws/install/setup.bash
```

**Why two sources?**  
`/opt/ros/humble/setup.bash` — the base ROS2 installation.  
`~/ros2_ws/install/setup.bash` — your workspace, layered on top.

---

## Step 6 — Run the Nodes (Three Methods)

### Method A: Two Terminals (Recommended for Learning)

**Terminal 1 — Motion Controller:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run formation_cosim_mini motion_controller
```

**Terminal 2 — Planner:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run formation_cosim_mini planner
```

### Method B: Launch File (One Command)
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch formation_cosim_mini cosim_mini_launch.py
```

### Method C: Debug Mode (Verbose Logging)
```bash
ros2 launch formation_cosim_mini cosim_mini_launch.py log_level:=debug
```

---

## Step 7 — Inspect the Running System (Crucial Learning Step)

While both nodes are running, open additional terminals and try these commands.
These are your primary debugging tools in ROS2.

### 7.1 List all active nodes
```bash
ros2 node list
# Output:
# /motion_controller
# /planner
```

### 7.2 List all active topics
```bash
ros2 topic list
# Output:
# /cmd_vel
# /diagnostics
# /odom
# /parameter_events
# /rosout
```

### 7.3 Watch messages in real time
```bash
# See the velocity commands being sent
ros2 topic echo /cmd_vel

# See the odometry being published
ros2 topic echo /odom

# See diagnostic reports
ros2 topic echo /diagnostics
```

### 7.4 Check publishing rates
```bash
ros2 topic hz /odom        # Should show ~20 Hz
ros2 topic hz /cmd_vel     # Should show ~5 Hz
ros2 topic hz /diagnostics # Should show ~1 Hz
```

### 7.5 Inspect a node's publishers and subscribers
```bash
ros2 node info /motion_controller
# Output:
#   Subscribers:
#     /cmd_vel: geometry_msgs/msg/Twist
#   Publishers:
#     /odom: nav_msgs/msg/Odometry
#     /diagnostics: diagnostic_msgs/msg/DiagnosticArray
```

### 7.6 Visualise the topic graph
```bash
rqt_graph
```
This opens a GUI showing nodes as ovals and topics as arrows — exactly the
diagram above.

### 7.7 Plot odometry values over time
```bash
rqt_plot /odom/pose/pose/position/x /odom/pose/pose/position/y
```
You should see sinusoidal curves — the robot is moving in a circle!

---

## Step 8 — Expected Output

**Terminal running motion_controller:**
```
[INFO] [motion_controller]: MotionControllerNode started (dt=0.050 s, 20 Hz)
[INFO] [motion_controller]: [DIAG] x= 0.010 m,  y= 0.001 m,  θ=  0.0°
[INFO] [motion_controller]: [DIAG] x= 0.203 m,  y= 0.101 m,  θ=  30.2°
...
```

**Terminal running planner:**
```
[INFO] [planner]: PlannerNode started  (planning rate = 5 Hz)
[INFO] [planner]: [PLAN]  t=  0.20 s   cmd → v=0.200 m/s,  ω=0.500 rad/s ...
[INFO] [planner]: [ODOM]  x= 0.010 m   y= 0.001 m   θ=  0.0°   v= 0.200 m/s
[INFO] [planner]: [DIAG]  motion_controller/unicycle_sim  [OK]  — x_m=0.0100, ...
...
[INFO] [planner]: [PLAN]  *** Completed 1 lap(s) of the demo circle ***
```

---

## Step 9 — What You Have Learned

| Concept                          | Where you saw it                                |
|----------------------------------|-------------------------------------------------|
| Node creation + naming           | `super().__init__('motion_controller')`         |
| Publisher creation               | `self.create_publisher(Twist, 'cmd_vel', 10)`   |
| Subscriber creation + callback   | `self.create_subscription(Odometry, ...)`       |
| Timer-driven simulation loop     | `self.create_timer(0.05, self.simulation_step)` |
| Unicycle kinematics in ROS2      | `simulation_step()` in motion_controller        |
| Quaternion encoding/decoding     | `_publish_odometry()` / `_odom_callback()`      |
| Structured diagnostics           | `DiagnosticArray`, `DiagnosticStatus`, `KeyValue`|
| Concurrent callbacks             | `MultiThreadedExecutor`, `ReentrantCallbackGroup`|
| ROS2 graph inspection            | `ros2 node list`, `ros2 topic echo`, `rqt_graph`|
| Launch files                     | `cosim_mini_launch.py`                          |

---

## Step 10 — What Changes for the Full Formation System

| Mini Platform                         | Full Formation System (Co-Sim Platform 1)       |
|---------------------------------------|-------------------------------------------------|
| One robot, circular trajectory        | Four robots, MPC formation trajectory           |
| `planner_node.py`: v, ω = fixed       | `coordinator_node.py` + hybrid MPC solver       |
| Topic: `/cmd_vel`                     | Topics: `/robot_1/cmd_vel`, ..., `/robot_4/cmd_vel` |
| `motion_controller_node.py`: Euler    | linorobot2 diff-drive controller (real or sim)  |
| Same machine (shared memory DDS)      | Platform 2: RPi4 over WiFi (UDP DDS)            |

The **topic interface** (`/cmd_vel` Twist, `/odom` Odometry) is identical between
this mini platform and the real linorobot2 robot. That is the whole point:
once you replace the motion_controller_node with the real RPi4, the planner
does not need to change at all.

---

## Troubleshooting

| Symptom                              | Cause                              | Fix                                      |
|--------------------------------------|------------------------------------|------------------------------------------|
| `ros2: command not found`            | ROS2 not sourced                   | `source /opt/ros/humble/setup.bash`      |
| `Package not found`                  | Workspace not sourced after build  | `source ~/ros2_ws/install/setup.bash`    |
| No messages on `/odom`               | motion_controller not running      | Start in a separate terminal             |
| `ModuleNotFoundError: rclpy`         | Wrong Python or virtualenv active  | Deactivate venv; use system Python       |
| Nodes visible but no data flowing    | ROS_DOMAIN_ID mismatch             | `export ROS_DOMAIN_ID=0` in both shells  |
| `colcon build` fails                 | Missing message package            | `sudo apt install ros-humble-nav-msgs`   |
