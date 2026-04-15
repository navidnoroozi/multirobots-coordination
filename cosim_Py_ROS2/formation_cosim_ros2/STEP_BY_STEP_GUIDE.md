# Formation Co-Simulation Platform 1 (Full) — Step-by-Step Guide
### Hybrid MPC Formation Control → ROS2 Unicycle Fleet on Ubuntu VM

---

## Architecture

```
Ubuntu 22.04 VM
┌─────────────────────────────────────────────────────────────────────────┐
│                                                                         │
│  ─── ZMQ layer (unchanged from pure-Python platform) ──────────────── │
│  │ coordinator_node.py │  │ controller_node.py ×4 │                   │
│  │   port 5555         │  │   ports 5601–5604      │                   │
│  └─────────────────────┘  └────────────────────────┘                   │
│            ↑↓ ZMQ REQ/REP                  ↑↓ ZMQ REQ/REP              │
│  ─── ROS2 layer (new) ─────────────────────────────────────────────── │
│                                                                         │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │  formation_planner_node   (replaces plant_node.py)             │    │
│  │                                                                │    │
│  │  ┌──────────────┐   ZMQ   ┌─────────────────────────────────┐ │    │
│  │  │ Planning     │ ──────→ │ coordinator_node.py             │ │    │
│  │  │ thread (MPC) │ ←────── │   + controller_node.py × 4     │ │    │
│  │  └──────┬───────┘         └─────────────────────────────────┘ │    │
│  │         │  u_safe (Cartesian)                                  │    │
│  │         │  → cartesian_to_twist(ux, uy, θ) → (v, ω)           │    │
│  │         ↓                                                      │    │
│  │  /robot_N/cmd_vel ─────────────────────────→ (Twist)           │    │
│  │  /robot_N/odom    ←────────────────────────── (Odometry)       │    │
│  │                                                                │    │
│  │  CSV logs → ros2_cosim_logs/                                   │    │
│  │    consensus_traj.csv     (identical to plant_node.py)         │    │
│  │    consensus_metrics.csv  (identical to plant_node.py)         │    │
│  │    consensus_outer.csv    (identical to plant_node.py)         │    │
│  │    hybrid_modes.csv       (identical to plant_node.py)         │    │
│  │    unicycle_state.csv     (NEW: x, y, θ, v, ω, heading_error) │    │
│  └────────────────────────────────────────────────────────────────┘    │
│                                                                         │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │  unicycle_fleet_node   (replaces single-integrator dynamics)   │    │
│  │                                                                │    │
│  │  Robot 1: /robot_1/cmd_vel ← /robot_1/odom → (20 Hz)         │    │
│  │  Robot 2: /robot_2/cmd_vel ← /robot_2/odom → (20 Hz)         │    │
│  │  Robot 3: /robot_3/cmd_vel ← /robot_3/odom → (20 Hz)         │    │
│  │  Robot 4: /robot_4/cmd_vel ← /robot_4/odom → (20 Hz)         │    │
│  └────────────────────────────────────────────────────────────────┘    │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## What Changes vs. the Pure-Python Platform

| Pure-Python platform           | ROS2 Co-Simulation Platform 1      |
|--------------------------------|------------------------------------|
| `plant_node.py`                | `formation_planner_node` (ROS2)    |
| Single-integrator dynamics     | Unicycle kinematics @ 20 Hz (ROS2) |
| Internal state `r += dt * u`   | `time.sleep(dt) + odom feedback`   |
| No topic communication         | `/robot_N/cmd_vel` + `/robot_N/odom` |
| 4 CSV files                    | Same 4 + `unicycle_state.csv`      |
| `coordinator_node.py`          | **Unchanged**                      |
| `controller_node.py × 4`       | **Unchanged**                      |

---

## File Structure

```
ros2_ws/
└── src/
    └── formation_cosim_ros2/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/formation_cosim_ros2
        ├── launch/
        │   └── formation_cosim_launch.py
        └── formation_cosim_ros2/
            ├── __init__.py
            ├── cosim_csv_logger.py
            ├── unicycle_fleet_node.py
            └── formation_planner_node.py
```

---

## Step 1 — Copy Package to Workspace

```bash
cp -r /path/to/formation_cosim_ros2  ~/ros2_ws/src/
```

---

## Step 2 — Install ROS2 Dependencies

```bash
sudo apt install ros-humble-nav-msgs ros-humble-diagnostic-msgs -y
pip install pyzmq numpy --break-system-packages
```

---

## Step 3 — Build

```bash
cd ~/ros2_ws
colcon build --packages-select formation_cosim_ros2
source install/setup.bash
```

---

## Step 4 — Start the ZMQ Processes (unchanged from pure-Python)

Open 6 terminals. In each, navigate to your project directory first.

**Terminal 1 — Coordinator:**
```bash
cd /path/to/your/project
python coordinator_node.py
```

**Terminals 2–5 — Controllers (one per agent):**
```bash
python controller_node.py --agent-id 1
python controller_node.py --agent-id 2
python controller_node.py --agent-id 3
python controller_node.py --agent-id 4
```

Wait until all five print their "REP bound at…" messages.

---

## Step 5 — Launch the ROS2 Layer

**Terminal 6:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Tell ROS2 where your project files are (consensus_config.py etc.)
export PYTHONPATH=/path/to/your/project:$PYTHONPATH

ros2 launch formation_cosim_ros2 formation_cosim_launch.py \
    project_dir:=/path/to/your/project
```

Or with custom parameters:
```bash
ros2 launch formation_cosim_ros2 formation_cosim_launch.py \
    project_dir:=/path/to/your/project   \
    outer_steps:=24                       \
    safety_enabled:=true                  \
    obstacles_enabled:=true               \
    k_heading:=4.0                        \
    k_speed:=1.0                          \
    log_dir:=ros2_cosim_logs
```

---

## Step 6 — Monitor the Running System

**In any sourced terminal:**

```bash
# Node graph
ros2 node list
# /unicycle_fleet
# /formation_planner

# Topic list
ros2 topic list
# /robot_1/cmd_vel   /robot_1/odom   /robot_1/diagnostics
# /robot_2/cmd_vel   /robot_2/odom   /robot_2/diagnostics
# ...

# Watch robot 1 position in real time
ros2 topic echo /robot_1/odom --field pose.pose.position

# Check rates
ros2 topic hz /robot_1/odom      # should be ~20 Hz
ros2 topic hz /robot_1/cmd_vel   # fires once per dt_inner (1 Hz at default settings)

# Visual graph
rqt_graph

# Plot trajectories
rqt_plot /robot_1/odom/pose/pose/position/x /robot_1/odom/pose/pose/position/y \
         /robot_2/odom/pose/pose/position/x /robot_2/odom/pose/pose/position/y
```

---

## Step 7 — Verify Safety Objectives from CSV Logs

After the run completes, `ros2_cosim_logs/` contains:

### Obstacle Avoidance (Objective O)
```python
import pandas as pd
df = pd.read_csv('ros2_cosim_logs/consensus_metrics.csv')
print("Min obstacle distance:", df['dmin_obstacles'].min())
# Must be > 0 (agents never penetrate obstacle + margin)
assert (df['dmin_obstacles'] > 0).all(), "SAFETY VIOLATION: obstacle penetration!"
```

### Inter-Agent Collision Avoidance (Objective C)
```python
d_safe = 1.10  # from NetConfig
print("Min inter-agent distance:", df['dmin_agents'].min())
assert (df['dmin_agents'] >= d_safe).all(), "SAFETY VIOLATION: agent collision!"
```

### Hybrid Mode Timeline
```python
df_modes = pd.read_csv('ros2_cosim_logs/hybrid_modes.csv')
# Count how many steps each agent spent in each mode
print(df_modes.groupby(['agent', 'mode']).size().unstack(fill_value=0))
```

### Unicycle Heading Tracking Quality
```python
df_uni = pd.read_csv('ros2_cosim_logs/unicycle_state.csv')
print("Max heading error (deg):", df_uni['heading_error_deg'].abs().max())
print("Mean heading error (deg):", df_uni['heading_error_deg'].abs().mean())
```

---

## Step 8 — Timing Model Explained

```
Outer loop (outer_j = 0..outer_steps-1):
│
├── ZMQ: coordinator_node → U_nom  (MPC solve, blocking ~50-200ms)
│
└── Inner loop (inner_t = 0..M-1):
    │
    ├── ZMQ: controller_node × N → u_safe  (hybrid filter, ~5-20ms per agent)
    ├── cartesian_to_twist(ux, uy, θ) → (v, ω)
    ├── Publish Twist to /robot_N/cmd_vel
    ├── time.sleep(cfg.dt = 1.0 s)  ← unicycle fleet integrates 20 × 50ms steps
    ├── Read /robot_N/odom → updated r, v, θ
    └── Log CSV
```

Total wall time ≈ outer_steps × M × cfg.dt seconds
= 18 × 5 × 1.0 = 90 seconds for default settings.

---

## Step 9 — Moving to Co-Simulation Platform 2 (RPi4)

The only change from Platform 1 to Platform 2:

| Platform 1 (this)              | Platform 2 (RPi4)                         |
|--------------------------------|-------------------------------------------|
| `unicycle_fleet_node` (VM)     | linorobot2 on each RPi4                   |
| `/robot_N/cmd_vel` via DDS     | `/robot_N/cmd_vel` via WiFi DDS           |
| Simulation @ 20 Hz             | Real motors @ ~50 Hz                      |
| formation_planner_node: **no change needed** | formation_planner_node: **no change needed** |

The topic interface (`/robot_N/cmd_vel` Twist, `/robot_N/odom` Odometry) is
identical. The planner does not know or care whether the subscriber is a
simulated fleet or a physical robot running linorobot2.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: consensus_config` | PYTHONPATH not set | `export PYTHONPATH=/path/to/project:$PYTHONPATH` |
| `[PLAN] Timed out waiting for odom` | `unicycle_fleet` not started | Check fleet node is running |
| ZMQ timeout on coordinator | coordinator_node.py not running | Start all 5 ZMQ processes first |
| `/robot_N/odom` rate < 20 Hz | Executor overloaded | Reduce n_agents or increase num_threads |
| `dmin_agents` < `d_safe` in CSV | Safety violation | Check safety_enabled:=true; inspect hybrid_modes.csv |
| Robots not moving after MPC solve | cartesian_to_twist heading error > 90° | Increase k_heading or reduce dt_inner |
