# Co-Simulation Platform 2 — Step-by-Step Guide
### Ubuntu VM ↔ Raspberry Pi 4 over WiFi

---

## Architecture

```
WiFi Network (same subnet, ROS_DOMAIN_ID=42)
                    │
        ┌───────────┴───────────────────────────────────────┐
        │                                                   │
  Ubuntu VM                                          Raspberry Pi 4
  ─────────────────────────────────          ────────────────────────────
  coordinator_node.py                        NOW (1 RPi4):
  controller_node.py × 4  (ZMQ)               robot_sim_node × 4
                 │                             /robot_1, /robot_2,
  formation_planner_node                       /robot_3, /robot_4
     │ /robot_N/cmd_vel ──────────WiFi────→    each: cmd_vel → Euler → odom
     └─ /robot_N/odom  ←──────────WiFi────     FUTURE (4 RPi4s):
                                                 Each RPi4: robot_sim_node
                                                 (swap → linorobot2 bringup)
```

**Key point:** `formation_planner_node.py` is **identical** to Platform 1.
Only the launch files change. The planner publishes to `/robot_N/cmd_vel` and
reads `/robot_N/odom` — it never knows or cares whether those topics are served
by a simulation on the same machine or a real robot across WiFi.

---

## Part A0 — Network Setup (do this once)

### ssh into pi
```bash
ssh pi@192.168.178.61
```
OR
```bash
ssh pi@robot1.local
```

### A.1 — Set the same ROS_DOMAIN_ID on all machines

Every machine participating in the ROS2 graph must use the same domain ID.
Add to `~/.bashrc` on **both** the VM and the RPi4:

```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

### A.2 — Verify both machines are on the same subnet

```bash
# On VM
ip addr show   # note the IP, e.g., 192.168.1.100

# On RPi4
ip addr show   # should be 192.168.1.xxx — same subnet
ping 192.168.1.100  # must reach the VM
```

### A.3 — Verify ROS2 cross-machine discovery

On VM, start a test publisher:
```bash
ros2 topic pub /test std_msgs/msg/String "data: 'hello'" --rate 1
```

On RPi4 (in a second terminal):
```bash
ros2 topic echo /test
```

You should see `data: hello` on the RPi4. If not, check:
- Both machines use the same `ROS_DOMAIN_ID`
- No firewall blocking UDP ports 7400–7500 (`sudo ufw allow 7400:7500/udp`)
- Both machines are on the same WiFi access point (not guest network isolation)

### A.4 — Install the package on the RPi4

The RPi4 needs the `formation_cosim_ros2` package AND `consensus_config.py`.

```bash
# On RPi4: clone/copy your project files
git clone <your-repo> ~/formation_project
# OR:  scp -r /path/to/project ubuntu@<rpi4-ip>:~/formation_project

# Build the ROS2 package on the RPi4
mkdir -p ~/ros2_ws/src
cp -r ~/formation_project/formation_cosim_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
sudo apt install ros-humble-nav-msgs ros-humble-diagnostic-msgs -y
colcon build --packages-select formation_cosim_ros2
source install/setup.bash
```

---

## Part B — Running Platform 2 (One RPi4, 4 Simulated Robots)

### Step 1 — Start the RPi4 fleet (run FIRST)

```bash
# On RPi4 terminal
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch formation_cosim_ros2 platform2_rpi4_fleet_launch.py \
    project_dir:=/home/ubuntu/formation_project
```

Wait until you see all four:
```
[robot_sim_1] [R1] Initial odom published — planner can start.
[robot_sim_2] [R2] Initial odom published — planner can start.
[robot_sim_3] [R3] Initial odom published — planner can start.
[robot_sim_4] [R4] Initial odom published — planner can start.
```

### Step 2 — Verify topics are visible from the VM

```bash
# On VM (separate terminal)
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
ros2 topic list | grep robot
# Should show: /robot_1/cmd_vel  /robot_1/odom  /robot_1/diagnostics  (× 4)

ros2 topic hz /robot_1/odom   # should show messages arriving
```

### Step 3 — Start the ZMQ processes on the VM

```bash
# VM terminal A
cd /path/to/project && python coordinator_node.py

# VM terminals B–E
python controller_node.py --agent-id 1
python controller_node.py --agent-id 2
python controller_node.py --agent-id 3
python controller_node.py --agent-id 4
```

### Step 4 — Launch the VM planner

```bash
# VM terminal F
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export PYTHONPATH=/path/to/project:$PYTHONPATH

ros2 launch formation_cosim_ros2 platform2_vm_launch.py \
    project_dir:=/path/to/project
```

### Step 5 — Monitor cross-machine communication

```bash
# On VM — watch robot 1 position arriving from RPi4:
ros2 topic echo /robot_1/odom --field pose.pose.position

# On RPi4 — watch cmd_vel arriving from VM:
ros2 topic echo /robot_1/cmd_vel

# Check rates:
ros2 topic hz /robot_1/cmd_vel   # fires at 1/cfg.dt Hz during active steps
ros2 topic hz /robot_1/odom      # fires in response to each cmd_vel
```

---

## Part C — Migrating to Four RPi4s (When They Arrive)

Each RPi4 runs one `robot_sim_node` for its assigned robot.

### On RPi4 for robot 1:
```bash
ros2 launch formation_cosim_ros2 platform2_rpi4_single_launch.py \
    robot_id:=1 \
    project_dir:=/home/ubuntu/formation_project
```

### On RPi4 for robot 2:
```bash
ros2 launch formation_cosim_ros2 platform2_rpi4_single_launch.py \
    robot_id:=2 \
    project_dir:=/home/ubuntu/formation_project
```

(Repeat for robots 3 and 4.)

The VM planner launch is unchanged. All four RPi4s must have the same
`ROS_DOMAIN_ID=42` and be on the same WiFi subnet.

---

## Part D — Migrating to Real Hardware (linorobot2 + ESP32)

When a robot's hardware is assembled and linorobot2 is installed on its RPi4:

### D.1 — Install linorobot2 on the RPi4 (follow linorobot2 docs)
```bash
cd ~/ros2_ws/src
git clone https://github.com/linorobot/linorobot2
# Follow linorobot2 installation instructions...
colcon build
```

### D.2 — Run bringup INSTEAD of robot_sim_node

On the RPi4 for robot i, replace:
```bash
ros2 launch formation_cosim_ros2 platform2_rpi4_single_launch.py \
    robot_id:=1 simulation:=true          # ← simulation
```

With:
```bash
ros2 launch formation_cosim_ros2 platform2_rpi4_single_launch.py \
    robot_id:=1 simulation:=false         # ← real hardware (linorobot2)
```

Or use linorobot2_bringup directly (after adding the topic remapping):
```bash
ros2 launch linorobot2_bringup bringup.launch.py robot_base:=2wd joy:=false
```

**The VM formation_planner_node requires ZERO code changes.**
It still publishes `/robot_1/cmd_vel` and reads `/robot_1/odom`.
linorobot2's diff_drive_controller subscribes to `/cmd_vel` and publishes
to `/odom`; you need a topic remapping to add the `/robot_i/` prefix.
See the commented block in `platform2_rpi4_single_launch.py`.

### D.3 — Mixed mode (some real, some simulated)

During the transition, robot 1 can be real while robots 2–4 are simulated:
- RPi4_1: `simulation:=false` → linorobot2 real hardware
- RPi4_2: `simulation:=true`  → robot_sim_node
- RPi4_3: `simulation:=true`  → robot_sim_node
- RPi4_4: `simulation:=true`  → robot_sim_node

The planner sees all four on the same topics and never knows the difference.

---

## Summary: What Changed vs. Platform 1

| Aspect                    | Platform 1 (VM only)          | Platform 2 (VM + RPi4)           |
|---------------------------|-------------------------------|-----------------------------------|
| Fleet node                | `unicycle_fleet` on VM        | `robot_sim` × N on RPi4          |
| Planner node              | `formation_planner` on VM     | same, unchanged                   |
| Topic transport           | Shared memory (loopback DDS)  | WiFi UDP (DDS multicast)          |
| Launch file (VM)          | `formation_cosim_launch.py`   | `platform2_vm_launch.py`          |
| Launch file (RPi4)        | —                             | `platform2_rpi4_fleet_launch.py`  |
| `formation_planner_node.py` | unchanged                   | unchanged                         |
| coordinator + controllers | unchanged                     | unchanged                         |
| startup_delay_s           | 2.0 s                         | 3.0 s (cross-machine DDS needs more time) |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| VM cannot see `/robot_1/odom` | Different `ROS_DOMAIN_ID` | Set same ID on both; restart nodes |
| VM cannot see `/robot_1/odom` | WiFi subnet isolation / firewall | `sudo ufw allow 7400:7500/udp` on both |
| Planner times out waiting for odom | RPi4 fleet not started first | Start RPi4 first, wait for "Initial odom published" |
| Planner times out waiting for odom | `startup_delay_s` too short | Increase to 5.0 in `platform2_vm_launch.py` |
| High latency on `/cmd_vel` | WiFi congestion | Use 5GHz band; reduce other WiFi traffic |
| RPi4 can't import consensus_config | `project_dir` path wrong | Verify path contains `consensus_config.py` |
| colcon build fails on RPi4 | Missing ROS2 packages | `sudo apt install ros-humble-nav-msgs ros-humble-diagnostic-msgs` |
