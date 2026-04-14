# Comprehensive Learning Plan: From Control Engineer to Robotics Builder

## Your Starting Point and Goal

**What you already know:** Control theory, MPC, hybrid systems, distributed consensus, Python, MATLAB/Simulink, optimization (CVXPY). You understand the *mathematics* behind your formation controller perfectly.

**What you need to learn:** How to make physical hardware obey that mathematics — which means understanding the software ecosystem (ROS2), the embedded firmware layer (ESP32 + micro-ROS), the sensor/actuator pipeline (encoders, IMU, motors), and the mechanical assembly that holds it all together.

**Goal:** Build four differential-drive robots running linorobot2, each receiving velocity commands from your distributed hybrid MPC controller over WiFi and returning odometry, so you can run your formation control experiment on real hardware.

---

## The Big Picture: How Everything Connects

Before diving into phases, here is the architecture you are building, so every concept below has a clear purpose:

```
┌─────────────────────────────────────────────────────────────────┐
│  YOUR LAPTOP (Planning Layer)                                   │
│                                                                 │
│  coordinator_node.py  ←→  controller_node.py (×4)              │
│       ↕ ZMQ                    ↕ ZMQ                           │
│  Each controller_node produces cmd_vel for one robot            │
│       ↕ WiFi (ROS2 topics)                                     │
├─────────────────────────────────────────────────────────────────┤
│  ROBOT i (RPi4 + ESP32)                                        │
│                                                                 │
│  RPi4 (Ubuntu 22.04 + ROS2 Humble):                           │
│    - micro_ros_agent bridges ESP32 ↔ ROS2 topics               │
│    - robot_localization EKF fuses wheel odom + IMU → /odom     │
│    - (optional) SLAM, Nav2, LiDAR driver                       │
│       ↕ USB serial                                             │
│  ESP32 (linorobot2_hardware firmware):                         │
│    - Subscribes to /cmd_vel (Twist msg)                        │
│    - PID-controls two DC motors via TB6612FNG                  │
│    - Reads quadrature encoders → publishes /odom/unfiltered    │
│    - Reads MPU-6050 IMU → publishes /imu/data                 │
│       ↕ Wires                                                  │
│  HARDWARE:                                                     │
│    Motors ← TB6612FNG ← ESP32 PWM                             │
│    Encoders → ESP32 GPIO (interrupt-driven)                    │
│    MPU-6050 → ESP32 I2C                                        │
└─────────────────────────────────────────────────────────────────┘
```

Every concept in this plan maps to a box or arrow in this diagram.

---

## Phase 0: Linux & Command-Line Foundations (Days 1–3)

### Why this matters
Your robots run Ubuntu 22.04. Everything — installing ROS2, flashing firmware, SSH-ing into robots, launching nodes — happens from a Linux terminal. If you are primarily a Windows user, you need basic fluency.

### What to learn
- **Linux filesystem:** `/home`, `/opt`, `/dev` (device files for USB serial), `/etc`
- **Essential commands:** `cd`, `ls`, `cat`, `nano`/`vim`, `sudo`, `apt`, `ssh`, `scp`, `chmod`, `grep`, `top`/`htop`, `systemctl`
- **Networking basics:** `ip addr`, `ping`, `hostname`, SSH key setup
- **Environment variables:** `export`, `.bashrc`, `source`, `echo $PATH` — critically important for ROS2
- **Package management:** `apt update`, `apt install`, `dpkg`

### Recommended resources
1. **"Linux for Robotics" — The Construct** (free course): Specifically designed for roboticists, covers exactly the terminal skills you need for ROS2. Available at theconstructsim.com.
2. **"The Linux Command Line" by William Shotts** — free PDF at linuxcommand.org. Read chapters 1–6 only (Navigation, Exploring, Manipulating, Working with Commands, I/O Redirection, Permissions). Skip scripting for now.
3. **YouTube: "Linux Terminal for Beginners" by NetworkChuck** (~20 min) — fast, practical overview.

### Milestone
You can SSH into a Raspberry Pi, navigate the filesystem, edit a config file with `nano`, install a package with `apt`, and set an environment variable in `.bashrc`.

---

## Phase 1: ROS2 Concepts — The Language Your Robot Speaks (Days 4–14)

### Why this matters
ROS2 (Robot Operating System 2) is not an operating system — it is a **middleware framework** that standardizes how software components on a robot communicate. Every piece of linorobot2 is a ROS2 node. Your MPC controller will send commands and receive data through ROS2 topics. Without understanding ROS2, you cannot debug or modify anything on the robot.

### Core concepts you must understand

#### 1. Nodes
A **node** is a single-purpose process. Think of it as one "module" in your distributed system. Examples: a motor driver node, a sensor driver node, an EKF node, a navigation node. Each robot runs many nodes simultaneously.

**Analogy from your background:** Each node is like one agent in your multi-agent system — it has local state, communicates with neighbors, and runs its own loop.

#### 2. Topics (publish-subscribe)
A **topic** is a named data channel. Nodes **publish** messages to topics and **subscribe** to topics. This is asynchronous, one-to-many communication. The publisher does not know who is listening.

Key topics for your project:
- `/cmd_vel` — your controller publishes velocity commands here (Twist message: linear.x = forward speed, angular.z = turning rate)
- `/odom` — the robot publishes its estimated position/velocity here (Odometry message)
- `/imu/data` — raw IMU readings (Imu message)
- `/tf` — coordinate frame transforms (where is the robot in the world?)

**Analogy:** Topics are like the ZMQ PUB/SUB sockets in your Python simulation — same pattern, different middleware.

#### 3. Messages
A **message** is a typed data structure. `geometry_msgs/msg/Twist` has fields `linear` (x, y, z) and `angular` (x, y, z). For a differential-drive robot on a plane, you only use `linear.x` (forward) and `angular.z` (yaw rate). All other fields are zero.

#### 4. Launch files
A **launch file** starts multiple nodes with configured parameters in one command. linorobot2 provides launch files that bring up the entire robot software stack.

#### 5. Packages and workspaces
A **package** is a unit of ROS2 software (like a Python package or a C++ library). A **workspace** is a directory where you build packages from source. You will `colcon build` inside a workspace.

#### 6. Parameters
Nodes accept **parameters** — runtime configuration values (like PID gains, sensor calibration, robot dimensions). You set these in YAML files or on the command line.

#### 7. TF2 (transforms)
**TF2** is the coordinate frame system. It maintains a tree of transforms: `map → odom → base_link → imu_link`, etc. When you ask "where is the robot?", TF2 answers by composing transforms. For your formation controller, the positions you receive come from this transform tree.

#### 8. URDF (Unified Robot Description Format)
A **URDF** is an XML file that describes your robot's geometry: link shapes, joint types, sensor locations. It tells ROS2 what your robot looks like physically. linorobot2 generates a URDF for your configuration.

#### 9. QoS (Quality of Service)
ROS2 uses DDS (a networking middleware) underneath. **QoS** settings control reliability, durability, and history of message delivery. Mismatched QoS between publisher and subscriber = no communication. This is a common gotcha.

### What to learn hands-on (on your laptop, no hardware needed)

Install ROS2 Humble on Ubuntu 22.04 (in a VM or WSL2 if you're on Windows) and practice:

```bash
# Source the ROS2 environment
source /opt/ros/humble/setup.bash

# Run the demo: two nodes talking via a topic
ros2 run demo_nodes_cpp talker        # Terminal 1
ros2 run demo_nodes_cpp listener      # Terminal 2

# Inspect the system
ros2 node list
ros2 topic list
ros2 topic echo /chatter
ros2 topic info /chatter
ros2 topic hz /chatter

# Manually publish a velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10
```

### Recommended resources

1. **Official ROS2 Humble Tutorials — docs.ros.org** → "Beginner: CLI Tools" and "Beginner: Client Libraries". Do every tutorial in order. This is the single most important learning activity.
   - URL: https://docs.ros.org/en/humble/Tutorials.html
   - Focus on: "Understanding Nodes", "Understanding Topics", "Understanding Services", "Understanding Parameters", "Using Launch Files", "Creating a Package"

2. **Articulated Robotics YouTube Series** — you are already watching this. The key videos for Phase 1:
   - Video 1–3: ROS2 overview, nodes, topics, messages
   - Video on "Simulating in Gazebo" (Phase 2 below)
   - Video on URDF and robot description

3. **"ROS2 for Beginners" by Edouard Renard (Udemy)** — highly rated paid course (~€15 on sale). Very structured, covers nodes/topics/services/actions/parameters systematically. Worth the investment if you prefer video lectures over reading.

4. **The Robotics Back-End (roboticsbackend.com)** — excellent written ROS2 tutorials with practical examples. Free.

### Milestone
You can write a minimal Python ROS2 node that subscribes to `/odom`, extracts the (x, y) position, and publishes a `/cmd_vel` Twist. You understand the output of `ros2 topic list`, `ros2 node list`, and `ros2 topic echo`.

---

## Phase 2: Simulation with Gazebo — Test Before You Build (Days 15–22)

### Why this matters
Gazebo is a physics simulator that lets you test your robot software without hardware. You can spawn a differential-drive robot, send `/cmd_vel`, and see it move. This is where you verify that your ROS2 understanding is correct before touching real motors.

### Core concepts

#### 1. Gazebo (Ignition Gazebo / Gazebo Classic)
Gazebo simulates physics (gravity, friction, collisions), sensors (LiDAR, camera, IMU), and actuators (motors). ROS2 Humble typically uses Gazebo Classic (Gazebo 11) or Ignition Fortress. linorobot2 supports Gazebo.

#### 2. SDF / URDF in simulation
Your robot's URDF gets loaded into Gazebo. The simulator creates a physics body from it and applies forces when you command velocities.

#### 3. The `ros2_control` framework
`ros2_control` is an abstraction layer between your controller and the hardware (or simulator). It provides a standard interface: you write a controller that outputs velocity commands, and a "hardware interface" plugin translates those to either simulated motors or real motors. linorobot2 uses this.

#### 4. `diff_drive_controller`
This is the specific `ros2_control` plugin for differential-drive robots. It takes a Twist command (linear.x, angular.z), computes individual wheel speeds using the differential-drive kinematics, and commands each wheel.

**The kinematics you should know:**
For a differential-drive robot with wheel radius $r$ and wheel separation $L$:
$$v_L = \frac{v - \omega L/2}{r}, \quad v_R = \frac{v + \omega L/2}{r}$$
where $v$ = linear.x, $\omega$ = angular.z, and $v_L, v_R$ are left/right wheel angular velocities.

This is the "unicycle-to-wheel" mapping — exactly what your co-simulation Simulink model does, but now handled by a ROS2 controller.

### Recommended resources

1. **Articulated Robotics YouTube** — the Gazebo simulation videos in the series. These walk through spawning the robot in Gazebo and driving it with `/cmd_vel`.
2. **Official Gazebo Tutorials** — https://classic.gazebosim.org/tutorials — "Quick Start" and "Build a Robot" sections.
3. **linorobot2 simulation guide** — the linorobot2 GitHub README has a simulation section. Follow it to launch the linorobot2 robot in Gazebo.

### Milestone
You can launch linorobot2 in Gazebo, drive the simulated robot with `ros2 topic pub /cmd_vel ...`, and see odometry on `/odom` updating in real time. You understand that the exact same topics and message types will be used on the real robot.

---

## Phase 3: Embedded Systems & Firmware Concepts (Days 23–30)

### Why this matters
The ESP32 runs the linorobot2_hardware firmware. It does the real-time, microsecond-level work: reading encoder pulses, computing PID, driving PWM to motors, reading the IMU over I2C. You don't need to write this firmware from scratch (linorobot2_hardware provides it), but you need to understand it well enough to configure, flash, and debug it.

### Core concepts

#### 1. Microcontroller vs. single-board computer
- **ESP32** = microcontroller. No OS, runs one program (firmware) in a loop. Deterministic timing. Handles real-time sensor/motor tasks.
- **RPi4** = single-board computer. Runs full Linux (Ubuntu). Handles high-level computation (ROS2, EKF, navigation, your MPC). Not real-time.

**Why two boards?** Linux is not real-time — it can pause your motor control for garbage collection or I/O. The ESP32 guarantees sub-millisecond control loops. The RPi4 does everything else.

#### 2. GPIO (General-Purpose Input/Output)
GPIO pins are the ESP32's interface to the physical world. Each pin can be configured as digital input, digital output, PWM output, or special function (I2C, SPI, UART). You connect encoder signals and motor driver signals to specific GPIO pins.

#### 3. PWM (Pulse-Width Modulation)
To control motor speed, the ESP32 outputs a square wave at a fixed frequency (e.g., 20 kHz). The **duty cycle** (fraction of time the signal is high) determines the average voltage delivered to the motor. 50% duty cycle ≈ 50% speed. The TB6612FNG motor driver amplifies these low-power PWM signals to drive the motors.

#### 4. Quadrature encoders
Each motor has an encoder disc with two channels (A and B) that produce square wave pulses as the motor shaft rotates. By counting pulses and checking which channel leads, the firmware determines both **speed** and **direction** of rotation. The ESP32 uses hardware interrupts to catch every pulse.

**The math:** If the encoder has $N$ counts per revolution (CPR) and the wheel radius is $r$, then wheel distance per count is $\frac{2\pi r}{N}$. Counting pulses over a time interval $\Delta t$ gives wheel velocity.

#### 5. PID control (on the ESP32)
The linorobot2_hardware firmware runs a PID loop for each wheel:
- **Setpoint:** desired wheel velocity (from `/cmd_vel` → differential-drive kinematics)
- **Measurement:** actual wheel velocity (from encoder counts / Δt)
- **Output:** PWM duty cycle to the motor driver

You will need to tune these PID gains. Start with P only, then add I and D.

#### 6. I2C (Inter-Integrated Circuit)
The MPU-6050 IMU communicates with the ESP32 over a two-wire I2C bus (SDA + SCL). The firmware reads accelerometer and gyroscope data at a fixed rate (typically 50–100 Hz) and publishes it as a ROS2 Imu message.

#### 7. micro-ROS
micro-ROS is a lightweight ROS2 implementation that runs on microcontrollers. The ESP32 firmware uses micro-ROS to publish topics (`/odom/unfiltered`, `/imu/data`) and subscribe to topics (`/cmd_vel`). On the RPi4 side, the `micro_ros_agent` process bridges the ESP32's serial connection to the full ROS2 network.

#### 8. PlatformIO
PlatformIO is the build system used to compile and flash the linorobot2_hardware firmware onto the ESP32. You configure it via `platformio.ini` and a header file that defines your pin assignments, motor type, IMU type, and robot dimensions.

### Recommended resources

1. **Articulated Robotics YouTube** — the videos on "Motor Drivers", "Encoders", and "Arduino/ESP32 firmware" in the robot build series.
2. **linorobot2_hardware README** — read the entire README carefully. It lists exactly which `#define` values to set for your configuration (2WD, ESP32, MPU6050, TB6612FNG).
3. **"ESP32 Getting Started" by Random Nerd Tutorials** (randomnerdtutorials.com) — excellent for understanding GPIO, PWM, I2C on the ESP32 specifically.
4. **YouTube: "PID Control — A brief introduction" by Brian Douglas** (~10 min) — you already know PID theory, but this video explains it in the embedded/motor context specifically.
5. **YouTube: "How Rotary Encoders Work" by How To Mechatronics** (~15 min) — visual explanation of quadrature encoding.

### Milestone
You can explain what every wire between the ESP32 and the motors/encoders/IMU does. You can open the linorobot2_hardware config header, set the correct `#define` values for your BOM (2WD, ESP32, TB6612FNG, MPU6050), and flash the firmware using PlatformIO.

---

## Phase 4: Sensor Fusion & State Estimation (Days 31–36)

### Why this matters
Your MPC needs agent positions. On a real robot, no single sensor gives you a clean position. Wheel odometry drifts (wheels slip). IMU gyroscope drifts (bias accumulates). You fuse them with an Extended Kalman Filter (EKF) to get a better position estimate. This is the `robot_localization` package in linorobot2.

### Core concepts

#### 1. Wheel odometry (dead reckoning)
The ESP32 firmware computes the robot's position by integrating wheel velocities:
$$x_{k+1} = x_k + v_k \cos(\theta_k) \Delta t, \quad y_{k+1} = y_k + v_k \sin(\theta_k) \Delta t, \quad \theta_{k+1} = \theta_k + \omega_k \Delta t$$
This is a unicycle model. It drifts over time due to wheel slip, uneven surfaces, and encoder quantization.

#### 2. IMU (Inertial Measurement Unit)
The MPU-6050 provides:
- **Accelerometer:** linear acceleration (gravity + motion) in 3 axes
- **Gyroscope:** angular velocity in 3 axes

For a ground robot, the gyroscope's z-axis gives yaw rate — the same $\omega$ as your unicycle model, but measured independently of the wheels. The gyroscope drifts slowly; the encoders drift with wheel slip. They have complementary error characteristics.

#### 3. Extended Kalman Filter (EKF) — `robot_localization`
The `robot_localization` package runs an EKF that fuses:
- Wheel odometry (from `/odom/unfiltered`) — provides position (x, y) and velocity
- IMU (from `/imu/data`) — provides orientation (yaw) and angular velocity

The output is a filtered `/odom` topic with a better position/orientation estimate than either source alone.

**As a control engineer, you already understand Kalman filtering.** The ROS2-specific part is just knowing which topics to configure in the `ekf.yaml` file and which state variables each sensor contributes.

#### 4. Coordinate frames and TF2 (revisited)
The EKF publishes the transform `odom → base_link`. This transform tells the rest of the system "the robot's base_link is at position (x, y, θ) relative to the odom frame." Your formation controller reads this position.

### Recommended resources

1. **Articulated Robotics YouTube** — the video on "Sensor Fusion" / "robot_localization" / "EKF" in the series.
2. **robot_localization documentation** — https://docs.ros.org/en/humble/p/robot_localization/ — read the "Preparing Your Data" and "Configuring robot_localization" sections.
3. **YouTube: "Sensor Fusion and Tracking" by MATLAB Tech Talks** — excellent visual explanation of Kalman filtering for robotics. You know the math, but seeing it applied to IMU + odometry is valuable.

### Milestone
You can explain what the `ekf.yaml` configuration does, which sensor provides which state variables, and what the output `/odom` topic contains. You can launch `robot_localization` in simulation and verify that the fused odometry is smoother than raw wheel odometry.

---

## Phase 5: Mechanical Assembly & Wiring (Days 37–44)

### Why this matters
Now you physically build one robot.

### What to learn (hands-on)

#### 1. Chassis assembly
Mount motors to the chassis plate, attach wheels, add a caster wheel for balance. The Articulated Robotics tutorials show this step-by-step.

#### 2. Wiring the motor driver (TB6612FNG)
The TB6612FNG has:
- **VM** → motor power supply (battery voltage)
- **VCC** → logic power (3.3V from ESP32)
- **AIN1, AIN2, PWMA** → direction and speed control for motor A (from ESP32 GPIO)
- **BIN1, BIN2, PWMB** → same for motor B
- **AO1, AO2** → motor A terminals
- **BO1, BO2** → motor B terminals
- **STBY** → must be pulled HIGH to enable the driver
- **GND** → common ground (ESP32 + battery)

#### 3. Wiring encoders
Each encoder has:
- **VCC** → 3.3V or 5V
- **GND** → ground
- **A, B** → two signal channels → two ESP32 GPIO pins (configured as interrupts)

#### 4. Wiring the IMU (MPU-6050)
- **VCC** → 3.3V
- **GND** → ground
- **SDA** → ESP32 I2C data pin (GPIO 21 on most ESP32 boards)
- **SCL** → ESP32 I2C clock pin (GPIO 22)

#### 5. Power distribution
- Motors are powered from the battery through the TB6612FNG
- ESP32 can be powered via USB from the RPi4
- RPi4 is powered from a USB-C power bank or a voltage regulator from the battery

### Recommended resources

1. **Articulated Robotics YouTube** — mechanical assembly and wiring videos. Follow exactly.
2. **linorobot2_hardware pinout documentation** — the README and config header specify which ESP32 pins to use. Follow them exactly.
3. **YouTube: "How to Use the TB6612FNG Motor Driver" by DroneBot Workshop** (~20 min) — clear walkthrough of wiring and testing.

### Milestone
One fully assembled robot with all wires connected, ESP32 flashed with linorobot2_hardware firmware, RPi4 running Ubuntu 22.04 + ROS2 Humble. You can power it on, SSH into the RPi4, and see the micro_ros_agent connecting to the ESP32.

---

## Phase 6: Bringup, Testing & Tuning (Days 45–52)

### What to do

#### 1. Run linorobot2 bringup
```bash
# On the RPi4:
ros2 launch linorobot2_bringup bringup.launch.py
```
This starts the micro-ROS agent, robot_localization EKF, and TF publishers.

#### 2. Test motors
From your laptop (on the same WiFi network):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10
```
Both wheels should spin forward. Verify direction. If one spins backward, swap its motor wires or flip the encoder direction in the firmware config.

#### 3. Test odometry
Drive the robot forward 1 meter by hand or command. Check `/odom`:
```bash
ros2 topic echo /odom
```
The `pose.pose.position.x` should read approximately 1.0. If it reads 0.5 or 2.0, your wheel radius or encoder CPR is wrong in the firmware config.

#### 4. Tune PID
Drive the robot at constant velocity and plot actual vs. commanded wheel speeds (use `rqt_plot` or `ros2 topic echo`). Adjust PID gains in the firmware until tracking is tight with minimal overshoot.

#### 5. Test IMU
```bash
ros2 topic echo /imu/data
```
Rotate the robot by hand. The `orientation` quaternion should change smoothly. The gyroscope z should show angular velocity.

### Recommended resources

1. **linorobot2 "Getting Started" wiki** on GitHub — step-by-step bringup instructions.
2. **Articulated Robotics YouTube** — bringup and testing videos.
3. **rqt_plot** — real-time plotting tool for ROS2 topics. Essential for PID tuning.

### Milestone
One robot drives straight when commanded, turns correctly, and reports accurate odometry. The EKF-fused `/odom` position drifts less than 5% over a 2-meter straight-line drive.

---

## Phase 7: Multi-Robot Networking & Integration (Days 53–60)

### Why this matters
You have four robots. Each needs its own ROS2 namespace to avoid topic collisions, and all must communicate over WiFi with your planning layer.

### Core concepts

#### 1. ROS2 namespaces
Each robot runs in a namespace: `/robot1`, `/robot2`, etc. This prefixes all topics: `/robot1/cmd_vel`, `/robot1/odom`, etc. linorobot2 supports this via a launch argument.

#### 2. ROS2 DDS and multi-machine communication
ROS2 uses DDS for discovery and transport. On the same WiFi network, nodes on different machines automatically discover each other (if using the same `ROS_DOMAIN_ID`). Set the same domain ID on all robots and your laptop:
```bash
export ROS_DOMAIN_ID=42
```

#### 3. Bridging ZMQ ↔ ROS2
Your planning layer uses ZMQ. You need a thin bridge node (running on your laptop or on each RPi4) that:
- Subscribes to `/robotN/odom` (ROS2) → forwards position to the ZMQ coordinator
- Receives control commands from ZMQ → publishes to `/robotN/cmd_vel` (ROS2)

This bridge replaces the `plant_node.py` in your simulation. The architecture is identical to the co-simulation bridge concept, but using ROS2 topics instead of MATLAB Engine.

#### 4. Clock synchronization
For distributed formation control, all robots should have roughly synchronized clocks. Use `chrony` or `systemd-timesyncd` with an NTP server on your WiFi network.

### Recommended resources

1. **ROS2 documentation — "Setting up a robot network"** — covers multi-machine setup, domain IDs, and DDS configuration.
2. **linorobot2 multi-robot guide** (if available) or community examples of multi-robot linorobot2 setups.

### Milestone
All four robots are on WiFi, discoverable via ROS2, and your laptop can command each one independently and read each one's odometry. You can replicate the same formation experiment you ran in Python simulation, now on real hardware.

---

## Summary Table: Concepts → Where They Appear

| Concept | Where it appears in your project | Phase |
|---|---|---|
| Linux terminal, SSH, apt | Every interaction with RPi4 | 0 |
| ROS2 nodes, topics, messages | All robot software communication | 1 |
| `/cmd_vel` Twist, `/odom` Odometry | Controller ↔ robot interface | 1 |
| TF2 coordinate frames | Position estimation chain | 1 |
| URDF robot description | linorobot2 configuration | 1 |
| Gazebo simulation | Testing before hardware | 2 |
| `ros2_control`, `diff_drive_controller` | Velocity → wheel speed mapping | 2 |
| GPIO, PWM | ESP32 drives motors | 3 |
| Quadrature encoders | Wheel speed measurement | 3 |
| PID control (embedded) | ESP32 motor speed control loop | 3 |
| I2C, MPU-6050 | IMU communication | 3 |
| micro-ROS, micro_ros_agent | ESP32 ↔ ROS2 bridge | 3 |
| PlatformIO | Firmware build & flash | 3 |
| EKF sensor fusion | robot_localization package | 4 |
| TB6612FNG wiring | Motor driver hookup | 5 |
| ROS2 namespaces | Multi-robot topic isolation | 7 |
| ROS2 DDS, domain IDs | Multi-machine networking | 7 |
| ZMQ ↔ ROS2 bridge | Planning layer integration | 7 |

---

## Curated Video Playlist (Watch in Order)

### Tier 1: Must watch before building
1. **Articulated Robotics — Full playlist** (youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT) — ~20 videos, ~20 min each. This is your primary tutorial. Watch all of it.
2. **"ROS2 Humble in 5 hours" by The Construct** (YouTube, free) — dense crash course covering all Phase 1 concepts.
3. **"How Rotary Encoders Work" by How To Mechatronics** — understand the sensor that measures wheel rotation.
4. **"TB6612FNG Motor Driver Tutorial" by DroneBot Workshop** — understand the IC that drives your motors.
5. **"PID Controller Explained" by Brian Douglas (MATLAB)** — PID in the embedded motor context.

### Tier 2: Watch as needed
6. **"ESP32 Tutorial Getting Started" by Random Nerd Tutorials** (YouTube + written) — GPIO, PWM, I2C basics.
7. **"micro-ROS on ESP32" by micro-ROS project** (YouTube) — how the ESP32 talks ROS2.
8. **"robot_localization Tutorial" by Automatic Addison** (YouTube + automaticaddison.com) — configuring the EKF.
9. **"Gazebo Sim with ROS2" — Official Gazebo YouTube channel** — if you get stuck on simulation.

### Tier 3: Deep dives for troubleshooting
10. **"Understanding DDS for ROS2" by Open Robotics** — when multi-machine communication doesn't work.
11. **"ROS2 Networking & Multi-Robot" by The Construct** — namespace and domain configuration.

---

## Concepts You Can Skip (For Now)

Given your specific project, you do **not** need to learn the following at this stage:

- **SLAM (Simultaneous Localization and Mapping):** Your formation experiment uses known starting positions and relative odometry, not a map. Add SLAM later if you want autonomous navigation.
- **Nav2 (Navigation Stack):** Your controller provides the velocity commands, not Nav2's path planner.
- **Computer vision / cameras:** Not in your BOM.
- **LiDAR:** Not in your initial BOM. Can add later for obstacle detection.
- **MoveIt (manipulation):** Not relevant — your robots don't have arms.
- **ROS2 services and actions in depth:** You only need topics for your project. Services/actions are for request-response patterns you won't use initially.
- **Writing custom URDF from scratch:** linorobot2 generates one for you from config parameters.
- **C++ ROS2 nodes:** Your planning layer is Python. The firmware is pre-written C++. You don't need to write C++ ROS2 code.

---

## Daily Time Estimate

Assuming ~2–3 hours per day of focused learning:

| Phase | Days | Hours | Mode |
|---|---|---|---|
| 0: Linux basics | 3 | 6–9 | Laptop only |
| 1: ROS2 concepts | 11 | 22–33 | Laptop (VM/WSL2) |
| 2: Gazebo simulation | 8 | 16–24 | Laptop |
| 3: Embedded concepts | 8 | 16–24 | Reading + videos, then ESP32 bench |
| 4: Sensor fusion | 6 | 12–18 | Laptop (theory) + RPi4 |
| 5: Mechanical build | 8 | 16–24 | Hands-on assembly |
| 6: Bringup & tuning | 8 | 16–24 | Robot on the floor |
| 7: Multi-robot integration | 8 | 16–24 | Four robots + laptop |
| **Total** | **~60** | **~120–180** | |

This is approximately 8–9 weeks at a steady pace. You can compress it by skipping simulation (Phase 2) if you're confident, or expand it if you want deeper understanding of any phase.

---

## A Note on Your Unique Advantage

Most robotics beginners struggle with the control theory — they know ROS2 but can't formulate an MPC or reason about stability. You have the opposite profile: deep theoretical foundations, learning the tooling. This is actually the easier direction. The concepts above are all concrete and well-documented; there is no ambiguity in "how to wire an encoder" the way there is in "how to prove convergence of a hybrid switched system."

Your formation controller is the hard part. Everything in this plan is plumbing to connect it to reality.
