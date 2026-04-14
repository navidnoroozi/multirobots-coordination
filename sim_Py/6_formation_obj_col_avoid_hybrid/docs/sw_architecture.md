# The System Architecture:

Home WiFi router (2.4 GHz, dedicated SSID recommended)
        │
        │  UDP/TCP/ZMQ over WiFi
        │
Host PC (Ubuntu 22.04, ROS2 Humble)
├── coordinator_node.py  (ZMQ REP)
├── controller_node.py × 4  (ZMQ REP, one per agent)
└── cosim_manager.py  (ZMQ REQ, MATLAB bridge)
        │
        │  ROS2 DDS over WiFi  (or your custom ZMQ UDP bridge)
        │
Robot 1 (RPi4)          Robot 2 (RPi4)        Robot 3 (RPi4)       Robot 4 (RPi4)
├── Ubuntu 22.04         ├── Ubuntu 22.04        ...                  ...
├── ROS2 Humble          ├── ROS2 Humble
├── robot_agent.py       ├── robot_agent.py
│   receives u_safe      │   receives u_safe
│   publishes odometry   │   publishes odometry
└── Arduino Nano         └── Arduino Nano
    ├── encoder ISR          ├── encoder ISR
    ├── PID velocity ctrl    ├── PID velocity ctrl
    └── TB6612FNG driver     └── TB6612FNG driver


# Implementation Phases:

## Phase 1 — Mechanical assembly:
Do this before touching any software. Estimated time: 2–3 hours for the first robot.

### Step 1.1 — Unbox and inventory. 
Lay all components on a table. 
Confirm you have: acrylic chassis plates (usually 2), 2 motors, 4 screws per motor, 2 wheels, 1 ball caster, all electronics.

### Step 1.2 — Mount the motors. 
Each JGA25-370 motor has two screw holes on its gearbox face. 
Align the motor so the output shaft points outward through the wheel cutout in the chassis bottom plate. 
Use M3 × 8 mm screws. Tighten firmly — a loose motor causes odometry drift.

### Step 1.3 — Attach wheels. 
Press-fit or screw the wheels onto the motor output shafts. 
The flat face of the D-shaped shaft must align with the flat inside the wheel hub.

### Step 1.4 — Mount ball caster. 
Screw the ball caster to the rear underside of the bottom chassis plate using M3 screws and the 10 mm standoffs if needed to level it.

### Step 1.5 — Stack the chassis layers. 
Most acrylic kits have two layers connected by M3 standoffs (~30–40 mm). 
Use 4 standoffs at the corners. This creates the electronics deck.

### Step 1.6 — Mount the Raspberry Pi 4. 
Attach 4 × 10 mm brass standoffs to the top deck using M3 screws from below. 
Then seat the RPi4 onto the standoffs and secure with M3 nuts from above. Orient the USB-C power port toward the rear.

### Step 1.7 — Stick heatsinks onto RPi4. 
Peel the adhesive backing and press the small copper heatsinks onto the BCM2711 chip, RAM chip, and USB controller chip. 
This prevents thermal throttling.


## Phase 2 — Wiring:
Keep wires short and tidy. Work on a non-conductive surface.

### Step 2.1 — Motor wires to TB6612FNG. 
The TB6612FNG board has two channels (A and B). Connect:

- Left motor: red → AO1, black → AO2
- Right motor: red → BO1, black → BO2

If a motor spins the wrong direction later in software testing, just swap those two wires for that motor.

### Step 2.2 — Power the TB6612FNG. 
VM pin → LiPo positive (7.4V). GND → LiPo negative. VCC pin → 3.3V from ESP32 (for logic level).

### Step 2.3 — TB6612FNG control pins to ESP32. 
Use Dupont wires:

TB6612FNGpin    ESP32GPIO      Function
---------------------------------------------------------
PWMA            GPIO 14         Left motor speed
---------------------------------------------------------
AIN1            GPIO 27         Left motor direction 1
---------------------------------------------------------
AIN2            GPIO 26         Left motor direction 2
---------------------------------------------------------
PWMB            GPIO 25         Right motor speed
---------------------------------------------------------
BIN1            GPIO 33         Right motor direction 1
---------------------------------------------------------
BIN2            GPIO 32         Right motor direction 2
---------------------------------------------------------
STBY            3.3V            Enable the driver (pull high)
---------------------------------------------------------

### Step 2.4 — Encoder wires to ESP32. 
Each JGA25-370 encoder has 6 wires: VCC, GND, A, B (+ sometimes index). Connect:
- Encoder VCC → 3.3V on ESP32
- Encoder GND → GND on ESP32
- Left encoder A → GPIO 34, Left encoder B → GPIO 35
- Right encoder A → GPIO 18, Right encoder B → GPIO 19

### Step 2.5 — MPU-6050 IMU to ESP32 via I2C:
VCC → 3.3V, GND → GND, SDA → GPIO 21, SCL → GPIO 22

### Step 2.6 — Power the RPi4. 
Buck converter input → LiPo (7.4V). 
Buck converter output (USB-C 5V) → RPi4 USB-C port. The buck converter regulates the LiPo voltage to a stable 5V for the RPi4.

### Step 2.7 — ESP32 to RPi4 via USB cable: 
ESP32 USB-C/micro → any RPi4 USB-A port. 
This carries both power (to ESP32 from RPi4) and the serial data link used by micro-ROS.

### Step 2.8 — Mount electronics with double-sided foam tape. 
Stick the TB6612FNG board, ESP32, and buck converter to the top deck of the chassis. Keep the LiPo accessible for charging — attach it with a Velcro strap to the underside of the bottom deck.


## Phase 3 — ESP32 firmware (linorobot2_hardware):
This replaces what Arduino firmware would have done. Estimated time: 1–2 hours.

### Step 3.1 — Install Arduino IDE 2 on your host PC. 
Download from arduino.cc.

### Step 3.2 — Add ESP32 board support. 
In Arduino IDE: File → Preferences → Additional Boards Manager URLs → paste https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json. Then Tools → Board → Boards Manager → search ESP32 → Install.

### Step 3.3 — Clone linorobot2_hardware. On your host PC terminal:
> git clone https://github.com/linorobot/linorobot2_hardware

### Step 3.4 — Install micro-ROS Arduino library. 
Follow the instructions in the linorobot2_hardware README: download the micro-ROS library for Arduino from the releases page and add it via Arduino IDE → Sketch → Include Library → Add .ZIP Library.

### Step 3.5 — Configure the firmware. 
Open <linorobot2_hardware/firmware/lib/config/lino_base_config.h> and set:
> #define LINO_BASE DIFFERENTIAL_DRIVE
> #define USE_TB6612_MOTOR_DRIVER
> #define USE_MPU6050_IMU
> // Set your encoder CPR and wheel diameter here
> #define COUNTS_PER_REV 44   // 11 CPR × 4 (quadrature)
> #define WHEEL_DIAMETER 0.065  // meters
> #define WHEEL_SEPARATION 0.17  // meters (measure on your chassis)>

### Step 3.6 — Set GPIO pin numbers in the motor driver config to match your wiring from Step 2.3 and 2.4.

### Step 3.7 — Flash the firmware. 
Connect ESP32 to your host PC via USB. In Arduino IDE: select board "ESP32 Dev Module", select the correct COM port, click Upload. 
The ESP32 now runs micro-ROS and waits for a ROS2 micro-ROS agent on the RPi4.


## Phase 4 — Raspberry Pi 4 software:

### Step 4.1 — Flash Ubuntu Server 22.04. 
On your host PC, download the Raspberry Pi Imager from raspberrypi.com. 
Insert the microSD card. 
Choose OS → Other general-purpose OS → Ubuntu → Ubuntu Server 22.04 LTS (64-bit). 
Before writing, click the gear icon and set: hostname = robot1, enable SSH, set WiFi SSID and password, set username = pi, password of your choice. 
Write to SD card. Insert SD into RPi4 and power on.

### Step 4.2 — SSH into the RPi4. 
From your host PC:
> ssh pi@robot1.local
Wait ~2 minutes for first boot. If robot1.local does not resolve, find the IP in your router's DHCP list.

### Step 4.3 — Install ROS2 Humble. On the RPi4 via SSH:
<sudo apt update && sudo apt upgrade -y>
<sudo apt install software-properties-common curl -y>
<sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg>
<echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list>
<sudo apt update>
<sudo apt install ros-humble-ros-base -y>
<echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc>
<source ~/.bashrc>

### Step 4.4 — Install linorobot2 on the RPi4. 
The project provides a one-line install script:
<source /opt/ros/humble/setup.bash>
<cd /tmp>
<wget https://raw.githubusercontent.com/linorobot/linorobot2/humble/install_linorobot2.bash>
# Clone missing packages
<cd ~/linorobot2_ws/src>
<git clone -b humble https://github.com/linorobot/linorobot2>
<git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent micro_ros_agent_pkg>
# Install dependencies
<cd ~/linorobot2_ws
rosdep install --from-path src --ignore-src -y \
  --skip-keys microxrcedds_agent \
  --skip-keys micro_ros_agent \
  --skip-keys python3-opencv-contrib-python \
  --skip-keys python3-pycollada \
  --skip-keys ros-humble-gazebo-ros-pkgs \
  --skip-keys gazebo_ros \
  --skip-keys gazebo_ros_pkgs \
  --skip-keys gazebo_plugins \
  --skip-keys gazebo_dev>
# Build while skipping Gazebo
<colcon build --packages-skip linorobot2_gazebo>
bash install_linorobot2.bash 2wd
This installs all dependencies and creates a ~/robot_ws workspace. It takes 10–20 minutes.

### Step 4.5 — Launch micro-ROS agent. 
This creates the ROS2 bridge between the ESP32 (USB serial) and the RPi4:
> source ~/robot_ws/install/setup.bash
> ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
Plug the USB cable (ESP32 → RPi4) and you should see connection messages appear.

### Step 4.6 — Verify odometry. 
In a second SSH terminal:
> source ~/robot_ws/install/setup.bash
> ros2 topic echo /odom
Manually spin a wheel by hand — the odometry values should change.

### Step 4.7 — Test motor commands. 
Send a velocity command:
> ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

👉The robot should move forward briefly.


## Phase 5 — Integration with your formation control system:
This is where your existing Python planning layer connects to the physical robots.

### Step 5.1 — Write the robot agent node. 
On each RPi4, create a small Python node ~/robot_ws/src/robot_agent.py that:
- Connects to your host PC ZMQ coordinator via WiFi
- Receives u_safe = (u_x, u_y) from cosim_manager.py
- Converts to (v_ref, ω) using the same motion-controller formula: v_ref = ||[u_x, u_y]||, θ_ref = atan2(u_y, u_x), ω = k_heading · wrap(θ_ref − θ)
- Publishes geometry_msgs/Twist to /cmd_vel
- Reads /odom and sends (x, y, θ, v, ω) back to the host via ZMQ

Step 5.2 — Update cosim_bridge.py on the host. 
Replace the matlab.engine step logic with a ZMQ UDP socket that sends u_safe to each robot's IP and receives odometry back. 
You already have this architecture designed — only the transport changes from MATLAB workspace to ZMQ WiFi.

Step 5.3 — Configure static IPs. 
Set each RPi4 to a static IP in your router: 192.168.1.21, .22, .23, .24. 
Update consensus_config.py accordingly.


Step 5.4 — Run the full stack. On the host:
> python cosim_experiment_runner.py --n-agents 4 --model single_integrator --outer-steps 18 --cosim-mode hardware

👉This starts coordinator, 4 controller nodes, and the cosim manager. The manager now talks to real robots over WiFi instead of MATLAB.


Recommended learning sequence:
Before attempting to run four robots simultaneously, follow this progression:
- Week 1 — Build one robot mechanically. Flash ESP32. Confirm motors spin when commanded.
- Week 2 — Set up RPi4 with linorobot2. Confirm odometry publishes. Drive robot manually with a keyboard using teleop_twist_keyboard.
- Week 3 — Connect robot to your host PC over WiFi. Write robot_agent.py. Confirm the ZMQ bridge works: host sends a velocity, robot moves, odometry comes back.
- Week 4 — Run your single-agent version of the planning layer (set n_agents=1). Close the loop: planning → robot → odometry → planning.
- Week 5–6 — Build robots 2, 3, 4 (much faster now that you know the process). Run the full 4-agent formation control experiment.
The Articulated Robotics YouTube playlist <youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT> is the best companion for Weeks 1–3, covering every step with clear explanations. 
The linorobot2 documentation at <github.com/linorobot/linorobot2> covers Week 4 onward.


| Layer      | Hardware       | Software                        | Role                               |
| ---------- | -------------- | ------------------------------- | ---------------------------------- |
| Low-Level  | ESP32          | linorobot2_hardware (micro-ROS) | PWM motor control, encoder reading |
| Mid-Level  | Raspberry Pi 4 | linorobot2 (ROS2 Humble)        | Odometry, kinematics, Wi-Fi bridge |
| High-Level | Ubuntu VM      | Your Python + rclpy             | Formation planning, logging        |