W, H = 960, 840

# palette
PU0="#EEEDFE"; PU1="#CECBF6"; PU6="#534AB7"; PU8="#3C3489"
TE0="#E1F5EE"; TE1="#9FE1CB"; TE6="#0F6E56"; TE8="#085041"
CO0="#FAECE7"; CO1="#F5C4B3"; CO6="#993C1D"; CO8="#712B13"
BL0="#E6F1FB"; BL1="#B5D4F4"; BL6="#185FA5"; BL8="#0C447C"
AM0="#FAEEDA"; AM1="#FAC775"; AM6="#854F0B"; AM8="#633806"
GY0="#F1EFE8"; GY1="#D3D1C7"; GY6="#5F5E5A"; GY8="#2C2C2A"
WHT="#FFFFFF"; BLK="#1a1a1a"
CMD="#C23B1A"; FBK="#0A6E56"

FONT = "-apple-system,BlinkMacSystemFont,Helvetica,Arial,sans-serif"

lines = []
def L(s): lines.append(s)

# ── primitives ────────────────────────────────────────────────────────────────
def box(x,y,w,h,fill,stroke,sw=0.8,rx=6):
    L(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

def hdr_strip(x,y,w,hh,fill,rx=6):
    """Filled header strip — top corners rounded, bottom square."""
    L(f'<rect x="{x}" y="{y}" width="{w}" height="{hh}" rx="{rx}" fill="{fill}"/>')
    L(f'<rect x="{x}" y="{y+hh-rx}" width="{w}" height="{rx}" fill="{fill}"/>')

def txt(x,y,s,a="middle",fs=11,fw=400,fill=BLK):
    L(f'<text x="{x}" y="{y}" text-anchor="{a}" font-family="{FONT}" '
      f'font-size="{fs}" font-weight="{fw}" fill="{fill}">{s}</text>')

def arr_down(cx,y1,y2,col,sw=2.0):
    """Downward arrow (command direction)."""
    L(f'<line x1="{cx}" y1="{y1}" x2="{cx}" y2="{y2}" stroke="{col}" stroke-width="{sw}" marker-end="url(#arr)"/>')

def arr_up(cx,y1,y2,col,sw=2.0):
    """Upward arrow: line from y1(bottom) to y2(top), head at top."""
    L(f'<line x1="{cx}" y1="{y1}" x2="{cx}" y2="{y2}" stroke="{col}" stroke-width="{sw}" marker-end="url(#arr)"/>')

def arr_right(x1,cx,y,col,sw=1.5):
    L(f'<line x1="{x1}" y1="{y}" x2="{cx}" y2="{y}" stroke="{col}" stroke-width="{sw}" marker-end="url(#arr)"/>')

def seg(x1,y1,x2,y2,col,sw=1.2,dash=""):
    da=f' stroke-dasharray="{dash}"' if dash else ""
    L(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{col}" stroke-width="{sw}"{da}/>')

# ── layout constants ─────────────────────────────────────────────────────────
# Sub-boxes: 4 per tier, equal width inside tier (margin 10px each side)
TIER_X1, TIER_X2 = 10, W-10
TIER_W  = TIER_X2 - TIER_X1          # 940
SB_GAP  = 8                           # gap between sub-boxes
SB_MARG = 12                          # inner left/right margin
SB_N    = 4
SB_W    = (TIER_W - 2*SB_MARG - (SB_N-1)*SB_GAP) // SB_N   # 218
SB_XS   = [TIER_X1 + SB_MARG + i*(SB_W+SB_GAP) for i in range(SB_N)]  # [22, 248, 474, 700]
HDR_H   = 20                          # tier header height
SB_HDR  = 15                          # sub-box header height
LSPC    = 13                          # line spacing in sub-boxes

# Vertical layout (y positions)
T1_Y1, T1_Y2   = 54,  220   # Tier 1 planning layer (166px)
IB1_Y1,IB1_Y2  = 220, 262   # Interface band 1 (42px)
T2_Y1, T2_Y2   = 262, 422   # Tier 2 RPi4 (160px)
IB2_Y1,IB2_Y2  = 422, 462   # Interface band 2 (40px)
T3_Y1, T3_Y2   = 462, 618   # Tier 3 ESP32 (156px)
IB3_Y1,IB3_Y2  = 618, 656   # Interface band 3 (38px)
T4_Y1, T4_Y2   = 656, 776   # Tier 4 Physical (120px — bh=82 for 4 rows)
LEG_Y           = 756        # Legend row

# Arrow channel x positions
CMD_X = W - 38              # command arrow (right side, downward)
FBK_X = 38                  # feedback arrow (left side, upward)

# ════════════════════════════════════════════════════════════════════════════
# SVG OPEN
# ════════════════════════════════════════════════════════════════════════════
L(f'<svg width="{W}" height="{H}" viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img">')
L('<title>Tier B Software Architecture — one robot</title>')
L(f'<desc>Tier-based software architecture for one Tier B mobile robot: planning layer on Host PC, ROS2 motion control on RPi4, micro-ROS firmware on ESP32, and physical actuation/sensing layer.</desc>')
L(f'''<defs>
  <marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
    <path d="M1 2L9 5L1 8" fill="none" stroke="context-stroke" stroke-width="1.4" stroke-linecap="round" stroke-linejoin="round"/>
  </marker>
</defs>''')
L(f'<rect width="{W}" height="{H}" fill="{WHT}"/>')

# ── Title ────────────────────────────────────────────────────────────────────
txt(W//2, 20, "Tier B Mobile Robot — Software Architecture (one robot, ×4 identical)", fs=15, fw=600)
txt(W//2, 38, "Python planning layer · ROS 2 Humble (RPi4) · micro-ROS / linorobot2_hardware (ESP32) · Physical", fs=10, fill=GY6)

# ════════════════════════════════════════════════════════════════════════════
# HELPER: draw one tier (outer box + header) and 4 sub-boxes
# ════════════════════════════════════════════════════════════════════════════
def draw_tier(y1, y2, bg, border, hdr_col, tier_label):
    box(TIER_X1, y1, TIER_W, y2-y1, bg, border, sw=1.0, rx=7)
    hdr_strip(TIER_X1, y1, TIER_W, HDR_H, hdr_col, rx=7)
    txt(W//2, y1+14, tier_label, fs=11, fw=600, fill=WHT)

def draw_sub(j, tier_y1, tier_y2, hdr_col, hdr_fill, title, rows):
    bx = SB_XS[j]
    by = tier_y1 + HDR_H + 6
    bh = tier_y2 - by - 8
    box(bx, by, SB_W, bh, WHT, hdr_fill, sw=0.7, rx=4)
    hdr_strip(bx, by, SB_W, SB_HDR, hdr_col, rx=4)
    txt(bx + SB_W//2, by + SB_HDR - 3, title, fs=9, fw=600, fill=WHT)
    for i, row in enumerate(rows):
        txt(bx+6, by + SB_HDR + 6 + i*LSPC, row, a="start", fs=9, fill=BLK)

# ════════════════════════════════════════════════════════════════════════════
# TIER 1 — Python Planning Layer  (Host PC · Ubuntu 22.04 · Python 3.12)
# ════════════════════════════════════════════════════════════════════════════
draw_tier(T1_Y1, T1_Y2, PU0, PU6, PU6,
    "Tier 1 — Python Planning Layer   ·   Host PC   ·   Ubuntu 22.04   ·   Python 3.12")

draw_sub(0, T1_Y1, T1_Y2, PU8, PU1, "coordinator_node.py", [
    "ZMQ REP server :5556",
    "Receives plant_step {r, v}",
    "Aggregates U_seq from agents",
    "Computes barycenter r̄_i(j)",
    "Distributes state to ctrls",
    "Outer step rate: 1 Hz",
    "Manages formation targets",
    "Closes outer feedback loop",
])

draw_sub(1, T1_Y1, T1_Y2, PU8, PU1, "controller_node.py  × 4", [
    "ZMQ REQ socket per agent",
    "Distributed MPC OCP solver",
    "Coords: y_i = z_i − c_i",
    "Terminal hull constraint",
    "Lexicographic OCP selection",
    "Horizon M, N agents",
    "Returns U_seq[0 : M]",
    "One instance per robot i",
])

draw_sub(2, T1_Y1, T1_Y2, PU8, PU1, "Hybrid safety supervisor", [
    "Mode F: nominal MPC",
    "Mode C: collision avoidance",
    "Mode O: obstacle avoidance",
    "Mode CO: blended safety",
    "Mode T: transition smooth",
    "Hysteresis d_enter / d_exit",
    "Blending weights β_C β_O",
    "Output: u_safe per agent",
])

draw_sub(3, T1_Y1, T1_Y2, PU8, PU1, "cosim_manager.py  (ZMQ bridge)", [
    "Replaces plant_node.py",
    "ZMQ REP: receives u_safe",
    "Hardware mode → robot_agent",
    "Sim mode → cosim_bridge",
    "Sends r, v to coordinator",
    "Dry-run without MATLAB",
    "Outer-loop timing control",
    "OdometryBundle aggregation",
])

# ════════════════════════════════════════════════════════════════════════════
# INTERFACE BAND 1 — WiFi / ZMQ
# ════════════════════════════════════════════════════════════════════════════
box(TIER_X1, IB1_Y1, TIER_W, IB1_Y2-IB1_Y1, AM0, AM6, sw=0.7, rx=0)

IB1_MID = (IB1_Y1 + IB1_Y2) // 2

# Command arrow (right, going DOWN through band)
arr_down(CMD_X, IB1_Y1+4, IB1_Y2-4, CMD, sw=2.2)
txt(CMD_X-4, IB1_MID+4, "u_safe →", a="end", fs=9, fw=600, fill=CMD)

# Feedback arrow (left, going UP through band)
arr_up(FBK_X, IB1_Y2-4, IB1_Y1+4, FBK, sw=2.2)
txt(FBK_X+4, IB1_MID+4, "← r, v (odometry)", a="start", fs=9, fw=600, fill=FBK)

# Band label center
txt(W//2, IB1_MID-5, "WiFi  802.11n/ac   ·   ZMQ REQ / REP   ·   per robot   ·   UDP transport", fs=10, fw=600, fill=AM8)
txt(W//2, IB1_MID+9, "u_safe: velocity command (Twist)   ←→   r, v: position + velocity (odometry)", fs=9, fill=AM8)

# ════════════════════════════════════════════════════════════════════════════
# TIER 2 — ROS 2 Motion Control Layer  (RPi 4B · Ubuntu 22.04 · ROS 2 Humble)
# ════════════════════════════════════════════════════════════════════════════
draw_tier(T2_Y1, T2_Y2, TE0, TE6, TE6,
    "Tier 2 — ROS 2 Motion Control Layer   ·   Raspberry Pi 4B   ·   Ubuntu 22.04   ·   ROS 2 Humble")

draw_sub(0, T2_Y1, T2_Y2, TE8, TE1, "robot_agent.py", [
    "rclpy Node (Python)",
    "ZMQ REP: receives u_safe",
    "Translates → geometry_msgs/Twist",
    "Publishes /cmd_vel @ 10 Hz",
    "Subscribes /odom",
    "Extracts r, v from Odometry",
    "ZMQ REQ: sends r, v",
    "ROS_DOMAIN_ID isolation",
])

draw_sub(1, T2_Y1, T2_Y2, TE8, TE1, "micro_ros_agent  (USB broker)", [
    "Bridges ROS 2 ↔ ESP32",
    "USB 2.0 serial transport",
    "Forwards /cmd_vel to ESP32",
    "Receives /odom/unfiltered",
    "Receives /imu/data",
    "DDS proxy for micro-ROS",
    "Auto-reconnect on drop",
    "rclc_executor lifecycle",
])

draw_sub(2, T2_Y1, T2_Y2, TE8, TE1, "robot_localization  (EKF)", [
    "Fuses wheel odom + IMU",
    "Input: /odom/unfiltered",
    "Input: /imu/data",
    "Output: /odom (fused)",
    "Covariance tuning (yaml)",
    "TF: odom→base_footprint",
    "robot_state_publisher",
    "joint_state_publisher",
])

draw_sub(3, T2_Y1, T2_Y2, TE8, TE1, "ROS 2 topic bus", [
    "Sub /cmd_vel → micro_ros_agent",
    "Pub /odom ← EKF output",
    "Pub /odom/unfiltered ← ESP32",
    "Pub /imu/data ← ESP32 IMU",
    "TF2 transform tree",
    "nav_msgs/Odometry",
    "sensor_msgs/Imu",
    "geometry_msgs/Twist",
])

# ════════════════════════════════════════════════════════════════════════════
# INTERFACE BAND 2 — USB serial / micro-ROS
# ════════════════════════════════════════════════════════════════════════════
box(TIER_X1, IB2_Y1, TIER_W, IB2_Y2-IB2_Y1, GY0, GY1, sw=0.7, rx=0)

IB2_MID = (IB2_Y1 + IB2_Y2) // 2

arr_down(CMD_X, IB2_Y1+4, IB2_Y2-4, CMD, sw=2.2)
txt(CMD_X-4, IB2_MID+4, "/cmd_vel →", a="end", fs=9, fw=600, fill=CMD)

arr_up(FBK_X, IB2_Y2-4, IB2_Y1+4, FBK, sw=2.2)
txt(FBK_X+4, IB2_MID+4, "← /odom/unfiltered  ·  /imu/data", a="start", fs=9, fw=600, fill=FBK)

txt(W//2, IB2_MID-5, "USB 2.0 Serial   ·   micro-ROS transport   ·   RPi4 USB-A → ESP32 USB-micro   ·   1 Mbps", fs=10, fw=600, fill=GY8)
txt(W//2, IB2_MID+9, "micro_ros_agent serial bridge   ·   ROS 2 topics tunnelled over UART", fs=9, fill=GY6)

# ════════════════════════════════════════════════════════════════════════════
# TIER 3 — Motor Drive Control Layer  (ESP32 · micro-ROS · linorobot2_hardware)
# ════════════════════════════════════════════════════════════════════════════
draw_tier(T3_Y1, T3_Y2, CO0, CO6, CO6,
    "Tier 3 — Motor Drive Control Layer   ·   ESP32 DevKit C   ·   micro-ROS   ·   linorobot2_hardware firmware")

draw_sub(0, T3_Y1, T3_Y2, CO8, CO1, "micro-ROS node  (rcl C++)", [
    "USB serial transport layer",
    "Sub: /cmd_vel (Twist)",
    "Pub: /odom/unfiltered",
    "Pub: /imu/data",
    "rclc_executor  @ 50 Hz",
    "Watchdog + auto-reconnect",
    "Differential kinematics",
    "ω_ref_L, ω_ref_R  →  PID",
])

draw_sub(1, T3_Y1, T3_Y2, CO8, CO1, "PID velocity controller", [
    "Discrete-time PID (per wheel)",
    "Left wheel loop",
    "Right wheel loop",
    "Setpoint: ω_ref  (rad/s)",
    "Measure: encoder ISR → ω",
    "Output: PWM duty cycle %",
    "50 Hz update rate",
    "Gains: kp, ki, kd (tunable)",
])

draw_sub(2, T3_Y1, T3_Y2, CO8, CO1, "TB6612FNG HAL driver", [
    "PWM output via LEDC API",
    "AIN1 / AIN2 direction (M1)",
    "PWMA duty → M1 speed",
    "BIN1 / BIN2 direction (M2)",
    "PWMB duty → M2 speed",
    "STBY pin enable logic",
    "Fwd / rev / brake / coast",
    "VM: 11.1 V motor rail",
])

draw_sub(3, T3_Y1, T3_Y2, CO8, CO1, "Encoder ISR", [
    "GPIO interrupt on ENC-A/B",
    "Quadrature decode (4-phase)",
    "Tick accumulator per wheel",
    "Δθ = 2π · Δn / N_CPR",
    "N_CPR = 11 (output shaft)",
    "Velocity estimate → PID",
    "Publishes /odom/unfiltered",
    "Feeds EKF on RPi4 via USB",
])

# ════════════════════════════════════════════════════════════════════════════
# INTERFACE BAND 3 — Physical signals
# ════════════════════════════════════════════════════════════════════════════
box(TIER_X1, IB3_Y1, TIER_W, IB3_Y2-IB3_Y1, BL0, BL1, sw=0.7, rx=0)

IB3_MID = (IB3_Y1 + IB3_Y2) // 2

arr_down(CMD_X, IB3_Y1+4, IB3_Y2-4, CMD, sw=2.2)
txt(CMD_X-4, IB3_MID+4, "PWM →", a="end", fs=9, fw=600, fill=CMD)

arr_up(FBK_X, IB3_Y2-4, IB3_Y1+4, FBK, sw=2.2)
txt(FBK_X+4, IB3_MID+4, "← ENC A/B", a="start", fs=9, fw=600, fill=FBK)

txt(W//2, IB3_MID-5,
    "AIN1/2 · BIN1/2 · PWMA/B (3.3 V logic, GPIO)   ·   ENC-A · ENC-B quadrature (11 CPR)   ·   I²C SDA/SCL (MPU-6050, 0x68)",
    fs=10, fw=600, fill=BL8)
txt(W//2, IB3_MID+9, "Motor drive signals from ESP32   ·   Encoder feedback to ESP32   ·   IMU data via I²C", fs=9, fill=BL8)

# ════════════════════════════════════════════════════════════════════════════
# TIER 4 — Physical Actuation & Sensing
# ════════════════════════════════════════════════════════════════════════════
draw_tier(T4_Y1, T4_Y2, BL0, BL6, BL6,
    "Tier 4 — Physical Actuation & Sensing")

draw_sub(0, T4_Y1, T4_Y2, BL8, BL1, "M1  —  left wheel", [
    "JGA25-370  ·  12 V rated",
    "259 RPM @ 11.1 V supply",
    "Quadrature encoder 11 CPR",
    "4 mm D-shaft  ·  65 mm wheel",
])

draw_sub(1, T4_Y1, T4_Y2, BL8, BL1, "M2  —  right wheel", [
    "JGA25-370  ·  12 V rated",
    "259 RPM @ 11.1 V supply",
    "Quadrature encoder 11 CPR",
    "4 mm D-shaft  ·  65 mm wheel",
])

draw_sub(2, T4_Y1, T4_Y2, BL8, BL1, "TB6612FNG  —  H-bridge", [
    "Dual H-bridge motor driver",
    "VM: 11.1 V  ·  VCC: 3.3 V",
    "1.2 A cont.  /  3.2 A peak",
    "AO1/AO2 → M1 · BO1/BO2 → M2",
])

draw_sub(3, T4_Y1, T4_Y2, BL8, BL1, "MPU-6050  —  6DOF IMU", [
    "3-axis accel + 3-axis gyro",
    "I²C addr 0x68  ·  VCC 3.3 V",
    "SDA/SCL → ESP32 GPIO 21/22",
    "Pub: /imu/data @ 50 Hz",
])

# ════════════════════════════════════════════════════════════════════════════
# CLOSED-LOOP SUMMARY ARROWS  (external vertical channels)
# ════════════════════════════════════════════════════════════════════════════
# Command channel (right side): draws continuous vertical line alongside tiers
# Feedback channel (left side): same

# Right-side continuous command channel line
seg(CMD_X, T1_Y2, CMD_X, T4_Y2, CMD, sw=1.0, dash="4,3")
# Left-side continuous feedback channel line
seg(FBK_X, T4_Y1, FBK_X, T1_Y1, FBK, sw=1.0, dash="4,3")

# ════════════════════════════════════════════════════════════════════════════
# LEGEND
# ════════════════════════════════════════════════════════════════════════════
box(TIER_X1, LEG_Y, TIER_W, H-LEG_Y-4, GY0, GY1, sw=0.6, rx=5)

lx = 22
# Command flow
seg(lx, LEG_Y+18, lx+30, LEG_Y+18, CMD, sw=2.5)
L(f'<polygon points="{lx+33},{LEG_Y+15} {lx+39},{LEG_Y+18} {lx+33},{LEG_Y+21}" fill="{CMD}"/>')
txt(lx+45, LEG_Y+22, "Command / control flow  (u_safe → /cmd_vel → PWM → motors)", a="start", fs=10, fill=BLK)
lx += 430

# Feedback flow
seg(lx, LEG_Y+18, lx+30, LEG_Y+18, FBK, sw=2.5)
L(f'<polygon points="{lx+33},{LEG_Y+15} {lx+39},{LEG_Y+18} {lx+33},{LEG_Y+21}" fill="{FBK}"/>')
txt(lx+45, LEG_Y+22, "Feedback  (encoders → /odom/unfiltered → EKF → /odom → r, v)", a="start", fs=10, fill=BLK)

# Second row of legend
lx2 = 22
txt(lx2, LEG_Y+42, "Color coding:", a="start", fs=10, fw=600, fill=GY8)
for col, label in [(PU6,"Tier 1: Python planning layer (Host PC)"),(TE6,"Tier 2: ROS 2 motion control (RPi4)"),(CO6,"Tier 3: Motor drive firmware (ESP32)"),(BL6,"Tier 4: Physical actuation/sensing")]:
    box(lx2+88, LEG_Y+33, 14, 12, col, col, rx=2)
    txt(lx2+106, LEG_Y+43, label, a="start", fs=10, fill=BLK)
    lx2 += len(label)*6 + 24

L('</svg>')

svg = '\n'.join(lines)
with open('/home/claude/sw_arch3b.svg','w') as f:
    f.write(svg)
print(f"Done — {len(svg)} bytes, {svg.count(chr(10))} lines")
