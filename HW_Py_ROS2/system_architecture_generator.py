"""Generate system and software architecture SVGs for Tier B mobile robot project."""

# ── Common SVG helpers ──────────────────────────────────────────────────────

def rect(x, y, w, h, fill, stroke, sw=0.8, rx=4):
    return f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>'

def text(x, y, s, anchor="middle", cls="ts", fill=None):
    f = f' fill="{fill}"' if fill else ""
    return f'<text x="{x}" y="{y}" text-anchor="{anchor}" class="{cls}"{f}>{s}</text>'

def line(x1, y1, x2, y2, stroke, sw=1.5, dash=""):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{stroke}" stroke-width="{sw}"{d}/>'

def arrow(x1, y1, x2, y2, stroke, sw=1.5, dash=""):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{stroke}" stroke-width="{sw}" marker-end="url(#arr)"{d}/>'

def path(d, stroke, sw=1.5, fill="none", dash=""):
    da = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{da}/>'

DEFS = '''<defs>
  <marker id="arr" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">
    <path d="M2 1L8 5L2 9" fill="none" stroke="context-stroke" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
  </marker>
  <marker id="arr2" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto">
    <path d="M2 1L8 5L2 9" fill="none" stroke="context-stroke" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
  </marker>
</defs>'''

STYLE = '''<style>
.th{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif;font-size:13px;font-weight:600;fill:#1a1a1a;}
.t {font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif;font-size:12px;font-weight:400;fill:#1a1a1a;}
.ts{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif;font-size:11px;font-weight:400;fill:#444441;}
.tl{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif;font-size:10px;font-weight:400;fill:#5f5e5a;}
.tc{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif;font-size:11px;font-weight:500;fill:#1a1a1a;}
.title{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif;font-size:15px;font-weight:600;fill:#1a1a1a;}
.sub  {font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif;font-size:11px;font-weight:400;fill:#5f5e5a;}
</style>'''

# ── Colour palette ──────────────────────────────────────────────────────────
C = {
    'gray50':'#F1EFE8','gray200':'#B4B2A9','gray600':'#5F5E5A','gray800':'#2C2C2A',
    'pu50':'#EEEDFE','pu100':'#CECBF6','pu200':'#AFA9EC','pu600':'#534AB7','pu800':'#3C3489',
    'te50':'#E1F5EE','te100':'#9FE1CB','te200':'#5DCAA5','te600':'#0F6E56','te800':'#085041',
    'co50':'#FAECE7','co100':'#F5C4B3','co200':'#F0997B','co600':'#993C1D','co800':'#712B13',
    'am50':'#FAEEDA','am100':'#FAC775','am200':'#EF9F27','am600':'#854F0B','am800':'#633806',
    'bl50':'#E6F1FB','bl100':'#B5D4F4','bl200':'#85B7EB','bl600':'#185FA5','bl800':'#0C447C',
    'gn50':'#EAF3DE','gn200':'#97C459','gn600':'#3B6D11','gn800':'#27500A',
    'wh':'#FFFFFF',
}

# ══════════════════════════════════════════════════════════════════════════════
#  DIAGRAM 1 — SYSTEM ARCHITECTURE  (960 × 640)
# ══════════════════════════════════════════════════════════════════════════════

def system_arch_svg():
    els = []
    W, H = 960, 648

    # ── background ──
    els.append(f'<rect width="{W}" height="{H}" fill="{C["wh"]}"/>')

    # ── title ──
    els.append(text(480, 20, "Tier B Mobile Robot — System Architecture", cls="title"))
    els.append(text(480, 36, "Systems Engineering view · 4 differential-drive robots · layered · traceable · modular", cls="sub"))

    # ─────────────────────────────────────────────────────────────────────────
    # L0 INFRASTRUCTURE  y=46..96
    # ─────────────────────────────────────────────────────────────────────────
    els.append(rect(10, 46, 940, 50, C['gray50'], C['gray200'], rx=6))
    els.append(text(22, 62, "L0", anchor="start", cls="tc", fill=C['gray600']))
    els.append(text(22, 76, "Infrastructure", anchor="start", cls="tl", fill=C['gray600']))

    # Router box: centered x=370..590
    els.append(rect(370, 52, 220, 38, C['wh'], C['gray600'], rx=4))
    els.append(text(480, 67, "Home WiFi Router", cls="tc", fill=C['gray800']))
    els.append(text(480, 81, "IEEE 802.11n/ac · 2.4/5 GHz", cls="tl", fill=C['gray600']))

    # ─────────────────────────────────────────────────────────────────────────
    # WiFi MEDIUM BAND  y=96..120
    # ─────────────────────────────────────────────────────────────────────────
    els.append(rect(10, 96, 940, 24, C['am50'], C['am200'], rx=0))
    els.append(text(480, 112, "Physical medium: WiFi 802.11n/ac  ·  TCP/UDP  ·  ZMQ over UDP  ·  ROS 2 DDS multicast", cls="tl", fill=C['am800']))

    # arrows from router to Host PC (x=218) and RPi4 (x=690), vertical
    els.append(arrow(440, 90, 218, 128, C['am600'], sw=1.5))
    els.append(arrow(520, 90, 690, 128, C['am600'], sw=1.5))

    # ─────────────────────────────────────────────────────────────────────────
    # L1 COMPUTE LAYER  y=120..308
    # ─────────────────────────────────────────────────────────────────────────
    els.append(rect(10, 120, 940, 188, C['pu50'], C['pu200'], rx=6))
    els.append(text(22, 136, "L1", anchor="start", cls="tc", fill=C['pu600']))
    els.append(text(22, 150, "Computing", anchor="start", cls="tl", fill=C['pu600']))

    # ── Host PC box: x=55..442 ──
    els.append(rect(55, 127, 387, 174, C['pu100'], C['pu600'], rx=5))
    els.append(text(248, 142, "Host PC  ·  Ubuntu 22.04  ·  ROS 2 Humble  ·  Python 3.12", cls="tc", fill=C['pu800']))

    # Planning layer sub-box: x=63..240
    els.append(rect(63, 150, 178, 143, C['wh'], C['pu600'], rx=4))
    els.append(rect(63, 150, 178, 16, C['pu600'], C['pu600'], rx=4))  # header fill
    els.append(text(152, 161, "Python planning layer", cls="tl", fill=C['wh']))
    lines_plan = [
        "coordinator_node.py",
        "controller_node.py × 4",
        "cosim_manager.py",
        "hybrid safety filter",
        "ZMQ REQ/REP mesh",
        "matlab.engine (opt.)",
    ]
    for i, ln in enumerate(lines_plan):
        els.append(text(152, 180 + i*17, ln, cls="tl", fill=C['pu800']))

    # Co-Sim Bridge sub-box: x=248..432
    els.append(rect(248, 150, 186, 143, C['wh'], C['pu600'], rx=4))
    els.append(rect(248, 150, 186, 16, C['pu600'], C['pu600'], rx=4))
    els.append(text(341, 161, "Co-simulation bridge layer", cls="tl", fill=C['wh']))
    lines_cosim = [
        "cosim_config.py",
        "cosim_bridge.py",
        "cosim_logger.py",
        "matlab.engine API",
        "4× CSV log streams",
        "MATLAB/Simulink (opt.)",
    ]
    for i, ln in enumerate(lines_cosim):
        els.append(text(341, 180 + i*17, ln, cls="tl", fill=C['pu800']))

    # ── Bidirectional WiFi arrows in center x=442..518 ──
    els.append(text(480, 195, "WiFi 802.11n", cls="tl", fill=C['am800']))
    # → u_safe
    els.append(arrow(443, 210, 517, 210, C['co600'], sw=1.5))
    els.append(text(480, 207, "u_safe (ZMQ)", cls="tl", fill=C['co800']))
    # ← odom
    els.append(arrow(518, 235, 443, 235, C['te600'], sw=1.5))
    els.append(text(480, 232, "r, v (ZMQ)", cls="tl", fill=C['te800']))
    # interface label
    els.append(text(480, 260, "TCP/UDP", cls="tl", fill=C['am600']))
    els.append(text(480, 273, "ZMQ REQ/REP", cls="tl", fill=C['am600']))

    # ── RPi4 box: x=518..942 ──
    els.append(rect(518, 127, 424, 174, C['te100'], C['te600'], rx=5))
    els.append(text(730, 142, "Raspberry Pi 4B × 4  ·  Ubuntu 22.04  ·  ROS 2 Humble", cls="tc", fill=C['te800']))

    # robot_agent sub-box: x=526..706
    els.append(rect(526, 150, 180, 143, C['wh'], C['te600'], rx=4))
    els.append(rect(526, 150, 180, 16, C['te600'], C['te600'], rx=4))
    els.append(text(616, 161, "Motion control layer", cls="tl", fill=C['wh']))
    lines_agent = [
        "robot_agent.py",
        "ZMQ ↔ ROS 2 bridge",
        "micro_ros_agent",
        "ROS2 pub: /cmd_vel",
        "ROS2 sub: /odom",
        "ROS2 sub: /imu/data",
    ]
    for i, ln in enumerate(lines_agent):
        els.append(text(616, 180 + i*17, ln, cls="tl", fill=C['te800']))

    # ROS2 interfaces sub-box: x=716..934
    els.append(rect(716, 150, 218, 143, C['wh'], C['te600'], rx=4))
    els.append(rect(716, 150, 218, 16, C['te600'], C['te600'], rx=4))
    els.append(text(825, 161, "ROS 2 interfaces", cls="tl", fill=C['wh']))
    lines_ros = [
        "/cmd_vel  geometry_msgs/Twist",
        "/odom  nav_msgs/Odometry",
        "/odom/unfiltered  (raw wheel)",
        "/imu/data  sensor_msgs/Imu",
        "TF: odom → base_footprint",
        "ROS2 DDS (local, per robot)",
    ]
    for i, ln in enumerate(lines_ros):
        els.append(text(825, 180 + i*17, ln, cls="tl", fill=C['te800']))

    # ─────────────────────────────────────────────────────────────────────────
    # USB SERIAL BAND  y=308..332
    # ─────────────────────────────────────────────────────────────────────────
    els.append(rect(10, 308, 940, 24, C['gray50'], C['gray200'], rx=0))
    els.append(text(480, 324, "Interface: USB 2.0 · micro-ROS serial agent · 1 Mbps · per robot (RPi4 USB-A → ESP32 USB micro)", cls="tl", fill=C['gray600']))
    els.append(arrow(730, 308, 730, 332, C['gray600'], sw=1.5))

    # ─────────────────────────────────────────────────────────────────────────
    # L2 EMBEDDED CONTROL  y=332..458
    # ─────────────────────────────────────────────────────────────────────────
    els.append(rect(10, 332, 940, 126, C['co50'], C['co200'], rx=6))
    els.append(text(22, 348, "L2", anchor="start", cls="tc", fill=C['co600']))
    els.append(text(22, 362, "Embedded", anchor="start", cls="tl", fill=C['co600']))
    els.append(text(22, 376, "control", anchor="start", cls="tl", fill=C['co600']))

    # Left side: Power distribution summary x=55..445
    els.append(rect(55, 340, 388, 110, C['wh'], C['co200'], rx=4))
    els.append(rect(55, 340, 388, 16, C['co200'], C['co200'], rx=4))
    els.append(text(249, 351, "Power distribution (per robot)", cls="tl", fill=C['co800']))
    pwr_lines = [
        "BAT1: Tattu 3S LiPo 11.1 V · 2300 mAh · XT60 connector",
        "F1:   5 A automotive blade fuse (inline, before switch)",
        "SW1:  SPST 2-pin toggle switch (main power disconnect)",
        "U1:   DC-DC Buck 7–35 V → 5 V / 3 A USB-C (powers RPi4)",
        "U2:   TB6612FNG VM ← 11.1 V  (motor supply rail)",
        "U4:   ESP32 VCC 3.3 V ← ESP32 onboard LDO",
    ]
    for i, ln in enumerate(pwr_lines):
        els.append(text(249, 372 + i*15, ln, cls="tl", fill=C['co800']))

    # ESP32 box: x=518..942
    els.append(rect(518, 340, 424, 110, C['co100'], C['co600'], rx=5))
    els.append(text(730, 354, "ESP32 DevKit C × 4  ·  micro-ROS  ·  linorobot2_hardware firmware", cls="tc", fill=C['co800']))

    # Three sub-boxes
    # micro-ROS sub: x=526..686
    els.append(rect(526, 362, 160, 80, C['wh'], C['co600'], rx=4))
    els.append(rect(526, 362, 160, 15, C['co600'], C['co600'], rx=4))
    els.append(text(606, 372, "micro-ROS node", cls="tl", fill=C['wh']))
    for i, ln in enumerate(["Sub: /cmd_vel","Pub: /odom/unfiltered","Pub: /imu/data","USB serial transport"]):
        els.append(text(606, 386 + i*14, ln, cls="tl", fill=C['co800']))

    # PID sub: x=696..836
    els.append(rect(696, 362, 140, 80, C['wh'], C['co600'], rx=4))
    els.append(rect(696, 362, 140, 15, C['co600'], C['co600'], rx=4))
    els.append(text(766, 372, "PID velocity ctrl", cls="tl", fill=C['wh']))
    for i, ln in enumerate(["Left wheel PID","Right wheel PID","50 Hz control loop","Wheel kinematics"]):
        els.append(text(766, 386 + i*14, ln, cls="tl", fill=C['co800']))

    # Driver+ISR sub: x=846..934
    els.append(rect(846, 362, 88, 80, C['wh'], C['co600'], rx=4))
    els.append(rect(846, 362, 88, 15, C['co600'], C['co600'], rx=4))
    els.append(text(890, 372, "Driver + ISR", cls="tl", fill=C['wh']))
    for i, ln in enumerate(["TB6612FNG","Encoder ISR","11 CPR/rev","MPU-6050 I²C"]):
        els.append(text(890, 386 + i*14, ln, cls="tl", fill=C['co800']))

    # ─────────────────────────────────────────────────────────────────────────
    # PHYSICAL SIGNALS BAND  y=458..480
    # ─────────────────────────────────────────────────────────────────────────
    els.append(rect(10, 458, 940, 22, C['bl50'], C['bl200'], rx=0))
    els.append(text(480, 473, "Interface: PWM/GPIO (AIN1/2·BIN1/2·PWMA/B) · Encoder A/B quadrature signals · I²C SDA/SCL · DC motor wires", cls="tl", fill=C['bl800']))
    els.append(arrow(730, 458, 730, 480, C['bl600'], sw=1.5))

    # ─────────────────────────────────────────────────────────────────────────
    # L3 PHYSICAL LAYER  y=480..638
    # ─────────────────────────────────────────────────────────────────────────
    els.append(rect(10, 480, 940, 158, C['bl50'], C['bl100'], rx=6))
    els.append(text(22, 496, "L3", anchor="start", cls="tc", fill=C['bl600']))
    els.append(text(22, 510, "Physical", anchor="start", cls="tl", fill=C['bl600']))
    els.append(text(22, 524, "actuation", anchor="start", cls="tl", fill=C['bl600']))

    # Left side: MPU-6050 + Ball caster x=55..445
    els.append(rect(55, 488, 175, 140, C['wh'], C['bl200'], rx=4))
    els.append(rect(55, 488, 175, 15, C['bl200'], C['bl200'], rx=4))
    els.append(text(142, 498, "U5 — MPU-6050 GY-521", cls="tl", fill=C['bl800']))
    for i, ln in enumerate(["6DOF IMU (3-axis accel + gyro)","I²C addr: 0x68","VCC: 3.3 V (from ESP32)","SDA → GPIO 21  SCL → GPIO 22","Publishes /imu/data @ 50 Hz"]):
        els.append(text(142, 512 + i*14, ln, cls="tl", fill=C['bl800']))

    els.append(rect(240, 488, 256, 140, C['wh'], C['bl200'], rx=4))
    els.append(rect(240, 488, 256, 15, C['bl200'], C['bl200'], rx=4))
    els.append(text(368, 498, "Mechanical (per robot)", cls="tl", fill=C['bl800']))
    for i, ln in enumerate(["2WD chassis · acrylic/aluminium plate","JGA25-370 motor mounts (25 mm body)","65 mm wheels × 2 (driven)","19 mm Pololu ball caster (rear passive)","Wheelbase ≈ 160 mm"]):
        els.append(text(368, 512 + i*14, ln, cls="tl", fill=C['bl800']))

    # TB6612FNG: x=518..658
    els.append(rect(518, 488, 140, 68, C['wh'], C['bl600'], rx=4))
    els.append(rect(518, 488, 140, 15, C['bl600'], C['bl600'], rx=4))
    els.append(text(588, 498, "U2 — TB6612FNG", cls="tl", fill=C['wh']))
    for i, ln in enumerate(["Dual H-bridge driver","VM: 11.1 V (motor rail)","1.2 A cont. / 3.2 A peak","VCC logic: 3.3 V"]):
        els.append(text(588, 512 + i*14, ln, cls="tl", fill=C['bl800']))

    # M1: x=668..788
    els.append(rect(668, 488, 120, 68, C['wh'], C['bl600'], rx=4))
    els.append(rect(668, 488, 120, 15, C['bl600'], C['bl600'], rx=4))
    els.append(text(728, 498, "M1 — left wheel", cls="tl", fill=C['wh']))
    for i, ln in enumerate(["JGA25-370 · 12 V","280 RPM (at 11.1 V: 259)","Quad encoder 11 CPR","4 mm D-shaft"]):
        els.append(text(728, 512 + i*14, ln, cls="tl", fill=C['bl800']))

    # M2: x=798..918
    els.append(rect(798, 488, 120, 68, C['wh'], C['bl600'], rx=4))
    els.append(rect(798, 488, 120, 15, C['bl600'], C['bl600'], rx=4))
    els.append(text(858, 498, "M2 — right wheel", cls="tl", fill=C['wh']))
    for i, ln in enumerate(["JGA25-370 · 12 V","280 RPM (at 11.1 V: 259)","Quad encoder 11 CPR","4 mm D-shaft"]):
        els.append(text(858, 512 + i*14, ln, cls="tl", fill=C['bl800']))

    # Second row: power train boxes y=566..628
    els.append(rect(518, 566, 100, 62, C['wh'], C['co600'], rx=4))
    els.append(rect(518, 566, 100, 14, C['co600'], C['co600'], rx=4))
    els.append(text(568, 575, "BAT1 · 3S LiPo", cls="tl", fill=C['wh']))
    for i, ln in enumerate(["11.1 V · 2300 mAh","XT60 · 45C","~2 h runtime"]):
        els.append(text(568, 588 + i*14, ln, cls="tl", fill=C['co800']))

    els.append(rect(628, 566, 100, 62, C['wh'], C['co600'], rx=4))
    els.append(rect(628, 566, 100, 14, C['co600'], C['co600'], rx=4))
    els.append(text(678, 575, "F1 · SW1", cls="tl", fill=C['wh']))
    for i, ln in enumerate(["5 A blade fuse","SPST 2-pin switch","Main disconnect"]):
        els.append(text(678, 588 + i*14, ln, cls="tl", fill=C['co800']))

    els.append(rect(738, 566, 110, 62, C['wh'], C['am600'], rx=4))
    els.append(rect(738, 566, 110, 14, C['am600'], C['am600'], rx=4))
    els.append(text(793, 575, "U1 · Buck conv.", cls="tl", fill=C['wh']))
    for i, ln in enumerate(["7–35 V → 5 V / 3 A","USB-C out → RPi4","Bauer Electronics"]):
        els.append(text(793, 588 + i*14, ln, cls="tl", fill=C['am800']))

    els.append(rect(858, 566, 80, 62, C['wh'], C['te600'], rx=4))
    els.append(rect(858, 566, 80, 14, C['te600'], C['te600'], rx=4))
    els.append(text(898, 575, "U3 · RPi 4B", cls="tl", fill=C['wh']))
    for i, ln in enumerate(["5 V USB-C in","Ubuntu 22.04","64-bit ARM"]):
        els.append(text(898, 588 + i*14, ln, cls="tl", fill=C['te800']))

    # Power chain arrows (low row)
    els.append(arrow(618, 597, 627, 597, C['co600'], sw=1.2))
    els.append(arrow(728, 597, 737, 597, C['co600'], sw=1.2))
    els.append(arrow(848, 597, 857, 597, C['am600'], sw=1.2))

    # ── "× 4 robots" annotation ──
    els.append(rect(518, 488, 420, 140, "none", C['te600'], rx=6, sw=2))
    els.append(text(940, 484, "× 4 robots (identical)", anchor="end", cls="tc", fill=C['te600']))

    svg = f'<svg width="960" height="{H}" viewBox="0 0 960 {H}" xmlns="http://www.w3.org/2000/svg" role="img">\n'
    svg += '<title>Tier B Mobile Robot — System Architecture</title>\n'
    svg += '<desc>Layered system architecture: L0 Infrastructure, L1 Computing (Host PC and RPi4), L2 Embedded control (ESP32), L3 Physical layer (motors, IMU, power)</desc>\n'
    svg += STYLE + '\n' + DEFS + '\n'
    svg += '\n'.join(els)
    svg += '\n</svg>'
    return svg


# ── Write file ───────────────────────────────────────────────────────────────
import os, re

svg = system_arch_svg()

# Ensure all bare & are XML-safe (must be &amp; in SVG/XML)
svg = re.sub(r'&(?!amp;|lt;|gt;|quot;|apos;|#)', '&amp;', svg)

out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'system_architecture.svg')
with open(out_path, 'w', encoding='utf-8') as f:
    f.write(svg)
print(f"Written: {out_path}  ({len(svg):,} bytes)")
