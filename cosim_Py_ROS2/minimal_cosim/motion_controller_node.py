"""
motion_controller_node.py
=========================
ROS2 Motion Control Layer  —  Formation Co-Sim Platform 1 (Mini)

WHAT THIS NODE DOES
-------------------
This node represents the **motion control layer** in your co-simulation.
In the real hardware stack this role is played by linorobot2 on the Raspberry Pi 4.
Here we run it on the same Ubuntu VM to learn the ROS2 communication model.

The node does three things in a loop:
  1. RECEIVES  velocity commands  (linear speed v,  angular rate ω)
               from the Python planning layer via the  /cmd_vel  topic.
  2. SIMULATES unicycle kinematics internally at 20 Hz to compute the pose (x, y, θ).
  3. PUBLISHES  the resulting odometry  (/odom)  and a  health/diagnostic report
               (/diagnostics)  back to the planning layer.

ROS2 CONCEPTS DEMONSTRATED
---------------------------
  • Node        — the fundamental unit of computation in ROS2
  • Subscriber  — receives messages on a named topic
  • Publisher   — sends messages on a named topic
  • Timer       — calls a callback at a fixed rate
  • Messages    — typed data structures shared between nodes
      - geometry_msgs/Twist     (velocity command: linear.x, angular.z)
      - nav_msgs/Odometry       (pose + velocity estimate)
      - diagnostic_msgs/DiagnosticArray  (health key-value pairs)
  • QoS (queue size) — how many messages to buffer if processing is slow
  • get_logger()     — ROS2 structured logging (replaces print())
  • get_clock()      — ROS2 time (so timestamps are simulation-aware)

UNICYCLE MODEL
--------------
  ẋ     = v · cos(θ)
  ẏ     = v · sin(θ)
  θ̇     = ω

This is the same model used in your MATLAB/Simulink co-simulation platform.
"""

import math

import rclpy                          # ROS2 Python client library — the entry point for everything
from rclpy.node import Node           # Base class for all ROS2 nodes

# ROS2 message types — these are the "data contracts" between nodes
from geometry_msgs.msg import Twist   # cmd_vel:  linear.x (m/s), angular.z (rad/s)
from nav_msgs.msg import Odometry     # odom:     pose (x, y, quat) + twist (v, ω)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# ═══════════════════════════════════════════════════════════════════════════════
class MotionControllerNode(Node):
    """
    ROS2 node that simulates a unicycle robot's motion control layer.

    Analogy with your existing platform
    ------------------------------------
    In your MATLAB co-simulation:
        cosim_manager.py  ← sends u_ref → MATLAB Engine → Simulink → returns odom

    Here, the equivalent chain is:
        planner_node.py   ← sends Twist → /cmd_vel topic → MotionControllerNode
                                                          → /odom topic → planner_node.py
    """

    def __init__(self):
        # ── Step 1: Initialise the Node ────────────────────────────────────
        # 'motion_controller' is the node name — must be unique in the ROS2 graph.
        # You will see this name when running: ros2 node list
        super().__init__('motion_controller')

        # ── Step 2: Robot state (the "plant" state) ────────────────────────
        # These are updated every simulation tick.
        self.x = 0.0        # [m]   world-frame x position
        self.y = 0.0        # [m]   world-frame y position
        self.theta = 0.0    # [rad] heading angle, 0 = pointing along +x axis
        self.v = 0.0        # [m/s] current linear speed (from last cmd_vel)
        self.omega = 0.0    # [rad/s] current angular rate (from last cmd_vel)

        # ── Step 3: Simulation parameters ─────────────────────────────────
        self.dt = 0.05  # [s] integration step = 1/20 Hz
                        # This is the "Simulink fixed-step" equivalent

        # ── Step 4: Create a SUBSCRIBER for velocity commands ──────────────
        #
        # create_subscription(msg_type, topic_name, callback, qos_depth)
        #
        # - msg_type   : Twist  — tells ROS2 what data format to expect
        # - topic_name : 'cmd_vel'  — the "channel name"; any node publishing
        #                to 'cmd_vel' on the same ROS_DOMAIN_ID will reach us
        # - callback   : self.cmd_vel_callback  — Python function called each time
        #                a new message arrives
        # - qos_depth  : 10  — buffer up to 10 unprocessed messages before dropping
        #
        # IMPORTANT: the subscriber is non-blocking. ROS2 calls cmd_vel_callback
        # asynchronously whenever a message arrives, via rclpy.spin().
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        # ── Step 5: Create a PUBLISHER for odometry ────────────────────────
        #
        # create_publisher(msg_type, topic_name, qos_depth)
        #
        # We will call odom_pub.publish(msg) inside the simulation timer.
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # ── Step 6: Create a PUBLISHER for diagnostics ─────────────────────
        # DiagnosticArray is the standard ROS2 health-reporting message.
        # In the real robot this channel carries motor temperature, battery
        # voltage, encoder error flags, etc.
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        # ── Step 7: Create TIMERS ──────────────────────────────────────────
        #
        # create_timer(period_seconds, callback)
        #
        # A timer fires its callback at the given rate, independently of
        # any received messages.  This is how we run the simulation loop.
        #
        # Simulation loop at 20 Hz — integrates kinematics + publishes /odom
        self.sim_timer = self.create_timer(self.dt, self.simulation_step)

        # Diagnostics at 1 Hz — health heartbeat
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

        # ── Ready ──────────────────────────────────────────────────────────
        # get_logger() is ROS2's structured logger.  Uses ROS_LOG_LEVEL env var.
        # In production you'd use .debug() for verbose, .warn() for issues.
        self.get_logger().info('MotionControllerNode started  (dt=%.3f s, 20 Hz)', self.dt)

    # ═══════════════════════════════════════════════════════════════════════
    # SUBSCRIBER CALLBACK — called by rclpy whenever a /cmd_vel message arrives
    # ═══════════════════════════════════════════════════════════════════════
    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Receive a velocity command from the planning layer.

        msg.linear.x   → desired linear speed  v   [m/s]
        msg.angular.z  → desired angular rate   ω   [rad/s]

        This is the EXACT same interface used by linorobot2:
        the Raspberry Pi subscribes to /cmd_vel Twist messages and
        converts them to left/right wheel PWM via the diff-drive controller.
        Here we just store the values for the simulation integrator.
        """
        self.v     = msg.linear.x
        self.omega = msg.angular.z
        self.get_logger().debug(
            'cmd_vel received:  v=%.3f m/s,  ω=%.3f rad/s', self.v, self.omega
        )

    # ═══════════════════════════════════════════════════════════════════════
    # TIMER CALLBACK — runs at 20 Hz
    # ═══════════════════════════════════════════════════════════════════════
    def simulation_step(self) -> None:
        """
        Integrate unicycle kinematics one step and publish odometry.

        This replaces what Simulink does in your MATLAB co-simulation:
            x(k+1) = x(k) + v·cos(θ)·dt
            y(k+1) = y(k) + v·sin(θ)·dt
            θ(k+1) = θ(k) + ω·dt
        """
        # Euler integration of unicycle kinematics
        self.x     += self.v * math.cos(self.theta) * self.dt
        self.y     += self.v * math.sin(self.theta) * self.dt
        self.theta += self.omega * self.dt

        # Keep θ in (−π, π] to avoid numerical drift
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish updated pose as Odometry message
        self._publish_odometry()

    def _publish_odometry(self) -> None:
        """
        Build and publish a nav_msgs/Odometry message.

        Odometry structure:
          header.stamp        — ROS2 timestamp (wall-clock or sim-clock)
          header.frame_id     — coordinate frame of the pose  ('odom')
          child_frame_id      — coordinate frame of the twist ('base_link')
          pose.pose.position  — (x, y, z)
          pose.pose.orientation — quaternion (x, y, z, w)
          twist.twist.linear  — (vx, vy, vz) in body frame
          twist.twist.angular — (wx, wy, wz) in body frame

        WHY QUATERNION?
        ROS2 always expresses orientations as unit quaternions to avoid
        gimbal lock and to be consistent across 3D and 2D use cases.
        For a 2D robot rotating about the Z-axis:
          qx = qy = 0
          qz = sin(θ/2),  qw = cos(θ/2)
        """
        msg = Odometry()

        # Timestamp — always use node clock so replayed bags stay consistent
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'       # parent frame: fixed world origin
        msg.child_frame_id  = 'base_link'  # child  frame: robot's body centre

        # Pose
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0  # planar robot — no Z motion

        # Quaternion from heading angle θ (rotation about Z axis only)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity (in the robot's body frame — standard for Odometry)
        msg.twist.twist.linear.x  = self.v      # forward speed
        msg.twist.twist.angular.z = self.omega  # yaw rate

        self.odom_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    # TIMER CALLBACK — runs at 1 Hz
    # ═══════════════════════════════════════════════════════════════════════
    def publish_diagnostics(self) -> None:
        """
        Publish a health/status report as a DiagnosticArray message.

        DiagnosticArray is the ROS2 standard for structured health reporting.
        In linorobot2, this channel carries:
          - motor driver status (OK / WARN / ERROR)
          - battery voltage
          - encoder counts
          - IMU status

        Here we report the simulated robot state as key-value pairs.
        The planner_node subscribes to this to demonstrate the feedback channel.
        """
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name    = 'motion_controller/unicycle_sim'
        status.level   = DiagnosticStatus.OK       # 0=OK, 1=WARN, 2=ERROR
        status.message = 'Simulation running normally'
        status.values  = [
            KeyValue(key='x_m',          value=f'{self.x:.4f}'),
            KeyValue(key='y_m',          value=f'{self.y:.4f}'),
            KeyValue(key='theta_deg',    value=f'{math.degrees(self.theta):.2f}'),
            KeyValue(key='linear_vel',   value=f'{self.v:.4f}'),
            KeyValue(key='angular_vel',  value=f'{self.omega:.4f}'),
            KeyValue(key='sim_dt_s',     value=f'{self.dt:.3f}'),
        ]

        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)
        self.get_logger().info(
            '[DIAG] x=%.3f m,  y=%.3f m,  θ=%.1f°',
            self.x, self.y, math.degrees(self.theta)
        )


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════
def main(args=None) -> None:
    """
    Standard ROS2 node main function.  Called by the entry_point defined in setup.py.

    rclpy.init()       — initialise the ROS2 Python client library (connects to DDS)
    rclpy.spin(node)   — hand control to the ROS2 event loop:
                           - calls subscriber callbacks when messages arrive
                           - fires timer callbacks at their scheduled rates
                           - blocks until Ctrl+C or rclpy.shutdown()
    node.destroy_node()— clean up subscriptions, publishers, timers
    rclpy.shutdown()   — disconnect from DDS, free ROS2 resources
    """
    rclpy.init(args=args)
    node = MotionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received — shutting down MotionControllerNode')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
