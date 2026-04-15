"""
planner_node.py
===============
Python Planning Layer  —  Formation Co-Sim Platform 1 (Mini)

WHAT THIS NODE DOES
-------------------
This node represents the **planning layer** — the role played by your Python
hybrid MPC formation app in the full system.

The node does three things in a loop:
  1. GENERATES a velocity command (v, ω) from a simple reference trajectory
     (a circle, for this demo) and PUBLISHES it to the /cmd_vel topic.
  2. RECEIVES odometry (pose feedback) from the motion controller via /odom.
  3. RECEIVES health reports via /diagnostics and logs them.

WHY THIS STRUCTURE MATTERS FOR YOUR FULL SYSTEM
------------------------------------------------
In your full formation system:
  coordinator_node.py + controller_node.py  →  planning layer (this node)
  linorobot2 on RPi4                        →  motion controller (other node)
  WiFi + ROS2 DDS                           →  replaces local topic communication

The topic names (/cmd_vel, /odom) stay identical — only the transport changes.
For multiple robots you add a namespace prefix (/robot_1/cmd_vel, etc.).

ROS2 CONCEPTS DEMONSTRATED
---------------------------
  • MultiThreadedExecutor  — running callbacks in parallel threads
  • ReentrantCallbackGroup — allow subscriber callbacks to run concurrently
  • QoS profiles           — reliability vs. latency tradeoff
  • Quaternion → angle     — decoding orientation from Odometry messages
  • rclpy.ok()             — checking if ROS2 is still running
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray


# ═══════════════════════════════════════════════════════════════════════════════
class PlannerNode(Node):
    """
    Python planning layer — publishes velocity commands, receives odometry.

    In the mini demo the 'plan' is just a fixed circular trajectory:
        v = 0.20 m/s,   ω = 0.50 rad/s   →  circle of radius 0.40 m

    In the full system this becomes:
        v, ω = hybrid_MPC_formation_controller(current_state, formation_target)
    """

    def __init__(self):
        super().__init__('planner')

        # ── Callback group ──────────────────────────────────────────────────
        # ReentrantCallbackGroup allows subscriber callbacks (odom, diagnostics)
        # to be executed concurrently.  This is important when the planning
        # step takes non-trivial time (as it will with the real MPC solver).
        self._cb_group = ReentrantCallbackGroup()

        # ── Current robot state (updated by odometry callback) ──────────────
        self.current_x     = 0.0   # [m]
        self.current_y     = 0.0   # [m]
        self.current_theta = 0.0   # [rad]
        self.current_v     = 0.0   # [m/s] — from odom twist
        self.current_omega = 0.0   # [rad/s]

        # Internal planning clock (elapsed time in seconds)
        self._t = 0.0

        # ── Planning rate ───────────────────────────────────────────────────
        # 5 Hz  ↔  every 0.2 s
        # Your Python MPC runs at a similar outer-loop rate (planning_dt).
        self._planning_dt = 0.2

        # ── Publisher: send velocity commands → motion controller ───────────
        #
        # create_publisher(topic_type, topic_name, qos_history_depth)
        # - topic_type: the ROS2 message type (here, geometry_msgs/Twist)
        # - topic_name: the name of the topic to publish to (here, 'cmd_vel')
        # - qos_history_depth: how many messages to buffer if the subscriber is
        #   not keeping up (10 is a common choice for control topics)
        # Topic name:  'cmd_vel'
        # Message type: geometry_msgs/Twist
        #   msg.linear.x   = forward speed v     [m/s]
        #   msg.angular.z  = yaw rate      ω     [rad/s]
        #
        # All other fields (linear.y, linear.z, angular.x, angular.y) are
        # zero for a differential-drive robot (it cannot move sideways).
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ── Subscriber: receive odometry ← motion controller ───────────────
        #
        # The callback_group=self._cb_group argument opts this subscriber into
        # concurrent execution.  Without it, ROS2 would serialise all callbacks
        # in the default MutuallyExclusiveCallbackGroup.
        # create_subscription(topic_type, topic_name, callback, qos_history_depth, callback_group)
        # - topic_type: the ROS2 message type (here, nav_msgs/Odometry)
        # - topic_name: the name of the topic to subscribe to (here, 'odom')
        # - callback: the function to call when a new message arrives (here, self._odom_callback)
        # - qos_history_depth: how many messages to buffer if the publisher is
        #   sending faster than we can process (10 is a common choice for control topics)
        # - callback_group: which callback group to assign this subscriber to (here, self._cb_group for concurrent execution)
        # Topic name:  'odom'
        # Message type: nav_msgs/Odometry
        #  msg.pose.pose.position.{x,y}  = position (x, y) [m]
        #  msg.pose.pose.orientation.{z,w} = heading θ encoded as quaternion
        #  msg.twist.twist.linear.x       = forward velocity v [m/s]
        #  msg.twist.twist.angular.z      = yaw rate ω [rad/s]
        # The motion controller publishes odometry at a high rate (e.g. 50 Hz) to provide
        # timely feedback for the MPC controller.  
        # The planner runs at a lower rate (e.g. 5 Hz) to generate commands, so we use a QoS history depth of 10 to buffer the latest messages.
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            10,
            callback_group=self._cb_group,
        )

        # ── Subscriber: receive diagnostics ← motion controller ────────────
        self.diag_sub = self.create_subscription(
            DiagnosticArray,
            'diagnostics',
            self._diagnostics_callback,
            10,
            callback_group=self._cb_group,
        )

        # ── Planning timer at 5 Hz ──────────────────────────────────────────
        # The timer calls the _planning_step function every self._planning_dt seconds.
        # This is where your MPC controller would be called in the full system.
        # create_timer(, , callback_group)
        # - callback_group=self._cb_group allows the timer callback to run concurrently with the subscribers.
        self.planning_timer = self.create_timer(
            self._planning_dt,
            self._planning_step,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'PlannerNode started  (planning rate = {1.0 / self._planning_dt:.0f} Hz)'
        )

    # ═══════════════════════════════════════════════════════════════════════
    # SUBSCRIBER CALLBACK — /odom
    # ═══════════════════════════════════════════════════════════════════════
    def _odom_callback(self, msg: Odometry) -> None:
        """
        Receive the robot's estimated pose from the motion controller.

        This is the feedback signal your MPC controller needs:
          r_i  = (x_i, y_i)   position
          v_i  = (vx, vy)     velocity (we reconstruct from v and θ)

        QUATERNION → ANGLE
        ------------------
        ROS2 encodes heading as a quaternion (qx, qy, qz, qw).
        For a 2D robot (rotation only around Z):
          θ = 2 · atan2(qz, qw)
        This is the inverse of the encoding in motion_controller_node.py.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Decode heading from quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)

        # Velocity is in the body frame of the child_frame_id ('base_link')
        self.current_v     = msg.twist.twist.linear.x
        self.current_omega = msg.twist.twist.angular.z

        self.get_logger().info(
            f'[ODOM]  x={self.current_x:6.3f} m   y={self.current_y:6.3f} m'
            f'   θ={math.degrees(self.current_theta):6.1f}°'
            f'   v={self.current_v:5.3f} m/s   ω={self.current_omega:5.3f} rad/s'
        )

    # ═══════════════════════════════════════════════════════════════════════
    # SUBSCRIBER CALLBACK — /diagnostics
    # ═══════════════════════════════════════════════════════════════════════
    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        """
        Receive health reports from the motion controller.

        In the full system you would use these to:
          - Detect motor driver faults → pause formation commands
          - Monitor battery voltage   → send robots to dock
          - Check encoder health      → flag odometry as unreliable
        """
        level_map = {0: 'OK', 1: 'WARN', 2: 'ERROR'}
        for status in msg.status:
            level_str = level_map.get(status.level, 'UNKNOWN')
            kv_str = ',  '.join(f'{kv.key}={kv.value}' for kv in status.values)
            self.get_logger().info(
                f'[DIAG]  {status.name}  [{level_str}]  — {kv_str}'
            )

    # ═══════════════════════════════════════════════════════════════════════
    # TIMER CALLBACK — planning loop at 5 Hz
    # ═══════════════════════════════════════════════════════════════════════
    def _planning_step(self) -> None:
        """
        Generate the velocity command for the current planning step.

        DEMO PLAN: circular trajectory
            v     = 0.20 m/s   (constant forward speed)
            ω     = 0.50 rad/s (constant yaw rate)
            → circle radius R = v / ω = 0.40 m
            → period T = 2π / ω ≈ 12.6 s

        IN THE FULL SYSTEM this function becomes:
            u_safe = hybrid_safety_filter(
                         mpc_solve(current_state, formation_target)
                     )
            v, ω = unicycle_velocity_from_cartesian(u_safe, current_theta)
        """
        self._t += self._planning_dt

        # ── Simple circular trajectory (demo) ──────────────────────────────
        v_cmd     = 0.20   # [m/s]  forward speed
        omega_cmd = 0.50   # [rad/s] yaw rate

        # ── Build and publish Twist message ────────────────────────────────
        cmd = Twist()
        cmd.linear.x  = v_cmd
        cmd.angular.z = omega_cmd
        self.cmd_vel_pub.publish(cmd)

        self.get_logger().info(
            f'[PLAN]  t={self._t:6.2f} s   cmd → v={v_cmd:.3f} m/s,  ω={omega_cmd:.3f} rad/s'
            f'   |  current pose: x={self.current_x:.3f}, y={self.current_y:.3f},'
            f' θ={math.degrees(self.current_theta):.1f}°'
        )

        # ── Formation convergence check (placeholder) ─────────────────────
        # In the full system you would check: |z_i - (c_centroid + c_i)| < ε
        # and switch the hybrid controller mode accordingly.
        # Here we just log the elapsed time.
        period = 2.0 * math.pi / omega_cmd
        laps   = self._t / period
        if laps > 0 and abs(laps - round(laps)) < self._planning_dt / period:
            self.get_logger().info(
                f'[PLAN]  *** Completed {round(laps):.0f} lap(s) of the demo circle ***'
            )


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════
def main(args=None) -> None:
    """
    Entry point for the planner node.

    We use MultiThreadedExecutor here so that:
      - the planning timer callback
      - the odom subscriber callback
      - the diagnostics subscriber callback
    can all run concurrently (in separate threads).

    With the default SingleThreadedExecutor they would be serialised, meaning
    a slow MPC solve would block the odom callback from updating — that is
    not what you want in a real-time control loop.
    """
    rclpy.init(args=args)
    node = PlannerNode()

    # MultiThreadedExecutor: spawns a thread pool, dispatches callbacks
    # to available threads, allowing concurrent execution of callbacks.
    # num_threads=4 is a common choice for a simple node with a few callbacks, but you can adjust based on your workload.
    # 4 threads means that up to 4 callbacks can run simultaneously.  In this node we have:
    # - 1 planning timer callback (every 0.2 s)
    # - 1 odom subscriber callback (up to 50 Hz)
    # - 1 diagnostics subscriber callback (up to 10 Hz)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received — shutting down PlannerNode')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()