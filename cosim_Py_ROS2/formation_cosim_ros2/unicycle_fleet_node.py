"""
unicycle_fleet_node.py
======================
ROS2 node: N simulated unicycle (differential-drive) robots.

ROLE IN THE PLATFORM
--------------------
This node replaces the single-integrator dynamics inside plant_node.py with
physically realistic unicycle kinematics, communicated through ROS2 topics:

  plant_node.py (old):
      r(k+1) = r(k) + dt * u_safe           ← algebraic, instantaneous

  unicycle_fleet_node (new):
      ẋ   = v · cos(θ)                        ← nonholonomic, continuous
      ẏ   = v · sin(θ)
      θ̇   = ω
  where v, ω are received via /robot_N/cmd_vel Twist messages from the
  formation_planner_node.

TOPICS PER ROBOT (N = 1..n_agents)
------------------------------------
  /robot_N/cmd_vel  ← geometry_msgs/Twist   (v m/s, ω rad/s from planner)
  /robot_N/odom     → nav_msgs/Odometry     (x, y, θ, v, ω back to planner)
  /robot_N/diagnostics → DiagnosticArray   (health heartbeat at 1 Hz)

TIMING
------
  sim_rate : 20 Hz internal integration step (dt_sim = 0.05 s)
  The planner publishes Twist at its planning inner-step rate (cfg.dt = 1.0 s).
  Within each cfg.dt second, the robot integrates 20 steps with the same
  Twist command (zero-order hold — identical to Simulink's fixed-step solver).

ROS2 PARAMETERS (set via launch file or ros2 param set)
-------------------------------------------------------
  n_agents   : int   — number of robots to simulate (default 4)
  sim_rate   : float — integration frequency in Hz   (default 20.0)
  diag_rate  : float — diagnostics publish rate Hz   (default 1.0)
"""

from __future__ import annotations

import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# ═══════════════════════════════════════════════════════════════════════════════
class UnicycleFleetNode(Node):
    """
    Simulates N differential-drive robots in a single ROS2 node.

    Each robot i has independent state (x_i, y_i, θ_i) and subscribes to its
    own /robot_i/cmd_vel topic.  All robots share a single 20 Hz simulation
    timer — exactly like a single Simulink model running N unicycle subsystems.
    """

    def __init__(self) -> None:
        super().__init__('unicycle_fleet')

        # ── ROS2 parameters ────────────────────────────────────────────────
        # Declaring parameters makes them visible to `ros2 param list` and
        # overrideable from the launch file with parameters=[{...}].
        self.declare_parameter('n_agents',  4)
        self.declare_parameter('sim_rate',  20.0)
        self.declare_parameter('diag_rate', 1.0)

        self._n   = self.get_parameter('n_agents').value
        sim_rate  = self.get_parameter('sim_rate').value
        diag_rate = self.get_parameter('diag_rate').value

        self._dt_sim = 1.0 / sim_rate  # integration step [s]

        # ── Per-robot state ────────────────────────────────────────────────
        # Indexed 1..n (1-based to match MPC agent numbering).
        self._x:     Dict[int, float] = {i: 0.0 for i in range(1, self._n + 1)}
        self._y:     Dict[int, float] = {i: 0.0 for i in range(1, self._n + 1)}
        self._theta: Dict[int, float] = {i: 0.0 for i in range(1, self._n + 1)}
        self._v:     Dict[int, float] = {i: 0.0 for i in range(1, self._n + 1)}
        self._omega: Dict[int, float] = {i: 0.0 for i in range(1, self._n + 1)}

        # ReentrantCallbackGroup: all cmd_vel callbacks can run concurrently.
        self._cb = ReentrantCallbackGroup()

        # ── Per-robot publishers and subscribers ───────────────────────────
        self._odom_pubs: Dict[int, object] = {}
        self._diag_pubs: Dict[int, object] = {}

        for i in range(1, self._n + 1):
            ns = f'robot_{i}'

            # Subscriber: receive Twist from formation_planner_node
            self.create_subscription(
                Twist,
                f'{ns}/cmd_vel',
                self._make_cmd_vel_callback(i),
                10,
                callback_group=self._cb,
            )

            # Publisher: send Odometry to formation_planner_node
            self._odom_pubs[i] = self.create_publisher(Odometry, f'{ns}/odom', 10)

            # Publisher: health diagnostics
            self._diag_pubs[i] = self.create_publisher(
                DiagnosticArray, f'{ns}/diagnostics', 10
            )

        # ── Simulation timer at sim_rate Hz ───────────────────────────────
        # This is the "Simulink fixed-step clock" for all N robots.
        self.create_timer(self._dt_sim, self._simulation_step, callback_group=self._cb)

        # ── Diagnostics timer at diag_rate Hz ─────────────────────────────
        self.create_timer(1.0 / diag_rate, self._publish_diagnostics, callback_group=self._cb)

        self.get_logger().info(
            f'UnicycleFleetNode started: {self._n} robots | '
            f'sim={sim_rate:.0f} Hz (dt={self._dt_sim*1000:.1f} ms)'
        )

    # ═══════════════════════════════════════════════════════════════════════
    # Subscriber callback factory
    # ═══════════════════════════════════════════════════════════════════════

    def _make_cmd_vel_callback(self, robot_id: int):
        """
        Returns a closure that captures robot_id.

        We cannot use a single callback with the topic name, because Python
        closures in a loop need explicit capture.  This pattern is the standard
        ROS2 Pythonic way to create N subscribers sharing a template callback.
        """
        def callback(msg: Twist) -> None:
            self._v[robot_id]     = msg.linear.x
            self._omega[robot_id] = msg.angular.z
            self.get_logger().debug(
                f'[R{robot_id}] cmd_vel: v={self._v[robot_id]:.3f} m/s, '
                f'ω={self._omega[robot_id]:.3f} rad/s'
            )
        return callback

    # ═══════════════════════════════════════════════════════════════════════
    # Simulation timer callback — runs at 20 Hz
    # ═══════════════════════════════════════════════════════════════════════

    def _simulation_step(self) -> None:
        """
        Integrate unicycle kinematics for all N robots and publish odometry.

        Euler integration (identical to cosim_step.m in MATLAB platform):
            x(k+1)     = x(k)     + v · cos(θ) · dt_sim
            y(k+1)     = y(k)     + v · sin(θ) · dt_sim
            θ(k+1)     = θ(k)     + ω · dt_sim
        """
        now = self.get_clock().now().to_msg()
        for i in range(1, self._n + 1):
            v     = self._v[i]
            omega = self._omega[i]

            self._x[i]     += v * math.cos(self._theta[i]) * self._dt_sim
            self._y[i]     += v * math.sin(self._theta[i]) * self._dt_sim
            self._theta[i] += omega * self._dt_sim
            # Wrap θ to (−π, π]
            self._theta[i] = math.atan2(
                math.sin(self._theta[i]), math.cos(self._theta[i])
            )
            self._publish_odom(i, now)

    def _publish_odom(self, robot_id: int, stamp) -> None:
        """
        Publish nav_msgs/Odometry for robot robot_id.

        Pose encoding:
          position.x, position.y = world-frame position
          orientation (quaternion) encodes heading θ about Z axis:
              qz = sin(θ/2), qw = cos(θ/2), qx = qy = 0

        Twist encoding (in body frame):
          linear.x  = v   (forward speed)
          angular.z = ω   (yaw rate)
        """
        θ   = self._theta[robot_id]
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = f'robot_{robot_id}/base_link'

        msg.pose.pose.position.x = self._x[robot_id]
        msg.pose.pose.position.y = self._y[robot_id]
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(θ / 2.0)
        msg.pose.pose.orientation.w = math.cos(θ / 2.0)

        msg.twist.twist.linear.x  = self._v[robot_id]
        msg.twist.twist.angular.z = self._omega[robot_id]

        self._odom_pubs[robot_id].publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    # Diagnostics timer callback — runs at 1 Hz
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_diagnostics(self) -> None:
        """
        Publish a DiagnosticArray for each robot.

        In the real hardware stack (linorobot2 on RPi4), this channel carries
        motor driver status, battery voltage, encoder counts, and IMU health.
        Here we report the simulation state as a health proxy.
        """
        stamp = self.get_clock().now().to_msg()
        for i in range(1, self._n + 1):
            arr = DiagnosticArray()
            arr.header.stamp = stamp

            st = DiagnosticStatus()
            st.name    = f'unicycle_fleet/robot_{i}'
            st.level   = DiagnosticStatus.OK
            st.message = 'Simulation running normally'
            st.values  = [
                KeyValue(key='x_m',         value=f'{self._x[i]:.4f}'),
                KeyValue(key='y_m',         value=f'{self._y[i]:.4f}'),
                KeyValue(key='theta_deg',   value=f'{math.degrees(self._theta[i]):.2f}'),
                KeyValue(key='v_ms',        value=f'{self._v[i]:.4f}'),
                KeyValue(key='omega_rads',  value=f'{self._omega[i]:.4f}'),
            ]
            arr.status.append(st)
            self._diag_pubs[i].publish(arr)

        # Print a compact fleet summary (easier to read than N separate log lines)
        poses = '  '.join(
            f'R{i}=({self._x[i]:.2f},{self._y[i]:.2f},{math.degrees(self._theta[i]):.0f}°)'
            for i in range(1, self._n + 1)
        )
        self.get_logger().info(f'[FLEET] {poses}')


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnicycleFleetNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C — shutting down UnicycleFleetNode')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
