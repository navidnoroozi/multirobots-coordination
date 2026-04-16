"""
unicycle_fleet_node.py
======================
ROS2 node: N simulated unicycle (differential-drive) robots.

ARCHITECTURE — EVENT-DRIVEN
-----------------------------
Simulation is event-driven, not timer-driven:
  cmd_vel received → motor lag filter → Euler step → odom published

STARTUP DEADLOCK BOOTSTRAP
----------------------------
On startup a one-shot timer fires after 0.5 s and publishes the initial
odom for all robots.  Without this, the planner and fleet wait on each
other forever:
  planner waits for /robot_N/odom  →  fleet waits for cmd_vel  →  deadlock

MOTOR DYNAMICS MODEL (first-order lag)
----------------------------------------
  v_actual  += α · (v_des  − v_actual)
  ω_actual  += α · (ω_des  − ω_actual)

  α = kv · dt,   stability requires 0 < α ≤ 1.
  α = 1.0  →  perfect instantaneous tracking  (default, best for MPC validation)
  α = 0.8  →  mild motor lag  (~88 ms time constant at dt=0.07 s)

TOPICS PER ROBOT (N = 1..n_agents)
------------------------------------
  /robot_N/cmd_vel     ← geometry_msgs/Twist    (v_des, ω_des from planner)
  /robot_N/odom        → nav_msgs/Odometry      (x, y, θ, v, ω to planner)
  /robot_N/diagnostics → DiagnosticArray        (1 Hz health heartbeat)

ROS2 PARAMETERS
---------------
  n_agents  : int   — number of robots (default 4)
  alpha     : float — motor lag coefficient ∈ (0, 1]  (default 1.0)
  diag_rate : float — diagnostics publish rate Hz (default 1.0)
"""

from __future__ import annotations

import math
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Import consensus_config for initial positions and dt.
# PYTHONPATH is set to the project directory by the launch file.
try:
    from consensus_config import NetConfig as _NetCfg
    _GLOBAL_CFG = _NetCfg()
except ImportError:
    _GLOBAL_CFG = None


def _load_initial_positions(n_agents: int, cfg) -> 'np.ndarray':
    import numpy as np
    r0 = np.zeros((n_agents, 2), dtype=float)
    if cfg is not None:
        r0_cfg = cfg.initial_positions()
        n_copy = min(n_agents, r0_cfg.shape[0])
        r0[:n_copy] = r0_cfg[:n_copy]
    return r0


# ═══════════════════════════════════════════════════════════════════════════════
class UnicycleFleetNode(Node):

    def __init__(self) -> None:
        super().__init__('unicycle_fleet')

        # ── ROS2 parameters ────────────────────────────────────────────────
        self.declare_parameter('n_agents',  4)
        self.declare_parameter('alpha',     1.0)
        self.declare_parameter('diag_rate', 1.0)

        self._n     = self.get_parameter('n_agents').value
        self._alpha = float(self.get_parameter('alpha').value)
        diag_rate   = self.get_parameter('diag_rate').value

        if not (0.0 < self._alpha <= 1.0):
            self.get_logger().warn(
                f'alpha={self._alpha} outside (0,1] — clamping to 1.0'
            )
            self._alpha = 1.0

        # ── Integration timestep from consensus_config ─────────────────────
        if _GLOBAL_CFG is not None:
            self._dt = _GLOBAL_CFG.dt
        else:
            self._dt = 0.07
            self.get_logger().warn('consensus_config not found — using dt=0.07 s')

        # ── Initial positions from consensus_config ────────────────────────
        r0 = _load_initial_positions(self._n, _GLOBAL_CFG)
        self.get_logger().info(
            'Initial positions: '
            + '  '.join(
                f'R{i+1}=({r0[i,0]:.2f},{r0[i,1]:.2f})'
                for i in range(self._n)
            )
        )

        # ── Per-robot kinematic state ──────────────────────────────────────
        self._x:            Dict[int, float] = {i: float(r0[i-1, 0]) for i in range(1, self._n+1)}
        self._y:            Dict[int, float] = {i: float(r0[i-1, 1]) for i in range(1, self._n+1)}
        self._theta:        Dict[int, float] = {i: 0.0 for i in range(1, self._n+1)}
        self._v_des:        Dict[int, float] = {i: 0.0 for i in range(1, self._n+1)}
        self._omega_des:    Dict[int, float] = {i: 0.0 for i in range(1, self._n+1)}
        self._v_actual:     Dict[int, float] = {i: 0.0 for i in range(1, self._n+1)}
        self._omega_actual: Dict[int, float] = {i: 0.0 for i in range(1, self._n+1)}

        # ── Callback group ─────────────────────────────────────────────────
        self._cb = ReentrantCallbackGroup()

        # ── Per-robot publishers and subscribers ───────────────────────────
        self._odom_pubs: Dict[int, object] = {}
        self._diag_pubs: Dict[int, object] = {}

        for i in range(1, self._n+1):
            ns = f'robot_{i}'
            self.create_subscription(
                Twist, f'{ns}/cmd_vel',
                self._make_cmd_vel_callback(i), 10,
                callback_group=self._cb,
            )
            self._odom_pubs[i] = self.create_publisher(Odometry, f'{ns}/odom', 10)
            self._diag_pubs[i] = self.create_publisher(
                DiagnosticArray, f'{ns}/diagnostics', 10
            )

        # ── Diagnostics timer (NOT a simulation timer) ─────────────────────
        self.create_timer(
            1.0 / diag_rate, self._publish_diagnostics, callback_group=self._cb
        )

        # ── One-shot startup timer: bootstrap initial odom ─────────────────
        # DEADLOCK FIX:
        # The planner waits for /robot_N/odom before starting its MPC loop.
        # The fleet only publishes odom when it receives a cmd_vel.
        # Without this timer both nodes wait on each other forever.
        #
        # This timer fires once after 0.5 s (enough for DDS discovery) and
        # publishes the initial pose for every robot so the planner's
        # _odom_received flags are all set before the MPC loop begins.
        # After firing it cancels itself and never fires again.
        self._startup_timer = self.create_timer(
            0.5, self._publish_initial_odom, callback_group=self._cb
        )

        self.get_logger().info(
            f'UnicycleFleetNode ready: {self._n} robots | '
            f'dt={self._dt*1000:.1f} ms | alpha={self._alpha:.2f} | '
            f'event-driven (one cmd_vel -> one sim step)'
        )

    # ═══════════════════════════════════════════════════════════════════════
    # One-shot bootstrap
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_initial_odom(self) -> None:
        """
        Publish the initial odom for all robots exactly once at startup.
        Sets the planner's _odom_received flags so the MPC loop can begin.
        Cancels itself immediately so it never fires again.
        """
        stamp = self.get_clock().now().to_msg()
        for i in range(1, self._n + 1):
            self._publish_odom(i, stamp)
        self.get_logger().info(
            '[FLEET] Initial odom published for all robots — planner can start.'
        )
        self._startup_timer.cancel()

    # ═══════════════════════════════════════════════════════════════════════
    # cmd_vel callback — simulation trigger
    # ═══════════════════════════════════════════════════════════════════════

    def _make_cmd_vel_callback(self, robot_id: int):
        """
        One cmd_vel message → one Euler integration step → one odom publish.

        Step 1 — Motor lag filter (first-order low-pass):
            v_actual  += alpha * (v_des  - v_actual)
            omega_actual += alpha * (omega_des - omega_actual)

            Equivalent to P-controller with gain kv = alpha/dt, pole at
            (1-alpha) in [0,1) — always stable, no overshoot.

            alpha=1.0: v_actual = v_des instantly (ideal, no motor lag).
            alpha<1.0: exponential convergence with tau = dt/(alpha).

        Step 2 — Unicycle kinematics (Euler, one step of length dt=cfg.dt):
            x     += v_actual * cos(theta) * dt
            y     += v_actual * sin(theta) * dt
            theta += omega_actual * dt

        Step 3 — Publish odom immediately so the planner reads the fresh
                  post-integration pose after its time.sleep(cfg.dt).

        On the real RPi4 + ESP32 (Platform 2), Steps 1-2 are replaced by
        the actual PID running at ~100 Hz on the ESP32 via linorobot2_hardware.
        """
        def callback(msg: Twist) -> None:
            alpha = self._alpha
            dt    = self._dt

            # Step 1: motor lag filter
            v_act = (self._v_actual[robot_id]
                     + alpha * (msg.linear.x - self._v_actual[robot_id]))
            w_act = (self._omega_actual[robot_id]
                     + alpha * (msg.angular.z - self._omega_actual[robot_id]))

            self._v_des[robot_id]        = msg.linear.x
            self._omega_des[robot_id]    = msg.angular.z
            self._v_actual[robot_id]     = v_act
            self._omega_actual[robot_id] = w_act

            # Step 2: unicycle kinematics
            th = self._theta[robot_id]
            self._x[robot_id]     += v_act * math.cos(th) * dt
            self._y[robot_id]     += v_act * math.sin(th) * dt
            self._theta[robot_id] += w_act * dt
            self._theta[robot_id]  = math.atan2(
                math.sin(self._theta[robot_id]),
                math.cos(self._theta[robot_id]),
            )

            # Step 3: publish updated odom
            self._publish_odom(robot_id, self.get_clock().now().to_msg())

            self.get_logger().debug(
                f'[R{robot_id}] '
                f'v={v_act:.3f} w={w_act:.3f} '
                f'x={self._x[robot_id]:.3f} y={self._y[robot_id]:.3f} '
                f'th={math.degrees(self._theta[robot_id]):.1f} deg'
            )

        return callback

    # ═══════════════════════════════════════════════════════════════════════
    # Odometry publisher
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_odom(self, robot_id: int, stamp) -> None:
        th  = self._theta[robot_id]
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = f'robot_{robot_id}/base_link'

        msg.pose.pose.position.x = self._x[robot_id]
        msg.pose.pose.position.y = self._y[robot_id]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(th / 2.0)
        msg.pose.pose.orientation.w = math.cos(th / 2.0)

        # Report actual (lag-filtered) velocity, not the commanded value
        msg.twist.twist.linear.x  = self._v_actual[robot_id]
        msg.twist.twist.angular.z = self._omega_actual[robot_id]

        self._odom_pubs[robot_id].publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    # Diagnostics timer — 1 Hz health heartbeat
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_diagnostics(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for i in range(1, self._n + 1):
            arr = DiagnosticArray()
            arr.header.stamp = stamp

            v_err = abs(self._v_des[i] - self._v_actual[i])
            w_err = abs(self._omega_des[i] - self._omega_actual[i])
            level = (DiagnosticStatus.OK
                     if (v_err < 0.5 and w_err < 1.0)
                     else DiagnosticStatus.WARN)

            st = DiagnosticStatus()
            st.name    = f'unicycle_fleet/robot_{i}'
            st.level   = level
            st.message = 'OK' if level == DiagnosticStatus.OK else f'v_err={v_err:.2f} w_err={w_err:.2f}'
            st.values  = [
                KeyValue(key='x_m',           value=f'{self._x[i]:.4f}'),
                KeyValue(key='y_m',           value=f'{self._y[i]:.4f}'),
                KeyValue(key='theta_deg',     value=f'{math.degrees(self._theta[i]):.2f}'),
                KeyValue(key='v_des_ms',      value=f'{self._v_des[i]:.4f}'),
                KeyValue(key='v_actual_ms',   value=f'{self._v_actual[i]:.4f}'),
                KeyValue(key='v_error_ms',    value=f'{v_err:.4f}'),
                KeyValue(key='w_des_rads',    value=f'{self._omega_des[i]:.4f}'),
                KeyValue(key='w_actual_rads', value=f'{self._omega_actual[i]:.4f}'),
                KeyValue(key='w_error_rads',  value=f'{w_err:.4f}'),
            ]
            arr.status.append(st)
            self._diag_pubs[i].publish(arr)

        poses = '  '.join(
            f'R{i}=({self._x[i]:.2f},{self._y[i]:.2f},'
            f'{math.degrees(self._theta[i]):.0f}deg,'
            f'v={self._v_actual[i]:.2f})'
            for i in range(1, self._n + 1)
        )
        self.get_logger().info(f'[FLEET] {poses}')


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