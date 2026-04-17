"""
robot_sim_node.py
=================
ROS2 node: simulates ONE differential-drive robot.

ROLE IN PLATFORM 2
-------------------
This node is the per-robot building block for Co-Simulation Platform 2.
It simulates exactly what ONE linorobot2 instance exposes over ROS2:

  TOPIC                            DIRECTION    TYPE
  /robot_{id}/cmd_vel              ← subscribe  geometry_msgs/Twist
  /robot_{id}/odom                 → publish    nav_msgs/Odometry
  /robot_{id}/diagnostics          → publish    diagnostic_msgs/DiagnosticArray

MIGRATION PATH TO REAL HARDWARE
---------------------------------
When a physical robot is ready (RPi4 + ESP32 + linorobot2), this node is
simply stopped and replaced with:

    ros2 launch linorobot2_bringup bringup.launch.py \
        robot_base:=2wd \
        joy:=false

running on the RPi4 in the same namespace.  The formation_planner_node on
the VM publishes to the exact same topic names and requires zero changes.

ONE-RPi4 DEPLOYMENT (now):
  Run N instances of this node on one RPi4, each with a different robot_id.
  The platform2_rpi4_fleet_launch.py does this automatically.

FOUR-RPi4 DEPLOYMENT (future):
  Run one instance per RPi4 using platform2_rpi4_single_launch.py.
  Each RPi4 SSHs in and: ros2 launch formation_cosim_ros2
  platform2_rpi4_single_launch.py robot_id:=<1|2|3|4>

ROS2 PARAMETERS
---------------
  robot_id      : int   — 1-based robot index (default 1)
  alpha         : float — motor lag ∈ (0,1]: 1.0=perfect, 0.8=mild lag
  init_x        : float — initial x position [m]  (overrides consensus_config)
  init_y        : float — initial y position [m]
  init_theta    : float — initial heading [rad]
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    from consensus_config import NetConfig as _NetCfg
    _GLOBAL_CFG = _NetCfg()
except ImportError:
    _GLOBAL_CFG = None


class RobotSimNode(Node):
    """
    Simulates one differential-drive robot.

    Identical dynamics model to UnicycleFleetNode, but scoped to a single
    robot so that N instances can be spread across N separate RPi4s.

    The topic structure mirrors linorobot2 exactly so that swapping this
    node for a real linorobot2 bringup requires zero changes in the planner.
    """

    def __init__(self) -> None:
        super().__init__('robot_sim')  # will be remapped by launch namespace

        # ── ROS2 parameters ────────────────────────────────────────────────
        self.declare_parameter('robot_id',    1)
        self.declare_parameter('alpha',       1.0)
        self.declare_parameter('init_x',      float('nan'))  # nan = use consensus_config
        self.declare_parameter('init_y',      float('nan'))
        self.declare_parameter('init_theta',  0.0)

        self._robot_id = self.get_parameter('robot_id').value
        self._alpha    = float(self.get_parameter('alpha').value)
        init_x         = self.get_parameter('init_x').value
        init_y         = self.get_parameter('init_y').value

        # Clamp alpha
        if not (0.0 < self._alpha <= 1.0):
            self.get_logger().warn(f'alpha={self._alpha} clamped to 1.0')
            self._alpha = 1.0

        # ── Timestep from consensus_config ─────────────────────────────────
        if _GLOBAL_CFG is not None:
            self._dt = _GLOBAL_CFG.dt
        else:
            self._dt = 0.07
            self.get_logger().warn('consensus_config not found — dt=0.07 s')

        # ── Initial position ───────────────────────────────────────────────
        # Priority: explicit ROS2 parameters > consensus_config > (0,0)
        if math.isnan(init_x) and _GLOBAL_CFG is not None:
            r0 = _GLOBAL_CFG.initial_positions()
            idx = self._robot_id - 1
            if idx < r0.shape[0]:
                init_x = float(r0[idx, 0])
                init_y = float(r0[idx, 1])
            else:
                init_x = init_y = 0.0
        elif math.isnan(init_x):
            init_x = init_y = 0.0

        # ── Robot state ────────────────────────────────────────────────────
        self._x     = init_x
        self._y     = init_y
        self._theta = self.get_parameter('init_theta').value

        self._v_des        = 0.0
        self._omega_des    = 0.0
        self._v_actual     = 0.0
        self._omega_actual = 0.0

        # ── Topic names — mirroring linorobot2 namespace convention ────────
        ns = f'robot_{self._robot_id}'
        self._cb = ReentrantCallbackGroup()

        self.create_subscription(
            Twist, f'{ns}/cmd_vel',
            self._cmd_vel_callback, 10,
            callback_group=self._cb,
        )
        self._odom_pub = self.create_publisher(Odometry, f'{ns}/odom', 10)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, f'{ns}/diagnostics', 10
        )

        # ── Diagnostics timer (1 Hz heartbeat only) ────────────────────────
        self.create_timer(1.0, self._publish_diagnostics, callback_group=self._cb)

        # ── Bootstrap: publish initial odom once after DDS discovery ───────
        # Breaks the planner/fleet deadlock (same logic as unicycle_fleet_node).
        self._startup_timer = self.create_timer(
            0.5, self._publish_initial_odom, callback_group=self._cb
        )

        self.get_logger().info(
            f'RobotSimNode robot_{self._robot_id} | '
            f'pos=({self._x:.2f},{self._y:.2f}) | '
            f'dt={self._dt*1000:.1f}ms | alpha={self._alpha:.2f}'
        )

    # ═══════════════════════════════════════════════════════════════════════
    # Bootstrap
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_initial_odom(self) -> None:
        """Publish initial pose once so the planner's _odom_received flag is set."""
        self._publish_odom(self.get_clock().now().to_msg())
        self.get_logger().info(
            f'[R{self._robot_id}] Initial odom published — planner can start.'
        )
        self._startup_timer.cancel()

    # ═══════════════════════════════════════════════════════════════════════
    # cmd_vel callback — simulation trigger (event-driven, not timer-driven)
    # ═══════════════════════════════════════════════════════════════════════

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        One cmd_vel → motor lag → Euler step → odom publish.

        This is the exact same logic as UnicycleFleetNode._make_cmd_vel_callback.
        On real hardware this callback is never needed — linorobot2's
        diff_drive_controller handles the cmd_vel → wheel PWM → odom pipeline.

        Motor lag model:
            v_actual  += alpha * (v_des  - v_actual)   [first-order lag]
            ω_actual  += alpha * (ω_des  - ω_actual)

        Unicycle kinematics (Euler, dt = cfg.dt):
            x     += v_actual * cos(θ) * dt
            y     += v_actual * sin(θ) * dt
            θ     += ω_actual * dt
        """
        alpha = self._alpha
        dt    = self._dt

        # Motor lag
        self._v_actual     += alpha * (msg.linear.x  - self._v_actual)
        self._omega_actual += alpha * (msg.angular.z - self._omega_actual)
        self._v_des         = msg.linear.x
        self._omega_des     = msg.angular.z

        # Unicycle kinematics
        self._x     += self._v_actual * math.cos(self._theta) * dt
        self._y     += self._v_actual * math.sin(self._theta) * dt
        self._theta += self._omega_actual * dt
        self._theta  = math.atan2(math.sin(self._theta), math.cos(self._theta))

        # Publish immediately so planner reads fresh pose after its sleep
        self._publish_odom(self.get_clock().now().to_msg())

    # ═══════════════════════════════════════════════════════════════════════
    # Odometry publisher
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_odom(self, stamp) -> None:
        """
        Publish nav_msgs/Odometry.

        Frame convention matches linorobot2:
          header.frame_id = 'odom'
          child_frame_id  = 'base_footprint'  (linorobot2 default)
        """
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_footprint'   # linorobot2 convention

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._theta / 2.0)

        msg.twist.twist.linear.x  = self._v_actual
        msg.twist.twist.angular.z = self._omega_actual

        self._odom_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════════════
    # Diagnostics
    # ═══════════════════════════════════════════════════════════════════════

    def _publish_diagnostics(self) -> None:
        v_err = abs(self._v_des - self._v_actual)
        w_err = abs(self._omega_des - self._omega_actual)
        level = DiagnosticStatus.OK if (v_err < 0.5 and w_err < 1.0) \
                else DiagnosticStatus.WARN

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        st = DiagnosticStatus()
        st.name    = f'robot_sim/robot_{self._robot_id}'
        st.level   = level
        st.message = 'OK' if level == DiagnosticStatus.OK \
                     else f'v_err={v_err:.2f} w_err={w_err:.2f}'
        st.values  = [
            KeyValue(key='x_m',           value=f'{self._x:.4f}'),
            KeyValue(key='y_m',           value=f'{self._y:.4f}'),
            KeyValue(key='theta_deg',     value=f'{math.degrees(self._theta):.2f}'),
            KeyValue(key='v_des_ms',      value=f'{self._v_des:.4f}'),
            KeyValue(key='v_actual_ms',   value=f'{self._v_actual:.4f}'),
            KeyValue(key='w_des_rads',    value=f'{self._omega_des:.4f}'),
            KeyValue(key='w_actual_rads', value=f'{self._omega_actual:.4f}'),
        ]
        arr.status.append(st)
        self._diag_pub.publish(arr)

        self.get_logger().info(
            f'[R{self._robot_id}] '
            f'({self._x:.2f},{self._y:.2f},{math.degrees(self._theta):.0f}deg) '
            f'v={self._v_actual:.2f} w={self._omega_actual:.2f}'
        )


# ═══════════════════════════════════════════════════════════════════════════════
def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotSimNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Ctrl+C — shutting down RobotSimNode robot_{node._robot_id}'
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
