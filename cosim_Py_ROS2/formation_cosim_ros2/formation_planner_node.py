"""
formation_planner_node.py
=========================
ROS2 node: MPC formation planning layer (replaces plant_node.py).

ROLE IN THE PLATFORM
--------------------
This node is the ROS2 equivalent of plant_node.py.  It orchestrates the same
distributed hybrid MPC formation control loop, but instead of integrating
single-integrator dynamics internally, it:

  1. Reads actual robot positions from /robot_N/odom  (feedback from unicycle fleet)
  2. Sends them to coordinator_node.py + controller_node.py × N via ZMQ (unchanged)
  3. Receives u_safe (Cartesian [ux, uy]) from the hybrid safety filters
  4. Converts u_safe → unicycle Twist (v, ω) via a proportional heading controller
  5. Publishes Twist to /robot_N/cmd_vel
  6. Waits cfg.dt seconds (zero-order hold — robot integrates at 20 Hz meanwhile)
  7. Logs 5 CSV streams identical to plant_node.py + unicycle_state.csv

ZMQ CONNECTIONS (identical ports to coordinator_node.py / controller_node.py)
---------------
  coordinator : tcp://127.0.0.1:5555   REQ  (plant_step requests)
  controller_1: tcp://127.0.0.1:5601   REQ  (hybrid_request)
  controller_2: tcp://127.0.0.1:5602   REQ
  controller_3: tcp://127.0.0.1:5603   REQ
  controller_4: tcp://127.0.0.1:5604   REQ

THREADING MODEL
---------------
  Main thread   : rclpy MultiThreadedExecutor → dispatches odom callbacks
  Planning thread: the full outer×inner MPC loop, sleeping cfg.dt between
                   inner steps to give the unicycle fleet time to integrate.

  The odom subscribers write to a thread-safe dict (protected by Lock).
  The planning thread reads from that dict after each sleep.

UNICYCLE HEADING CONTROLLER
----------------------------
The MPC outputs Cartesian velocity u_safe = (ux, uy).
The heading controller converts this to unicycle commands:

    θ_ref = atan2(uy, ux)
    e_θ   = wrap(θ_ref − θ)             ← heading error
    ω     = k_heading · e_θ             ← proportional heading control
    v     = k_speed · ‖u_safe‖ · max(cos e_θ, 0)  ← forward speed (no reverse)

This is identical to cosim_step.m in the MATLAB co-simulation platform.

ROS2 PARAMETERS
---------------
  n_agents        : int   — must match coordinator/controllers (default 4)
  model           : str   — 'single_integrator' or 'double_integrator'
  outer_steps     : int   — total MPC outer iterations (default 18)
  safety_enabled  : bool  — enable hybrid safety filter (default True)
  obstacles_enabled: bool — enable obstacle avoidance (default True)
  dt_inner        : float — integration timestep = cfg.dt (default 1.0 s)
  k_heading       : float — proportional heading gain (default 4.0)
  k_speed         : float — speed feedforward scale  (default 1.0)
  log_dir         : str   — output directory for CSV logs (default 'ros2_cosim_logs')
  startup_delay_s : float — wait before starting the MPC loop (default 2.0 s)
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import zmq

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ── Import from the existing MPC project ──────────────────────────────────────
# PYTHONPATH must include the project directory that contains these modules.
# See the launch file for how to set this up.
from consensus_comm import dumps, loads, make_envelope
from consensus_config import NetConfig

from formation_cosim_ros2.cosim_csv_logger import CosimCSVLogger


# ═══════════════════════════════════════════════════════════════════════════════
# Helpers (identical to plant_node.py — kept here to keep the node self-contained)
# ═══════════════════════════════════════════════════════════════════════════════

def _clip(x: np.ndarray, lo: float, hi: float) -> np.ndarray:
    return np.minimum(np.maximum(x, lo), hi)


def _pairwise_diameter(X: np.ndarray) -> float:
    if X.ndim != 2 or X.shape[0] <= 1:
        return 0.0
    dmax = 0.0
    for i in range(X.shape[0]):
        for k in range(i + 1, X.shape[0]):
            dmax = max(dmax, float(np.linalg.norm(X[i] - X[k])))
    return dmax


def _pairwise_min_distance(X: np.ndarray) -> float:
    if X.ndim != 2 or X.shape[0] <= 1:
        return float("inf")
    dmin = float("inf")
    for i in range(X.shape[0]):
        for k in range(i + 1, X.shape[0]):
            dmin = min(dmin, float(np.linalg.norm(X[i] - X[k])))
    return dmin


def _global_obstacle_distance(cfg: NetConfig, r: np.ndarray) -> float:
    if not cfg.obstacles_enabled or not cfg.obstacles_circles:
        return float("inf")
    dmin = float("inf")
    for p in r:
        for (cx, cy, rad) in cfg.obstacles_circles:
            dd = float(np.linalg.norm(p[:2] - np.array([cx, cy])) - (rad + cfg.obstacle_margin))
            dmin = min(dmin, dd)
    return dmin


def _get_keyed(d: dict, i: int, default: Any) -> Any:
    """Handles both int and str keys — coordinator may return str-keyed dicts."""
    if not isinstance(d, dict):
        return default
    if i in d:
        return d[i]
    si = str(i)
    if si in d:
        return d[si]
    return default


def _make_req_socket(ctx: zmq.Context, endpoint: str, timeout_ms: int, linger_ms: int) -> zmq.Socket:
    s = ctx.socket(zmq.REQ)
    s.setsockopt(zmq.LINGER, linger_ms)
    s.setsockopt(zmq.REQ_RELAXED, 1)
    s.setsockopt(zmq.REQ_CORRELATE, 1)
    s.RCVTIMEO = timeout_ms
    s.SNDTIMEO = timeout_ms
    s.connect(endpoint)
    return s


def _reset_socket(ctx: zmq.Context, sock: zmq.Socket, endpoint: str, timeout_ms: int, linger_ms: int) -> zmq.Socket:
    try:
        sock.close(0)
    except Exception:
        pass
    return _make_req_socket(ctx, endpoint, timeout_ms, linger_ms)


# ═══════════════════════════════════════════════════════════════════════════════
class FormationPlannerNode(Node):
    """
    MPC formation planning node — the ROS2 replacement for plant_node.py.

    Analogy with the MATLAB co-simulation
    ---------------------------------------
    MATLAB platform                      This ROS2 node
    ─────────────────────────────────    ─────────────────────────────────────
    plant_node.py (loop)             ←→  _planning_loop() thread
    MATLAB Engine API call           ←→  ZMQ REQ to coordinator_node.py
    Simulink unicycle step           ←→  time.sleep(dt_inner) + unicycle_fleet
    cosim_logger.py                  ←→  CosimCSVLogger
    cosim_bridge.py (cartesian→uni.) ←→  _cartesian_to_twist()
    """

    def __init__(self) -> None:
        super().__init__('formation_planner')

        # ── Declare ROS2 parameters ────────────────────────────────────────
        self.declare_parameter('n_agents',         4)
        self.declare_parameter('model',            'single_integrator')
        self.declare_parameter('outer_steps',      18)
        self.declare_parameter('safety_enabled',   True)
        self.declare_parameter('obstacles_enabled', True)
        self.declare_parameter('dt_inner',         1.0)
        self.declare_parameter('k_heading',        4.0)
        self.declare_parameter('k_speed',          1.0)
        self.declare_parameter('log_dir',          'ros2_cosim_logs')
        self.declare_parameter('startup_delay_s',  2.0)

        n_agents    = self.get_parameter('n_agents').value
        model       = self.get_parameter('model').value
        outer_steps = self.get_parameter('outer_steps').value
        safety_en   = self.get_parameter('safety_enabled').value
        obs_en      = self.get_parameter('obstacles_enabled').value
        log_dir     = self.get_parameter('log_dir').value
        startup_s   = self.get_parameter('startup_delay_s').value

        self._dt_inner  = self.get_parameter('dt_inner').value
        self._k_heading = self.get_parameter('k_heading').value
        self._k_speed   = self.get_parameter('k_speed').value

        # ── Build NetConfig from parameters ───────────────────────────────
        # NetConfig is frozen so we reconstruct it from the default with overrides.
        from dataclasses import replace
        base_cfg = NetConfig()
        self._cfg = replace(
            base_cfg,
            n_agents=n_agents,
            model=model,
            outer_steps=outer_steps,
            safety_enabled=safety_en,
            obstacles_enabled=obs_en,
        )
        self._M = self._cfg.horizon_M()
        self._n = n_agents

        self.get_logger().info(
            f'FormationPlannerNode: n={self._n} | model={model} | '
            f'M={self._M} | outer_steps={outer_steps} | '
            f'safety={safety_en} | obstacles={obs_en} | dt_inner={self._dt_inner}s'
        )

        # ── Thread-safe odom state ─────────────────────────────────────────
        # Each entry: {'x': float, 'y': float, 'theta': float,
        #              'vx': float, 'vy': float}
        # Written by odom callbacks (executor thread pool).
        # Read by planning loop thread.
        self._odom_lock  = threading.Lock()
        self._odom_state: Dict[int, Dict[str, float]] = {
            i: {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'vx': 0.0, 'vy': 0.0}
            for i in range(1, self._n + 1)
        }
        # Track whether we have received at least one odom message per robot
        self._odom_received: Dict[int, bool] = {i: False for i in range(1, self._n + 1)}

        # ── ROS2 subscribers (odom) + publishers (cmd_vel) ─────────────────
        self._cb_group = ReentrantCallbackGroup()
        self._cmd_vel_pubs: Dict[int, object] = {}

        for i in range(1, self._n + 1):
            ns = f'robot_{i}'
            self.create_subscription(
                Odometry,
                f'{ns}/odom',
                self._make_odom_callback(i),
                10,
                callback_group=self._cb_group,
            )
            self._cmd_vel_pubs[i] = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)

        # ── ZMQ context and sockets ────────────────────────────────────────
        # Identical setup to plant_node.py.
        self._zmq_ctx = zmq.Context.instance()

        self._coord_sock = _make_req_socket(
            self._zmq_ctx,
            self._cfg.plant_to_coord_rep,
            self._cfg.req_timeout_ms,
            self._cfg.req_linger_ms,
        )
        self.get_logger().info(
            f'ZMQ REQ → coordinator at {self._cfg.plant_to_coord_rep}'
        )

        self._hybrid_socks: Dict[int, zmq.Socket] = {}
        self._hybrid_eps:   Dict[int, str]        = {}
        for i in range(1, self._n + 1):
            ep = self._cfg.controller_endpoint(i)
            self._hybrid_eps[i]  = ep
            self._hybrid_socks[i] = _make_req_socket(
                self._zmq_ctx, ep,
                self._cfg.hybrid_timeout_ms,
                self._cfg.req_linger_ms,
            )
            self.get_logger().info(f'ZMQ REQ → controller_{i} hybrid at {ep}')

        # ── CSV logger ─────────────────────────────────────────────────────
        self._csv = CosimCSVLogger(log_dir=log_dir)
        self.get_logger().info(f'CSV logs → {log_dir}/')

        # ── Start the planning loop in a daemon thread ─────────────────────
        # A daemon thread exits automatically when the main process exits,
        # so Ctrl+C in the terminal will cleanly terminate the loop.
        self._plan_thread = threading.Thread(
            target=self._planning_loop,
            args=(startup_s,),
            daemon=True,
            name='formation_mpc_loop',
        )
        self._plan_thread.start()

    # ═══════════════════════════════════════════════════════════════════════
    # Odom subscriber callbacks
    # ═══════════════════════════════════════════════════════════════════════

    def _make_odom_callback(self, robot_id: int):
        """
        Returns a closure that writes Odometry data into the thread-safe dict.

        Cartesian velocity in world frame:
            vx_world = v_forward * cos(θ)
            vy_world = v_forward * sin(θ)
        where v_forward is the body-frame linear.x from the Odometry message.
        """
        def callback(msg: Odometry) -> None:
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            theta = 2.0 * math.atan2(qz, qw)

            v_fwd = msg.twist.twist.linear.x
            vx_world = v_fwd * math.cos(theta)
            vy_world = v_fwd * math.sin(theta)

            with self._odom_lock:
                self._odom_state[robot_id] = {
                    'x':     msg.pose.pose.position.x,
                    'y':     msg.pose.pose.position.y,
                    'theta': theta,
                    'vx':    vx_world,
                    'vy':    vy_world,
                }
                self._odom_received[robot_id] = True

        return callback

    # ═══════════════════════════════════════════════════════════════════════
    # Unicycle heading controller
    # ═══════════════════════════════════════════════════════════════════════

    def _cartesian_to_twist(
        self, ux: float, uy: float, theta: float
    ) -> Tuple[float, float]:
        """
        Convert Cartesian MPC velocity command (ux, uy) to unicycle (v, ω).

        Implements the same heading controller as cosim_step.m:

            θ_ref = atan2(uy, ux)
            e_θ   = wrap(θ_ref − θ)
            ω     = k_heading · e_θ
            v     = k_speed · ‖u‖ · max(cos(e_θ), 0)

        The max(cos(e_θ), 0) term prevents backward driving: the robot waits
        for its heading to align before accelerating.

        PHYSICAL INTERPRETATION
        -----------------------
        If e_θ is large (robot pointing the wrong way):
          • ω is large → robot rotates quickly toward target heading
          • v ≈ 0     → robot doesn't drive forward while turning

        If e_θ ≈ 0 (robot aligned with MPC command):
          • ω ≈ 0     → no rotation
          • v = k_speed · ‖u‖  → full forward speed
        """
        u_mag = math.sqrt(ux * ux + uy * uy)
        if u_mag < 1e-6:
            return 0.0, 0.0  # zero command → stop

        theta_ref = math.atan2(uy, ux)
        e_theta   = math.atan2(
            math.sin(theta_ref - theta),
            math.cos(theta_ref - theta),
        )

        omega = self._k_heading * e_theta
        v     = self._k_speed * u_mag * max(math.cos(e_theta), 0.0)
        return v, omega

    # ═══════════════════════════════════════════════════════════════════════
    # ZMQ communication — coordinator (identical to plant_node.py)
    # ═══════════════════════════════════════════════════════════════════════

    def _zmq_coordinator_step(
        self, outer_j: int, r: np.ndarray, v: np.ndarray
    ) -> Tuple[Optional[np.ndarray], dict]:
        """
        Send plant_step to coordinator_node.py → receive U_nom + diagnostics.
        Returns (U_nom array shape (M, n, dim), diag_dict) or (zeros, {}) on failure.
        """
        cfg = self._cfg
        M   = self._M
        try:
            self._coord_sock.send(
                dumps(make_envelope(
                    "plant_step",
                    {"outer_index": outer_j, "r_all": r.tolist(), "v_all": v.tolist()},
                    src="planner_ros2",
                    dst="coord",
                ))
            )
            reply = loads(self._coord_sock.recv())["payload"]
        except Exception as e:
            self.get_logger().warn(
                f'[PLAN] outer_j={outer_j}: coordinator ZMQ error: {e} — using zeros'
            )
            return np.zeros((M, self._n, cfg.dim), dtype=float), {}

        U_nom = np.array(
            reply.get("U_seq", np.zeros((M, self._n, cfg.dim))), dtype=float
        )
        diag  = reply.get("diag", {}) if isinstance(reply, dict) else {}
        return U_nom, diag

    # ═══════════════════════════════════════════════════════════════════════
    # ZMQ communication — hybrid safety filter (identical to plant_node.py)
    # ═══════════════════════════════════════════════════════════════════════

    def _zmq_hybrid_step(
        self,
        agent_id: int,
        r: np.ndarray,
        v: np.ndarray,
        u_nom_i: np.ndarray,
        bary_i: List[float],
    ) -> Tuple[np.ndarray, dict]:
        """
        Send hybrid_request to controller_node_{agent_id} → receive u_safe + mode diag.
        Returns (u_safe array shape (dim,), diag dict) or (u_nom_i, fallback_diag).
        """
        payload = {
            "agent_id": agent_id,
            "r_all":    r.tolist(),
            "v_all":    v.tolist(),
            "u_nom":    u_nom_i.tolist(),
            "bary_r":   bary_i,
        }
        try:
            self._hybrid_socks[agent_id].send(
                dumps(make_envelope(
                    "hybrid_request", payload,
                    src="planner_ros2",
                    dst=f"controller_node{agent_id}",
                ))
            )
            rep = loads(self._hybrid_socks[agent_id].recv())["payload"]
            u_safe = np.array(rep["u_safe"], dtype=float)
            diag   = rep.get("diag", {})
            return u_safe, diag
        except Exception as e:
            self.get_logger().warn(
                f'[PLAN] hybrid ZMQ error agent {agent_id}: {e} — passing u_nom through'
            )
            self._hybrid_socks[agent_id] = _reset_socket(
                self._zmq_ctx,
                self._hybrid_socks[agent_id],
                self._hybrid_eps[agent_id],
                self._cfg.hybrid_timeout_ms,
                self._cfg.req_linger_ms,
            )
            fallback_diag = {
                "mode": "F", "desired_mode": "F", "effective_mode": "F",
                "d_agent_min": float("nan"), "d_obs_min": float("nan"),
                "active_pairs": 0, "active_obstacles": 0, "obstructed": False,
                "target_waypoint": [float("nan"), float("nan")],
                "_h_pair_num": 0, "_circle_barrier_num": 0,
                "formation_hold_active": False,
            }
            return u_nom_i.copy(), fallback_diag

    # ═══════════════════════════════════════════════════════════════════════
    # Read current odom state (thread-safe snapshot)
    # ═══════════════════════════════════════════════════════════════════════

    def _read_odom(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Return (r, v, theta) from the latest odom messages.

        r     : (n, 2) float  — world-frame positions
        v     : (n, 2) float  — world-frame velocities (zeros for SI)
        theta : (n,)   float  — heading angles [rad]
        """
        r     = np.zeros((self._n, self._cfg.dim), dtype=float)
        v_vec = np.zeros((self._n, self._cfg.dim), dtype=float)
        theta = np.zeros(self._n, dtype=float)

        with self._odom_lock:
            for i in range(1, self._n + 1):
                s = self._odom_state[i]
                r[i - 1, 0]     = s['x']
                r[i - 1, 1]     = s['y']
                v_vec[i - 1, 0] = s['vx']
                v_vec[i - 1, 1] = s['vy']
                theta[i - 1]    = s['theta']

        if self._cfg.model == 'single_integrator':
            v_vec = np.zeros_like(v_vec)

        return r, v_vec, theta

    # ═══════════════════════════════════════════════════════════════════════
    # Main planning loop (runs in a dedicated thread)
    # ═══════════════════════════════════════════════════════════════════════

    def _planning_loop(self, startup_delay: float) -> None:
        """
        The full outer × inner MPC control loop.

        Structure identical to plant_node.py's main() loop, but:
          • dynamics feedback comes from /robot_N/odom via ROS2
          • output goes to /robot_N/cmd_vel via ROS2
          • time.sleep(dt_inner) replaces the algebraic single-integrator update
          • CSV logging includes the new unicycle_state.csv stream
        """
        cfg = self._cfg
        M   = self._M

        # Wait for startup (fleet needs time to initialise)
        self.get_logger().info(f'[PLAN] Waiting {startup_delay}s for fleet to start…')
        time.sleep(startup_delay)

        # Wait until at least one odom message received per robot
        wait_start = time.time()
        while True:
            with self._odom_lock:
                all_ready = all(self._odom_received[i] for i in range(1, self._n + 1))
            if all_ready:
                break
            if time.time() - wait_start > 30.0:
                self.get_logger().error(
                    '[PLAN] Timed out waiting for odom from all robots. '
                    'Is unicycle_fleet_node running?'
                )
                return
            time.sleep(0.1)

        self.get_logger().info('[PLAN] All robots publishing odom. Starting MPC loop.')

        k_global = 0

        for outer_j in range(cfg.outer_steps):

            # ── 1. Read current state from odom ───────────────────────────
            r, v, theta = self._read_odom()

            # ── 2. MPC solve: send plant_step to coordinator ───────────────
            U_nom, diag = self._zmq_coordinator_step(outer_j, r, v)

            # ── 3. Log outer-step MPC diagnostics ─────────────────────────
            ok_coord = isinstance(diag, dict) and bool(diag)
            if ok_coord:
                bary_dict = diag.get("bary_r", {})
                for i in range(1, self._n + 1):
                    d_i = {
                        key: _get_keyed(diag.get(key, {}), i, np.nan)
                        for key in [
                            "objective_primary", "lex_used", "phi_terminal",
                            "ri_terminal", "diam_C",
                            "t_primary_ms", "t_lex_ms", "t_total_ms",
                            "n_var_primary", "n_eq_primary", "n_ineq_primary",
                            "n_var_lex", "n_eq_lex", "n_ineq_lex",
                            "nominal_fallback_used", "nominal_status",
                        ]
                    }
                    rt  = _get_keyed(diag.get("r_term",  {}), i, [np.nan, np.nan])
                    bry = _get_keyed(bary_dict,              i, [r[i-1,0], r[i-1,1]])
                    if not (isinstance(rt,  (list, tuple)) and len(rt)  >= 2): rt  = [np.nan, np.nan]
                    if not (isinstance(bry, (list, tuple)) and len(bry) >= 2): bry = [r[i-1,0], r[i-1,1]]
                    d_i["r_term"] = rt
                    d_i["bary_r"] = bry
                    self._csv.log_outer(outer_j, i, cfg.model, M, d_i)
            else:
                self._csv.log_outer_fail(outer_j, self._n, cfg.model, M, "coord_fail")
                bary_dict = {}

            # ── 4. Inner loop over MPC horizon ────────────────────────────
            Vr = Vv = dmin_agents = dmin_obstacles = 0.0  # defined for final print

            for inner_t in range(M):
                u_nom_now = U_nom[inner_t, :, :]   # shape (n, dim)

                U_hybrid        = np.zeros_like(u_nom_now)
                count_modes     = {m: 0 for m in ["F", "C", "O", "CO", "T"]}
                h_pair_total    = 0
                circle_total    = 0

                # ── 4a. Hybrid safety filter + Twist publish ───────────────
                for i in range(1, self._n + 1):
                    bary_i = _get_keyed(bary_dict, i, [r[i-1, 0], r[i-1, 1]])
                    if not (isinstance(bary_i, (list, tuple)) and len(bary_i) >= 2):
                        bary_i = [r[i-1, 0], r[i-1, 1]]

                    u_safe_i, mode_diag = self._zmq_hybrid_step(
                        i, r, v, u_nom_now[i-1], bary_i
                    )
                    U_hybrid[i - 1] = u_safe_i

                    # Convert Cartesian u_safe → unicycle Twist
                    v_cmd, omega_cmd = self._cartesian_to_twist(
                        float(u_safe_i[0]), float(u_safe_i[1]), float(theta[i - 1])
                    )

                    # Publish Twist to /robot_i/cmd_vel
                    twist_msg          = Twist()
                    twist_msg.linear.x = v_cmd
                    twist_msg.angular.z = omega_cmd
                    self._cmd_vel_pubs[i].publish(twist_msg)

                    # Accumulate mode counts and barrier totals
                    mode = str(mode_diag.get("mode", "F"))
                    count_modes[mode] = count_modes.get(mode, 0) + 1
                    h_pair_total += int(mode_diag.get("_h_pair_num", 0))
                    circle_total += int(mode_diag.get("_circle_barrier_num", 0))

                    # ── CSV: hybrid mode ───────────────────────────────────
                    self._csv.log_hybrid_mode(k_global, outer_j, inner_t, i, mode_diag)

                    # ── CSV: unicycle state ────────────────────────────────
                    with self._odom_lock:
                        s = self._odom_state[i]
                    self._csv.log_unicycle(
                        k_global, outer_j, inner_t, i,
                        s['x'], s['y'], s['theta'],
                        v_cmd, omega_cmd,
                        float(u_safe_i[0]), float(u_safe_i[1]),
                    )

                # ── 4b. Zero-order hold: wait dt_inner for the fleet ──────
                # During this sleep the MultiThreadedExecutor continues
                # dispatching odom callbacks, keeping _odom_state current.
                time.sleep(self._dt_inner)

                # ── 4c. Read updated robot positions after integration ─────
                r, v, theta = self._read_odom()

                # ── 4d. Clip u (matches plant_node.py) ────────────────────
                u = _clip(U_hybrid, cfg.u_min, cfg.u_max)

                # ── 4e. Safety metrics ─────────────────────────────────────
                Vr   = _pairwise_diameter(r)
                Vv   = _pairwise_diameter(v)
                speeds     = np.linalg.norm(v, axis=1)
                max_speed  = float(np.max(speeds))  if speeds.size else 0.0
                mean_speed = float(np.mean(speeds)) if speeds.size else 0.0
                dmin_agents    = _pairwise_min_distance(r)
                dmin_obstacles = _global_obstacle_distance(cfg, r)

                # ── 4f. CSV: trajectory ────────────────────────────────────
                for i in range(self._n):
                    vi_log = [v[i, 0], v[i, 1]] if cfg.model == 'double_integrator' else [0.0, 0.0]
                    self._csv.log_trajectory(
                        k_global, outer_j, inner_t, i + 1,
                        [r[i, 0], r[i, 1]], vi_log, [u[i, 0], u[i, 1]],
                    )

                # ── 4g. CSV: metrics ───────────────────────────────────────
                self._csv.log_metrics(
                    k_global, outer_j, inner_t,
                    Vr, Vv, max_speed, mean_speed,
                    dmin_agents, dmin_obstacles,
                    count_modes, h_pair_total, circle_total,
                )

                # Flush every inner step so the CSV is readable during a run
                if k_global % 5 == 0:
                    self._csv.flush()

                k_global += 1

            self.get_logger().info(
                f'[PLAN] outer_j={outer_j + 1}/{cfg.outer_steps} | '
                f'Vr={Vr:.3f} | dmin_agents={dmin_agents:.3f} | '
                f'dmin_obs={dmin_obstacles:.3f}'
            )

        # ── 5. Formation complete — stop all robots ────────────────────────
        self.get_logger().info('[PLAN] All outer steps complete. Stopping fleet…')
        for i in range(1, self._n + 1):
            stop = Twist()  # all zeros
            self._cmd_vel_pubs[i].publish(stop)

        # ── 6. ZMQ shutdown ────────────────────────────────────────────────
        try:
            self._coord_sock.send(
                dumps(make_envelope("shutdown", {}, src="planner_ros2", dst="coord"))
            )
            _ = loads(self._coord_sock.recv())
            self.get_logger().info('[PLAN] Coordinator shutdown acknowledged.')
        except Exception:
            self.get_logger().warn('[PLAN] Coordinator shutdown: no response (ok to ignore).')

        # ── 7. Close CSV logs ──────────────────────────────────────────────
        self._csv.close()
        self.get_logger().info(f'[PLAN] CSV logs written to {self.get_parameter("log_dir").value}/')


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormationPlannerNode()

    # MultiThreadedExecutor runs odom callbacks in parallel with the planning thread.
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C — shutting down FormationPlannerNode')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
