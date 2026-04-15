"""
cosim_csv_logger.py
===================
Structured CSV logger for the ROS2 Co-Simulation Platform 1.

STREAM DESIGN
-------------
This logger reproduces the EXACT same 4 CSV files that plant_node.py writes,
so every existing analysis script, plot generator, or post-processor you have
for the pure-Python simulation works identically on the ROS2 co-simulation data.

  1. consensus_traj.csv    — per-agent state (r, v, u) at every inner step
  2. consensus_metrics.csv — per-step safety metrics (dmin_agents, dmin_obs, modes)
  3. consensus_outer.csv   — per-outer-step MPC diagnostics (solve times, Φ, etc.)
  4. hybrid_modes.csv      — per-agent hybrid controller mode at every inner step

PLUS one new stream specific to the unicycle model:

  5. unicycle_state.csv    — (x, y, θ, v_cmd, ω_cmd, ux_mpc, uy_mpc) per agent
                             Lets you verify how well the unicycle tracks the
                             Cartesian MPC commands and measure heading error.

SAFETY VERIFICATION
-------------------
The safety objectives from your paper can be verified directly from these logs:

  Objective Avoidance (Obstacle):
      consensus_metrics.csv → column 'dmin_obstacles'
      Invariant: dmin_obstacles > 0 at every row (agent stays outside obstacle+margin)

  Inter-Agent Collision Avoidance:
      consensus_metrics.csv → column 'dmin_agents'
      Invariant: dmin_agents > d_safe at every row

  Hybrid mode activations:
      hybrid_modes.csv → columns mode, desired_mode, effective_mode
      n_C / n_O / n_CO in consensus_metrics.csv show when safety filters fire
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Dict, List, Optional


class CosimCSVLogger:

    # ── Column headers (identical to plant_node.py where applicable) ────────

    TRAJ_HEADER = [
        "k_global", "outer_j", "inner_t", "agent",
        "r0", "r1", "v0", "v1", "u0", "u1",
    ]
    METRICS_HEADER = [
        "k_global", "outer_j", "inner_t",
        "Vr", "Vv", "max_speed", "mean_speed",
        "dmin_agents", "dmin_obstacles",
        "n_F", "n_C", "n_O", "n_CO", "n_T",
        "_h_pair_num", "_circle_barrier_num",
    ]
    OUTER_HEADER = [
        "outer_j", "agent", "model", "M",
        "objective_primary", "lex_used",
        "phi_terminal", "ri_terminal",
        "r_term0", "r_term1", "diam_C",
        "t_primary_ms", "t_lex_ms", "t_total_ms",
        "n_var_primary", "n_eq_primary", "n_ineq_primary",
        "n_var_lex", "n_eq_lex", "n_ineq_lex",
        "nominal_fallback_used", "nominal_status",
        "bary_r0", "bary_r1",
    ]
    MODES_HEADER = [
        "k_global", "outer_j", "inner_t", "agent",
        "mode", "desired_mode", "effective_mode",
        "d_agent_min", "d_obs_min",
        "active_pairs", "active_obstacles", "obstructed",
        "waypoint0", "waypoint1",
        "_h_pair_num", "_circle_barrier_num", "formation_hold_active",
    ]
    # New stream — unicycle-specific, no equivalent in plant_node.py
    UNICYCLE_HEADER = [
        "k_global", "outer_j", "inner_t", "agent",
        "x_m", "y_m", "theta_deg",       # actual unicycle pose
        "v_cmd_ms", "omega_cmd_rads",     # Twist command sent to robot
        "ux_mpc_ms", "uy_mpc_ms",         # Cartesian MPC command (u_safe)
        "heading_error_deg",              # atan2(uy,ux) - theta  [tracks MPC tracking quality]
    ]

    def __init__(self, log_dir: str = "ros2_cosim_logs") -> None:
        self._dir = Path(log_dir)
        self._dir.mkdir(parents=True, exist_ok=True)

        self._f_traj  = (self._dir / "consensus_traj.csv").open("w", newline="")
        self._f_met   = (self._dir / "consensus_metrics.csv").open("w", newline="")
        self._f_out   = (self._dir / "consensus_outer.csv").open("w", newline="")
        self._f_modes = (self._dir / "hybrid_modes.csv").open("w", newline="")
        self._f_uni   = (self._dir / "unicycle_state.csv").open("w", newline="")

        self._w_traj  = csv.writer(self._f_traj)
        self._w_met   = csv.writer(self._f_met)
        self._w_out   = csv.writer(self._f_out)
        self._w_modes = csv.writer(self._f_modes)
        self._w_uni   = csv.writer(self._f_uni)

        self._w_traj.writerow(self.TRAJ_HEADER)
        self._w_met.writerow(self.METRICS_HEADER)
        self._w_out.writerow(self.OUTER_HEADER)
        self._w_modes.writerow(self.MODES_HEADER)
        self._w_uni.writerow(self.UNICYCLE_HEADER)

    # ═══════════════════════════════════════════════════════════════════════
    # Per-inner-step writers
    # ═══════════════════════════════════════════════════════════════════════

    def log_trajectory(
        self,
        k_global: int, outer_j: int, inner_t: int, agent_1based: int,
        r: List[float], v: List[float], u: List[float],
    ) -> None:
        self._w_traj.writerow([
            k_global, outer_j, inner_t, agent_1based,
            r[0], r[1], v[0], v[1], u[0], u[1],
        ])

    def log_metrics(
        self,
        k_global: int, outer_j: int, inner_t: int,
        Vr: float, Vv: float, max_speed: float, mean_speed: float,
        dmin_agents: float, dmin_obstacles: float,
        count_modes: Dict[str, int],
        h_pair_total: int, circle_barrier_total: int,
    ) -> None:
        self._w_met.writerow([
            k_global, outer_j, inner_t,
            Vr, Vv, max_speed, mean_speed,
            dmin_agents, dmin_obstacles,
            count_modes.get("F", 0),
            count_modes.get("C", 0),
            count_modes.get("O", 0),
            count_modes.get("CO", 0),
            count_modes.get("T", 0),
            h_pair_total, circle_barrier_total,
        ])

    def log_hybrid_mode(
        self,
        k_global: int, outer_j: int, inner_t: int, agent_1based: int,
        diag: Dict[str, Any],
    ) -> None:
        wp = diag.get("target_waypoint", [float("nan"), float("nan")])
        if not (isinstance(wp, (list, tuple)) and len(wp) >= 2):
            wp = [float("nan"), float("nan")]
        self._w_modes.writerow([
            k_global, outer_j, inner_t, agent_1based,
            diag.get("mode", "F"),
            diag.get("desired_mode", "F"),
            diag.get("effective_mode", "F"),
            diag.get("d_agent_min", float("nan")),
            diag.get("d_obs_min", float("nan")),
            diag.get("active_pairs", 0),
            diag.get("active_obstacles", 0),
            int(bool(diag.get("obstructed", False))),
            float(wp[0]), float(wp[1]),
            int(diag.get("_h_pair_num", 0)),
            int(diag.get("_circle_barrier_num", 0)),
            int(bool(diag.get("formation_hold_active", False))),
        ])

    def log_unicycle(
        self,
        k_global: int, outer_j: int, inner_t: int, agent_1based: int,
        x: float, y: float, theta_rad: float,
        v_cmd: float, omega_cmd: float,
        ux_mpc: float, uy_mpc: float,
    ) -> None:
        # Heading error: angle between desired MPC direction and actual heading
        theta_ref = math.atan2(uy_mpc, ux_mpc) if (abs(ux_mpc) > 1e-9 or abs(uy_mpc) > 1e-9) else theta_rad
        e_theta = math.atan2(math.sin(theta_ref - theta_rad), math.cos(theta_ref - theta_rad))
        self._w_uni.writerow([
            k_global, outer_j, inner_t, agent_1based,
            round(x, 6), round(y, 6), round(math.degrees(theta_rad), 4),
            round(v_cmd, 6), round(omega_cmd, 6),
            round(ux_mpc, 6), round(uy_mpc, 6),
            round(math.degrees(e_theta), 4),
        ])

    # ═══════════════════════════════════════════════════════════════════════
    # Per-outer-step writer
    # ═══════════════════════════════════════════════════════════════════════

    def log_outer(
        self,
        outer_j: int, agent_1based: int, model: str, M: int,
        diag_i: Dict[str, Any],
    ) -> None:
        """
        Write one row of consensus_outer.csv for agent agent_1based.
        diag_i is the per-agent diagnostic dict extracted from the coordinator reply.
        """
        NAN = float("nan")
        rt = diag_i.get("r_term", [NAN, NAN])
        if not (isinstance(rt, (list, tuple)) and len(rt) >= 2):
            rt = [NAN, NAN]
        bary = diag_i.get("bary_r", [NAN, NAN])
        if not (isinstance(bary, (list, tuple)) and len(bary) >= 2):
            bary = [NAN, NAN]

        self._w_out.writerow([
            outer_j, agent_1based, model, M,
            diag_i.get("objective_primary", NAN),
            int(bool(diag_i.get("lex_used", 0))),
            diag_i.get("phi_terminal", NAN),
            int(bool(diag_i.get("ri_terminal", 0))),
            float(rt[0]), float(rt[1]),
            diag_i.get("diam_C", NAN),
            diag_i.get("t_primary_ms", NAN),
            diag_i.get("t_lex_ms", NAN),
            diag_i.get("t_total_ms", NAN),
            int(diag_i.get("n_var_primary", 0)),
            int(diag_i.get("n_eq_primary", 0)),
            int(diag_i.get("n_ineq_primary", 0)),
            int(diag_i.get("n_var_lex", 0)),
            int(diag_i.get("n_eq_lex", 0)),
            int(diag_i.get("n_ineq_lex", 0)),
            int(bool(diag_i.get("nominal_fallback_used", False))),
            diag_i.get("nominal_status", ""),
            float(bary[0]), float(bary[1]),
        ])

    def log_outer_fail(
        self, outer_j: int, n_agents: int, model: str, M: int, reason: str,
    ) -> None:
        """Write n_agents rows of zeros when the coordinator fails."""
        NAN = float("nan")
        for i in range(1, n_agents + 1):
            self._w_out.writerow([
                outer_j, i, model, M,
                NAN, 0, NAN, 0, NAN, NAN, NAN,
                NAN, NAN, NAN, 0, 0, 0, 0, 0, 0,
                1, reason, NAN, NAN,
            ])

    # ═══════════════════════════════════════════════════════════════════════
    # Lifecycle
    # ═══════════════════════════════════════════════════════════════════════

    def flush(self) -> None:
        for f in [self._f_traj, self._f_met, self._f_out, self._f_modes, self._f_uni]:
            f.flush()

    def close(self) -> None:
        for f in [self._f_traj, self._f_met, self._f_out, self._f_modes, self._f_uni]:
            try:
                f.flush()
                f.close()
            except Exception:
                pass
