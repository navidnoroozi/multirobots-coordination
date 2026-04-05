"""cosim_logger.py
Structured CSV and diagnostic logger for the co-simulation platform.

Four output streams
-------------------
1. **planning.csv**  – one row per (outer_j, inner_t, agent) capturing the
   planning layer's state: r, v, u_nom, u_safe, hybrid mode, objectives.
2. **odometry.csv**  – one row per (outer_j, inner_t, agent) from Simulink:
   x, y, θ, v, ω.
3. **hybrid_modes.csv** – mirrors plant_node.py's existing hybrid_modes.csv
   but with the co-sim outer/inner index.
4. **health.csv**    – one row per outer_j with timing, error counts, and
   convergence indicators useful for diagnosing the co-sim platform.

Design principles
-----------------
* All writers are opened once and kept open (buffered writes, flushed every
  ``flush_every`` outer steps) to avoid per-row open/close overhead.
* The logger is completely self-contained – it never imports from the planning
  layer so that it can be used in isolation for unit-tests.
* All numeric values are Python float / int – no numpy scalars reach CSV.
"""
from __future__ import annotations

import csv
import logging
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from cosim_config import CoSimConfig
from cosim_bridge import OdometryBundle

log = logging.getLogger(__name__)


class CoSimLogger:
    """Multi-stream CSV logger for the co-simulation platform."""

    def __init__(self, cfg: CoSimConfig) -> None:
        self.cfg = cfg
        self._log_dir = cfg.log_dir_path
        self._log_dir.mkdir(parents=True, exist_ok=True)

        # file handles
        self._f_planning = None
        self._f_odometry = None
        self._f_health = None
        self._f_modes = None

        # csv writers
        self._w_planning = None
        self._w_odometry = None
        self._w_health = None
        self._w_modes = None

        self._outer_since_flush = 0
        self._start_wall: float = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Open all CSV files and write headers."""
        self._start_wall = time.time()
        d = self._log_dir

        self._f_planning = open(d / self.cfg.log_planning_file, "w", newline="")
        self._w_planning = csv.writer(self._f_planning)
        self._w_planning.writerow([
            "k_global", "outer_j", "inner_t", "agent",
            "r_x", "r_y", "v_x", "v_y",
            "u_nom_x", "u_nom_y",
            "u_safe_x", "u_safe_y",
            "objective_primary", "lex_used",
        ])

        self._f_odometry = open(d / self.cfg.log_odometry_file, "w", newline="")
        self._w_odometry = csv.writer(self._f_odometry)
        self._w_odometry.writerow([
            "k_global", "outer_j", "inner_t", "agent",
            "sim_time",
            "odo_x", "odo_y", "odo_theta",
            "odo_v", "odo_omega",
            # position error vs planning layer
            "err_x", "err_y", "err_pos",
        ])

        self._f_health = open(d / self.cfg.log_health_file, "w", newline="")
        self._w_health = csv.writer(self._f_health)
        self._w_health.writerow([
            "outer_j",
            "wall_time_s",
            "elapsed_s",
            "planning_ok",
            "n_failed_ctrl",
            "Vr",
            "dmin_agents",
            "dmin_obs",
            "n_F", "n_C", "n_O", "n_CO", "n_T",
            "odo_max_err_pos",
            "matlab_step_ms",
        ])

        self._f_modes = open(d / self.cfg.log_modes_file, "w", newline="")
        self._w_modes = csv.writer(self._f_modes)
        self._w_modes.writerow([
            "k_global", "outer_j", "inner_t", "agent",
            "mode", "desired_mode", "effective_mode",
            "d_agent_min", "d_obs_min",
            "active_pairs", "active_obstacles",
            "obstructed", "waypoint_x", "waypoint_y",
            "_h_pair_num", "_circle_barrier_num",
            "formation_hold_active",
        ])

        log.info("[Logger] Opened log files in %s", self._log_dir)

    def close(self) -> None:
        """Flush and close all open files."""
        for fh in [self._f_planning, self._f_odometry, self._f_health, self._f_modes]:
            if fh is not None:
                try:
                    fh.flush()
                    fh.close()
                except Exception:
                    pass
        log.info("[Logger] All log files closed.")

    # ------------------------------------------------------------------
    # Per-step writers
    # ------------------------------------------------------------------

    def log_planning_step(
        self,
        k_global: int,
        outer_j: int,
        inner_t: int,
        r: np.ndarray,
        v: np.ndarray,
        u_nom: np.ndarray,
        u_safe: np.ndarray,
        diag: Dict[str, Any],
    ) -> None:
        """One row per agent per inner step – planning-layer quantities."""
        n = self.cfg.n_agents
        obj_d = diag.get("objective_primary", {}) if isinstance(diag, dict) else {}
        lex_d = diag.get("lex_used", {}) if isinstance(diag, dict) else {}

        for i in range(1, n + 1):
            obj_i = _get_keyed(obj_d, i, float("nan"))
            lex_i = int(bool(_get_keyed(lex_d, i, 0)))
            self._w_planning.writerow([
                k_global, outer_j, inner_t, i,
                _f(r[i - 1, 0]), _f(r[i - 1, 1]),
                _f(v[i - 1, 0] if v.shape[1] > 0 else 0),
                _f(v[i - 1, 1] if v.shape[1] > 1 else 0),
                _f(u_nom[i - 1, 0]), _f(u_nom[i - 1, 1]),
                _f(u_safe[i - 1, 0]), _f(u_safe[i - 1, 1]),
                _f(obj_i), lex_i,
            ])

    def log_odometry_step(
        self,
        k_global: int,
        outer_j: int,
        inner_t: int,
        bundle: OdometryBundle,
        r_planning: np.ndarray,
    ) -> None:
        """One row per agent – Simulink odometry vs planning-layer position."""
        n = self.cfg.n_agents
        for i in range(1, n + 1):
            x_odo = bundle.positions[i - 1, 0]
            y_odo = bundle.positions[i - 1, 1]
            x_plan = float(r_planning[i - 1, 0])
            y_plan = float(r_planning[i - 1, 1])
            err_x = x_odo - x_plan
            err_y = y_odo - y_plan
            err_pos = float(np.hypot(err_x, err_y))

            self._w_odometry.writerow([
                k_global, outer_j, inner_t, i,
                _f(bundle.sim_time),
                _f(x_odo), _f(y_odo),
                _f(bundle.headings[i - 1]),
                _f(bundle.linear_speeds[i - 1]),
                _f(bundle.angular_rates[i - 1]),
                _f(err_x), _f(err_y), _f(err_pos),
            ])

    def log_hybrid_modes_step(
        self,
        k_global: int,
        outer_j: int,
        inner_t: int,
        mode_data: List[Dict[str, Any]],
    ) -> None:
        """One row per agent – hybrid-mode diagnostics."""
        for i, d in enumerate(mode_data, start=1):
            wp = d.get("target_waypoint", [float("nan"), float("nan")])
            if not (isinstance(wp, (list, tuple)) and len(wp) >= 2):
                wp = [float("nan"), float("nan")]
            self._w_modes.writerow([
                k_global, outer_j, inner_t, i,
                d.get("mode", "F"),
                d.get("desired_mode", "F"),
                d.get("effective_mode", "F"),
                _f(d.get("d_agent_min", float("nan"))),
                _f(d.get("d_obs_min", float("nan"))),
                int(d.get("active_pairs", 0)),
                int(d.get("active_obstacles", 0)),
                int(bool(d.get("obstructed", False))),
                _f(wp[0]), _f(wp[1]),
                int(d.get("_h_pair_num", 0)),
                int(d.get("_circle_barrier_num", 0)),
                int(bool(d.get("formation_hold_active", False))),
            ])

    def log_health(
        self,
        outer_j: int,
        planning_ok: bool,
        n_failed_ctrl: int,
        Vr: float,
        dmin_agents: float,
        dmin_obs: float,
        mode_counts: Dict[str, int],
        odo_max_err: float,
        matlab_step_ms: float,
    ) -> None:
        """One row per outer step – platform health summary."""
        elapsed = time.time() - self._start_wall
        self._w_health.writerow([
            outer_j,
            _f(time.time()),
            _f(elapsed),
            int(planning_ok),
            n_failed_ctrl,
            _f(Vr),
            _f(dmin_agents),
            _f(dmin_obs),
            mode_counts.get("F", 0),
            mode_counts.get("C", 0),
            mode_counts.get("O", 0),
            mode_counts.get("CO", 0),
            mode_counts.get("T", 0),
            _f(odo_max_err),
            _f(matlab_step_ms),
        ])

        self._outer_since_flush += 1
        if self._outer_since_flush >= self.cfg.log_flush_every:
            self._flush_all()
            self._outer_since_flush = 0

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _flush_all(self) -> None:
        for fh in [self._f_planning, self._f_odometry, self._f_health, self._f_modes]:
            if fh is not None:
                fh.flush()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _f(x: Any) -> Any:
    """Convert numpy scalars to plain Python float for CSV safety."""
    try:
        return float(x)
    except (TypeError, ValueError):
        return x


def _get_keyed(d: Dict, i: int, default: Any) -> Any:
    if not isinstance(d, dict):
        return default
    if i in d:
        return d[i]
    si = str(i)
    if si in d:
        return d[si]
    return default
