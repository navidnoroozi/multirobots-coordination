"""cosim_bridge.py
Python ↔ MATLAB/Simulink bridge.

Responsibilities
----------------
1. Start and own a ``matlab.engine`` session.
2. Build (or reload) the Simulink model on first run via ``build_unicycle_model.m``.
3. Write per-agent velocity reference arrays into the MATLAB workspace before
   each Simulink sub-step.
4. Run the Simulink model for exactly one planning outer step (``planning_dt``
   seconds of simulated time) via ``sim()``.
5. Read back per-agent odometry arrays from the MATLAB workspace.
6. Expose a clean ``step()`` interface so ``cosim_manager.py`` never touches
   MATLAB internals directly.

Interface contract
------------------
``step(outer_j, u_safe_all, current_sim_time)``
    u_safe_all : ndarray of shape (n_agents, dim) — hybrid-filtered velocities
                 for the current planning step.
    Returns    : OdometryBundle — positions, headings, speeds, angular rates.

Notes on the ``sim()`` approach
---------------------------------
``matlab.engine`` calls ``sim('model', SimulationMode='normal', ...)`` in a
blocking fashion from Python.  For each planning step we run the Simulink model
from ``t_start`` to ``t_start + planning_dt``, passing the new ``u_ref``
values through MATLAB workspace variables (``From Workspace`` blocks inside
Simulink read them as timeseries).

This avoids the need for the Instrument Control Toolbox (UDP Send/Receive blocks
are NOT required) and works with a base MATLAB + Simulink installation.

If real-time hardware-in-the-loop is later required, replace this module with a
UDP-socket variant; ``cosim_manager.py`` depends only on the ``step()`` API.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from cosim_config import CoSimConfig

log = logging.getLogger(__name__)


@dataclass
class OdometryBundle:
    """Odometry data returned by Simulink after one planning step."""
    # shape (n_agents, 2) — x, y in planning-layer coordinates
    positions: np.ndarray
    # shape (n_agents,) — heading θ in radians
    headings: np.ndarray
    # shape (n_agents,) — linear speed v [unit/s]
    linear_speeds: np.ndarray
    # shape (n_agents,) — angular rate ω [rad/s]
    angular_rates: np.ndarray
    # simulation wall-clock time for this bundle
    sim_time: float = 0.0
    # per-agent raw dict (keyed 1..N) for logging
    raw: Dict[int, Dict[str, float]] = field(default_factory=dict)


class MatlabBridge:
    """Owns the matlab.engine session and the Simulink model lifecycle."""

    def __init__(self, cfg: CoSimConfig) -> None:
        self.cfg = cfg
        self._eng: Optional[Any] = None          # matlab.engine.MatlabEngine
        self._sim_time: float = 0.0              # current simulated time [s]
        self._model_loaded: bool = False
        self._matlab_available: bool = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start MATLAB, build/load the Simulink model."""
        try:
            import matlab.engine  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "matlab.engine is not importable.  Install MATLAB Engine API "
                "for Python: from your MATLAB root run "
                "'cd(fullfile(matlabroot,\"extern\",\"engines\",\"python\")); "
                "system(\"pip install .\")'"
            ) from exc

        log.info("[Bridge] Starting MATLAB engine …")
        t0 = time.perf_counter()
        self._eng = matlab.engine.start_matlab(self.cfg.matlab_options)
        elapsed = time.perf_counter() - t0
        log.info("[Bridge] MATLAB started in %.1f s.", elapsed)
        self._matlab_available = True

        self._ensure_model()

    def stop(self) -> None:
        """Gracefully quit the MATLAB engine."""
        if self._eng is not None:
            try:
                log.info("[Bridge] Stopping Simulink model …")
                self._eng.eval(
                    f"if bdIsLoaded('{self.cfg.model_name}'),"
                    f"  close_system('{self.cfg.model_name}', 0);"
                    f"end",
                    nargout=0,
                )
            except Exception:
                pass
            try:
                log.info("[Bridge] Quitting MATLAB engine …")
                self._eng.quit()
            except Exception:
                pass
            self._eng = None
        self._matlab_available = False

    # ------------------------------------------------------------------
    # Core step interface
    # ------------------------------------------------------------------

    def step(
        self,
        outer_j: int,
        u_safe_all: np.ndarray,
        inner_t: int = 0,
    ) -> OdometryBundle:
        """Run Simulink for one inner step and return odometry.

        Parameters
        ----------
        outer_j   : outer planning index (for workspace variable naming)
        u_safe_all: (n_agents, dim) velocity reference from the hybrid layer
        inner_t   : inner MPC step index within the current outer step
        """
        if not self._matlab_available:
            return self._dummy_odometry(u_safe_all)

        # --- write u_ref to MATLAB workspace ---
        self._write_u_ref(u_safe_all, inner_t)

        # --- advance Simulink by simulink_dt ---
        t_end = self._sim_time + self.cfg.simulink_dt
        self._run_sim_step(t_end)
        self._sim_time = t_end

        # --- read odometry ---
        bundle = self._read_odometry()
        return bundle

    def step_outer(
        self,
        outer_j: int,
        u_safe_sequence: np.ndarray,
    ) -> List[OdometryBundle]:
        """Run a full planning outer step (M inner steps) and return all bundles.

        Parameters
        ----------
        u_safe_sequence : (M, n_agents, dim) — the full inner sequence from MPC
        """
        M = u_safe_sequence.shape[0]
        bundles: List[OdometryBundle] = []
        for inner_t in range(M):
            b = self.step(outer_j, u_safe_sequence[inner_t], inner_t)
            bundles.append(b)
        return bundles

    def reset_sim_time(self) -> None:
        """Reset the internal sim-time counter (call before a new experiment)."""
        self._sim_time = 0.0

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _ensure_model(self) -> None:
        """Build the .slx if needed and load it into MATLAB."""
        eng = self._eng
        matlab_dir = str(self.cfg.matlab_dir_path)
        model_name = self.cfg.model_name
        slx_path = self.cfg.model_slx

        # Add matlab/ directory to MATLAB path
        eng.addpath(matlab_dir, nargout=0)

        if not slx_path.exists():
            log.info("[Bridge] .slx not found – running build_unicycle_model.m …")
            eng.eval(
                f"build_unicycle_model('{model_name}', '{matlab_dir}', "
                f"{self.cfg.n_agents}, {self.cfg.simulink_dt}, "
                f"{self.cfg.k_heading}, {self.cfg.k_speed}, "
                f"{self.cfg.max_linear_speed}, {self.cfg.max_angular_speed})",
                nargout=0,
            )
            log.info("[Bridge] Model built: %s", slx_path)
        else:
            log.info("[Bridge] Loading existing model: %s", slx_path)

        eng.eval(f"load_system('{model_name}')", nargout=0)

        # Configure solver
        eng.eval(
            f"set_param('{model_name}', "
            f"'SolverType', 'Fixed-step', "
            f"'FixedStep', '{self.cfg.simulink_dt}', "
            f"'Solver', 'ode1');",
            nargout=0,
        )
        self._model_loaded = True
        log.info("[Bridge] Simulink model ready.")

    def _write_u_ref(self, u_safe_all: np.ndarray, inner_t: int) -> None:
        """Pack per-agent velocity references into MATLAB workspace."""
        import matlab  # type: ignore

        for i in range(1, self.cfg.n_agents + 1):
            u_i = u_safe_all[i - 1, :]   # shape (dim,)
            var_name = self.cfg.workspace_u_name(i)
            # Simulink 'From Workspace' expects a struct with .time and .signals.values
            # We pass it as a simple [u_x, u_y] vector via a MATLAB variable;
            # the From Workspace block is configured with the variable name.
            self._eng.workspace[var_name] = matlab.double(u_i.tolist())

    def _run_sim_step(self, t_end: float) -> None:
        """Advance the Simulink model from current time to t_end."""
        eng = self._eng
        model = self.cfg.model_name
        # Use simset / sim with explicit time limits
        eng.eval(
            f"set_param('{model}', 'StopTime', '{t_end:.6f}');",
            nargout=0,
        )
        if self._sim_time == 0.0 or not self._is_running():
            # First step: start the simulation
            eng.eval(
                f"set_param('{model}', 'SimulationCommand', 'start');",
                nargout=0,
            )
            # Wait until it reaches t_end
            self._wait_for_sim(t_end)
        else:
            # Subsequent steps: simulation is paused, advance by one step
            eng.eval(
                f"set_param('{model}', 'SimulationCommand', 'continue');",
                nargout=0,
            )
            self._wait_for_sim(t_end)

    def _wait_for_sim(self, t_end: float, poll_interval: float = 0.001) -> None:
        """Poll simulation status until it reaches t_end or pauses."""
        eng = self._eng
        model = self.cfg.model_name
        deadline = time.perf_counter() + 10.0  # 10-s safety timeout
        while time.perf_counter() < deadline:
            status = eng.eval(f"get_param('{model}', 'SimulationStatus')", nargout=1)
            if str(status) in ("paused", "stopped"):
                return
            cur_t = float(eng.eval(f"get_param('{model}', 'SimulationTime')", nargout=1))
            if cur_t >= t_end - 1e-9:
                # Pause at t_end so we can update inputs
                eng.eval(
                    f"set_param('{model}', 'SimulationCommand', 'pause');",
                    nargout=0,
                )
                return
            time.sleep(poll_interval)
        log.warning("[Bridge] Simulation step timed out at t=%.3f", t_end)

    def _is_running(self) -> bool:
        """Check if the Simulink model is currently in a running or paused state."""
        try:
            status = str(
                self._eng.eval(
                    f"get_param('{self.cfg.model_name}', 'SimulationStatus')",
                    nargout=1,
                )
            )
            return status in ("running", "paused")
        except Exception:
            return False

    def _read_odometry(self) -> OdometryBundle:
        """Read per-agent odometry variables from MATLAB workspace."""
        n = self.cfg.n_agents
        positions = np.zeros((n, 2), dtype=float)
        headings = np.zeros(n, dtype=float)
        linear_speeds = np.zeros(n, dtype=float)
        angular_rates = np.zeros(n, dtype=float)
        raw: Dict[int, Dict[str, float]] = {}

        for i in range(1, n + 1):
            var = self.cfg.workspace_odo_name(i)
            try:
                odo = self._eng.workspace[var]
                # Expected layout: [x, y, theta, v, omega]
                arr = np.array(odo, dtype=float).flatten()
                if arr.size >= 5:
                    positions[i - 1, 0] = arr[0]
                    positions[i - 1, 1] = arr[1]
                    headings[i - 1] = arr[2]
                    linear_speeds[i - 1] = arr[3]
                    angular_rates[i - 1] = arr[4]
                    raw[i] = dict(x=arr[0], y=arr[1], theta=arr[2], v=arr[3], omega=arr[4])
            except Exception as exc:
                log.debug("[Bridge] Could not read %s: %s", var, exc)

        return OdometryBundle(
            positions=positions,
            headings=headings,
            linear_speeds=linear_speeds,
            angular_rates=angular_rates,
            sim_time=self._sim_time,
            raw=raw,
        )

    def _dummy_odometry(self, u_safe_all: np.ndarray) -> OdometryBundle:
        """Return zero odometry when MATLAB is not available (dry-run mode)."""
        n = self.cfg.n_agents
        return OdometryBundle(
            positions=np.zeros((n, 2)),
            headings=np.zeros(n),
            linear_speeds=np.zeros(n),
            angular_rates=np.zeros(n),
            sim_time=self._sim_time,
        )
