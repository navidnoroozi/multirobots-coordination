"""cosim_bridge.py
Python ↔ MATLAB/Simulink bridge.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
from consensus_config import NetConfig 

from cosim_config import CoSimConfig

log = logging.getLogger(__name__)

@dataclass
class OdometryBundle:
    """Odometry data returned by Simulink after one planning step."""
    positions: np.ndarray
    headings: np.ndarray
    linear_speeds: np.ndarray
    angular_rates: np.ndarray
    sim_time: float = 0.0
    raw: Dict[int, Dict[str, float]] = field(default_factory=dict)

class MatlabBridge:
    """Owns the matlab.engine session and the Simulink model lifecycle."""

    def __init__(self, cfg: CoSimConfig) -> None:
        self.cfg = cfg
        self._eng: Optional[Any] = None
        self._sim_time: float = 0.0
        self._model_loaded: bool = False
        self._matlab_available: bool = False

    def start(self) -> None:
        """Start MATLAB, build/load the Simulink model."""
        try:
            import matlab.engine  # type: ignore
        except ImportError as exc:
            raise RuntimeError("matlab.engine is not importable.") from exc

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
                    f" close_system('{self.cfg.model_name}', 0);"
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

    def step(self, outer_j: int, u_safe_all: np.ndarray, inner_t: int = 0) -> OdometryBundle:
        """Run Simulink for one inner step and return odometry."""
        if not self._matlab_available:
            return self._dummy_odometry(u_safe_all)

        self._write_u_ref(u_safe_all, inner_t)

        # CRITICAL FIX: Advance Simulink by the MPC planning interval, not the internal solver step size!
        t_end = self._sim_time + self.cfg.planning_dt
        self._run_sim_step(t_end)
        self._sim_time = t_end

        return self._read_odometry()

    def step_outer(self, outer_j: int, u_safe_sequence: np.ndarray) -> List[OdometryBundle]:
        M = u_safe_sequence.shape[0]
        bundles: List[OdometryBundle] = []
        for inner_t in range(M):
            b = self.step(outer_j, u_safe_sequence[inner_t], inner_t)
            bundles.append(b)
        return bundles

    def reset_sim_time(self) -> None:
        self._sim_time = 0.0

    def _ensure_model(self) -> None:
        """Build the .slx if needed and load it into MATLAB."""
        eng = self._eng
        matlab_dir = str(self.cfg.matlab_dir_path)
        model_name = self.cfg.model_name
        slx_path = self.cfg.model_slx

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

        # Configure solver AND strictly force ReturnWorkspaceOutputs to OFF dynamically
        eng.eval(
            f"set_param('{model_name}', "
            f"'SolverType', 'Fixed-step', "
            f"'FixedStep', '{self.cfg.simulink_dt}', "
            f"'Solver', 'ode1', "
            f"'ReturnWorkspaceOutputs', 'off');",
            nargout=0,
        )

        # CRITICAL FIX: Inject Python initial positions into the Simulink integrators
        # so agents start at the correct coordinates instead of (0,0)
        from consensus_config import NetConfig
        net_cfg = NetConfig()
        r0 = net_cfg.initial_positions()
        
        for i in range(1, self.cfg.n_agents + 1):
            x_init = float(r0[i - 1, 0])
            y_init = float(r0[i - 1, 1])
            # Set initial states for x, y, and theta (default heading to 0 rad)
            eng.eval(
                f"set_param('{model_name}/Agent_{i}/Unicycle/Int_x', 'InitialCondition', '{x_init}');",
                nargout=0
            )
            eng.eval(
                f"set_param('{model_name}/Agent_{i}/Unicycle/Int_y', 'InitialCondition', '{y_init}');",
                nargout=0
            )
            eng.eval(
                f"set_param('{model_name}/Agent_{i}/Unicycle/Int_theta', 'InitialCondition', '0');",
                nargout=0
            )

        self._model_loaded = True
        log.info("[Bridge] Simulink model ready with applied initial conditions.")


    def _write_u_ref(self, u_safe_all: np.ndarray, inner_t: int) -> None:
        """Inject per-agent velocity references directly into Constant blocks."""
        model = self.cfg.model_name
        for i in range(1, self.cfg.n_agents + 1):
            u_i = u_safe_all[i - 1, :]
            
            # Format the array as a MATLAB string: '[x, y]'
            val_str = f"[{float(u_i[0])}, {float(u_i[1])}]"
            
            # Dynamically set the value in the paused Simulink model
            self._eng.eval(
                f"set_param('{model}/Agent_{i}/Constant_u', 'Value', '{val_str}');", 
                nargout=0
            )

    def _run_sim_step(self, t_end: float) -> None:
        eng = self._eng
        model = self.cfg.model_name
        eng.eval(f"set_param('{model}', 'StopTime', '{t_end:.6f}');", nargout=0)

        if self._sim_time == 0.0 or not self._is_running():
            eng.eval(f"set_param('{model}', 'SimulationCommand', 'start');", nargout=0)
            self._wait_for_sim(t_end)
        else:
            eng.eval(f"set_param('{model}', 'SimulationCommand', 'continue');", nargout=0)
            self._wait_for_sim(t_end)

    def _wait_for_sim(self, t_end: float, poll_interval: float = 0.001) -> None:
        eng = self._eng
        model = self.cfg.model_name
        deadline = time.perf_counter() + 10.0
        while time.perf_counter() < deadline:
            status = eng.eval(f"get_param('{model}', 'SimulationStatus')", nargout=1)
            if str(status) in ("paused", "stopped"):
                return
            cur_t = float(eng.eval(f"get_param('{model}', 'SimulationTime')", nargout=1))
            if cur_t >= t_end - 1e-9:
                eng.eval(f"set_param('{model}', 'SimulationCommand', 'pause');", nargout=0)
                return
            time.sleep(poll_interval)
        log.warning("[Bridge] Simulation step timed out at t=%.3f", t_end)

    def _is_running(self) -> bool:
        try:
            status = str(self._eng.eval(f"get_param('{self.cfg.model_name}', 'SimulationStatus')", nargout=1))
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
                arr = np.array(odo, dtype=float)

                # CRITICAL FIX: To Workspace appends data over time. Extract the last row!
                if arr.ndim == 2 and arr.shape[0] > 0:
                    latest = arr[-1, :]
                else:
                    latest = arr.flatten()[-5:]

                if latest.size >= 5:
                    positions[i - 1, 0] = latest[0]
                    positions[i - 1, 1] = latest[1]
                    headings[i - 1] = latest[2]
                    linear_speeds[i - 1] = latest[3]
                    angular_rates[i - 1] = latest[4]
                    raw[i] = dict(x=latest[0], y=latest[1], theta=latest[2], v=latest[3], omega=latest[4])
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
        n = self.cfg.n_agents
        return OdometryBundle(
            positions=np.zeros((n, 2)),
            headings=np.zeros(n),
            linear_speeds=np.zeros(n),
            angular_rates=np.zeros(n),
            sim_time=self._sim_time,
        )