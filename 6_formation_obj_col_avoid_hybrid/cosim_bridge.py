# cosim_bridge.py  -- patched core methods

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from cosim_config import CoSimConfig

log = logging.getLogger(__name__)


@dataclass
class OdometryBundle:
    positions: np.ndarray
    headings: np.ndarray
    linear_speeds: np.ndarray
    angular_rates: np.ndarray
    sim_time: float = 0.0
    raw: Dict[int, Dict[str, float]] = field(default_factory=dict)


class MatlabBridge:
    def __init__(self, cfg: CoSimConfig) -> None:
        self.cfg = cfg
        self._eng: Optional[Any] = None
        self._sim_time: float = 0.0
        self._model_loaded: bool = False
        self._matlab_available: bool = False

    def start(self) -> None:
        try:
            import matlab.engine  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "matlab.engine is not importable. Install MATLAB Engine API for Python."
            ) from exc

        log.info("[Bridge] Starting MATLAB engine …")
        t0 = time.perf_counter()
        self._eng = matlab.engine.start_matlab(self.cfg.matlab_options)
        log.info("[Bridge] MATLAB started in %.1f s.", time.perf_counter() - t0)

        self._matlab_available = True
        self._ensure_model()
        self.reset_experiment()

    def stop(self) -> None:
        if self._eng is not None:
            try:
                self._eng.eval(
                    f"if bdIsLoaded('{self.cfg.model_name}'), "
                    f"set_param('{self.cfg.model_name}','SimulationCommand','stop'); "
                    f"close_system('{self.cfg.model_name}', 0); "
                    f"end",
                    nargout=0,
                )
            except Exception:
                pass
            try:
                self._eng.quit()
            except Exception:
                pass
            self._eng = None
        self._matlab_available = False

    def reset_experiment(self) -> None:
        """Reset simulation state to t=0 for a fresh experiment run.

        ROOT-CAUSE FIX (Bug 6 — agents jump to origin on first step):
        ---------------------------------------------------------------
        build_unicycle_model.m sets InitialCondition='0' on every integrator
        (Int_x, Int_y, Int_theta).  This places all agents at (0, 0, 0)
        regardless of the planning layer's initial positions.

        After the first Simulink step, cosim_manager reads back odometry ≈ (0, 0)
        and updates r.  The planning layer teleports from the true initial
        positions to the origin.  The hybrid supervisor then sees all agents
        collapsed at the origin (inside the obstacle), fires at full saturation,
        and produces the "impulsive" aggressive jumps visible in the animation.

        Fix: call set_param() on Int_x, Int_y, Int_theta for every agent,
        injecting the correct initial conditions from CoSimConfig.initial_positions
        and CoSimConfig.initial_headings before the first simulation run.

        NOTE (Bug 2 fix, kept): StopTime is NOT modified here.
        cosim_step.m sets StopTime='inf' on first-start and owns it.
        """
        self._sim_time = 0.0
        if not self._matlab_available:
            return

        model = self.cfg.model_name

        # Stop simulation if currently running or paused
        self._eng.eval(
            f"try; set_param('{model}', 'SimulationCommand', 'stop'); catch; end",
            nargout=0,
        )

        # Clear all odometry and velocity-reference workspace variables
        self._eng.eval(
            "try; evalin('base', 'clear odo_* u_ref_*'); catch; end",
            nargout=0,
        )

        # Reset StartTime so the next 'start' begins at t=0
        # (StopTime intentionally left as-is; cosim_step.m owns it)
        self._eng.eval(
            f"set_param('{model}', 'StartTime', '0.0');",
            nargout=0,
        )

        # ------------------------------------------------------------------
        # KEY FIX: Inject true initial conditions into every integrator.
        #
        # Without this, Simulink uses IC=0 for all integrators, placing all
        # agents at (0,0,0).  The first odometry feedback then teleports the
        # planning layer's r array to the origin in a single step, causing
        # all safety violations and the impulsive behavior.
        # ------------------------------------------------------------------
        for i in range(1, self.cfg.n_agents + 1):
            # Initial position from CoSimConfig
            if self.cfg.initial_positions and len(self.cfg.initial_positions) >= i:
                x0 = float(self.cfg.initial_positions[i - 1][0])
                y0 = float(self.cfg.initial_positions[i - 1][1])
            else:
                x0, y0 = 0.0, 0.0
                log.warning(
                    "[Bridge] Agent %d has no initial_position in CoSimConfig. "
                    "Integrators will start at (0,0). "
                    "Pass initial_positions to CoSimConfig to fix the origin-jump bug.",
                    i,
                )

            # Initial heading from CoSimConfig
            if self.cfg.initial_headings and len(self.cfg.initial_headings) >= i:
                theta0 = float(self.cfg.initial_headings[i - 1])
            else:
                theta0 = 0.0

            self._eng.eval(
                f"set_param('{model}/Agent_{i}/Unicycle/Int_x',     "
                f"  'InitialCondition', '{x0:.15g}'); "
                f"set_param('{model}/Agent_{i}/Unicycle/Int_y',     "
                f"  'InitialCondition', '{y0:.15g}'); "
                f"set_param('{model}/Agent_{i}/Unicycle/Int_theta', "
                f"  'InitialCondition', '{theta0:.15g}');",
                nargout=0,
            )
            log.info(
                "[Bridge] Agent %d integrator ICs → x0=%.4f  y0=%.4f  theta0=%.4f",
                i, x0, y0, theta0,
            )


    def step(self, outer_j: int, u_safe_all: np.ndarray, inner_t: int = 0) -> OdometryBundle:
        if not self._matlab_available:
            return self._dummy_odometry(u_safe_all)

        import matlab  # type: ignore

        u = np.asarray(u_safe_all, dtype=float)
        if u.shape != (self.cfg.n_agents, self.cfg.dim):
            raise ValueError(
                f"u_safe_all must have shape {(self.cfg.n_agents, self.cfg.dim)}, got {u.shape}"
            )

        t_start = float(self._sim_time)
        t_end = float(t_start + self.cfg.simulink_dt)

        odo_all = self._eng.cosim_step(
            self.cfg.model_name,
            matlab.double(u.tolist()),
            float(t_start),
            float(t_end),
            float(self.cfg.simulink_dt),
            nargout=1,
        )

        odo_np = np.array(odo_all, dtype=float)
        self._sim_time = t_end
        return self._bundle_from_matrix(odo_np, sim_time=t_end)

    def _bundle_from_matrix(self, odo_all: np.ndarray, sim_time: float) -> OdometryBundle:
        n = self.cfg.n_agents
        odo_all = np.asarray(odo_all, dtype=float)

        if odo_all.shape != (n, 5):
            raise RuntimeError(f"Expected odometry matrix {(n, 5)}, got {odo_all.shape}")

        positions = odo_all[:, 0:2].copy()
        headings = odo_all[:, 2].copy()
        linear_speeds = odo_all[:, 3].copy()
        angular_rates = odo_all[:, 4].copy()

        raw: Dict[int, Dict[str, float]] = {}
        for i in range(n):
            raw[i + 1] = {
                "x": float(positions[i, 0]),
                "y": float(positions[i, 1]),
                "theta": float(headings[i]),
                "v": float(linear_speeds[i]),
                "omega": float(angular_rates[i]),
            }

        return OdometryBundle(
            positions=positions,
            headings=headings,
            linear_speeds=linear_speeds,
            angular_rates=angular_rates,
            sim_time=sim_time,
            raw=raw,
        )

    def _ensure_model(self) -> None:
        eng = self._eng
        matlab_dir = str(self.cfg.matlab_dir_path)
        model_name = self.cfg.model_name
        slx_path = self.cfg.model_slx

        eng.addpath(matlab_dir, nargout=0)

        if not slx_path.exists():
            eng.eval(
                f"build_unicycle_model('{model_name}', '{matlab_dir}', "
                f"{self.cfg.n_agents}, {self.cfg.simulink_dt}, "
                f"{self.cfg.k_heading}, {self.cfg.k_speed}, "
                f"{self.cfg.max_linear_speed}, {self.cfg.max_angular_speed})",
                nargout=0,
            )

        eng.eval(f"load_system('{model_name}')", nargout=0)
        eng.eval(
            f"set_param('{model_name}', "
            f"'SolverType', 'Fixed-step', "
            f"'FixedStep', '{self.cfg.simulink_dt:.12g}', "
            f"'Solver', 'ode1');",
            nargout=0,
        )
        self._model_loaded = True

    def _dummy_odometry(self, u_safe_all: np.ndarray) -> OdometryBundle:
        n = self.cfg.n_agents
        return OdometryBundle(
            positions=np.zeros((n, 2)),
            headings=np.zeros(n),
            linear_speeds=np.zeros(n),
            angular_rates=np.zeros(n),
            sim_time=self._sim_time,
        )