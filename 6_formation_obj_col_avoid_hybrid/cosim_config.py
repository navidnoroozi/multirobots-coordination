"""cosim_config.py
Configuration for the Python–MATLAB/Simulink co-simulation layer.

This module is intentionally separate from consensus_config.py so that
the two layers can be tuned independently and the Simulink side can be
swapped out without touching the planning layer.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Tuple


@dataclass(frozen=True)
class CoSimConfig:
    # ---------------------------------------------------------------
    # Simulink model
    # ---------------------------------------------------------------
    model_name: str = "unicycle_cosim"
    """Name of the .slx file (without extension)."""

    matlab_dir: str = "matlab"
    """Directory that holds the .m build script and the generated .slx."""

    simulink_dt: float = 0.01
    """Fixed simulation step size inside Simulink [s].
    Must divide planning_dt exactly."""

    planning_dt: float = 1.0
    """Duration of one planning outer step [s].
    Each outer step runs planning_dt / simulink_dt Simulink steps."""

    n_agents: int = 4
    """Must match NetConfig.n_agents."""

    dim: int = 2
    """Spatial dimension (2 for 2-D plane)."""

    # ---------------------------------------------------------------
    # Unicycle / motion-controller parameters
    # ---------------------------------------------------------------
    k_heading: float = 4.0
    """Proportional gain for the heading error controller [rad/(rad·s)]."""

    k_speed: float = 1.0
    """Speed scaling applied to v_ref (feedforward gain, dimensionless)."""

    v_min_threshold: float = 0.05
    """Minimum |u_ref| below which heading is frozen to avoid singularity."""

    max_linear_speed: float = 2.0
    """Saturation on v (linear wheel speed) [m/s or unit/s]."""

    max_angular_speed: float = 6.0
    """Saturation on ω (angular rate) [rad/s]."""

    # ---------------------------------------------------------------
    # Initial headings (one per agent, radians)
    # ---------------------------------------------------------------
    initial_headings: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0)

    # ---------------------------------------------------------------
    # MATLAB engine / startup
    # ---------------------------------------------------------------
    matlab_startup_timeout: float = 60.0
    """Maximum seconds to wait for MATLAB engine to start."""

    matlab_options: str = "-nodesktop -nosplash"
    """Flags forwarded to matlab.engine.start_matlab()."""

    simulink_build_timeout: float = 30.0
    """Maximum seconds to allow the model-build script to run."""

    # ---------------------------------------------------------------
    # Logging
    # ---------------------------------------------------------------
    log_dir: str = "cosim_logs"
    """Directory where all CSV and diagnostic files are written."""

    log_odometry_file: str = "odometry.csv"
    log_planning_file: str = "planning.csv"
    log_health_file: str = "health.csv"
    log_modes_file: str = "hybrid_modes.csv"
    log_flush_every: int = 5
    """Write-flush interval in outer steps (reduces I/O overhead)."""

    # ---------------------------------------------------------------
    # Derived helpers
    # ---------------------------------------------------------------
    @property
    def steps_per_outer(self) -> int:
        """Number of Simulink fixed steps per planning outer step."""
        ratio = self.planning_dt / self.simulink_dt
        if abs(ratio - round(ratio)) > 1e-9:
            raise ValueError(
                f"planning_dt ({self.planning_dt}) must be an exact "
                f"multiple of simulink_dt ({self.simulink_dt})."
            )
        return int(round(ratio))

    @property
    def matlab_dir_path(self) -> Path:
        return Path(self.matlab_dir).resolve()

    @property
    def model_slx(self) -> Path:
        return self.matlab_dir_path / f"{self.model_name}.slx"

    @property
    def log_dir_path(self) -> Path:
        return Path(self.log_dir).resolve()

    def workspace_u_name(self, agent_id: int) -> str:
        """MATLAB workspace variable name for u_ref of agent i."""
        return f"u_ref_{agent_id}"

    def workspace_odo_name(self, agent_id: int) -> str:
        """MATLAB workspace variable name for odometry of agent i."""
        return f"odo_{agent_id}"


# ---------------------------------------------------------------------------
# Default singleton – import and use directly in most modules
# ---------------------------------------------------------------------------
DEFAULT_COSIM_CONFIG = CoSimConfig()
