from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from typing import Dict, List, Literal, Optional, Tuple

import numpy as np

ModelType = Literal["single_integrator", "double_integrator"]


def _pairwise_diameter(X: np.ndarray) -> float:
    if X.ndim != 2 or X.shape[0] <= 1:
        return 0.0
    dmax = 0.0
    for i in range(X.shape[0]):
        for k in range(i + 1, X.shape[0]):
            dmax = max(dmax, float(np.linalg.norm(X[i] - X[k])))
    return dmax


def _str2bool(v: str | bool) -> bool:
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    if s in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if s in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {v}")


@dataclass(frozen=True)
class NetConfig:
    n_agents: int = 4
    dim: int = 2
    model: ModelType = "single_integrator"
    outer_steps: int = 10
    auto_M: bool = False
    M_manual: int = 5
    alpha_gamma: float = 0.80

    r_min: float = -5.0
    r_max: float = 5.0
    u_min: float = -2.0
    u_max: float = 2.0
    u_mag: float = 2.0
    v_min: float = -10.0
    v_max: float = 10.0

    w_track: float = 10.0
    w_du: float = 1.0
    w_u: float = 0.0
    w_v: float = 0.10

    use_lexicographic: bool = True
    lex_cost_tol: float = 1e-6
    phi_tol: float = 1e-19
    diam_tol: float = 1e-15
    lex_only_if_phi_zero: bool = True

    plant_to_coord_rep: str = "tcp://127.0.0.1:5555"
    ctrl_base_port: int = 5600
    logger_endpoint: str = "tcp://127.0.0.1:5700"
    graph: Literal["ring_timevarying", "complete"] = "ring_timevarying"

    r0_single: Tuple[Tuple[float, float], ...] = (
        (-4.0, 2.0),
        (3.5, 4.0),
        (4.5, -3.5),
        (-2.5, -4.0),
    )
    r0_double: Tuple[Tuple[float, float], ...] = (
        (-4.0, 2.0),
        (3.5, 4.0),
        (4.5, -3.5),
        (-2.5, -4.0),
    )
    v0_double: Tuple[Tuple[float, float], ...] = (
        (1.0, 0.0),
        (0.0, -1.0),
        (-1.0, 0.0),
        (0.0, 1.0),
    )

    startup_delay_s: float = 2.0
    req_timeout_ms: int = 12000
    coord_controller_timeout_ms: int = 15000
    hybrid_timeout_ms: int = 5000
    req_linger_ms: int = 0

    # Hybrid supervisor
    hybrid_enabled: bool = True
    dt: float = 1.0
    d_safe: float = 0.90

    # tuned hysteresis: enter earlier, release later
    d_hold_down: float = 1.02
    d_hold_up: float = 1.18
    d_warn_down: float = 1.48
    d_warn_up: float = 1.72
    cluster_diam_hold: float = 2.05
    cluster_diam_release: float = 2.65

    nominal_dwell_min: int = 2
    hold_dwell_min: int = 2
    emergency_dwell_min: int = 2

    # anchor shell
    hybrid_anchor_min_radius: float = 0.58
    hybrid_anchor_max_radius: float = 0.95

    # single-integrator tuning
    hold_position_gain_si: float = 0.52
    hold_repulsion_gain_si: float = 0.26
    hold_velocity_damping_si: float = 0.00
    emergency_repulsion_gain_si: float = 0.60
    emergency_direction_gain_si: float = 0.78
    emergency_velocity_damping_si: float = 0.00
    hybrid_hold_max_correction_si: float = 0.52
    hybrid_emergency_max_correction_si: float = 0.95

    # double-integrator tuning: more damping, less aggressive position push
    hold_position_gain_di: float = 0.22
    hold_repulsion_gain_di: float = 0.18
    hold_velocity_damping_di: float = 1.15
    emergency_repulsion_gain_di: float = 0.38
    emergency_direction_gain_di: float = 0.42
    emergency_velocity_damping_di: float = 1.65
    hybrid_hold_max_correction_di: float = 0.34
    hybrid_emergency_max_correction_di: float = 0.62

    # backward-compatible legacy names used by CLI / runner
    safety_enabled: bool = True
    safety_method: str = "hybrid"

    def controller_endpoint(self, agent_id_1based: int) -> str:
        return f"tcp://127.0.0.1:{self.ctrl_base_port + agent_id_1based}"

    def neighbors(self, outer_j: int) -> Dict[int, List[int]]:
        n = self.n_agents

        def ring_neighbors() -> Dict[int, List[int]]:
            nbrs: Dict[int, List[int]] = {}
            for i in range(1, n + 1):
                left = i - 1 if i > 1 else n
                right = i + 1 if i < n else 1
                nbrs[i] = [left, right]
            return nbrs

        if self.graph == "complete":
            return {i: [k for k in range(1, n + 1) if k != i] for i in range(1, n + 1)}

        nbrs = ring_neighbors()
        if outer_j % 5 == 0 and n >= 3 and 3 in nbrs[2]:
            nbrs[2] = [k for k in nbrs[2] if k != 3]
        return nbrs

    def initial_positions(self) -> np.ndarray:
        return np.array(self.r0_single if self.model == "single_integrator" else self.r0_double, dtype=float)

    def initial_velocities(self) -> np.ndarray:
        if self.model == "double_integrator":
            return np.array(self.v0_double, dtype=float)
        return np.zeros((self.n_agents, self.dim), dtype=float)

    def V0_single(self) -> float:
        return _pairwise_diameter(self.initial_positions())

    def Vr0_double(self) -> float:
        return _pairwise_diameter(self.initial_positions())

    def horizon_M(self) -> int:
        if not self.auto_M:
            return int(self.M_manual)
        umin = float(self.u_mag)
        if umin <= 0:
            return int(self.M_manual)
        if self.model == "single_integrator":
            return max(1, int(np.ceil(self.V0_single() / umin)))
        term1 = int(np.ceil((2.0 * float(self.v_max)) / umin))
        term2 = int(np.ceil(np.sqrt(max(self.Vr0_double(), 0.0) / umin)))
        return int(max(2, term1 + 2 * term2))


def add_common_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument("--n-agents", type=int, default=None)
    parser.add_argument("--model", choices=["single_integrator", "double_integrator"], default=None)
    parser.add_argument("--outer-steps", type=int, default=None)
    parser.add_argument("--use-lexicographic", type=_str2bool, default=None)
    parser.add_argument("--safety-enabled", type=_str2bool, default=None)
    parser.add_argument("--safety-method", default=None)
    parser.add_argument("--agent-id", type=int, default=None)
    parser.add_argument("--startup-delay-s", type=float, default=None)
    return parser


def config_from_namespace(ns: argparse.Namespace) -> NetConfig:
    cfg = NetConfig()
    updates = {}
    for key in ["n_agents", "model", "outer_steps", "use_lexicographic", "startup_delay_s"]:
        v = getattr(ns, key, None)
        if v is not None:
            updates[key] = v
    v = getattr(ns, "safety_enabled", None)
    if v is not None:
        updates["hybrid_enabled"] = bool(v)
        updates["safety_enabled"] = bool(v)
    sm = getattr(ns, "safety_method", None)
    if sm is not None:
        updates["safety_method"] = str(sm)
    return replace(cfg, **updates)


def parse_config_args(argv: Optional[list[str]] = None) -> tuple[NetConfig, argparse.Namespace]:
    parser = add_common_args(argparse.ArgumentParser())
    ns = parser.parse_args(argv)
    return config_from_namespace(ns), ns