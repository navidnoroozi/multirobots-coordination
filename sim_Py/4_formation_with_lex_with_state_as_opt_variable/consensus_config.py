from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from typing import Dict, List, Literal, Optional, Tuple

import numpy as np

ModelType = Literal["single_integrator", "double_integrator"]


def _pairwise_diameter(X: np.ndarray) -> float:
    if X.ndim != 2 or X.shape[0] == 0:
        return 0.0
    dmax = 0.0
    for i in range(X.shape[0]):
        for k in range(i + 1, X.shape[0]):
            d = float(np.linalg.norm(X[i] - X[k]))
            if d > dmax:
                dmax = d
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
    # --- Network ---
    n_agents: int = 4
    dim: int = 2

    # --- Model ---
    model: ModelType = "single_integrator"

    # --- DMPC ---
    outer_steps: int = 11
    auto_M: bool = True
    M_manual: int = 3
    alpha_gamma: float = 0.90

    # --- Constraints ---
    r_min: float = -5.0
    r_max: float = 5.0

    u_min: float = -2.0
    u_max: float = 2.0
    u_mag: float = 2.0

    v_min: float = -10.0
    v_max: float = 10.0

    # --- Cost weights ---
    w_track: float = 10.0
    w_du: float = 1.0
    w_u: float = 0.1
    w_v: float = 0.10

    # --- Lexicographic tie-break ---
    use_lexicographic: bool = True
    lex_cost_tol: float = 1e-6
    phi_tol: float = 1e-19
    diam_tol: float = 1e-15
    lex_only_if_phi_zero: bool = True

    # --- ZMQ / runtime ---
    plant_to_coord_rep: str = "tcp://127.0.0.1:5555"
    ctrl_base_port: int = 5600
    graph: Literal["ring_timevarying", "complete"] = "ring_timevarying"

    startup_delay_s: float = 2.0
    req_timeout_ms: int = 12000
    coord_controller_timeout_ms: int = 15000
    req_linger_ms: int = 0

    # --- Initial conditions ---
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

    # --- Solution 2: safe-formation redesign ---
    objective_mode: Literal["consensus", "safe_formation"] = "safe_formation"

    # desired safe spacing parameter (also used by plotting animation circles)
    d_safe: float = 0.90

    # regular-polygon formation shell
    formation_margin: float = 0.2
    formation_rotation_rad: float = 0.0
    formation_radius_override: float = 0.0  # if >0, max(auto_radius, override)

    # backward-compatible CLI fields
    safety_enabled: bool = True
    safety_method: str = "safe_formation"

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
        if self.model == "single_integrator":
            return np.array(self.r0_single, dtype=float)
        return np.array(self.r0_double, dtype=float)

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
            M = int(np.ceil(self.V0_single() / umin))
            return max(1, M)

        term1 = int(np.ceil((2.0 * float(self.v_max)) / umin))
        term2 = int(np.ceil(np.sqrt(max(self.Vr0_double(), 0.0) / umin)))
        return int(max(2, term1 + 2 * term2))

    def formation_offsets(self) -> np.ndarray:
        """
        Regular-polygon offsets c_i in R^dim with zero sum.
        For dim > 2, pad zeros in the extra coordinates.
        """
        n = self.n_agents
        d = self.dim
        if n <= 1:
            return np.zeros((n, d), dtype=float)

        # radius so that chord length >= d_safe + formation_margin
        denom = 2.0 * np.sin(np.pi / float(n))
        auto_radius = 0.5 * (self.d_safe + self.formation_margin) / max(denom, 1e-9)
        radius = max(auto_radius, float(self.formation_radius_override))

        angles = self.formation_rotation_rad + 2.0 * np.pi * np.arange(n) / float(n)
        C = np.zeros((n, d), dtype=float)
        C[:, 0] = radius * np.cos(angles)
        if d >= 2:
            C[:, 1] = radius * np.sin(angles)

        # numerically enforce zero sum
        C = C - np.mean(C, axis=0, keepdims=True)
        return C


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
        updates["safety_enabled"] = bool(v)

    sm = getattr(ns, "safety_method", None)
    if sm is not None:
        updates["safety_method"] = str(sm)

    return replace(cfg, **updates)


def parse_config_args(argv: Optional[list[str]] = None) -> tuple[NetConfig, argparse.Namespace]:
    parser = add_common_args(argparse.ArgumentParser())
    ns = parser.parse_args(argv)
    return config_from_namespace(ns), ns