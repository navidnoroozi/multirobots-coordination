# consensus_config.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Literal, Optional, Tuple

import numpy as np

ModelType = Literal["single_integrator", "double_integrator"]


def _pairwise_diameter(X: np.ndarray) -> float:
    """
    Diameter of a finite set of points: max_{i,k} ||X_i - X_k||_2.
    X shape: (N, d)
    """
    if X.ndim != 2 or X.shape[0] == 0:
        return 0.0
    # O(N^2) is fine for small N=4..20; keep it simple & robust.
    dmax = 0.0
    for i in range(X.shape[0]):
        for k in range(i + 1, X.shape[0]):
            d = float(np.linalg.norm(X[i] - X[k]))
            if d > dmax:
                dmax = d
    return dmax


@dataclass(frozen=True)
class NetConfig:
    # --- Network ---
    n_agents: int = 4
    dim: int = 2  # position dimension

    # --- Model selection ---
    model: ModelType = "double_integrator" #  "single_integrator" # or "double_integrator"

    # --- MPC / Algorithm 2 ---
    outer_steps: int = 11  # number of MPC re-optimizations (outer steps)
    auto_M: bool = True
    M_manual: int = 3  # used only if auto_M=False

    # Terminal contraction factor used in OCP terminal inequality:
    # ||r_M - bary|| <= alpha_gamma * ||r_0 - bary||  (positions)
    alpha_gamma: float = 0.80

    # --- Constraints (positions) ---
    r_min: float = -5.0
    r_max: float = 5.0

    # --- Constraints (control / acceleration) ---
    # Componentwise box bounds: u in [u_min, u_max]^dim
    u_min: float = -1.0
    u_max: float = 1.0

    # A positive "control authority" scalar used in Corollary 2/3 formulas.
    # For symmetric bounds [-u_max, u_max], a conservative choice is u_mag=u_max.
    u_mag: float = 1.0

    # --- Double integrator (velocity constraints) ---
    v_min: float = -2.0
    v_max: float = 2.0

    # --- Cost weights (applied per inner stage) ---
    w_track: float = 10.0   # ||r_t - bary||^2
    w_du: float = 1.0       # ||u_t - u_{t-1}||^2
    w_u: float = 0.0       # ||u_t||^2
    w_v: float = 0.10       # (double integrator) ||v_t||^2 (optional regularization)

    # --- Lexicographic tie-break ---
    use_lexicographic: bool = True
    lex_cost_tol: float = 1e-6  # allow tiny slack in primary objective

    # --- Lex activation based on boundary + non-singleton hull ---
    phi_tol: float = 1e-19 # Thereshold for "close to boundary" in phi; also used for lex_only_if_phi_zero gating
    diam_tol: float = 1e-15 # Threshold for detecting non-singleton hull (e.g., due to numerical issues)
    lex_only_if_phi_zero: bool = True  # keep name; means "phi <= phi_tol" gating is enabled

    # --- ZMQ endpoints ---
    plant_to_coord_rep: str = "tcp://127.0.0.1:5555"  # coordinator REP binds here
    ctrl_base_port: int = 5600  # controllers REP bind at tcp://127.0.0.1:(ctrl_base_port + id)

    # --- Graph ---
    # Use a mild time-varying ring by default (still spanning-tree connected).
    graph: Literal["ring_timevarying", "complete"] = "ring_timevarying"

    # --- Initial conditions ---
    # Single integrator uses r0_single (shape (n, dim))
    r0_single: Tuple[Tuple[float, float], ...] = (
        (-4.0,  2.0),
        ( 3.5,  4.0),
        ( 4.5, -3.5),
        (-2.5, -4.0),
    )

    # Double integrator uses (r0_double, v0_double)
    r0_double: Tuple[Tuple[float, float], ...] = (
        (-4.0,  2.0),
        ( 3.5,  4.0),
        ( 4.5, -3.5),
        (-2.5, -4.0),
    )
    v0_double: Tuple[Tuple[float, float], ...] = (
        ( 1.0,  0.0),
        ( 0.0, -1.0),
        (-1.0,  0.0),
        ( 0.0,  1.0),
    )

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

        # ring_timevarying
        nbrs = ring_neighbors()
        if outer_j % 5 == 0 and n >= 3:
            # drop one edge temporarily but keep spanning connectivity
            if 3 in nbrs[2]:
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
        """V(z(0)) for Corollary 2: diameter of positions."""
        return _pairwise_diameter(self.initial_positions())

    def Vr0_double(self) -> float:
        """V_r(0) for Corollary 3: diameter of positions."""
        return _pairwise_diameter(self.initial_positions())

    def horizon_M(self) -> int:
        """Auto-select horizon M according to Corollary 2/3."""
        if not self.auto_M:
            return int(self.M_manual)

        umin = float(self.u_mag)
        if umin <= 0:
            # Fall back to manual if misconfigured; but keep deterministic.
            return int(self.M_manual)

        if self.model == "single_integrator":
            V0 = self.V0_single()
            M = int(np.ceil(V0 / umin))
            return max(1, M)

        # double_integrator
        Vr0 = self.Vr0_double()
        vmax = float(self.v_max)
        term1 = int(np.ceil((2.0 * vmax) / umin))
        term2 = int(np.ceil(np.sqrt(max(Vr0, 0.0) / umin)))
        M = max(2, term1 + 2 * term2)
        return int(M)