# consensus_config.py
from dataclasses import dataclass
from typing import Dict, List, Tuple

@dataclass(frozen=True)
class NetConfig:
    n_agents: int = 4
    dim: int = 2  # 2D states to make convex-hull geometry meaningful
    M: int = 3    # multi-step horizon applied open-loop
    outer_steps: int = 35  # number of MPC re-optimizations

    # Simple integrator dynamics: x+ = x + u
    # Hard constraints
    x_min: float = -5.0
    x_max: float = 5.0
    u_min: float = -1.0
    u_max: float = 1.0

    # Cost weights
    w_track: float = 10.0   # ||x - barycenter||^2
    w_du: float = 1.0       # ||u_t - u_{t-1}||^2
    w_u: float = 0.05       # ||u||^2

    # Strict convexity weights lower bound
    use_strict_weights: bool = True
    eta: float = 0.05       # if strict, enforce a_k >= eta; fallback to eta=0 if infeasible

    # ZMQ endpoints
    plant_to_coord_rep: str = "tcp://127.0.0.1:5555"  # coordinator REP binds here
    ctrl_base_port: int = 5600  # controllers REP bind at tcp://127.0.0.1:(ctrl_base_port + id)

def controller_endpoint(cfg: NetConfig, agent_id_1based: int) -> str:
    return f"tcp://127.0.0.1:{cfg.ctrl_base_port + agent_id_1based}"

def ring_neighbors(n: int) -> Dict[int, List[int]]:
    """
    1-based node indexing. Ring graph:
      i connected to i-1 and i+1 (wrap-around).
    """
    nbrs: Dict[int, List[int]] = {}
    for i in range(1, n + 1):
        left = i - 1 if i > 1 else n
        right = i + 1 if i < n else 1
        nbrs[i] = [left, right]
    return nbrs

def time_varying_neighbors(n: int, j: int) -> Dict[int, List[int]]:
    """
    Mild time-variation:
    - baseline ring
    - every 5th outer step, drop one direction for agent 2 (simulate link variation)
    Still keeps a spanning tree (in this ring it will).
    """
    nbrs = ring_neighbors(n)
    if j % 5 == 0:
        # drop agent 2's right neighbor temporarily
        if 3 in nbrs[2]:
            nbrs[2] = [k for k in nbrs[2] if k != 3]
    return nbrs
