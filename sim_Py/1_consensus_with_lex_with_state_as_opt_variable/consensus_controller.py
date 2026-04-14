# consensus_controller.py
import argparse
from typing import Dict, List, Tuple, Any

import numpy as np
import zmq
import cvxpy as cp

from consensus_config import NetConfig, controller_endpoint
from consensus_comm import make_envelope, dumps, loads

def clamp_box_constraints(x_var, x_min, x_max):
    return [x_var >= x_min, x_var <= x_max]

def solve_local_mpc(
    cfg: NetConfig,
    agent_id: int,
    z_i: np.ndarray,
    z_neighbors: List[np.ndarray],
    u_prev: np.ndarray,
) -> Tuple[Dict[str, Any], bool]:
    """
    Local M-step MPC for integrator dynamics x_{t+1} = x_t + u_t, t=0..M-1.

    Hard constraints:
      - box constraints on x and u
      - terminal convex-hull inclusion: x_M = sum_k a_k z_k, a>=0, sum(a)=1

    Added in this version:
      - finite-step terminal inequality:
            ||x_M - bary|| <= alpha_gamma * ||x_0 - bary||
      - lexicographic (tie-breaking) selection:
            Stage 1: minimize primary cost
            Stage 2: among solutions with cost <= J* + tol, maximize t = min_k a_k
        This does NOT shrink the feasible set; it only selects among (near-)optimal solutions.

    Returns (result_dict, feasible_ok).
    """
    M = cfg.M
    d = cfg.dim

    S = [z_i] + list(z_neighbors)
    m = len(S)
    S_mat = np.stack(S, axis=0)  # (m,d)

    bary = np.mean(S_mat, axis=0)  # local barycenter in R^d
    rhs0 = float(np.linalg.norm(z_i - bary))  # ||x_0 - bary||, since x_0 == z_i
    rhs = cfg.alpha_gamma * rhs0

    # decision variables
    x = cp.Variable((M + 1, d))
    u = cp.Variable((M, d))
    a = cp.Variable((m,))         # convex weights for terminal hull
    tmin = cp.Variable()          # min weight proxy for interiority (stage-2 only)

    constraints: List[Any] = []
    constraints += [x[0, :] == z_i]

    for k in range(M):
        constraints += [x[k + 1, :] == x[k, :] + u[k, :]]
        constraints += clamp_box_constraints(x[k, :], cfg.x_min, cfg.x_max)
        constraints += clamp_box_constraints(u[k, :], cfg.u_min, cfg.u_max)
    constraints += clamp_box_constraints(x[M, :], cfg.x_min, cfg.x_max)

    # convex weights (NO strict lower bounds here)
    constraints += [a >= 0.0, cp.sum(a) == 1.0]

    # terminal hull inclusion
    constraints += [x[M, :] == a @ S_mat]

    # finite-step terminal inequality (SOC)
    # If rhs0==0 this forces x_M == bary (still convex).
    constraints += [cp.norm(x[M, :] - bary, 2) <= rhs]

    # primary cost
    primary_cost = 0
    for k in range(M):
        primary_cost += cfg.w_track * cp.sum_squares(x[k, :] - bary)
        primary_cost += cfg.w_u * cp.sum_squares(u[k, :])
        if k == 0:
            primary_cost += cfg.w_du * cp.sum_squares(u[k, :] - u_prev)
        else:
            primary_cost += cfg.w_du * cp.sum_squares(u[k, :] - u[k - 1, :])

    # -------- Stage 1: primary solve --------
    prob1 = cp.Problem(cp.Minimize(primary_cost), constraints)
    try:
        prob1.solve(solver=cp.ECOS, warm_start=True, verbose=False)
    except Exception:
        try:
            prob1.solve(solver=cp.SCS, warm_start=True, verbose=False)
        except Exception:
            return {}, False

    if prob1.status not in ("optimal", "optimal_inaccurate"):
        return {}, False

    J_star = float(prob1.value)

    # save stage-1 solution in case stage-2 fails
    x1 = np.array(x.value)
    u1 = np.array(u.value)
    a1 = np.array(a.value).reshape(-1)

    # -------- Stage 2: lexicographic tie-break (optional) --------
    lex_used = False
    t_val = float(np.min(a1)) if a1.size else 0.0

    if cfg.use_lexicographic:
        lex_constraints = list(constraints)
        # keep within primary optimum set up to tolerance
        lex_constraints += [primary_cost <= J_star + cfg.lex_cost_tol]
        # maximize min weight
        lex_constraints += [tmin >= 0.0, a >= tmin]
        prob2 = cp.Problem(cp.Maximize(tmin), lex_constraints)
        try:
            prob2.solve(solver=cp.ECOS, warm_start=True, verbose=False)
        except Exception:
            try:
                prob2.solve(solver=cp.SCS, warm_start=True, verbose=False)
            except Exception:
                prob2 = None

        if prob2 is not None and prob2.status in ("optimal", "optimal_inaccurate"):
            lex_used = True
            x2 = np.array(x.value)
            u2 = np.array(u.value)
            a2 = np.array(a.value).reshape(-1)
            # replace with stage-2 solution
            x1, u1, a1 = x2, u2, a2
            t_val = float(tmin.value)

    terminal_residual = x1[M, :] - (a1 @ S_mat)

    out = {
        "agent_id": int(agent_id),
        "barycenter": bary.tolist(),
        "x_pred": x1.tolist(),
        "u_seq": u1.tolist(),
        "a_weights": a1.tolist(),
        "t_min_weight": float(t_val),
        "lex_used": bool(lex_used),
        "alpha_gamma": float(cfg.alpha_gamma),
        "terminal_residual": terminal_residual.tolist(),
        "objective": float(J_star),
        "neighbors_count": int(len(z_neighbors)),
        "rhs0_norm_x0_minus_bary": float(rhs0),
        "rhs_norm_bound": float(rhs),
    }
    return out, True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, required=True, help="Agent id (1..n)")
    args = parser.parse_args()

    cfg = NetConfig()
    agent_id = args.id
    endpoint = controller_endpoint(cfg, agent_id)

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REP)
    sock.bind(endpoint)
    print(f"[Controller {agent_id}] REP bound at {endpoint}")

    while True:
        msg = loads(sock.recv())
        if msg.get("type") == "shutdown":
            sock.send(dumps(make_envelope("shutdown_ack", {"ok": True}, src=f"ctrl{agent_id}")))
            break

        if msg.get("type") != "mpc_request":
            sock.send(dumps(make_envelope("error", {"error": "unknown msg type"}, src=f"ctrl{agent_id}")))
            continue

        p = msg["payload"]
        z_i = np.array(p["z_i"], dtype=float)
        z_neighbors = [np.array(v, dtype=float) for v in p["z_neighbors"]]
        u_prev = np.array(p["u_prev"], dtype=float)

        res, ok = solve_local_mpc(cfg, agent_id, z_i, z_neighbors, u_prev)

        if not ok:
            sock.send(dumps(make_envelope(
                "mpc_reply",
                {"ok": False, "agent_id": int(agent_id), "reason": "infeasible_or_solver_failed"},
                src=f"ctrl{agent_id}"
            )))
        else:
            sock.send(dumps(make_envelope(
                "mpc_reply",
                {"ok": True, **res},
                src=f"ctrl{agent_id}"
            )))

if __name__ == "__main__":
    main()
