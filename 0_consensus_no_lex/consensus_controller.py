# consensus_controller.py
import argparse
from typing import Dict, List, Tuple

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
    eta_try: float,
) -> Tuple[Dict, bool]:
    """
    Local M-step MPC for integrator dynamics x_{t+1} = x_t + u_t, t=0..M-1
    Terminal constraint: x_M = sum_k a_k * z_k  (convex combination of {i} U neighbors)
    Stage cost: sum_t w_track*||x_t - bary||^2 + w_u*||u_t||^2 + w_du*||u_t - u_{t-1}||^2
    Hard constraints on x,u.

    Returns (result_dict, feasible).
    """
    M = cfg.M
    d = cfg.dim

    # Build the point set S = {z_i} U neighbors
    S = [z_i] + list(z_neighbors)
    m = len(S)

    bary = np.mean(np.stack(S, axis=0), axis=0)  # local barycenter
    # decision variables
    x = cp.Variable((M + 1, d))
    u = cp.Variable((M, d))
    a = cp.Variable((m,))  # convex weights for terminal hull

    constraints = []
    # initial
    constraints += [x[0, :] == z_i]

    # dynamics + bounds
    for t in range(M):
        constraints += [x[t + 1, :] == x[t, :] + u[t, :]]
        constraints += clamp_box_constraints(x[t, :], cfg.x_min, cfg.x_max)
        constraints += clamp_box_constraints(u[t, :], cfg.u_min, cfg.u_max)
    constraints += clamp_box_constraints(x[M, :], cfg.x_min, cfg.x_max)

    # convex weights constraints (optionally strict)
    if eta_try > 0:
        constraints += [a >= eta_try]
    else:
        constraints += [a >= 0.0]
    constraints += [cp.sum(a) == 1.0]

    # terminal hull inclusion
    S_mat = np.stack(S, axis=0)  # (m,d)
    constraints += [x[M, :] == a @ S_mat]  # a (1xm) times (mxd) -> (d,)

    # cost
    cost = 0
    for t in range(M):
        cost += cfg.w_track * cp.sum_squares(x[t, :] - bary)
        cost += cfg.w_u * cp.sum_squares(u[t, :])
        if t == 0:
            cost += cfg.w_du * cp.sum_squares(u[t, :] - u_prev)
        else:
            cost += cfg.w_du * cp.sum_squares(u[t, :] - u[t - 1, :])

    prob = cp.Problem(cp.Minimize(cost), constraints)

    # Solve
    try:
        prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
    except Exception:
        # fallback solver
        try:
            prob.solve(solver=cp.ECOS, warm_start=True, verbose=False)
        except Exception:
            return {}, False

    if prob.status not in ("optimal", "optimal_inaccurate"):
        return {}, False

    x_val = np.array(x.value)
    u_val = np.array(u.value)
    a_val = np.array(a.value).reshape(-1)

    terminal_residual = x_val[M, :] - (a_val @ S_mat)

    out = {
        "agent_id": agent_id,
        "barycenter": bary.tolist(),
        "x_pred": x_val.tolist(),
        "u_seq": u_val.tolist(),  # length M
        "a_weights": a_val.tolist(),
        "eta_used": float(eta_try),
        "terminal_residual": terminal_residual.tolist(),
        "objective": float(prob.value),
        "neighbors_count": int(len(z_neighbors)),
    }
    return out, True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, required=True, help="Agent id (1..4)")
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

        # strict weights attempt -> fallback to non-strict
        eta0 = cfg.eta if cfg.use_strict_weights else 0.0
        res, ok = solve_local_mpc(cfg, agent_id, z_i, z_neighbors, u_prev, eta_try=eta0)
        if not ok and eta0 > 0:
            res, ok = solve_local_mpc(cfg, agent_id, z_i, z_neighbors, u_prev, eta_try=0.0)

        if not ok:
            sock.send(dumps(make_envelope(
                "mpc_reply",
                {"ok": False, "agent_id": agent_id, "reason": "infeasible"},
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
