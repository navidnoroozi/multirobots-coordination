# consensus_coordinator.py
from typing import Dict, List, Tuple

import numpy as np
import zmq

from consensus_config import NetConfig, controller_endpoint, time_varying_neighbors
from consensus_comm import make_envelope, dumps, loads

def main():
    cfg = NetConfig()
    ctx = zmq.Context.instance()

    # Coordinator REP (plant talks to this)
    rep = ctx.socket(zmq.REP)
    rep.bind(cfg.plant_to_coord_rep)
    print(f"[Coordinator] REP bound at {cfg.plant_to_coord_rep}")

    # Coordinator REQ sockets to controllers (one per agent)
    req_socks: Dict[int, zmq.Socket] = {}
    for i in range(1, cfg.n_agents + 1):
        s = ctx.socket(zmq.REQ)
        s.connect(controller_endpoint(cfg, i))
        req_socks[i] = s
        print(f"[Coordinator] Connected REQ -> Controller {i} at {controller_endpoint(cfg, i)}")

    # keep last applied control for Δu penalty
    u_prev = {i: np.zeros((cfg.dim,), dtype=float) for i in range(1, cfg.n_agents + 1)}

    outer_index = 0
    while True:
        msg = loads(rep.recv())
        mtype = msg.get("type")

        if mtype == "shutdown":
            # propagate shutdown
            for i, s in req_socks.items():
                s.send(dumps(make_envelope("shutdown", {}, src="coord", dst=f"ctrl{i}")))
                _ = loads(s.recv())
            rep.send(dumps(make_envelope("shutdown_ack", {"ok": True}, src="coord")))
            break

        if mtype != "plant_step":
            rep.send(dumps(make_envelope("error", {"error": "unknown msg type"}, src="coord")))
            continue

        p = msg["payload"]
        z_all = np.array(p["z_all"], dtype=float)  # shape (n_agents, dim)
        outer_index = int(p["outer_index"])

        nbrs = time_varying_neighbors(cfg.n_agents, outer_index)

        # query each controller
        replies = {}
        for i in range(1, cfg.n_agents + 1):
            z_i = z_all[i - 1, :].tolist()
            z_neighbors = [z_all[k - 1, :].tolist() for k in nbrs[i]]
            payload = {
                "outer_index": outer_index,
                "agent_id": i,
                "z_i": z_i,
                "z_neighbors": z_neighbors,
                "u_prev": u_prev[i].tolist(),
                "neighbors": nbrs[i],
            }
            req_socks[i].send(dumps(make_envelope("mpc_request", payload, src="coord", dst=f"ctrl{i}")))
            rep_i = loads(req_socks[i].recv())["payload"]
            replies[i] = rep_i

        # assemble M-step control sequences
        ok_all = all(replies[i].get("ok", False) for i in range(1, cfg.n_agents + 1))
        if not ok_all:
            # If any infeasible, reply with error and zero controls (safe fallback)
            U = np.zeros((cfg.M, cfg.n_agents, cfg.dim), dtype=float)
            out = {
                "ok": False,
                "outer_index": outer_index,
                "neighbors": nbrs,
                "U_seq": U.tolist(),
                "replies": replies,
                "note": "At least one agent infeasible; returning zeros.",
            }
            rep.send(dumps(make_envelope("coord_reply", out, src="coord")))
            continue

        U = np.zeros((cfg.M, cfg.n_agents, cfg.dim), dtype=float)
        costs = {}
        residuals = {}
        etas_used = {}

        for i in range(1, cfg.n_agents + 1):
            u_seq = np.array(replies[i]["u_seq"], dtype=float)  # (M,dim)
            U[:, i - 1, :] = u_seq
            costs[i] = float(replies[i]["objective"])
            residuals[i] = replies[i]["terminal_residual"]
            etas_used[i] = float(replies[i]["eta_used"])

            # update u_prev for next outer iteration as the last applied within this open-loop block
            u_prev[i] = u_seq[-1, :].copy()

        out = {
            "ok": True,
            "outer_index": outer_index,
            "neighbors": nbrs,
            "U_seq": U.tolist(),  # shape (M,n,dim)
            "costs": costs,
            "terminal_residuals": residuals,
            "eta_used": etas_used,
        }
        rep.send(dumps(make_envelope("coord_reply", out, src="coord")))

if __name__ == "__main__":
    main()
