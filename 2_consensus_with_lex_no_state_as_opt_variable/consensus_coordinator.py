# consensus_coordinator.py
from __future__ import annotations

from typing import Any, Dict

import numpy as np
import zmq

from consensus_comm import dumps, loads, make_envelope
from consensus_config import NetConfig


def main() -> None:
    cfg = NetConfig()
    M = cfg.horizon_M()

    ctx = zmq.Context.instance()

    rep = ctx.socket(zmq.REP)
    rep.bind(cfg.plant_to_coord_rep)
    rep.RCVTIMEO = 10_000
    rep.SNDTIMEO = 10_000
    print(f"[Coordinator] REP bound at {cfg.plant_to_coord_rep} | model={cfg.model} | M={M}")

    req_socks: Dict[int, zmq.Socket] = {}
    for i in range(1, cfg.n_agents + 1):
        s = ctx.socket(zmq.REQ)
        s.connect(cfg.controller_endpoint(i))
        s.RCVTIMEO = 5_000
        s.SNDTIMEO = 5_000
        req_socks[i] = s
        print(f"[Coordinator] REQ -> ctrl{i} at {cfg.controller_endpoint(i)}")

    u_prev = {i: np.zeros((cfg.dim,), dtype=float) for i in range(1, cfg.n_agents + 1)}

    while True:
        try:
            msg = loads(rep.recv())
        except zmq.Again:
            print("[Coordinator] Waiting for plant... (no message yet)")
            continue

        mtype = msg.get("type")

        if mtype == "shutdown":
            print("[Coordinator] Shutdown requested by plant.")
            for i, s in req_socks.items():
                try:
                    s.send(dumps(make_envelope("shutdown", {}, src="coord", dst=f"ctrl{i}")))
                    _ = loads(s.recv())
                except Exception:
                    pass
            rep.send(dumps(make_envelope("shutdown_ack", {"ok": True}, src="coord")))
            break

        if mtype != "plant_step":
            rep.send(dumps(make_envelope("error", {"error": "unknown msg type"}, src="coord")))
            continue

        p = msg["payload"]
        outer_j = int(p["outer_index"])

        r_all = np.array(p["r_all"], dtype=float)
        v_all = np.array(p["v_all"], dtype=float)

        nbrs = cfg.neighbors(outer_j)

        replies: Dict[int, Dict[str, Any]] = {}
        missing: Dict[int, str] = {}

        for i in range(1, cfg.n_agents + 1):
            r_i = r_all[i - 1, :].tolist()
            v_i = v_all[i - 1, :].tolist()
            r_neighbors = [r_all[k - 1, :].tolist() for k in nbrs[i]]

            payload = {
                "outer_index": outer_j,
                "agent_id": i,
                "r_i": r_i,
                "v_i": v_i,
                "r_neighbors": r_neighbors,
                "u_prev": u_prev[i].tolist(),
                "neighbors": nbrs[i],
            }

            try:
                req_socks[i].send(dumps(make_envelope("mpc_request", payload, src="coord", dst=f"ctrl{i}")))
                rep_i = loads(req_socks[i].recv())["payload"]
                replies[i] = rep_i
            except zmq.Again:
                missing[i] = "timeout"
                replies[i] = {"ok": False, "agent_id": i, "reason": "timeout"}
            except Exception as e:
                missing[i] = f"error: {e}"
                replies[i] = {"ok": False, "agent_id": i, "reason": f"error: {e}"}

        ok_all = all(bool(replies[i].get("ok", False)) for i in range(1, cfg.n_agents + 1))
        if not ok_all:
            if missing:
                print(f"[Coordinator] outer_j={outer_j}: missing/failed controllers: {missing}")

            U = np.zeros((M, cfg.n_agents, cfg.dim), dtype=float)
            rep.send(
                dumps(
                    make_envelope(
                        "coord_reply",
                        {
                            "ok": False,
                            "outer_index": outer_j,
                            "neighbors": nbrs,
                            "model": cfg.model,
                            "M": M,
                            "U_seq": U.tolist(),
                            "replies": replies,
                            "note": "At least one agent infeasible/missing; returning zeros.",
                        },
                        src="coord",
                    )
                )
            )
            continue

        U = np.zeros((M, cfg.n_agents, cfg.dim), dtype=float)

        diag_objective_primary = {}
        diag_lex_used = {}
        diag_phi_terminal = {}
        diag_ri_terminal = {}
        diag_r_term = {}
        diag_diam_C = {}
        diag_t_primary_ms = {}
        diag_t_lex_ms = {}
        diag_t_total_ms = {}
        diag_n_var_primary = {}
        diag_n_eq_primary = {}
        diag_n_ineq_primary = {}
        diag_n_var_lex = {}
        diag_n_eq_lex = {}
        diag_n_ineq_lex = {}

        for i in range(1, cfg.n_agents + 1):
            rep_i = replies[i]

            # controls
            u_seq = np.array(rep_i["u_seq"], dtype=float)  # (M,dim)
            U[:, i - 1, :] = u_seq

            # diagnostics (NEW schema)
            diag_objective_primary[i] = float(rep_i.get("objective_primary", np.nan))
            diag_lex_used[i] = bool(rep_i.get("lex_used", False))
            diag_phi_terminal[i] = float(rep_i.get("phi_terminal", np.nan))
            diag_ri_terminal[i] = bool(rep_i.get("ri_terminal", False))
            diag_r_term[i] = rep_i.get("r_term", [np.nan, np.nan])
            diag_diam_C[i] = float(rep_i.get("diam_C", np.nan))

            diag_t_primary_ms[i] = float(rep_i.get("t_primary_ms", np.nan))
            diag_t_lex_ms[i] = float(rep_i.get("t_lex_ms", np.nan))
            diag_t_total_ms[i] = float(rep_i.get("t_total_ms", np.nan))

            diag_n_var_primary[i] = int(rep_i.get("n_var_primary", 0))
            diag_n_eq_primary[i]  = int(rep_i.get("n_eq_primary", 0))
            diag_n_ineq_primary[i]= int(rep_i.get("n_ineq_primary", 0))

            diag_n_var_lex[i] = int(rep_i.get("n_var_lex", 0))
            diag_n_eq_lex[i]  = int(rep_i.get("n_eq_lex", 0))
            diag_n_ineq_lex[i]= int(rep_i.get("n_ineq_lex", 0))

            # update u_prev for next outer iteration
            u_prev[i] = u_seq[-1, :].copy()

        out = {
            "ok": True,
            "outer_index": outer_j,
            "neighbors": nbrs,
            "U_seq": U.tolist(),
            "diag": {
                "objective_primary": diag_objective_primary,
                "lex_used": diag_lex_used,
                "phi_terminal": diag_phi_terminal,
                "ri_terminal": diag_ri_terminal,
                "r_term": diag_r_term,
                "diam_C": diag_diam_C,
                "t_primary_ms": diag_t_primary_ms,
                "t_lex_ms": diag_t_lex_ms,
                "t_total_ms": diag_t_total_ms,
                "n_var_primary": diag_n_var_primary,
                "n_eq_primary": diag_n_eq_primary,
                "n_ineq_primary": diag_n_ineq_primary,
                "n_var_lex": diag_n_var_lex,
                "n_eq_lex": diag_n_eq_lex,
                "n_ineq_lex": diag_n_ineq_lex,
            },
        }
        rep.send(dumps(make_envelope("coord_reply", out, src="coord")))


if __name__ == "__main__":
    main()