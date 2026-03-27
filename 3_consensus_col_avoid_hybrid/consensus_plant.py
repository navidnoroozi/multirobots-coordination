from __future__ import annotations

import argparse
from typing import Dict

import numpy as np
import zmq

from consensus_comm import dumps, loads, make_envelope
from consensus_config import add_common_args, config_from_namespace


def _clip(x: np.ndarray, lo: float, hi: float) -> np.ndarray:
    return np.minimum(np.maximum(x, lo), hi)


def _pairwise_diameter(X: np.ndarray) -> float:
    if X.ndim != 2 or X.shape[0] <= 1:
        return 0.0
    dmax = 0.0
    for i in range(X.shape[0]):
        for k in range(i + 1, X.shape[0]):
            dmax = max(dmax, float(np.linalg.norm(X[i] - X[k])))
    return dmax


def _pairwise_min_distance(X: np.ndarray) -> float:
    if X.ndim != 2 or X.shape[0] <= 1:
        return float("inf")
    dmin = float("inf")
    for i in range(X.shape[0]):
        for k in range(i + 1, X.shape[0]):
            dmin = min(dmin, float(np.linalg.norm(X[i] - X[k])))
    return dmin


def _make_req_socket(ctx: zmq.Context, endpoint: str, timeout_ms: int, linger_ms: int) -> zmq.Socket:
    s = ctx.socket(zmq.REQ)
    s.setsockopt(zmq.LINGER, linger_ms)
    s.setsockopt(zmq.REQ_RELAXED, 1)
    s.setsockopt(zmq.REQ_CORRELATE, 1)
    s.RCVTIMEO = timeout_ms
    s.SNDTIMEO = timeout_ms
    s.connect(endpoint)
    return s


def _reset_socket(ctx: zmq.Context, sock: zmq.Socket, endpoint: str, timeout_ms: int, linger_ms: int) -> zmq.Socket:
    try:
        sock.close(0)
    except Exception:
        pass
    return _make_req_socket(ctx, endpoint, timeout_ms, linger_ms)


def main() -> None:
    parser = add_common_args(argparse.ArgumentParser())
    ns = parser.parse_args()
    cfg = config_from_namespace(ns)
    M = cfg.horizon_M()
    print(f"[Plant] model={cfg.model} | auto_M={cfg.auto_M} | M={M} | outer_steps={cfg.outer_steps} | hybrid enabled={cfg.hybrid_enabled}")

    ctx = zmq.Context.instance()
    coord_req = _make_req_socket(ctx, cfg.plant_to_coord_rep, cfg.req_timeout_ms, cfg.req_linger_ms)
    print(f"[Plant] REQ -> Coordinator at {cfg.plant_to_coord_rep}")

    hybrid_reqs: Dict[int, zmq.Socket] = {}
    hybrid_eps: Dict[int, str] = {}
    for i in range(1, cfg.n_agents + 1):
        ep = cfg.controller_endpoint(i)
        hybrid_eps[i] = ep
        hybrid_reqs[i] = _make_req_socket(ctx, ep, cfg.hybrid_timeout_ms, cfg.req_linger_ms)
        print(f"[Plant] REQ -> controller_node{i} hybrid endpoint {ep}")

    logger = ctx.socket(zmq.PUSH)
    logger.setsockopt(zmq.LINGER, cfg.req_linger_ms)
    logger.connect(cfg.logger_endpoint)
    print(f"[Plant] PUSH -> Logger at {cfg.logger_endpoint}")

    r = cfg.initial_positions().copy()
    v = cfg.initial_velocities().copy()
    k_global = 0

    for outer_j in range(cfg.outer_steps):
        try:
            coord_req.send(dumps(make_envelope("plant_step", {"outer_index": outer_j, "r_all": r.tolist(), "v_all": v.tolist()}, src="plant", dst="coord")))
            reply = loads(coord_req.recv())["payload"]
        except Exception as e:
            print(f"[Plant] outer_j={outer_j}: coordinator error {e}. Using zero inputs.")
            coord_req = _reset_socket(ctx, coord_req, cfg.plant_to_coord_rep, cfg.req_timeout_ms, cfg.req_linger_ms)
            reply = {"ok": False, "U_seq": np.zeros((M, cfg.n_agents, cfg.dim)).tolist(), "diag": {}}

        U = np.array(reply.get("U_seq", np.zeros((M, cfg.n_agents, cfg.dim))), dtype=float)
        diag = reply.get("diag", {}) if isinstance(reply, dict) else {}

        for i in range(1, cfg.n_agents + 1):
            rt = diag.get("r_term", {}).get(i, diag.get("r_term", {}).get(str(i), [np.nan, np.nan])) if isinstance(diag, dict) else [np.nan, np.nan]
            row = [outer_j, i, cfg.model, M,
                   diag.get("objective_primary", {}).get(i, diag.get("objective_primary", {}).get(str(i), np.nan)) if isinstance(diag, dict) else np.nan,
                   int(bool(diag.get("lex_used", {}).get(i, diag.get("lex_used", {}).get(str(i), 0)))) if isinstance(diag, dict) else 0,
                   diag.get("phi_terminal", {}).get(i, diag.get("phi_terminal", {}).get(str(i), np.nan)) if isinstance(diag, dict) else np.nan,
                   int(bool(diag.get("ri_terminal", {}).get(i, diag.get("ri_terminal", {}).get(str(i), 0)))) if isinstance(diag, dict) else 0,
                   float(rt[0]) if isinstance(rt, (list, tuple)) and len(rt) >= 2 else np.nan,
                   float(rt[1]) if isinstance(rt, (list, tuple)) and len(rt) >= 2 else np.nan,
                   diag.get("diam_C", {}).get(i, diag.get("diam_C", {}).get(str(i), np.nan)) if isinstance(diag, dict) else np.nan,
                   diag.get("t_primary_ms", {}).get(i, diag.get("t_primary_ms", {}).get(str(i), np.nan)) if isinstance(diag, dict) else np.nan,
                   diag.get("t_lex_ms", {}).get(i, diag.get("t_lex_ms", {}).get(str(i), np.nan)) if isinstance(diag, dict) else np.nan,
                   diag.get("t_total_ms", {}).get(i, diag.get("t_total_ms", {}).get(str(i), np.nan)) if isinstance(diag, dict) else np.nan,
                   diag.get("n_var_primary", {}).get(i, diag.get("n_var_primary", {}).get(str(i), 0)) if isinstance(diag, dict) else 0,
                   diag.get("n_eq_primary", {}).get(i, diag.get("n_eq_primary", {}).get(str(i), 0)) if isinstance(diag, dict) else 0,
                   diag.get("n_ineq_primary", {}).get(i, diag.get("n_ineq_primary", {}).get(str(i), 0)) if isinstance(diag, dict) else 0,
                   diag.get("n_var_lex", {}).get(i, diag.get("n_var_lex", {}).get(str(i), 0)) if isinstance(diag, dict) else 0,
                   diag.get("n_eq_lex", {}).get(i, diag.get("n_eq_lex", {}).get(str(i), 0)) if isinstance(diag, dict) else 0,
                   diag.get("n_ineq_lex", {}).get(i, diag.get("n_ineq_lex", {}).get(str(i), 0)) if isinstance(diag, dict) else 0]
            logger.send(dumps(make_envelope("log_outer", {"row": row}, src="plant", dst="logger")))

        for inner_t in range(M):
            u_nom = _clip(U[inner_t, :, :], cfg.u_min, cfg.u_max)
            U_cmd = u_nom.copy()
            mode_counts = {1: 0, 2: 0, 3: 0}
            n_switched = 0
            diags: Dict[int, Dict] = {}

            if cfg.hybrid_enabled:
                for i in range(1, cfg.n_agents + 1):
                    payload = {"agent_id": i, "r_all": r.tolist(), "v_all": v.tolist(), "u_nom_all": u_nom.tolist()}
                    try:
                        hybrid_reqs[i].send(dumps(make_envelope("hybrid_request", payload, src="plant", dst=f"controller_node{i}")))
                        rep_i = loads(hybrid_reqs[i].recv())["payload"]
                        U_cmd[i - 1, :] = np.array(rep_i["u_cmd"], dtype=float)
                        diags[i] = rep_i.get("diag", {})
                    except Exception:
                        hybrid_reqs[i] = _reset_socket(ctx, hybrid_reqs[i], hybrid_eps[i], cfg.hybrid_timeout_ms, cfg.req_linger_ms)
                        U_cmd[i - 1, :] = u_nom[i - 1, :]
                        diags[i] = {"mode": 1, "mode_name": "nominal", "switched": False, "mode_steps": 0, "dmin_local": np.nan, "local_diameter": np.nan, "n_warn_neighbors": 0, "anchor_norm": 0.0}
            else:
                for i in range(1, cfg.n_agents + 1):
                    diags[i] = {"mode": 1, "mode_name": "nominal", "switched": False, "mode_steps": 0, "dmin_local": np.nan, "local_diameter": np.nan, "n_warn_neighbors": 0, "anchor_norm": 0.0}

            u = _clip(U_cmd, cfg.u_min, cfg.u_max)
            if cfg.model == "single_integrator":
                r = _clip(r + cfg.dt * u, cfg.r_min, cfg.r_max)
                v = np.zeros_like(v)
            else:
                r = _clip(r + cfg.dt * v, cfg.r_min, cfg.r_max)
                v = _clip(v + cfg.dt * u, cfg.v_min, cfg.v_max)

            Vr = _pairwise_diameter(r)
            Vv = _pairwise_diameter(v)
            speeds = np.linalg.norm(v, axis=1)
            max_speed = float(np.max(speeds)) if speeds.size else 0.0
            mean_speed = float(np.mean(speeds)) if speeds.size else 0.0
            dmin = _pairwise_min_distance(r)

            for i in range(1, cfg.n_agents + 1):
                d = diags[i]
                mode = int(d.get("mode", 1))
                mode_counts[mode] = mode_counts.get(mode, 0) + 1
                n_switched += int(bool(d.get("switched", False)))
                logger.send(dumps(make_envelope("log_mode", {"row": [k_global, outer_j, inner_t, i, mode, d.get("mode_name", "nominal"), int(bool(d.get("switched", False))), int(d.get("mode_steps", 0)), float(d.get("dmin_local", np.nan)), float(d.get("local_diameter", np.nan)), int(d.get("n_warn_neighbors", 0)), float(d.get("anchor_norm", 0.0))]}, src="plant", dst="logger")))

            logger.send(dumps(make_envelope("log_metrics", {"row": [k_global, outer_j, inner_t, Vr, Vv, max_speed, mean_speed, dmin, mode_counts.get(1, 0), mode_counts.get(2, 0), mode_counts.get(3, 0), n_switched]}, src="plant", dst="logger")))

            for i in range(cfg.n_agents):
                logger.send(dumps(make_envelope("log_traj", {"row": [k_global, outer_j, inner_t, i + 1, r[i, 0], r[i, 1], v[i, 0], v[i, 1], u[i, 0], u[i, 1]]}, src="plant", dst="logger")))

            k_global += 1

        print(f"[Plant] finished outer_j={outer_j + 1}/{cfg.outer_steps} | Vr={Vr:.3f} | dmin={dmin:.3f}")

    try:
        coord_req.send(dumps(make_envelope("shutdown", {}, src="plant", dst="coord")))
        _ = loads(coord_req.recv())
        print("[Plant] Shutdown complete.")
    except Exception:
        print("[Plant] Shutdown: coordinator did not respond (ok to ignore).")
    try:
        logger.send(dumps(make_envelope("shutdown", {}, src="plant", dst="logger")))
    except Exception:
        pass


if __name__ == "__main__":
    main()