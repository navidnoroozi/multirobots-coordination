from __future__ import annotations

import argparse
import csv
from pathlib import Path
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


def _global_obstacle_distance(cfg, r: np.ndarray) -> float:
    if not cfg.obstacles_enabled or len(cfg.obstacles_circles) == 0:
        return float("inf")
    dmin = float("inf")
    for p in r:
        for (cx, cy, rad) in cfg.obstacles_circles:
            dd = float(np.linalg.norm(p[:2] - np.array([cx, cy], dtype=float)) - (rad + cfg.obstacle_margin))
            dmin = min(dmin, dd)
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


def _get_keyed(d: dict, i: int, default):
    if not isinstance(d, dict):
        return default
    if i in d:
        return d[i]
    si = str(i)
    if si in d:
        return d[si]
    return default


def main() -> None:
    parser = add_common_args(argparse.ArgumentParser())
    ns = parser.parse_args()
    cfg = config_from_namespace(ns)
    M = cfg.horizon_M()

    print(
        f"[Plant] model={cfg.model} | auto_M={cfg.auto_M} | M={M} | "
        f"outer_steps={cfg.outer_steps} | safety={cfg.safety_method} enabled={cfg.safety_enabled}"
    )

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

    r = cfg.initial_positions().copy()
    v = cfg.initial_velocities().copy()

    traj_path = Path("consensus_traj.csv")
    metrics_path = Path("consensus_metrics.csv")
    outer_path = Path("consensus_outer.csv")
    hybrid_modes_path = Path("hybrid_modes.csv")

    with (
        traj_path.open("w", newline="") as f_traj,
        metrics_path.open("w", newline="") as f_met,
        outer_path.open("w", newline="") as f_out,
        hybrid_modes_path.open("w", newline="") as f_modes,
    ):
        w_traj = csv.writer(f_traj)
        w_met = csv.writer(f_met)
        w_out = csv.writer(f_out)
        w_modes = csv.writer(f_modes)

        w_traj.writerow(["k_global", "outer_j", "inner_t", "agent", "r0", "r1", "v0", "v1", "u0", "u1"])
        w_met.writerow([
            "k_global", "outer_j", "inner_t",
            "Vr", "Vv", "max_speed", "mean_speed",
            "dmin_agents", "dmin_obstacles",
            "n_F", "n_C", "n_O", "n_CO", "n_T",
            "_h_pair_num", "_circle_barrier_num",
        ])
        w_out.writerow([
            "outer_j", "agent", "model", "M",
            "objective_primary", "lex_used",
            "phi_terminal", "ri_terminal",
            "r_term0", "r_term1",
            "diam_C",
            "t_primary_ms", "t_lex_ms", "t_total_ms",
            "n_var_primary", "n_eq_primary", "n_ineq_primary",
            "n_var_lex", "n_eq_lex", "n_ineq_lex",
            "nominal_fallback_used", "nominal_status",
            "bary_r0", "bary_r1",
        ])
        w_modes.writerow([
            "k_global", "outer_j", "inner_t", "agent",
            "mode", "desired_mode", "effective_mode",
            "d_agent_min", "d_obs_min",
            "active_pairs", "active_obstacles",
            "eta_C", "eta_O",
            "_h_pair_num", "_circle_barrier_num",
            "solver_status", "fallback",
            "waypoint0", "waypoint1",
        ])

        k_global = 0

        for outer_j in range(cfg.outer_steps):
            try:
                coord_req.send(
                    dumps(
                        make_envelope(
                            "plant_step",
                            {"outer_index": outer_j, "r_all": r.tolist(), "v_all": v.tolist()},
                            src="plant",
                            dst="coord",
                        )
                    )
                )
                reply = loads(coord_req.recv())["payload"]
            except Exception as e:
                print(f"[Plant] outer_j={outer_j}: coordinator error {e}. Using zero nominal inputs.")
                reply = {"ok": False, "U_seq": np.zeros((M, cfg.n_agents, cfg.dim)).tolist(), "diag": {}}

            U_nom = np.array(reply.get("U_seq", np.zeros((M, cfg.n_agents, cfg.dim))), dtype=float)
            diag = reply.get("diag", {}) if isinstance(reply, dict) else {}

            bary_dict = diag.get("bary_r", {}) if isinstance(diag, dict) else {}

            if reply.get("ok", False) and isinstance(diag, dict) and diag:
                for i in range(1, cfg.n_agents + 1):
                    obj_i = _get_keyed(diag.get("objective_primary", {}), i, np.nan)
                    lex_i = _get_keyed(diag.get("lex_used", {}), i, 0)
                    phi_i = _get_keyed(diag.get("phi_terminal", {}), i, np.nan)
                    ri_i = _get_keyed(diag.get("ri_terminal", {}), i, 0)
                    rt_i = _get_keyed(diag.get("r_term", {}), i, [np.nan, np.nan])
                    diam_i = _get_keyed(diag.get("diam_C", {}), i, np.nan)
                    tprim_i = _get_keyed(diag.get("t_primary_ms", {}), i, np.nan)
                    tlex_i = _get_keyed(diag.get("t_lex_ms", {}), i, np.nan)
                    ttot_i = _get_keyed(diag.get("t_total_ms", {}), i, np.nan)
                    nvp_i = _get_keyed(diag.get("n_var_primary", {}), i, 0)
                    neqp_i = _get_keyed(diag.get("n_eq_primary", {}), i, 0)
                    ninp_i = _get_keyed(diag.get("n_ineq_primary", {}), i, 0)
                    nvl_i = _get_keyed(diag.get("n_var_lex", {}), i, 0)
                    neql_i = _get_keyed(diag.get("n_eq_lex", {}), i, 0)
                    ninl_i = _get_keyed(diag.get("n_ineq_lex", {}), i, 0)
                    fb_i = _get_keyed(diag.get("nominal_fallback_used", {}), i, False)
                    st_i = _get_keyed(diag.get("nominal_status", {}), i, "")
                    bary_i = _get_keyed(bary_dict, i, [np.nan, np.nan])

                    if not (isinstance(rt_i, (list, tuple)) and len(rt_i) >= 2):
                        rt_i = [np.nan, np.nan]
                    if not (isinstance(bary_i, (list, tuple)) and len(bary_i) >= 2):
                        bary_i = [np.nan, np.nan]

                    w_out.writerow([
                        outer_j, i, cfg.model, M,
                        obj_i, int(bool(lex_i)),
                        phi_i, int(bool(ri_i)),
                        float(rt_i[0]), float(rt_i[1]),
                        diam_i,
                        tprim_i, tlex_i, ttot_i,
                        int(nvp_i), int(neqp_i), int(ninp_i),
                        int(nvl_i), int(neql_i), int(ninl_i),
                        int(bool(fb_i)), st_i,
                        float(bary_i[0]), float(bary_i[1]),
                    ])
            else:
                for i in range(1, cfg.n_agents + 1):
                    w_out.writerow([outer_j, i, cfg.model, M, np.nan, 0, np.nan, 0, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, 0, 0, 0, 0, 0, 0, 1, "coord_fail", np.nan, np.nan])

            for inner_t in range(M):
                u_nom_now = U_nom[inner_t, :, :]
                U_hybrid = np.zeros_like(u_nom_now)

                count_modes = {m: 0 for m in ["F", "C", "O", "CO", "T"]}

                step_pair_barriers = 0
                step_circle_barriers = 0

                for i in range(1, cfg.n_agents + 1):
                    bary_i = _get_keyed(bary_dict, i, [r[i - 1, 0], r[i - 1, 1]])
                    payload = {
                        "agent_id": i,
                        "r_all": r.tolist(),
                        "v_all": v.tolist(),
                        "u_nom": u_nom_now[i - 1, :].tolist(),
                        "bary_r": bary_i,
                        "u_nom_all": u_nom_now.tolist(),
                    }

                    try:
                        hybrid_reqs[i].send(dumps(make_envelope("hybrid_request", payload, src="plant", dst=f"controller_node{i}")))
                        rep_i = loads(hybrid_reqs[i].recv())["payload"]
                        U_hybrid[i - 1, :] = np.array(rep_i["u_safe"], dtype=float)
                        d = rep_i.get("diag", {})
                    except Exception:
                        hybrid_reqs[i] = _reset_socket(ctx, hybrid_reqs[i], hybrid_eps[i], cfg.hybrid_timeout_ms, cfg.req_linger_ms)
                        U_hybrid[i - 1, :] = u_nom_now[i - 1, :]
                        d = {
                            "mode": "F",
                            "desired_mode": "F",
                            "effective_mode": "F",
                            "d_agent_min": np.nan,
                            "d_obs_min": np.nan,
                            "active_pairs": 0,
                            "active_obstacles": 0,
                            "eta_C": 0,
                            "eta_O": 0,
                            "_h_pair_num": 0,
                            "_circle_barrier_num": 0,
                            "solver_status": "hybrid_timeout",
                            "fallback": True,
                            "target_waypoint": [np.nan, np.nan],
                        }

                    step_pair_barriers += int(d.get("_h_pair_num", 0) or 0)
                    step_circle_barriers += int(d.get("_circle_barrier_num", 0) or 0)

                    mode = str(d.get("mode", "F"))
                    if mode not in count_modes:
                        count_modes[mode] = 0
                    count_modes[mode] += 1

                    wp = d.get("target_waypoint", [np.nan, np.nan])
                    if not (isinstance(wp, (list, tuple)) and len(wp) >= 2):
                        wp = [np.nan, np.nan]

                    w_modes.writerow([
                        k_global,
                        outer_j,
                        inner_t,
                        i,
                        d.get("mode", "F"),
                        d.get("desired_mode", "F"),
                        d.get("effective_mode", "F"),
                        d.get("d_agent_min", np.nan),
                        d.get("d_obs_min", np.nan),
                        d.get("active_pairs", 0),
                        d.get("active_obstacles", 0),
                        d.get("eta_C", 0),
                        d.get("eta_O", 0),
                        d.get("_h_pair_num", 0),
                        d.get("_circle_barrier_num", 0),
                        d.get("solver_status", ""),
                        int(bool(d.get("fallback", False))),
                        float(wp[0]),
                        float(wp[1]),
                    ])

                u = _clip(U_hybrid, cfg.u_min, cfg.u_max)

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
                dmin_agents = _pairwise_min_distance(r)
                dmin_obstacles = _global_obstacle_distance(cfg, r)

                w_met.writerow([
                    k_global, outer_j, inner_t,
                    Vr, Vv, max_speed, mean_speed,
                    dmin_agents, dmin_obstacles,
                    count_modes.get("F", 0),
                    count_modes.get("C", 0),
                    count_modes.get("O", 0),
                    count_modes.get("CO", 0),
                    count_modes.get("T", 0),
                    step_pair_barriers, step_circle_barriers,
                ])

                for i in range(cfg.n_agents):
                    w_traj.writerow([
                        k_global, outer_j, inner_t, i + 1,
                        r[i, 0], r[i, 1],
                        v[i, 0], v[i, 1],
                        u[i, 0], u[i, 1],
                    ])

                k_global += 1

            print(
                f"[Plant] finished outer_j={outer_j + 1}/{cfg.outer_steps} | "
                f"Vr={Vr:.3f} | dmin_agents={dmin_agents:.3f} | dmin_obs={dmin_obstacles:.3f}"
            )

    try:
        coord_req.send(dumps(make_envelope("shutdown", {}, src="plant", dst="coord")))
        _ = loads(coord_req.recv())
        print("[Plant] Shutdown complete.")
    except Exception:
        print("[Plant] Shutdown: coordinator did not respond (ok to ignore).")


if __name__ == "__main__":
    main()