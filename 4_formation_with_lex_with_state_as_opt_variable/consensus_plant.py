from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import zmq

from consensus_comm import dumps, loads, make_envelope
from consensus_config import add_common_args, config_from_namespace


def _clip(x: np.ndarray, lo: float, hi: float) -> np.ndarray:
    return np.minimum(np.maximum(x, lo), hi)


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
        f"outer_steps={cfg.outer_steps} | objective_mode={cfg.objective_mode}"
    )

    ctx = zmq.Context.instance()
    req = ctx.socket(zmq.REQ)
    req.setsockopt(zmq.LINGER, cfg.req_linger_ms)
    req.connect(cfg.plant_to_coord_rep)
    req.RCVTIMEO = cfg.req_timeout_ms
    req.SNDTIMEO = cfg.req_timeout_ms
    print(f"[Plant] REQ -> Coordinator at {cfg.plant_to_coord_rep}")

    r = cfg.initial_positions().copy()
    v = cfg.initial_velocities().copy()

    traj_path = Path("consensus_traj.csv")
    metrics_path = Path("consensus_metrics.csv")
    outer_path = Path("consensus_outer.csv")

    with traj_path.open("w", newline="") as f_traj, metrics_path.open("w", newline="") as f_met, outer_path.open("w", newline="") as f_out:
        w_traj = csv.writer(f_traj)
        w_met = csv.writer(f_met)
        w_out = csv.writer(f_out)

        w_traj.writerow(["k_global", "outer_j", "inner_t", "agent", "r0", "r1", "v0", "v1", "u0", "u1"])
        w_met.writerow(["k_global", "outer_j", "inner_t", "Vr", "Vv", "max_speed", "mean_speed"])
        w_out.writerow([
            "outer_j", "agent", "model", "M",
            "objective_primary", "lex_used",
            "phi_terminal", "ri_terminal",
            "r_term0", "r_term1",
            "diam_C",
            "t_primary_ms", "t_lex_ms", "t_total_ms",
            "n_var_primary", "n_eq_primary", "n_ineq_primary",
            "n_var_lex", "n_eq_lex", "n_ineq_lex",
        ])

        k_global = 0

        for outer_j in range(cfg.outer_steps):
            try:
                req.send(
                    dumps(
                        make_envelope(
                            "plant_step",
                            {"outer_index": outer_j, "r_all": r.tolist(), "v_all": v.tolist()},
                            src="plant",
                            dst="coord",
                        )
                    )
                )
                reply = loads(req.recv())["payload"]
            except zmq.Again:
                print(f"[Plant] outer_j={outer_j}: coordinator timeout. Using zero inputs.")
                reply = {"ok": False, "U_seq": np.zeros((M, cfg.n_agents, cfg.dim)).tolist(), "diag": {}}
            except Exception as e:
                print(f"[Plant] outer_j={outer_j}: coordinator error {e}. Using zero inputs.")
                reply = {"ok": False, "U_seq": np.zeros((M, cfg.n_agents, cfg.dim)).tolist(), "diag": {}}

            U = np.array(reply.get("U_seq", np.zeros((M, cfg.n_agents, cfg.dim))), dtype=float)

            diag = reply.get("diag", {}) if isinstance(reply, dict) else {}
            if reply.get("ok", False) and isinstance(diag, dict) and diag:
                obj = diag.get("objective_primary", {})
                lex = diag.get("lex_used", {})
                phi = diag.get("phi_terminal", {})
                ri = diag.get("ri_terminal", {})
                rterm = diag.get("r_term", {})
                diamC = diag.get("diam_C", {})
                tprim = diag.get("t_primary_ms", {})
                tlex = diag.get("t_lex_ms", {})
                ttot = diag.get("t_total_ms", {})

                nvp = diag.get("n_var_primary", {})
                neqp = diag.get("n_eq_primary", {})
                ninp = diag.get("n_ineq_primary", {})
                nvl = diag.get("n_var_lex", {})
                neql = diag.get("n_eq_lex", {})
                ninl = diag.get("n_ineq_lex", {})

                for i in range(1, cfg.n_agents + 1):
                    obj_i = _get_keyed(obj, i, np.nan)
                    lex_i = _get_keyed(lex, i, 0)
                    phi_i = _get_keyed(phi, i, np.nan)
                    ri_i = _get_keyed(ri, i, 0)
                    rt = _get_keyed(rterm, i, [np.nan, np.nan])
                    diam_i = _get_keyed(diamC, i, np.nan)

                    tprim_i = _get_keyed(tprim, i, np.nan)
                    tlex_i = _get_keyed(tlex, i, np.nan)
                    ttot_i = _get_keyed(ttot, i, np.nan)

                    nvp_i = _get_keyed(nvp, i, 0)
                    neqp_i = _get_keyed(neqp, i, 0)
                    ninp_i = _get_keyed(ninp, i, 0)
                    nvl_i = _get_keyed(nvl, i, 0)
                    neql_i = _get_keyed(neql, i, 0)
                    ninl_i = _get_keyed(ninl, i, 0)

                    try:
                        obj_i = float(obj_i)
                    except Exception:
                        obj_i = np.nan
                    lex_i = int(bool(lex_i))
                    try:
                        phi_i = float(phi_i)
                    except Exception:
                        phi_i = np.nan
                    ri_i = int(bool(ri_i))
                    if not (isinstance(rt, (list, tuple)) and len(rt) >= 2):
                        rt = [np.nan, np.nan]
                    try:
                        diam_i = float(diam_i)
                    except Exception:
                        diam_i = np.nan

                    try:
                        tprim_i = float(tprim_i)
                    except Exception:
                        tprim_i = np.nan
                    try:
                        tlex_i = float(tlex_i)
                    except Exception:
                        tlex_i = np.nan
                    try:
                        ttot_i = float(ttot_i)
                    except Exception:
                        ttot_i = np.nan

                    try:
                        nvp_i = int(nvp_i)
                    except Exception:
                        nvp_i = 0
                    try:
                        neqp_i = int(neqp_i)
                    except Exception:
                        neqp_i = 0
                    try:
                        ninp_i = int(ninp_i)
                    except Exception:
                        ninp_i = 0
                    try:
                        nvl_i = int(nvl_i)
                    except Exception:
                        nvl_i = 0
                    try:
                        neql_i = int(neql_i)
                    except Exception:
                        neql_i = 0
                    try:
                        ninl_i = int(ninl_i)
                    except Exception:
                        ninl_i = 0

                    w_out.writerow([
                        outer_j, i, cfg.model, M,
                        obj_i, lex_i, phi_i, ri_i,
                        float(rt[0]), float(rt[1]),
                        diam_i,
                        tprim_i, tlex_i, ttot_i,
                        nvp_i, neqp_i, ninp_i,
                        nvl_i, neql_i, ninl_i,
                    ])
            else:
                for i in range(1, cfg.n_agents + 1):
                    w_out.writerow([outer_j, i, cfg.model, M, np.nan, 0, np.nan, 0, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, 0, 0, 0, 0, 0, 0])

            for inner_t in range(M):
                u = _clip(U[inner_t, :, :], cfg.u_min, cfg.u_max)

                if cfg.model == "single_integrator":
                    r = _clip(r + u, cfg.r_min, cfg.r_max)
                    v = np.zeros_like(v)
                else:
                    r = _clip(r + v, cfg.r_min, cfg.r_max)
                    v = _clip(v + u, cfg.v_min, cfg.v_max)

                Vr = _pairwise_diameter(r)
                Vv = _pairwise_diameter(v)
                speeds = np.linalg.norm(v, axis=1)
                max_speed = float(np.max(speeds)) if speeds.size else 0.0
                mean_speed = float(np.mean(speeds)) if speeds.size else 0.0

                w_met.writerow([k_global, outer_j, inner_t, Vr, Vv, max_speed, mean_speed])

                for i in range(cfg.n_agents):
                    w_traj.writerow([
                        k_global, outer_j, inner_t, i + 1,
                        r[i, 0], r[i, 1],
                        v[i, 0], v[i, 1],
                        u[i, 0], u[i, 1],
                    ])

                k_global += 1

            print(f"[Plant] finished outer_j={outer_j + 1}/{cfg.outer_steps} | Vr={Vr:.3f}")

    print(
        "[Plant] Done.\n"
        f"  traj: {traj_path.resolve()}\n"
        f"  metrics: {metrics_path.resolve()}\n"
        f"  outer: {outer_path.resolve()}"
    )

    try:
        req.send(dumps(make_envelope("shutdown", {}, src="plant", dst="coord")))
        _ = loads(req.recv())
        print("[Plant] Shutdown complete.")
    except Exception:
        print("[Plant] Shutdown: coordinator did not respond (ok to ignore).")


if __name__ == "__main__":
    main()