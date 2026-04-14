"""cosim_manager.py
Co-simulation manager: replaces ``plant_node.py`` in the experiment pipeline.
"""
from __future__ import annotations

import argparse
import logging
import time
from typing import Any, Dict, List

import numpy as np
import zmq

from consensus_comm import dumps, loads, make_envelope
from consensus_config import NetConfig, add_common_args, config_from_namespace
from cosim_bridge import MatlabBridge, OdometryBundle
from cosim_config import CoSimConfig
from cosim_logger import CoSimLogger, _get_keyed

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("CosimManager")

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def _add_cosim_args(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    parser.add_argument(
        "--cosim-mode",
        choices=["matlab", "dry-run"],
        default="matlab",
        help="'matlab' starts MATLAB/Simulink; 'dry-run' skips it.",
    )
    parser.add_argument(
        "--simulink-dt",
        type=float,
        default=None,
        help="Simulink fixed step size in seconds.",
    )
    parser.add_argument(
        "--k-heading",
        type=float,
        default=None,
        help="Proportional heading gain for unicycle controller.",
    )
    return parser

# ---------------------------------------------------------------------------
# State update helpers
# ---------------------------------------------------------------------------

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

def _pairwise_min_dist(X: np.ndarray) -> float:
    if X.ndim != 2 or X.shape[0] <= 1:
        return float("inf")
    dmin = float("inf")
    for i in range(X.shape[0]):
        for k in range(i + 1, X.shape[0]):
            dmin = min(dmin, float(np.linalg.norm(X[i] - X[k])))
    return dmin

def _global_obs_dist(cfg: NetConfig, r: np.ndarray) -> float:
    if not cfg.obstacles_enabled or len(cfg.obstacles_circles) == 0:
        return float("inf")
    dmin = float("inf")
    for p in r:
        for cx, cy, rad in cfg.obstacles_circles:
            dd = float(
                np.linalg.norm(p[:2] - np.array([cx, cy], dtype=float))
                - (rad + cfg.obstacle_margin)
            )
            dmin = min(dmin, dd)
    return dmin

def _make_req_sock(
    ctx: zmq.Context, ep: str, timeout_ms: int, linger_ms: int
) -> zmq.Socket:
    s = ctx.socket(zmq.REQ)
    s.setsockopt(zmq.LINGER, linger_ms)
    s.setsockopt(zmq.REQ_RELAXED, 1)
    s.setsockopt(zmq.REQ_CORRELATE, 1)
    s.RCVTIMEO = timeout_ms
    s.SNDTIMEO = timeout_ms
    s.connect(ep)
    return s

def _reset_sock(
    ctx: zmq.Context, sock: zmq.Socket, ep: str, timeout_ms: int, linger_ms: int
) -> zmq.Socket:
    try:
        sock.close(0)
    except Exception:
        pass
    return _make_req_sock(ctx, ep, timeout_ms, linger_ms)

def _propagate_fallback(cfg, dt_inner, r_prev, v_prev, u_safe_all):
    """Fallback state propagation if MATLAB is unavailable or returns invalid odometry."""
    if cfg.model == "single_integrator":
        r_pred = _clip(r_prev + dt_inner * u_safe_all, cfg.r_min, cfg.r_max)
        v_pred = np.zeros_like(v_prev)
    else:
        r_pred = _clip(
            r_prev + dt_inner * v_prev + 0.5 * (dt_inner ** 2) * u_safe_all,
            cfg.r_min, cfg.r_max,
        )
        v_pred = _clip(v_prev + dt_inner * u_safe_all, cfg.v_min, cfg.v_max)
    return r_pred, v_pred

# ---------------------------------------------------------------------------
# Main manager
# ---------------------------------------------------------------------------

def main() -> None:
    parser = _add_cosim_args(add_common_args(argparse.ArgumentParser()))
    ns = parser.parse_args()

    cfg: NetConfig = config_from_namespace(ns)
    M: int = cfg.horizon_M()
    cosim_mode: str = getattr(ns, "cosim_mode", "matlab")

    # Mathematically enforce time-base synchronization.
    # The MPC outer step (cfg.dt) must be perfectly sub-divided into M inner steps.
    # FIX (Bug 4): round(cfg.dt / M, 4) gave 0.1667 for M=6, so 0.1667×6=1.0002≠1.0.
    # Use cfg.dt / M directly; cosim_step.m uses count-based stepping so the tiny
    # float residual in e.g. 1/6 has zero impact on step counts.
    derived_simulink_dt = cfg.dt / M          # exact: e.g. 1.0/6 = 0.16666...
    user_dt = getattr(ns, "simulink_dt", None)

    if user_dt is not None and abs(user_dt - derived_simulink_dt) > 1e-6:
        log.warning(
            "CLI --simulink-dt=%s ignored. Enforcing derived "
            "simulink_dt=%.10f = dt/M = %.4f/%d.",
            user_dt, derived_simulink_dt, cfg.dt, M
        )

    # Convert initial positions to tuple-of-tuples for CoSimConfig.
    # ROOT-CAUSE FIX (Bug 6): These are passed to cosim_bridge.reset_experiment()
    # which sets the Simulink integrator InitialCondition parameters before the
    # first simulation step.  Without this, all agents start at (0,0,0) in
    # Simulink, the first odometry feedback teleports the planning layer's r array
    # to the origin, and the hybrid safety supervisor fires at full saturation,
    # producing the "impulsive" behavior seen in the animation.
    r0_np = cfg.initial_positions()          # ndarray (n_agents, dim)
    initial_pos_tuple = tuple(
        tuple(float(v) for v in row) for row in r0_np.tolist()
    )

    cosim_cfg = CoSimConfig(
        n_agents=cfg.n_agents,
        dim=cfg.dim,
        planning_dt=cfg.dt,
        simulink_dt=derived_simulink_dt,
        k_heading=getattr(ns, "k_heading", None) or 4.0,
        initial_positions=initial_pos_tuple,       # ← KEY FIX
    )

    log.info(
        "CosimManager | model=%s M=%d outer_steps=%d cosim_mode=%s "
        "simulink_dt=%.6f initial_positions=%s",
        cfg.model, M, cfg.outer_steps, cosim_mode,
        derived_simulink_dt, initial_pos_tuple,
    )

    # --- MATLAB bridge ---
    bridge = MatlabBridge(cosim_cfg)
    if cosim_mode == "matlab":
        bridge.start()

    # --- Logger ---
    logger = CoSimLogger(cosim_cfg)
    logger.open()

    # --- ZMQ context ---
    ctx = zmq.Context.instance()

    # Coordinator socket
    coord_req = _make_req_sock(
        ctx, cfg.plant_to_coord_rep, cfg.req_timeout_ms, cfg.req_linger_ms
    )
    log.info("REQ -> Coordinator at %s", cfg.plant_to_coord_rep)

    # Hybrid controller sockets
    hybrid_reqs: Dict[int, zmq.Socket] = {}
    hybrid_eps: Dict[int, str] = {}
    for i in range(1, cfg.n_agents + 1):
        ep = cfg.controller_endpoint(i)
        hybrid_eps[i] = ep
        hybrid_reqs[i] = _make_req_sock(ctx, ep, cfg.hybrid_timeout_ms, cfg.req_linger_ms)
        log.info("REQ -> controller_node %d (hybrid) at %s", i, ep)

    # --- State ---
    r = cfg.initial_positions().copy()
    v = cfg.initial_velocities().copy()
    k_global = 0

    try:
        # Guarantee Simulink starts fresh from 0 at the beginning of the experiment
        if hasattr(bridge, 'reset_experiment'):
            bridge.reset_experiment()
            
        for outer_j in range(cfg.outer_steps):
            outer_t0 = time.perf_counter()

            # ---- 1. Request nominal MPC sequence ----
            try:
                coord_req.send(
                    dumps(
                        make_envelope(
                            "plant_step",
                            {"outer_index": outer_j, "r_all": r.tolist(), "v_all": v.tolist()},
                            src="cosim_manager",
                            dst="coord",
                        )
                    )
                )
                reply = loads(coord_req.recv())["payload"]
            except Exception as exc:
                log.warning("outer_j=%d coordinator error: %s", outer_j, exc)
                reply = {
                    "ok": False,
                    "U_seq": np.zeros((M, cfg.n_agents, cfg.dim)).tolist(),
                    "diag": {},
                }

            planning_ok: bool = bool(reply.get("ok", False))
            U_nom = np.array(
                reply.get("U_seq", np.zeros((M, cfg.n_agents, cfg.dim))), dtype=float
            )
            diag: Dict[str, Any] = (
                reply.get("diag", {}) if isinstance(reply, dict) else {}
            )
            bary_dict = diag.get("bary_r", {}) if isinstance(diag, dict) else {}

            # ---- 2. Inner steps ----
            count_modes: Dict[str, int] = {m: 0 for m in ["F", "C", "O", "CO", "T"]}
            odo_errors: List[float] = []
            matlab_step_ms_list: List[float] = []

            for inner_t in range(M):
                r_prev = r.copy()
                v_prev = v.copy()

                u_nom_now = U_nom[inner_t, :, :]
                u_safe_all = np.zeros_like(u_nom_now)
                mode_data: List[Dict[str, Any]] = []

                # ---- 2a. Hybrid safety filter ----
                for i in range(1, cfg.n_agents + 1):
                    bary_i = _get_keyed(
                        bary_dict, i, [r_prev[i - 1, 0], r_prev[i - 1, 1]]
                    )
                    payload = {
                        "agent_id": i,
                        "r_all": r_prev.tolist(),
                        "v_all": v_prev.tolist(),
                        "u_nom": u_nom_now[i - 1, :].tolist(),
                        "bary_r": bary_i,
                        "dt_inner": float(derived_simulink_dt),
                    }
                    try:
                        hybrid_reqs[i].send(
                            dumps(
                                make_envelope(
                                    "hybrid_request", payload,
                                    src="cosim_manager",
                                    dst=f"controller_node{i}",
                                )
                            )
                        )
                        rep_i = loads(hybrid_reqs[i].recv())["payload"]
                        u_safe_all[i - 1, :] = np.array(rep_i["u_safe"], dtype=float)
                        d = rep_i.get("diag", {})
                    except Exception:
                        hybrid_reqs[i] = _reset_sock(
                            ctx, hybrid_reqs[i], hybrid_eps[i],
                            cfg.hybrid_timeout_ms, cfg.req_linger_ms,
                        )
                        u_safe_all[i - 1, :] = u_nom_now[i - 1, :]
                        d = {"mode": "F", "desired_mode": "F", "effective_mode": "F"}

                    mode = str(d.get("mode", "F"))
                    count_modes[mode] = count_modes.get(mode, 0) + 1
                    mode_data.append(d)

                u_safe_all = _clip(u_safe_all, cfg.u_min, cfg.u_max)

                # predict theoretical position for error checking
                r_pred, v_pred = _propagate_fallback(
                    cfg, cosim_cfg.simulink_dt, r_prev, v_prev, u_safe_all
                )

                # ---- 2b. Simulink step ----
                sim_t0 = time.perf_counter()
                bundle: OdometryBundle = bridge.step(outer_j, u_safe_all, inner_t)
                matlab_step_ms = (time.perf_counter() - sim_t0) * 1e3
                matlab_step_ms_list.append(matlab_step_ms)

                have_valid_odo = (
                    cosim_mode == "matlab"
                    and bundle.positions.shape == (cfg.n_agents, cfg.dim)
                    and np.isfinite(bundle.positions).all()
                )

                # ---- 2c. Update planning-layer state ----
                if have_valid_odo:
                    r_next = _clip(bundle.positions.copy(), cfg.r_min, cfg.r_max)
                    v_next = np.zeros_like(v_prev)
                    for i in range(cfg.n_agents):
                        v_next[i, 0] = bundle.linear_speeds[i] * np.cos(bundle.headings[i])
                        v_next[i, 1] = bundle.linear_speeds[i] * np.sin(bundle.headings[i])
                    v_next = _clip(v_next, cfg.v_min, cfg.v_max)
                else:
                    r_next, v_next = r_pred, v_pred

                # ---- 2d. Log ----
                logger.log_planning_step(
                    k_global, outer_j, inner_t, r_prev, v_prev,
                    u_nom_now, u_safe_all, diag,
                )
                # FIX (secondary logging bug): log odometry error against r_pred
                # (kinematic single-integrator prediction), not against r_next.
                # Previously r_next WAS bundle.positions, so err_pos was always 0.
                # The tracking error = Simulink position minus what the planning
                # model predicted — this is the meaningful quantity.
                logger.log_odometry_step(k_global, outer_j, inner_t, bundle, r_pred)
                logger.log_hybrid_modes_step(k_global, outer_j, inner_t, mode_data)

                if have_valid_odo:
                    for i in range(cfg.n_agents):
                        err = float(np.linalg.norm(bundle.positions[i] - r_pred[i]))
                        odo_errors.append(err)

                # Commit step
                r = r_next
                v = v_next
                k_global += 1

            # ---- 3. Outer-step health log ----
            Vr = _pairwise_diameter(r)
            dmin_a = _pairwise_min_dist(r)
            dmin_o = _global_obs_dist(cfg, r)
            odo_max_err = max(odo_errors) if odo_errors else 0.0
            avg_matlab_ms = (
                sum(matlab_step_ms_list) / len(matlab_step_ms_list)
                if matlab_step_ms_list else 0.0
            )

            logger.log_health(
                outer_j,
                planning_ok=planning_ok,
                n_failed_ctrl=0,
                Vr=Vr,
                dmin_agents=dmin_a,
                dmin_obs=dmin_o,
                mode_counts=count_modes,
                odo_max_err=odo_max_err,
                matlab_step_ms=avg_matlab_ms,
            )

            outer_elapsed = (time.perf_counter() - outer_t0) * 1e3
            log.info(
                "outer_j=%d/%d | Vr=%.3f | dmin_a=%.3f | dmin_o=%.3f | "
                "odo_err_max=%.4f | matlab_avg=%.1f ms | total=%.0f ms",
                outer_j + 1, cfg.outer_steps,
                Vr, dmin_a, dmin_o, odo_max_err, avg_matlab_ms, outer_elapsed,
            )

    finally:
        # ---- Shutdown ----
        try:
            coord_req.send(
                dumps(make_envelope("shutdown", {}, src="cosim_manager", dst="coord"))
            )
            _ = loads(coord_req.recv())
        except Exception:
            pass

        if cosim_mode == "matlab":
            bridge.stop()

        logger.close()
        log.info("CosimManager finished.")

if __name__ == "__main__":
    main()