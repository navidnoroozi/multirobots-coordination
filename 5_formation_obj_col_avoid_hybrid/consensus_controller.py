from __future__ import annotations

import time
from typing import Any, Dict, List, Tuple

import cvxpy as cp
import numpy as np

from consensus_config import NetConfig


def _clamp_box(var, lo: float, hi: float) -> List[Any]:
    return [var >= lo, var <= hi]


def _clip_vec(u: np.ndarray, lo: float, hi: float) -> np.ndarray:
    return np.minimum(np.maximum(u, lo), hi)


def _cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _diameter(points: np.ndarray) -> float:
    if points.ndim != 2 or points.shape[0] <= 1:
        return 0.0
    dmax = 0.0
    for i in range(points.shape[0]):
        for k in range(i + 1, points.shape[0]):
            d = float(np.linalg.norm(points[i] - points[k]))
            if d > dmax:
                dmax = d
    return float(dmax)


def _convex_hull_2d(points: np.ndarray) -> np.ndarray:
    pts = np.unique(points, axis=0)
    if pts.shape[0] <= 1:
        return pts
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def build_half(pts_arr):
        half = []
        for p in pts_arr:
            while len(half) >= 2:
                a = half[-2]
                b = half[-1]
                if _cross2(b - a, p - b) <= 1e-12:
                    half.pop()
                else:
                    break
            half.append(p)
        return half

    lower = build_half(pts)
    upper = build_half(pts[::-1])
    hull = np.array(lower[:-1] + upper[:-1], dtype=float)
    if hull.shape[0] == 0:
        return pts[:1]
    return hull


def _dist_point_to_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-15:
        return float(np.linalg.norm(p - a))
    t = float(np.dot(p - a, ab) / denom)
    t = max(0.0, min(1.0, t))
    proj = a + t * ab
    return float(np.linalg.norm(p - proj))


def _phi_dist_to_boundary_conv2d(points: np.ndarray, x: np.ndarray) -> float:
    hull = _convex_hull_2d(points)
    if hull.shape[0] <= 1:
        return 0.0
    if hull.shape[0] == 2:
        return float(min(np.linalg.norm(x - hull[0]), np.linalg.norm(x - hull[1])))

    dmin = float("inf")
    for i in range(hull.shape[0]):
        a = hull[i]
        b = hull[(i + 1) % hull.shape[0]]
        d = _dist_point_to_segment(x, a, b)
        if d < dmin:
            dmin = d
    return float(dmin)


def _prob_metrics(prob):
    sm = prob.size_metrics
    n_var = int(getattr(sm, "num_scalar_variables", 0))
    n_eq = int(getattr(sm, "num_scalar_eq_constr", 0))
    n_ineq = int(getattr(sm, "num_scalar_leq_constr", 0))
    return n_var, n_eq, n_ineq


def formation_data(cfg: NetConfig, agent_id: int, r_i: np.ndarray, r_neighbors: List[np.ndarray], neighbors: List[int]):
    C = cfg.formation_offsets()
    idx_i = int(agent_id) - 1
    c_i = C[idx_i, :]
    c_neighbors = [C[int(k) - 1, :] for k in neighbors]

    Y_list = [r_i - c_i] + [rk - ck for rk, ck in zip(r_neighbors, c_neighbors)]
    Y_mat = np.stack(Y_list, axis=0)
    bary_y = np.mean(Y_mat, axis=0)
    bary_r = bary_y + c_i
    return c_i, Y_mat, bary_y, bary_r


def _simulate_single(r0: np.ndarray, u_seq: np.ndarray) -> np.ndarray:
    M, d = u_seq.shape
    r = np.zeros((M + 1, d), dtype=float)
    r[0, :] = r0
    for k in range(M):
        r[k + 1, :] = r[k, :] + u_seq[k, :]
    return r


def _simulate_double(r0: np.ndarray, v0: np.ndarray, u_seq: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    M, d = u_seq.shape
    r = np.zeros((M + 1, d), dtype=float)
    v = np.zeros((M + 1, d), dtype=float)
    r[0, :] = r0
    v[0, :] = v0
    for k in range(M):
        r[k + 1, :] = r[k, :] + v[k, :]
        v[k + 1, :] = v[k, :] + u_seq[k, :]
    return r, v


def _fallback_single(cfg: NetConfig, agent_id: int, r_i: np.ndarray, r_neighbors: List[np.ndarray], neighbors: List[int]) -> Dict[str, Any]:
    M = cfg.horizon_M()
    d = cfg.dim
    c_i, Y_mat, bary_y, bary_r = formation_data(cfg, agent_id, r_i, r_neighbors, neighbors)

    u_seq = np.zeros((M, d), dtype=float)
    r = r_i.copy()
    for k in range(M):
        rem = max(M - k, 1)
        u = (bary_r - r) / float(rem)
        u = _clip_vec(u, cfg.u_min, cfg.u_max)
        r_next = _clip_vec(r + u, cfg.r_min, cfg.r_max)
        u = r_next - r
        u_seq[k, :] = u
        r = r_next

    r_sol = _simulate_single(r_i, u_seq)
    r_term = r_sol[-1, :]
    y_term = r_term - c_i
    phi = _phi_dist_to_boundary_conv2d(Y_mat, y_term) if cfg.dim == 2 else 0.0

    return {
        "ok": True,
        "agent_id": int(agent_id),
        "model": cfg.model,
        "M": int(M),
        "bary_r": bary_r.tolist(),
        "r_pred": r_sol.tolist(),
        "u_seq": u_seq.tolist(),
        "a_weights": [],
        "lex_used": False,
        "objective_primary": np.nan,
        "r_term": r_term.tolist(),
        "phi_terminal": float(phi),
        "ri_terminal": bool(phi > cfg.phi_tol),
        "diam_C": float(_diameter(Y_mat)),
        "t_primary_ms": 0.0,
        "t_lex_ms": 0.0,
        "t_total_ms": 0.0,
        "n_var_primary": 0,
        "n_eq_primary": 0,
        "n_ineq_primary": 0,
        "n_var_lex": 0,
        "n_eq_lex": 0,
        "n_ineq_lex": 0,
        "nominal_fallback_used": True,
        "nominal_status": "fallback_single",
    }


def _fallback_double(cfg: NetConfig, agent_id: int, r_i: np.ndarray, v_i: np.ndarray, r_neighbors: List[np.ndarray], neighbors: List[int]) -> Dict[str, Any]:
    M = cfg.horizon_M()
    d = cfg.dim
    c_i, Y_mat, bary_y, bary_r = formation_data(cfg, agent_id, r_i, r_neighbors, neighbors)

    kp = 0.35
    kd = 0.85

    u_seq = np.zeros((M, d), dtype=float)
    r = r_i.copy()
    v = v_i.copy()

    for k in range(M):
        u = kp * (bary_r - r) - kd * v
        u = _clip_vec(u, cfg.u_min, cfg.u_max)

        r_next = _clip_vec(r + v, cfg.r_min, cfg.r_max)
        v_next = _clip_vec(v + u, cfg.v_min, cfg.v_max)

        for q in range(d):
            if r_next[q] <= cfg.r_min + 1e-9 or r_next[q] >= cfg.r_max - 1e-9:
                v_next[q] *= 0.25

        u_seq[k, :] = _clip_vec(v_next - v, cfg.u_min, cfg.u_max)
        r, v = r_next, _clip_vec(v + u_seq[k, :], cfg.v_min, cfg.v_max)

    r_sol, v_sol = _simulate_double(r_i, v_i, u_seq)
    r_term = r_sol[-1, :]
    y_term = r_term - c_i
    phi = _phi_dist_to_boundary_conv2d(Y_mat, y_term) if cfg.dim == 2 else 0.0

    return {
        "ok": True,
        "agent_id": int(agent_id),
        "model": cfg.model,
        "M": int(M),
        "bary_r": bary_r.tolist(),
        "r_pred": r_sol.tolist(),
        "v_pred": v_sol.tolist(),
        "u_seq": u_seq.tolist(),
        "a_weights": [],
        "lex_used": False,
        "objective_primary": np.nan,
        "r_term": r_term.tolist(),
        "phi_terminal": float(phi),
        "ri_terminal": bool(phi > cfg.phi_tol),
        "diam_C": float(_diameter(Y_mat)),
        "t_primary_ms": 0.0,
        "t_lex_ms": 0.0,
        "t_total_ms": 0.0,
        "n_var_primary": 0,
        "n_eq_primary": 0,
        "n_ineq_primary": 0,
        "n_var_lex": 0,
        "n_eq_lex": 0,
        "n_ineq_lex": 0,
        "nominal_fallback_used": True,
        "nominal_status": "fallback_double",
    }


def _solve_single_integrator(
    cfg: NetConfig,
    agent_id: int,
    r_i: np.ndarray,
    r_neighbors: List[np.ndarray],
    neighbors: List[int],
    u_prev: np.ndarray,
) -> Tuple[Dict[str, Any], bool]:
    M = cfg.horizon_M()
    d = cfg.dim

    c_i, Y_mat, bary_y, bary_r = formation_data(cfg, agent_id, r_i, r_neighbors, neighbors)
    rhs0 = float(np.linalg.norm((r_i - c_i) - bary_y))
    rhs = cfg.alpha_gamma * rhs0

    u = cp.Variable((M, d))
    a = cp.Variable((Y_mat.shape[0],))
    tmin = cp.Variable()

    r_expr = [cp.Constant(r_i)]
    for k in range(1, M + 1):
        r_expr.append(cp.Constant(r_i) + cp.sum(u[:k, :], axis=0))

    cons: List[Any] = []
    for k in range(M):
        cons += _clamp_box(u[k, :], cfg.u_min, cfg.u_max)
        cons += _clamp_box(r_expr[k], cfg.r_min, cfg.r_max)
    cons += _clamp_box(r_expr[M], cfg.r_min, cfg.r_max)

    cons += [a >= 0.0, cp.sum(a) == 1.0]
    cons += [r_expr[M] - c_i == a @ Y_mat]
    cons += [cp.norm((r_expr[M] - c_i) - bary_y, 2) <= rhs]

    J = 0
    for k in range(M):
        J += cfg.w_track * cp.sum_squares(r_expr[k] - bary_r)
        J += cfg.w_u * cp.sum_squares(u[k, :])
        if k == 0:
            J += cfg.w_du * cp.sum_squares(u[k, :] - u_prev)
        else:
            J += cfg.w_du * cp.sum_squares(u[k, :] - u[k - 1, :])

    prob1 = cp.Problem(cp.Minimize(J), cons)
    n_var_prim, n_eq_prim, n_ineq_prim = _prob_metrics(prob1)

    t0 = time.perf_counter()
    try:
        prob1.solve(solver=cp.ECOS, warm_start=True, verbose=False)
    except Exception:
        try:
            prob1.solve(solver=cp.SCS, warm_start=True, verbose=False)
        except Exception:
            return _fallback_single(cfg, agent_id, r_i, r_neighbors, neighbors), True
    t_primary_ms = 1e3 * (time.perf_counter() - t0)

    if prob1.status not in ("optimal", "optimal_inaccurate") or u.value is None:
        return _fallback_single(cfg, agent_id, r_i, r_neighbors, neighbors), True

    J_star = float(prob1.value)
    u_sol = np.array(u.value)
    a_sol = np.array(a.value).reshape(-1)

    r_sol = np.zeros((M + 1, d), dtype=float)
    r_sol[0, :] = r_i
    for k in range(M):
        r_sol[k + 1, :] = r_sol[k, :] + u_sol[k, :]

    y_term_primary = r_sol[M, :] - c_i
    phi_primary = _phi_dist_to_boundary_conv2d(Y_mat, y_term_primary) if cfg.dim == 2 else 0.0
    diam_C = _diameter(Y_mat)

    t_lex_ms = 0.0
    n_var_lex = n_eq_lex = n_ineq_lex = 0
    lex_used = False

    run_lex = bool(cfg.use_lexicographic)
    if run_lex and bool(getattr(cfg, "lex_only_if_phi_zero", False)):
        phi_ok = (phi_primary <= float(getattr(cfg, "phi_tol", 1e-9)))
        diam_ok = (diam_C > float(getattr(cfg, "diam_tol", 1e-6)))
        run_lex = bool(phi_ok and diam_ok)

    if run_lex:
        cons2 = list(cons)
        cons2 += [J <= J_star + cfg.lex_cost_tol]
        cons2 += [tmin >= 0.0, a >= tmin]
        prob2 = cp.Problem(cp.Maximize(tmin), cons2)

        n_var_lex, n_eq_lex, n_ineq_lex = _prob_metrics(prob2)
        t1 = time.perf_counter()
        try:
            prob2.solve(solver=cp.ECOS, warm_start=True, verbose=False)
        except Exception:
            try:
                prob2.solve(solver=cp.SCS, warm_start=True, verbose=False)
            except Exception:
                prob2 = None
        t_lex_ms = 1e3 * (time.perf_counter() - t1)

        if prob2 is not None and prob2.status in ("optimal", "optimal_inaccurate") and u.value is not None:
            lex_used = True
            u_sol = np.array(u.value)
            a_sol = np.array(a.value).reshape(-1)
            r_sol = _simulate_single(r_i, u_sol)

    r_term = r_sol[M, :].copy()
    y_term = r_term - c_i
    phi = _phi_dist_to_boundary_conv2d(Y_mat, y_term) if cfg.dim == 2 else 0.0

    out = {
        "ok": True,
        "agent_id": int(agent_id),
        "model": cfg.model,
        "M": int(M),
        "bary_r": bary_r.tolist(),
        "r_pred": r_sol.tolist(),
        "u_seq": u_sol.tolist(),
        "a_weights": a_sol.tolist(),
        "lex_used": bool(lex_used),
        "objective_primary": float(J_star),
        "r_term": r_term.tolist(),
        "phi_terminal": float(phi),
        "ri_terminal": bool(phi > cfg.phi_tol),
        "diam_C": float(diam_C),
        "t_primary_ms": float(t_primary_ms),
        "t_lex_ms": float(t_lex_ms),
        "t_total_ms": float(t_primary_ms + t_lex_ms),
        "n_var_primary": int(n_var_prim),
        "n_eq_primary": int(n_eq_prim),
        "n_ineq_primary": int(n_ineq_prim),
        "n_var_lex": int(n_var_lex),
        "n_eq_lex": int(n_eq_lex),
        "n_ineq_lex": int(n_ineq_lex),
        "nominal_fallback_used": False,
        "nominal_status": str(prob1.status),
    }
    return out, True


def _solve_double_integrator(
    cfg: NetConfig,
    agent_id: int,
    r_i: np.ndarray,
    v_i: np.ndarray,
    r_neighbors: List[np.ndarray],
    neighbors: List[int],
    u_prev: np.ndarray,
) -> Tuple[Dict[str, Any], bool]:
    M = cfg.horizon_M()
    d = cfg.dim

    c_i, Y_mat, bary_y, bary_r = formation_data(cfg, agent_id, r_i, r_neighbors, neighbors)
    rhs0 = float(np.linalg.norm((r_i - c_i) - bary_y))
    rhs = max(cfg.alpha_gamma * rhs0, 0.25)

    u = cp.Variable((M, d))
    a = cp.Variable((Y_mat.shape[0],))
    tmin = cp.Variable()

    v_expr = [cp.Constant(v_i)]
    for k in range(1, M + 1):
        v_expr.append(cp.Constant(v_i) + cp.sum(u[:k, :], axis=0))

    r_expr = [cp.Constant(r_i)]
    for k in range(1, M + 1):
        r_k = cp.Constant(r_i)
        for s in range(k):
            r_k = r_k + v_expr[s]
        r_expr.append(r_k)

    cons: List[Any] = []
    for k in range(M):
        cons += _clamp_box(u[k, :], cfg.u_min, cfg.u_max)
        cons += _clamp_box(r_expr[k], cfg.r_min, cfg.r_max)
        cons += _clamp_box(v_expr[k], cfg.v_min, cfg.v_max)
    cons += _clamp_box(r_expr[M], cfg.r_min, cfg.r_max)
    cons += _clamp_box(v_expr[M], cfg.v_min, cfg.v_max)

    cons += [a >= 0.0, cp.sum(a) == 1.0]
    cons += [r_expr[M] - c_i == a @ Y_mat]
    cons += [cp.norm((r_expr[M] - c_i) - bary_y, 2) <= rhs]

    J = 0
    for k in range(M):
        J += cfg.w_track * cp.sum_squares(r_expr[k] - bary_r)
        J += cfg.w_v * cp.sum_squares(v_expr[k])
        J += cfg.w_u * cp.sum_squares(u[k, :])
        if k == 0:
            J += cfg.w_du * cp.sum_squares(u[k, :] - u_prev)
        else:
            J += cfg.w_du * cp.sum_squares(u[k, :] - u[k - 1, :])

    prob1 = cp.Problem(cp.Minimize(J), cons)
    n_var_prim, n_eq_prim, n_ineq_prim = _prob_metrics(prob1)

    t0 = time.perf_counter()
    try:
        prob1.solve(solver=cp.ECOS, warm_start=True, verbose=False)
    except Exception:
        try:
            prob1.solve(solver=cp.SCS, warm_start=True, verbose=False)
        except Exception:
            return _fallback_double(cfg, agent_id, r_i, v_i, r_neighbors, neighbors), True
    t_primary_ms = 1e3 * (time.perf_counter() - t0)

    if prob1.status not in ("optimal", "optimal_inaccurate") or u.value is None:
        return _fallback_double(cfg, agent_id, r_i, v_i, r_neighbors, neighbors), True

    J_star = float(prob1.value)
    u_sol = np.array(u.value)
    a_sol = np.array(a.value).reshape(-1)

    r_sol, v_sol = _simulate_double(r_i, v_i, u_sol)
    y_term_primary = r_sol[M, :] - c_i
    phi_primary = _phi_dist_to_boundary_conv2d(Y_mat, y_term_primary) if cfg.dim == 2 else 0.0
    diam_C = _diameter(Y_mat)

    t_lex_ms = 0.0
    n_var_lex = n_eq_lex = n_ineq_lex = 0
    lex_used = False

    run_lex = bool(cfg.use_lexicographic)
    if run_lex and bool(getattr(cfg, "lex_only_if_phi_zero", False)):
        phi_ok = (phi_primary <= float(getattr(cfg, "phi_tol", 1e-9)))
        diam_ok = (diam_C > float(getattr(cfg, "diam_tol", 1e-6)))
        run_lex = bool(phi_ok and diam_ok)

    if run_lex:
        cons2 = list(cons)
        cons2 += [J <= J_star + cfg.lex_cost_tol]
        cons2 += [tmin >= 0.0, a >= tmin]
        prob2 = cp.Problem(cp.Maximize(tmin), cons2)

        n_var_lex, n_eq_lex, n_ineq_lex = _prob_metrics(prob2)
        t1 = time.perf_counter()
        try:
            prob2.solve(solver=cp.ECOS, warm_start=True, verbose=False)
        except Exception:
            try:
                prob2.solve(solver=cp.SCS, warm_start=True, verbose=False)
            except Exception:
                prob2 = None
        t_lex_ms = 1e3 * (time.perf_counter() - t1)

        if prob2 is not None and prob2.status in ("optimal", "optimal_inaccurate") and u.value is not None:
            lex_used = True
            u_sol = np.array(u.value)
            a_sol = np.array(a.value).reshape(-1)
            r_sol, v_sol = _simulate_double(r_i, v_i, u_sol)

    r_term = r_sol[M, :].copy()
    y_term = r_term - c_i
    phi = _phi_dist_to_boundary_conv2d(Y_mat, y_term) if cfg.dim == 2 else 0.0

    out = {
        "ok": True,
        "agent_id": int(agent_id),
        "model": cfg.model,
        "M": int(M),
        "bary_r": bary_r.tolist(),
        "r_pred": r_sol.tolist(),
        "v_pred": v_sol.tolist(),
        "u_seq": u_sol.tolist(),
        "a_weights": a_sol.tolist(),
        "lex_used": bool(lex_used),
        "objective_primary": float(J_star),
        "r_term": r_term.tolist(),
        "phi_terminal": float(phi),
        "ri_terminal": bool(phi > cfg.phi_tol),
        "diam_C": float(diam_C),
        "t_primary_ms": float(t_primary_ms),
        "t_lex_ms": float(t_lex_ms),
        "t_total_ms": float(t_primary_ms + t_lex_ms),
        "n_var_primary": int(n_var_prim),
        "n_eq_primary": int(n_eq_prim),
        "n_ineq_primary": int(n_ineq_prim),
        "n_var_lex": int(n_var_lex),
        "n_eq_lex": int(n_eq_lex),
        "n_ineq_lex": int(n_ineq_lex),
        "nominal_fallback_used": False,
        "nominal_status": str(prob1.status),
    }
    return out, True


def solve_mpc_request(cfg: NetConfig, payload: Dict[str, Any]) -> Tuple[Dict[str, Any], bool]:
    agent_id = int(payload["agent_id"])
    r_i = np.array(payload["r_i"], dtype=float)
    v_i = np.array(payload["v_i"], dtype=float)
    r_neighbors = [np.array(x, dtype=float) for x in payload["r_neighbors"]]
    u_prev = np.array(payload["u_prev"], dtype=float)
    neighbors = [int(k) for k in payload.get("neighbors", [])]

    if cfg.model == "single_integrator":
        return _solve_single_integrator(cfg, agent_id, r_i, r_neighbors, neighbors, u_prev)
    return _solve_double_integrator(cfg, agent_id, r_i, v_i, r_neighbors, neighbors, u_prev)