from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import cvxpy as cp
except Exception:  # pragma: no cover
    cp = None

from consensus_config import NetConfig

MODE_F = "F"
MODE_C = "C"
MODE_O = "O"
MODE_CO = "CO"
MODE_T = "T"


@dataclass
class HybridResult:
    u_safe: np.ndarray
    diag: Dict[str, Any]


class ExplicitHybridController:
    """
    Practical distributed hybrid safety layer.

    For single integrators, the safety modes can be enforced directly with one-step position
    constraints. For double integrators, the manuscript's one-step specialization (53)-(54) does
    not constrain the current acceleration because r(t+1)=r(t)+v(t). In the app, we therefore use
    a conservative practical repair consisting of
      1) earlier activation based on closing speed and braking distance,
      2) affine constraints on the next relative/obstacle-normal velocity, and
      3) two-step projected-position constraints together with a tangent guidance field around
         local obstacles so the agents can move around obstacles instead of stalling in front of
         them.
    """

    def __init__(self, cfg: NetConfig, agent_id: int):
        self.cfg = cfg
        self.agent_id = int(agent_id)
        self.idx = self.agent_id - 1
        self.mode = MODE_F
        self.pending_mode: Optional[str] = None
        self.transition_counter = 0
        self.last_u = np.zeros((cfg.dim,), dtype=float)
        self.last_target_u = np.zeros((cfg.dim,), dtype=float)
        self.eta_C = False
        self.eta_O = False
        self._orbit_sign: Dict[int, float] = {}

    @staticmethod
    def _norm(x: np.ndarray) -> float:
        return float(np.linalg.norm(x))

    @staticmethod
    def _unit(x: np.ndarray, fallback: Optional[np.ndarray] = None) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        n = float(np.linalg.norm(x))
        if n <= 1e-12:
            if fallback is None:
                y = np.zeros_like(x)
                if y.size:
                    y[0] = 1.0
                return y
            return ExplicitHybridController._unit(fallback)
        return x / n

    @staticmethod
    def _perp2(x: np.ndarray) -> np.ndarray:
        return np.array([-float(x[1]), float(x[0])], dtype=float)

    def _clip_u(self, u: np.ndarray) -> np.ndarray:
        u = np.asarray(u, dtype=float).reshape(self.cfg.dim)
        u = np.minimum(np.maximum(u, self.cfg.u_min), self.cfg.u_max)
        mag = float(getattr(self.cfg, "u_mag", 0.0))
        if mag > 0.0:
            n = self._norm(u)
            if n > mag:
                u = (mag / max(n, 1e-12)) * u
        return u

    def _inflated_obstacles(self) -> List[Tuple[np.ndarray, float]]:
        if not self.cfg.obstacles_enabled:
            return []
        out: List[Tuple[np.ndarray, float]] = []
        for (cx, cy, rad) in self.cfg.obstacles_circles:
            out.append((np.array([float(cx), float(cy)], dtype=float), float(rad) + float(self.cfg.obstacle_margin)))
        return out

    def _pairwise_distances(self, r_all: np.ndarray) -> Tuple[float, List[int]]:
        ri = r_all[self.idx]
        nbrs: List[int] = []
        dmin = float("inf")
        for j in range(r_all.shape[0]):
            if j == self.idx:
                continue
            d = float(np.linalg.norm(ri - r_all[j]))
            if d <= float(self.cfg.safety_warning_radius):
                nbrs.append(j)
            dmin = min(dmin, d)
        return dmin, nbrs

    def _obstacle_distances(self, r_i: np.ndarray) -> Tuple[float, Optional[int], List[int]]:
        active: List[int] = []
        dmin = float("inf")
        arg = None
        for q, (c, rad) in enumerate(self._inflated_obstacles()):
            d = float(np.linalg.norm(r_i[:2] - c) - rad)
            if d <= float(self.cfg.obstacle_warning_radius):
                active.append(q)
            if d < dmin:
                dmin = d
                arg = q
        if not active:
            return dmin, arg, []
        return dmin, arg, active

    def _segment_intersects_circle(self, a: np.ndarray, b: np.ndarray, c: np.ndarray, radius: float) -> bool:
        a2 = np.asarray(a[:2], dtype=float)
        b2 = np.asarray(b[:2], dtype=float)
        ab = b2 - a2
        denom = float(np.dot(ab, ab))
        if denom <= 1e-12:
            return float(np.linalg.norm(a2 - c)) <= radius
        t = float(np.dot(c - a2, ab) / denom)
        t = max(0.0, min(1.0, t))
        proj = a2 + t * ab
        return float(np.linalg.norm(proj - c)) <= radius

    def _path_blocked(self, r_i: np.ndarray, goal: np.ndarray, obs_ids: List[int]) -> Optional[int]:
        obstacles = self._inflated_obstacles()
        pad = float(getattr(self.cfg, "di_path_block_margin", 0.25))
        for q in obs_ids:
            c, rad = obstacles[q]
            if self._segment_intersects_circle(r_i, goal, c, rad + pad):
                return q
        return None

    def _dynamic_pair_enter_distance(self, r_i: np.ndarray, v_i: np.ndarray, r_j: np.ndarray, v_j: np.ndarray) -> float:
        if self.cfg.model != "double_integrator":
            return float(self.cfg.d_agent_enter)
        dt = float(self.cfg.dt)
        a_eff = max(float(getattr(self.cfg, "di_brake_accel", self.cfg.u_mag)), 1e-6)
        delta = r_i - r_j
        n = self._unit(delta)
        v_rel = v_i - v_j
        closing = max(0.0, -float(np.dot(n, v_rel)))
        brake = dt * closing + 0.5 * (closing ** 2) / a_eff
        return max(float(self.cfg.d_agent_enter), float(self.cfg.d_safe) + float(getattr(self.cfg, "safety_pair_buffer", 0.0)) + float(getattr(self.cfg, "di_dynamic_trigger_buffer", 0.25)) + brake)

    def _dynamic_obs_enter_distance(self, r_i: np.ndarray, v_i: np.ndarray, c: np.ndarray, rad: float) -> float:
        if self.cfg.model != "double_integrator":
            return float(self.cfg.d_obs_enter)
        dt = float(self.cfg.dt)
        a_eff = max(float(getattr(self.cfg, "di_brake_accel", self.cfg.u_mag)), 1e-6)
        n = self._unit(r_i[:2] - c)
        closing = max(0.0, -float(np.dot(n, v_i[:2])))
        brake = dt * closing + 0.5 * (closing ** 2) / a_eff
        return max(float(self.cfg.d_obs_enter), float(getattr(self.cfg, "di_dynamic_trigger_buffer", 0.25)) + brake)

    def _desired_mode(self, r_all: np.ndarray, v_all: np.ndarray) -> Tuple[str, bool, bool, float, float, List[int], List[int]]:
        r_i = r_all[self.idx]
        v_i = v_all[self.idx]
        d_agent_min, nbrs = self._pairwise_distances(r_all)
        d_obs_min, _, obs_ids = self._obstacle_distances(r_i)

        pair_trigger = float("inf")
        for j in nbrs:
            pair_trigger = min(pair_trigger, self._dynamic_pair_enter_distance(r_i, v_i, r_all[j], v_all[j]))
        if pair_trigger == float("inf"):
            pair_trigger = float(self.cfg.d_agent_enter)

        obs_trigger = float("inf")
        obstacles = self._inflated_obstacles()
        for q in obs_ids:
            c, rad = obstacles[q]
            obs_trigger = min(obs_trigger, self._dynamic_obs_enter_distance(r_i, v_i, c, rad))
        if obs_trigger == float("inf"):
            obs_trigger = float(self.cfg.d_obs_enter)

        if d_agent_min <= pair_trigger:
            self.eta_C = True
        elif d_agent_min >= float(self.cfg.d_agent_exit):
            self.eta_C = False

        if d_obs_min <= obs_trigger:
            self.eta_O = True
        elif d_obs_min >= float(self.cfg.d_obs_exit):
            self.eta_O = False

        if self.eta_C and self.eta_O:
            desired = MODE_CO
        elif self.eta_C:
            desired = MODE_C
        elif self.eta_O:
            desired = MODE_O
        else:
            desired = MODE_F
        return desired, bool(self.eta_C), bool(self.eta_O), d_agent_min, d_obs_min, nbrs, obs_ids

    def _formation_hold(self, mode: str, u_nom: np.ndarray, v_i: np.ndarray) -> Optional[np.ndarray]:
        if mode != MODE_F:
            return None
        if self.cfg.model == "single_integrator":
            if self._norm(u_nom) <= float(getattr(self.cfg, "steady_u_tol_si", 0.03)):
                return np.zeros_like(u_nom)
            return None
        if self._norm(v_i) <= float(getattr(self.cfg, "steady_v_tol_di", 0.08)) and self._norm(u_nom) <= float(getattr(self.cfg, "steady_u_tol_di", 0.06)):
            return np.zeros_like(u_nom)
        if self._norm(u_nom) <= float(getattr(self.cfg, "steady_u_tol_di", 0.06)):
            return self._clip_u(-float(getattr(self.cfg, "steady_kd_di", 1.2)) * v_i)
        return None

    def _predicted_neighbor_input(self, u_nom_all: np.ndarray, j: int) -> np.ndarray:
        if u_nom_all.ndim == 2 and 0 <= j < u_nom_all.shape[0]:
            return np.asarray(u_nom_all[j], dtype=float)
        return np.zeros((self.cfg.dim,), dtype=float)

    def _mode_reference(
        self,
        effective_mode: str,
        u_nom: np.ndarray,
        bary_r: np.ndarray,
        r_all: np.ndarray,
        v_all: np.ndarray,
        pair_ids: List[int],
        obs_ids: List[int],
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        r_i = r_all[self.idx]
        v_i = v_all[self.idx]
        hold_u = self._formation_hold(effective_mode, u_nom, v_i)
        if hold_u is not None:
            return hold_u, {"waypoint": bary_r[:2].tolist() if bary_r.size >= 2 else [np.nan, np.nan], "guidance": "hold"}
        if self.cfg.model == "single_integrator" or effective_mode == MODE_F:
            return u_nom, {"waypoint": bary_r[:2].tolist() if bary_r.size >= 2 else [np.nan, np.nan], "guidance": "nominal"}

        # double-integrator safety guidance
        goal = bary_r if bary_r.shape[0] == self.cfg.dim else r_i + u_nom
        to_goal = goal - r_i
        goal_dir = self._unit(to_goal, fallback=u_nom)
        guide_vec = 0.45 * np.asarray(u_nom, dtype=float) - float(getattr(self.cfg, "di_nominal_damping", 0.35)) * v_i
        waypoint = goal[:2].tolist() if goal.size >= 2 else [np.nan, np.nan]
        guidance = []

        if effective_mode in {MODE_O, MODE_CO} and obs_ids:
            obstacles = self._inflated_obstacles()
            q_block = self._path_blocked(r_i, goal, obs_ids)
            q_use = q_block if q_block is not None else int(min(obs_ids, key=lambda q: float(np.linalg.norm(r_i[:2] - obstacles[q][0]) - obstacles[q][1])))
            c, rad = obstacles[q_use]
            outward = self._unit(r_i[:2] - c, fallback=goal_dir[:2])
            tangent = self._perp2(outward)
            sign_seed = float(np.cross(np.r_[outward, 0.0], np.r_[goal_dir[:2], 0.0])[2])
            if abs(sign_seed) <= 1e-9:
                sign_seed = float(np.cross(np.r_[outward, 0.0], np.r_[u_nom[:2], 0.0])[2])
            sign = 1.0 if sign_seed >= 0.0 else -1.0
            if q_use in self._orbit_sign:
                prev_sign = float(self._orbit_sign[q_use])
                align = float(np.dot(sign * tangent, prev_sign * tangent))
                if align < 0.0 and float(np.linalg.norm(to_goal[:2])) < 1.5 * float(self.cfg.obstacle_warning_radius):
                    sign = prev_sign
            self._orbit_sign[q_use] = sign
            tangent = sign * tangent
            clearance = float(np.linalg.norm(r_i[:2] - c) - rad)
            tangential_strength = float(getattr(self.cfg, "di_tangent_speed", 1.1))
            outward_strength = float(getattr(self.cfg, "di_outward_speed", 0.8)) * max(0.0, 1.0 - clearance / max(float(self.cfg.d_obs_exit), 1e-6))
            vel_des2 = tangential_strength * tangent + outward_strength * outward
            if float(np.dot(vel_des2, goal_dir[:2])) < -0.1:
                vel_des2 += 0.75 * goal_dir[:2]
            vel_des = np.zeros(self.cfg.dim, dtype=float)
            vel_des[:2] = vel_des2
            guide_vec += float(getattr(self.cfg, "di_guidance_kv", 1.4)) * (vel_des - v_i)
            waypoint = (c + (rad + float(getattr(self.cfg, "di_path_block_margin", 0.25)) + 0.35) * tangent).tolist()
            guidance.append("obstacle_orbit")

        if effective_mode in {MODE_C, MODE_CO} and pair_ids:
            sep_vec = np.zeros(self.cfg.dim, dtype=float)
            for j in pair_ids:
                diff = r_i - r_all[j]
                d = max(self._norm(diff), 1e-8)
                n = diff / d
                weight = max(0.0, (float(self.cfg.safety_warning_radius) - d) / max(float(self.cfg.safety_warning_radius) - float(self.cfg.d_safe), 1e-6))
                sep_vec += weight * n
            if self._norm(sep_vec) > 1e-9:
                vel_des = float(getattr(self.cfg, "di_pair_sep_speed", 0.9)) * self._unit(sep_vec)
                guide_vec += float(getattr(self.cfg, "di_pair_sep_gain", 1.6)) * (vel_des - v_i)
                guidance.append("pair_sep")

        if not guidance:
            guide_vec = np.asarray(u_nom, dtype=float) - float(getattr(self.cfg, "di_nominal_damping", 0.35)) * v_i
            guidance.append("nominal_damped")

        return self._clip_u(guide_vec), {"waypoint": waypoint, "guidance": "+".join(guidance)}

    def _qp_filter(
        self,
        u_ref: np.ndarray,
        r_all: np.ndarray,
        v_all: np.ndarray,
        u_nom_all: np.ndarray,
        pair_ids: List[int],
        obs_ids: List[int],
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        dt = float(self.cfg.dt)
        r_i = r_all[self.idx]
        v_i = v_all[self.idx]
        dim = self.cfg.dim
        d_safe = float(self.cfg.d_safe)
        pair_margin = float(getattr(self.cfg, "safety_pair_buffer", 0.03))
        obs_margin = float(getattr(self.cfg, "safety_obs_buffer", 0.03))
        q_vel = float(getattr(self.cfg, "safety_qp_w_vel", 0.5))
        q_u = float(getattr(self.cfg, "safety_qp_w_u", 1.0))
        kappa_pair = float(getattr(self.cfg, "di_kappa_pair", 0.55))
        kappa_obs = float(getattr(self.cfg, "di_kappa_obs", 0.55))

        pair_num = len(pair_ids)
        circle_num = len(obs_ids)

        if pair_num == 0 and circle_num == 0:
            return self._clip_u(u_ref), {
                "solver_status": "inactive",
                "_h_pair_num": 0,
                "_circle_barrier_num": 0,
                "fallback": False,
                "used_two_step_filter": self.cfg.model == "double_integrator",
            }

        if cp is None:
            return self._emergency_fallback(u_ref, r_all, v_all, pair_ids, obs_ids), {
                "solver_status": "cvxpy_unavailable",
                "_h_pair_num": pair_num,
                "_circle_barrier_num": circle_num,
                "fallback": True,
                "used_two_step_filter": self.cfg.model == "double_integrator",
            }

        u = cp.Variable(dim)
        cons = [u >= self.cfg.u_min, u <= self.cfg.u_max]
        if float(getattr(self.cfg, "u_mag", 0.0)) > 0.0:
            cons += [cp.norm(u, 2) <= float(self.cfg.u_mag)]

        if self.cfg.model == "single_integrator":
            p_i_aff = r_i + dt * u
        else:
            p_i_aff = r_i + 2.0 * dt * v_i + (dt * dt) * u
            v_i_next = v_i + dt * u

        for j in pair_ids:
            r_j = r_all[j]
            v_j = v_all[j]
            u_j_nom = self._predicted_neighbor_input(u_nom_all, j)
            delta = r_i - r_j
            nvec = self._unit(delta)
            if self.cfg.model == "single_integrator":
                p_j_pred = r_j + dt * u_j_nom
            else:
                p_j_pred = r_j + 2.0 * dt * v_j + (dt * dt) * u_j_nom
                v_j_next = v_j + dt * u_j_nom
                slack = max(float(np.linalg.norm(delta)) - (d_safe + pair_margin), 0.0)
                cons += [nvec @ (v_i_next - v_j_next) >= -kappa_pair * slack]
            cons += [nvec @ (p_i_aff - p_j_pred) >= d_safe + pair_margin]

        obstacles = self._inflated_obstacles()
        for q in obs_ids:
            c, rad = obstacles[q]
            outward = self._unit(r_i[:2] - c)
            if self.cfg.model == "double_integrator":
                slack = max(float(np.linalg.norm(r_i[:2] - c)) - rad - obs_margin, 0.0)
                cons += [outward @ (v_i_next[:2]) >= -kappa_obs * slack]
            cons += [outward @ (p_i_aff[:2] - c) >= rad + obs_margin]

        obj = q_u * cp.sum_squares(u - u_ref)
        if self.cfg.model == "double_integrator":
            obj += q_vel * cp.sum_squares(v_i_next)
        prob = cp.Problem(cp.Minimize(obj), cons)

        status = "error"
        for solver in ("ECOS", "OSQP", "SCS"):
            try:
                prob.solve(solver=getattr(cp, solver), warm_start=True, verbose=False)
                status = str(prob.status)
                if prob.status in ("optimal", "optimal_inaccurate") and u.value is not None:
                    break
            except Exception:
                status = f"{solver.lower()}_failed"
        if prob.status not in ("optimal", "optimal_inaccurate") or u.value is None:
            return self._emergency_fallback(u_ref, r_all, v_all, pair_ids, obs_ids), {
                "solver_status": status,
                "_h_pair_num": pair_num,
                "_circle_barrier_num": circle_num,
                "fallback": True,
                "used_two_step_filter": self.cfg.model == "double_integrator",
            }

        u_sol = self._clip_u(np.asarray(u.value).reshape(dim))
        return u_sol, {
            "solver_status": status,
            "_h_pair_num": pair_num,
            "_circle_barrier_num": circle_num,
            "fallback": False,
            "used_two_step_filter": self.cfg.model == "double_integrator",
        }

    def _emergency_fallback(self, u_ref: np.ndarray, r_all: np.ndarray, v_all: np.ndarray, pair_ids: List[int], obs_ids: List[int]) -> np.ndarray:
        r_i = r_all[self.idx]
        v_i = v_all[self.idx]
        u = np.asarray(u_ref, dtype=float).copy()

        for j in pair_ids:
            diff = r_i - r_all[j]
            dirn = self._unit(diff)
            gain = float(getattr(self.cfg, "emergency_pair_gain", 2.0))
            u += gain * dirn

        for q in obs_ids:
            c, _ = self._inflated_obstacles()[q]
            diff2 = r_i[:2] - c
            dir2 = self._unit(diff2)
            gain = float(getattr(self.cfg, "emergency_obs_gain", 2.5))
            u[:2] += gain * dir2

        if self.cfg.model == "double_integrator":
            u += -float(getattr(self.cfg, "emergency_damping_di", 1.8)) * v_i
        return self._clip_u(u)

    def _transition_law(self, u_old: np.ndarray, u_new: np.ndarray, r_all: np.ndarray, v_all: np.ndarray, u_nom_all: np.ndarray, pair_ids: List[int], obs_ids: List[int]) -> np.ndarray:
        steps = int(getattr(self.cfg, "transition_steps", 1))
        if steps <= 1:
            return u_new
        s = min(self.transition_counter + 1, steps) / float(steps)
        lam0 = float(getattr(self.cfg, "transition_lambda_start", 0.35))
        lam1 = float(getattr(self.cfg, "transition_lambda_end", 1.0))
        lam = (1.0 - s) * lam0 + s * lam1
        u_blend = self._clip_u((1.0 - lam) * u_old + lam * u_new)
        u_checked, info = self._qp_filter(u_blend, r_all, v_all, u_nom_all, pair_ids, obs_ids)
        if info.get("fallback", False):
            return u_new
        return u_checked

    def step(self, payload: Dict[str, Any]) -> HybridResult:
        r_all = np.asarray(payload["r_all"], dtype=float)
        v_all = np.asarray(payload["v_all"], dtype=float)
        u_nom = np.asarray(payload["u_nom"], dtype=float)
        u_nom_all = np.asarray(payload.get("u_nom_all", np.zeros_like(r_all)), dtype=float)
        bary_r = np.asarray(payload.get("bary_r", r_all[self.idx]), dtype=float)

        desired, eta_C, eta_O, d_agent_min, d_obs_min, pair_ids, obs_ids = self._desired_mode(r_all, v_all)

        if self.mode == MODE_T:
            self.transition_counter += 1
            if self.transition_counter >= int(getattr(self.cfg, "transition_steps", 1)):
                self.mode = self.pending_mode if self.pending_mode is not None else desired
                self.pending_mode = None
                self.transition_counter = 0
        else:
            if desired != self.mode:
                self.pending_mode = desired
                self.mode = MODE_T
                self.transition_counter = 0

        effective_mode = self.pending_mode if (self.mode == MODE_T and self.pending_mode is not None) else self.mode
        u_ref, ref_info = self._mode_reference(effective_mode, u_nom, bary_r, r_all, v_all, pair_ids, obs_ids)

        if effective_mode == MODE_F:
            u_target = self._clip_u(u_ref)
            qp_info = {
                "solver_status": "mode_F",
                "_h_pair_num": 0,
                "_circle_barrier_num": 0,
                "fallback": False,
                "used_two_step_filter": self.cfg.model == "double_integrator",
            }
        else:
            active_pairs = pair_ids if effective_mode in {MODE_C, MODE_CO} else []
            active_obs = obs_ids if effective_mode in {MODE_O, MODE_CO} else []
            u_target, qp_info = self._qp_filter(u_ref, r_all, v_all, u_nom_all, active_pairs, active_obs)

        if self.mode == MODE_T:
            u_safe = self._transition_law(self.last_u, u_target, r_all, v_all, u_nom_all, pair_ids, obs_ids)
        else:
            u_safe = u_target

        self.last_target_u = np.asarray(u_target, dtype=float).copy()
        self.last_u = np.asarray(u_safe, dtype=float).copy()

        diag = {
            "mode": self.mode,
            "desired_mode": desired,
            "effective_mode": effective_mode,
            "eta_C": int(bool(eta_C)),
            "eta_O": int(bool(eta_O)),
            "d_agent_min": float(d_agent_min),
            "d_obs_min": float(d_obs_min),
            "active_pairs": int(len(pair_ids)),
            "active_obstacles": int(len(obs_ids)),
            "target_waypoint": ref_info.get("waypoint", [np.nan, np.nan]),
            "guidance": ref_info.get("guidance", ""),
            **qp_info,
        }
        return HybridResult(u_safe=u_safe, diag=diag)
