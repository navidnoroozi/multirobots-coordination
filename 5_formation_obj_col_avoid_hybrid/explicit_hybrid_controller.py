from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

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
    def __init__(self, cfg: NetConfig, agent_id: int):
        self.cfg = cfg
        self.agent_id = int(agent_id)
        self.idx = self.agent_id - 1
        self.mode = MODE_F
        self.pending_mode: Optional[str] = None
        self.transition_counter = 0
        self.last_u = np.zeros((cfg.dim,), dtype=float)

    @staticmethod
    def _clip(u: np.ndarray, lo: float, hi: float) -> np.ndarray:
        return np.minimum(np.maximum(u, lo), hi)

    @staticmethod
    def _norm(x: np.ndarray) -> float:
        return float(np.linalg.norm(x))

    def _inflated_obstacles(self) -> List[Tuple[float, float, float]]:
        if not self.cfg.obstacles_enabled:
            return []
        return [
            (float(cx), float(cy), float(rad) + float(self.cfg.obstacle_margin))
            for (cx, cy, rad) in self.cfg.obstacles_circles
        ]

    def _local_safety_neighbors(self, r_all: np.ndarray) -> List[int]:
        out: List[int] = []
        rw = float(self.cfg.safety_warning_radius)
        ri = r_all[self.idx]
        for j in range(r_all.shape[0]):
            if j == self.idx:
                continue
            if float(np.linalg.norm(ri - r_all[j])) <= rw:
                out.append(j)
        return out

    def _local_obstacles(self, r_i: np.ndarray) -> List[Tuple[float, float, float, float]]:
        out = []
        for obs_idx, (cx, cy, rad_eff) in enumerate(self._inflated_obstacles()):
            d = float(np.linalg.norm(r_i[:2] - np.array([cx, cy], dtype=float)) - rad_eff)
            if d <= float(self.cfg.obstacle_warning_radius):
                out.append((cx, cy, rad_eff, float(obs_idx)))
        return out

    def _agent_margin(self, r_all: np.ndarray) -> tuple[float, List[int]]:
        nbrs = self._local_safety_neighbors(r_all)
        if len(nbrs) == 0:
            return float("inf"), []
        ri = r_all[self.idx]
        dvals = [float(np.linalg.norm(ri - r_all[j])) for j in nbrs]
        return min(dvals), nbrs

    def _obstacle_margin(self, r_i: np.ndarray) -> tuple[float, Optional[int], List[Tuple[float, float, float, float]]]:
        """
        Returns the minimum distance to obstacles, the index of the closest obstacle, and the list of local obstacles.
        If there are no local obstacles, returns infinity for the distance and None for the index.
        """
        local_obs = self._local_obstacles(r_i)
        if len(local_obs) == 0:
            return float("inf"), None, []
        dvals = [float(np.linalg.norm(r_i[:2] - np.array([cx, cy], dtype=float)) - rad_eff) for (cx, cy, rad_eff, _) in local_obs]
        k = int(np.argmin(dvals))
        return dvals[k], int(local_obs[k][3]), local_obs

    def _segment_intersects_circle(self, a: np.ndarray, b: np.ndarray, circle: Tuple[float, float, float]) -> bool:
        c = np.array([circle[0], circle[1]], dtype=float)
        R = float(circle[2])
        a2 = a[:2]
        b2 = b[:2]
        ab = b2 - a2
        denom = float(np.dot(ab, ab))
        if denom <= 1e-12:
            return float(np.linalg.norm(a2 - c)) <= R
        t = float(np.dot(c - a2, ab) / denom)
        t = max(0.0, min(1.0, t))
        proj = a2 + t * ab
        return float(np.linalg.norm(proj - c)) <= R

    def _path_obstructed(self, r_i: np.ndarray, target: np.ndarray) -> tuple[bool, Optional[int]]:
        for (cx, cy, rad_eff) in self._inflated_obstacles():
            if self._segment_intersects_circle(r_i, target, (cx, cy, rad_eff)):
                idx = None
                for k, obs in enumerate(self._inflated_obstacles()):
                    if abs(obs[0] - cx) < 1e-12 and abs(obs[1] - cy) < 1e-12 and abs(obs[2] - rad_eff) < 1e-12:
                        idx = k
                        break
                return True, idx
        return False, None

    def _projection_on_circle(self, p: np.ndarray, circle: Tuple[float, float, float]) -> np.ndarray:
        c = np.array([circle[0], circle[1]], dtype=float)
        v = p[:2] - c
        nrm = self._norm(v)
        if nrm <= 1e-9:
            v = np.array([1.0, 0.0], dtype=float)
            nrm = 1.0
        return c + (float(circle[2]) / nrm) * v

    def _tangent_waypoint(self, r_i: np.ndarray, target: np.ndarray, circle: Tuple[float, float, float]) -> np.ndarray:
        """
        Computes a tangential waypoint on the circle around the obstacle to help the agent navigate around it while moving towards the target.
        Mathematically, it first projects the agent's current position onto the circle to find the closest point on the circle.
        Then, it computes the normal vector from the agent to this projection point and normalizes it.
        Using this normal vector, it calculates two potential tangential waypoints by rotating the normal vector 90 degrees in both directions (clockwise and counterclockwise) and scaling it by a specified radius.
        Finally, it selects the tangential waypoint that is closer to the target position and returns it as the desired waypoint for navigation.
        """
        proj = self._projection_on_circle(r_i, circle)
        n = r_i[:2] - proj
        nrm = self._norm(n)
        if nrm <= 1e-9:
            n = np.array([1.0, 0.0], dtype=float)
            nrm = 1.0
        n = n / nrm

        Rcw = np.array([[0.0, 1.0], [-1.0, 0.0]])
        Rccw = np.array([[0.0, -1.0], [1.0, 0.0]])

        t_plus = Rccw @ n
        t_minus = Rcw @ n
        rho = float(self.cfg.tangential_waypoint_radius)

        tau_plus = proj + rho * t_plus
        tau_minus = proj + rho * t_minus

        tau2 = tau_plus if self._norm(tau_plus - target[:2]) <= self._norm(tau_minus - target[:2]) else tau_minus

        tau = np.zeros((self.cfg.dim,), dtype=float)
        tau[:2] = tau2
        tau = self._clip(tau, self.cfg.r_min, self.cfg.r_max)
        return tau

    def _repulsion_vector(self, r_all: np.ndarray, nbrs: List[int]) -> np.ndarray:
        """
        Computes a repulsion vector based on the positions of neighboring agents to help maintain a safe distance from them.
        For each neighboring agent, it calculates the difference in position and the distance to that agent.
        If the distance is very small (indicating a potential collision), it generates a repulsion vector in a random direction to help separate the agents. If the distance is within a specified exit threshold, it adds a repulsion component that increases as the distance decreases, encouraging the agent to move away from its neighbors.
        The resulting repulsion vector is returned as an output.
        Mathematically, the repulsion vector is computed as the sum of contributions from each neighboring agent, where each contribution is calculated as:
        rep_j = (1/d - 1/d_exit) * (diff / d)
        """
        if len(nbrs) == 0:
            return np.zeros((self.cfg.dim,), dtype=float)

        ri = r_all[self.idx]
        out = np.zeros((self.cfg.dim,), dtype=float)
        for j in nbrs:
            diff = ri - r_all[j]
            d = self._norm(diff)
            if d <= 1e-9:
                angle = 2.0 * np.pi * (self.idx / max(r_all.shape[0], 1))
                diff = np.array([np.cos(angle), np.sin(angle)], dtype=float)
                if self.cfg.dim > 2:
                    diff = np.pad(diff, (0, self.cfg.dim - 2))
                d = self._norm(diff)
            if d < float(self.cfg.d_agent_exit):
                out += (1.0 / max(d, 1e-6) - 1.0 / float(self.cfg.d_agent_exit)) * diff / max(d, 1e-6)
        return out

    def _single_mode_control(self, mode: str, r_all: np.ndarray, u_nom: np.ndarray, bary_r: np.ndarray, nbrs: List[int], closest_obs_idx: Optional[int]) -> tuple[np.ndarray, Optional[np.ndarray]]:
        ri = r_all[self.idx]
        rep = self._repulsion_vector(r_all, nbrs)

        target_waypoint = None
        u = u_nom.copy()

        if mode == MODE_F:
            u = u_nom.copy()

        elif mode == MODE_C:
            u = float(self.cfg.nominal_blend_C_si) * u_nom + float(self.cfg.modeC_repulsion_gain_si) * rep

        elif mode == MODE_O:
            if closest_obs_idx is not None:
                circle = self._inflated_obstacles()[closest_obs_idx]
                tau = self._tangent_waypoint(ri, bary_r, circle)
                target_waypoint = tau
                u = float(self.cfg.nominal_blend_O_si) * u_nom + float(self.cfg.modeO_target_gain_si) * (tau - ri)

        elif mode == MODE_CO:
            """
            In mode CO, the control input is computed as a blend of the nominal control input, a target-seeking component that directs the agent towards a tangential waypoint around the closest obstacle, and a repulsion component that helps maintain a safe distance from neighboring agents.
            If there is a closest obstacle, the controller calculates a tangential waypoint on the circle around the obstacle and blends the control input accordingly.
            If there is no closest obstacle, it blends only the nominal control input and the repulsion component.
            Mathematically, the control input u in mode CO is computed as:
            If there is a closest obstacle:
            u = nominal_blend_CO_si * u_nom + modeCO_target_gain_si * (tau - ri) + modeCO_repulsion_gain_si * rep
            If there is no closest obstacle:
            u = nominal_blend_CO_si * u_nom + modeCO_repulsion_gain_si * rep
            where tau is the tangential waypoint computed based on the closest obstacle, ri is the current position of the agent, u_nom is the nominal control input, rep is the repulsion vector from neighboring agents, and the various gain parameters determine the weighting of each component in the blended control input.
            """
            if closest_obs_idx is not None:
                circle = self._inflated_obstacles()[closest_obs_idx]
                tau = self._tangent_waypoint(ri, bary_r, circle)
                target_waypoint = tau
                u = (
                    float(self.cfg.nominal_blend_CO_si) * u_nom
                    + float(self.cfg.modeCO_target_gain_si) * (tau - ri)
                    + float(self.cfg.modeCO_repulsion_gain_si) * rep
                )
            else:
                u = float(self.cfg.nominal_blend_CO_si) * u_nom + float(self.cfg.modeCO_repulsion_gain_si) * rep

        return self._clip(u, self.cfg.u_min, self.cfg.u_max), target_waypoint

    def _double_mode_control(self, mode: str, r_all: np.ndarray, v_all: np.ndarray, u_nom: np.ndarray, bary_r: np.ndarray, nbrs: List[int], closest_obs_idx: Optional[int]) -> tuple[np.ndarray, Optional[np.ndarray]]:
        ri = r_all[self.idx]
        vi = v_all[self.idx]
        rep = self._repulsion_vector(r_all, nbrs)

        target_waypoint = None
        u = u_nom.copy()

        if mode == MODE_F:
            u = u_nom.copy()

        elif mode == MODE_C:
            u = (
                float(self.cfg.nominal_blend_C_di) * u_nom
                + float(self.cfg.modeC_repulsion_gain_di) * rep
                - float(self.cfg.modeC_damping_di) * vi
            )

        elif mode == MODE_O:
            if closest_obs_idx is not None:
                circle = self._inflated_obstacles()[closest_obs_idx]
                tau = self._tangent_waypoint(ri, bary_r, circle)
                target_waypoint = tau
                u = (
                    float(self.cfg.nominal_blend_O_di) * u_nom
                    + float(self.cfg.modeO_kp_di) * (tau - ri)
                    - float(self.cfg.modeO_kd_di) * vi
                )

        elif mode == MODE_CO:
            if closest_obs_idx is not None:
                circle = self._inflated_obstacles()[closest_obs_idx]
                tau = self._tangent_waypoint(ri, bary_r, circle)
                target_waypoint = tau
                u = (
                    float(self.cfg.nominal_blend_CO_di) * u_nom
                    + float(self.cfg.modeCO_kp_di) * (tau - ri)
                    - float(self.cfg.modeCO_kd_di) * vi
                    + float(self.cfg.modeC_repulsion_gain_di) * rep
                )
            else:
                u = (
                    float(self.cfg.nominal_blend_CO_di) * u_nom
                    + float(self.cfg.modeC_repulsion_gain_di) * rep
                    - float(self.cfg.modeC_damping_di) * vi
                )

        return self._clip(u, self.cfg.u_min, self.cfg.u_max), target_waypoint

    def _desired_mode(self, r_all: np.ndarray, bary_r: np.ndarray) -> tuple[str, float, float, List[int], Optional[int], bool]:
        r_i = r_all[self.idx]

        d_agent_min, nbrs = self._agent_margin(r_all)
        d_obs_min, obs_idx, _ = self._obstacle_margin(r_i)
        obstructed, obs_idx_from_path = self._path_obstructed(r_i, bary_r)

        if obs_idx is None and obs_idx_from_path is not None:
            obs_idx = obs_idx_from_path

        active_C = False
        active_O = False

        if d_agent_min <= float(self.cfg.d_agent_enter):
            active_C = True
        elif self.mode in {MODE_C, MODE_CO} and d_agent_min <= float(self.cfg.d_agent_exit):
            active_C = True

        if d_obs_min <= float(self.cfg.d_obs_enter) or obstructed:
            active_O = True
        elif self.mode in {MODE_O, MODE_CO} and d_obs_min <= float(self.cfg.d_obs_exit):
            active_O = True

        if active_C and active_O:
            desired = MODE_CO
        elif active_C:
            desired = MODE_C
        elif active_O:
            desired = MODE_O
        else:
            desired = MODE_F

        return desired, d_agent_min, d_obs_min, nbrs, obs_idx, obstructed

    def _blend_transition(self, u_target: np.ndarray) -> np.ndarray:
        """
        Blends the control input during a transition between modes using a lambda parameter that changes over the course of the transition steps.
        The blending is done between the last control input and the target control input for the new mode, with the lambda parameter determining the weighting of each. If the number of transition steps is 1 or less, it will immediately switch to the target control input without blending.
        Mathematically, the blended control input u is computed as:
        u = (1 - lam) * last_u + lam * u_target
        where lam is the transition lambda parameter that changes from transition_lambda_start to transition_lambda_end over the course of transition_steps.
        """
        if int(self.cfg.transition_steps) <= 1:
            # If transition_steps is 1 or less, immediately switch to the target control input without blending
            lam = float(self.cfg.transition_lambda_end)
        else:
            # Compute the transition lambda parameter based on the current transition step, transitioning from transition_lambda_start to transition_lambda_end over the course of transition_steps
            # s is a normalized value in the range [0, 1] that represents the progress of the transition, where 0 corresponds to the start of the transition and 1 corresponds to the end of the transition.
            # It is computed as the current transition step (transition_counter + 1) divided by the total number of transition steps (transition_steps), and is capped at 1 to ensure it does not exceed the end of the transition.
            s = min(self.transition_counter + 1, int(self.cfg.transition_steps)) / float(int(self.cfg.transition_steps))
            lam = (1.0 - s) * float(self.cfg.transition_lambda_start) + s * float(self.cfg.transition_lambda_end)
        u = (1.0 - lam) * self.last_u + lam * u_target
        return self._clip(u, self.cfg.u_min, self.cfg.u_max)

    def step(self, payload: Dict[str, Any]) -> HybridResult:
        """
        Performs a single step of the hybrid controller.
        It first extracts the necessary information from the input payload, including the positions and velocities of all agents, the nominal control input, and the barycenter position.
        Then, it determines the desired mode based on the current state of the system, including the minimum distances to neighboring agents and obstacles, and whether the path to the target is obstructed.
        Mathematically, the desired mode is determined based on the following conditions:
        - If the minimum distance to neighboring agents is less than or equal to d_agent_enter, the desired mode is set to MODE_C (collision avoidance mode).
        - If the minimum distance to obstacles is less than or equal to d_obs_enter or if the path to the target is obstructed, the desired mode is set to MODE_O (obstacle avoidance mode).
        - If both conditions are true, the desired mode is set to MODE_CO (combined collision and obstacle avoidance mode).
        - If neither condition is true, the desired mode is set to MODE_F (formation control mode).
        Output control inputs are then computed based on the current mode, and if the controller is in a transition state (MODE_T), it blends the control input accordingly. 
        Finally, it returns the safe control input along with diagnostic information about the current mode and state of the system.
        Outputs:
        - u_safe: The safe control input computed by the controller after considering the current mode and blending if in transition.
        - diag: A dictionary containing diagnostic information about the current mode, desired mode, effective mode
        """
        r_all = np.array(payload["r_all"], dtype=float)
        v_all = np.array(payload["v_all"], dtype=float)
        u_nom = np.array(payload["u_nom"], dtype=float)
        bary_r = np.array(payload["bary_r"], dtype=float)

        desired, d_agent_min, d_obs_min, nbrs, obs_idx, obstructed = self._desired_mode(r_all, bary_r)

        if self.mode == MODE_T:
            self.transition_counter += 1
            if self.transition_counter >= int(self.cfg.transition_steps):
                self.mode = self.pending_mode if self.pending_mode is not None else desired
                self.pending_mode = None
                self.transition_counter = 0
        else:
            if desired != self.mode:
                self.pending_mode = desired
                self.mode = MODE_T
                self.transition_counter = 0

        eval_mode = self.pending_mode if self.mode == MODE_T and self.pending_mode is not None else self.mode

        if self.cfg.model == "single_integrator":
            u_target, target_waypoint = self._single_mode_control(eval_mode, r_all, u_nom, bary_r, nbrs, obs_idx)
        else:
            u_target, target_waypoint = self._double_mode_control(eval_mode, r_all, v_all, u_nom, bary_r, nbrs, obs_idx)

        if self.mode == MODE_T:
            u_safe = self._blend_transition(u_target)
        else:
            u_safe = self._clip(u_target, self.cfg.u_min, self.cfg.u_max)

        self.last_u = u_safe.copy()

        diag = {
            "mode": self.mode,
            "desired_mode": desired,
            "effective_mode": eval_mode,
            "d_agent_min": float(d_agent_min),
            "d_obs_min": float(d_obs_min),
            "active_pairs": int(len(nbrs)),
            "active_obstacles": int(0 if obs_idx is None else 1),
            "obstructed": bool(obstructed),
            "target_waypoint": target_waypoint.tolist() if target_waypoint is not None else [np.nan, np.nan],
        }
        return HybridResult(u_safe=u_safe, diag=diag)