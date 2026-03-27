from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List

import numpy as np

from consensus_config import NetConfig

MODE_NOMINAL = 1
MODE_HOLD = 2
MODE_EMERGENCY = 3


@dataclass
class HybridStepResult:
    u_cmd: np.ndarray
    diag: Dict[str, Any]


class HybridController:
    """Per-agent hybrid controller wrapping the nominal DMPC command."""

    def __init__(self, cfg: NetConfig, agent_index: int):
        self.cfg = cfg
        self.i = int(agent_index)
        self.mode = MODE_NOMINAL
        self.mode_steps = 0
        self.anchor_offset = np.zeros((cfg.dim,), dtype=float)
        self.anchor_valid = False

    def _clip(self, u: np.ndarray) -> np.ndarray:
        return np.minimum(np.maximum(u, self.cfg.u_min), self.cfg.u_max)

    @staticmethod
    def _norm(x: np.ndarray) -> float:
        return float(np.linalg.norm(x))

    def _gains(self) -> Dict[str, float]:
        if self.cfg.model == "single_integrator":
            return {
                "hold_position_gain": self.cfg.hold_position_gain_si,
                "hold_repulsion_gain": self.cfg.hold_repulsion_gain_si,
                "hold_velocity_damping": self.cfg.hold_velocity_damping_si,
                "emergency_repulsion_gain": self.cfg.emergency_repulsion_gain_si,
                "emergency_direction_gain": self.cfg.emergency_direction_gain_si,
                "emergency_velocity_damping": self.cfg.emergency_velocity_damping_si,
                "hold_max_corr": self.cfg.hybrid_hold_max_correction_si,
                "emergency_max_corr": self.cfg.hybrid_emergency_max_correction_si,
            }
        return {
            "hold_position_gain": self.cfg.hold_position_gain_di,
            "hold_repulsion_gain": self.cfg.hold_repulsion_gain_di,
            "hold_velocity_damping": self.cfg.hold_velocity_damping_di,
            "emergency_repulsion_gain": self.cfg.emergency_repulsion_gain_di,
            "emergency_direction_gain": self.cfg.emergency_direction_gain_di,
            "emergency_velocity_damping": self.cfg.emergency_velocity_damping_di,
            "hold_max_corr": self.cfg.hybrid_hold_max_correction_di,
            "emergency_max_corr": self.cfg.hybrid_emergency_max_correction_di,
        }

    def _neighbors_within(self, r: np.ndarray, radius: float) -> List[int]:
        out: List[int] = []
        ri = r[self.i]
        for j in range(r.shape[0]):
            if j == self.i:
                continue
            if self._norm(ri - r[j]) <= radius:
                out.append(j)
        return out

    def _dmin_local(self, r: np.ndarray, neighbors: List[int]) -> float:
        if not neighbors:
            return float("inf")
        ri = r[self.i]
        return min(self._norm(ri - r[j]) for j in neighbors)

    def _local_diameter(self, r: np.ndarray, neighbors: List[int]) -> float:
        pts = [r[self.i]] + [r[j] for j in neighbors]
        if len(pts) <= 1:
            return 0.0
        dmax = 0.0
        for a in range(len(pts)):
            for b in range(a + 1, len(pts)):
                dmax = max(dmax, self._norm(pts[a] - pts[b]))
        return dmax

    def _nearest_neighbor_dir(self, r: np.ndarray, neighbors: List[int]) -> np.ndarray:
        if not neighbors:
            return np.zeros((self.cfg.dim,), dtype=float)
        ri = r[self.i]
        j_star = min(neighbors, key=lambda j: self._norm(ri - r[j]))
        diff = ri - r[j_star]
        nrm = self._norm(diff)
        if nrm <= 1e-9:
            e = np.zeros((self.cfg.dim,), dtype=float)
            e[self.i % self.cfg.dim] = 1.0
            return e
        return diff / nrm

    def _repulsion(self, r: np.ndarray, neighbors: List[int], radius: float, gain: float) -> np.ndarray:
        ri = r[self.i]
        acc = np.zeros((self.cfg.dim,), dtype=float)
        eps = 1e-6
        for j in neighbors:
            diff = ri - r[j]
            rho = self._norm(diff)
            if rho <= eps or rho >= radius:
                continue
            dir_away = diff / rho
            scale = gain * ((1.0 / max(rho, eps)) - (1.0 / radius)) / max(rho * rho, eps)
            weight = max(0.0, min(1.0, (radius - rho) / max(radius - self.cfg.d_safe, 1e-6)))
            acc += weight * scale * dir_away
        return acc

    def _store_anchor(self, r: np.ndarray) -> None:
        bary = np.mean(r, axis=0)
        raw = r[self.i] - bary
        nrm = self._norm(raw)
        target_radius = max(self.cfg.hybrid_anchor_min_radius, min(self.cfg.hybrid_anchor_max_radius, nrm))
        if nrm <= 1e-9:
            angle = 2.0 * np.pi * (self.i / max(r.shape[0], 1))
            raw = np.array([np.cos(angle), np.sin(angle)], dtype=float)
            if self.cfg.dim > 2:
                raw = np.pad(raw, (0, self.cfg.dim - 2))
            nrm = self._norm(raw)
        self.anchor_offset = (target_radius / max(nrm, 1e-9)) * raw
        self.anchor_valid = True

    def _desired_mode(self, r: np.ndarray, v: np.ndarray, u_nom: np.ndarray) -> tuple[int, Dict[str, float]]:
        warn_neighbors = self._neighbors_within(r, self.cfg.d_warn_up)
        dmin = self._dmin_local(r, warn_neighbors)
        ldiam = self._local_diameter(r, warn_neighbors)

        desired = self.mode
        if self.mode == MODE_NOMINAL:
            if dmin <= self.cfg.d_warn_down or ldiam <= self.cfg.cluster_diam_hold:
                desired = MODE_HOLD
        elif self.mode == MODE_HOLD:
            if dmin <= self.cfg.d_hold_down:
                desired = MODE_EMERGENCY
            elif dmin >= self.cfg.d_warn_up and ldiam >= self.cfg.cluster_diam_release:
                desired = MODE_NOMINAL
            else:
                desired = MODE_HOLD
        else:
            if dmin >= self.cfg.d_hold_up:
                desired = MODE_HOLD
            else:
                desired = MODE_EMERGENCY

        return desired, {"dmin_local": dmin, "local_diameter": ldiam, "n_warn_neighbors": len(warn_neighbors)}

    def _respect_dwell(self, desired: int) -> int:
        if desired == self.mode:
            return desired
        min_dwell = {
            MODE_NOMINAL: self.cfg.nominal_dwell_min,
            MODE_HOLD: self.cfg.hold_dwell_min,
            MODE_EMERGENCY: self.cfg.emergency_dwell_min,
        }[self.mode]
        if self.mode_steps < int(min_dwell):
            return self.mode
        return desired

    def _hold_control(self, r: np.ndarray, v: np.ndarray, u_nom: np.ndarray, neighbors: List[int]) -> np.ndarray:
        g = self._gains()
        if not self.anchor_valid:
            self._store_anchor(r)
        bary = np.mean(r, axis=0)
        target = bary + self.anchor_offset
        pos_term = g["hold_position_gain"] * (target - r[self.i])
        rep_term = self._repulsion(r, neighbors, self.cfg.d_hold_up, g["hold_repulsion_gain"])
        if self.cfg.model == "single_integrator":
            u = pos_term + rep_term
        else:
            vel_term = -g["hold_velocity_damping"] * v[self.i]
            u = pos_term + vel_term + rep_term
        u = self._clip(u)
        delta = u - u_nom[self.i]
        nrm = self._norm(delta)
        if nrm > g["hold_max_corr"]:
            u = u_nom[self.i] + (g["hold_max_corr"] / max(nrm, 1e-9)) * delta
        return self._clip(u)

    def _emergency_control(self, r: np.ndarray, v: np.ndarray, u_nom: np.ndarray, neighbors: List[int]) -> np.ndarray:
        g = self._gains()
        diff_dir = self._nearest_neighbor_dir(r, neighbors)
        rep_term = self._repulsion(r, neighbors, self.cfg.d_hold_up, g["emergency_repulsion_gain"])
        emergency_push = g["emergency_direction_gain"] * diff_dir
        if self.cfg.model == "single_integrator":
            u = emergency_push + rep_term
        else:
            vel_term = -g["emergency_velocity_damping"] * v[self.i]
            u = emergency_push + rep_term + vel_term
        u = self._clip(u)
        delta = u - u_nom[self.i]
        nrm = self._norm(delta)
        if nrm > g["emergency_max_corr"]:
            u = u_nom[self.i] + (g["emergency_max_corr"] / max(nrm, 1e-9)) * delta
        return self._clip(u)

    def step(self, r: np.ndarray, v: np.ndarray, u_nom_all: np.ndarray) -> HybridStepResult:
        neighbors = self._neighbors_within(r, self.cfg.d_warn_up)
        desired, ind = self._desired_mode(r, v, u_nom_all)
        desired = self._respect_dwell(desired)
        switched = desired != self.mode
        if switched:
            self.mode = desired
            self.mode_steps = 0
            if self.mode == MODE_HOLD:
                self._store_anchor(r)
            if self.mode == MODE_NOMINAL:
                self.anchor_valid = False
        self.mode_steps += 1

        if self.mode == MODE_NOMINAL or not self.cfg.hybrid_enabled:
            u_cmd = self._clip(u_nom_all[self.i])
        elif self.mode == MODE_HOLD:
            hold_neighbors = self._neighbors_within(r, self.cfg.d_hold_up)
            u_cmd = self._hold_control(r, v, u_nom_all, hold_neighbors)
        else:
            emer_neighbors = self._neighbors_within(r, self.cfg.d_hold_up)
            u_cmd = self._emergency_control(r, v, u_nom_all, emer_neighbors)

        diag = {
            "mode": int(self.mode),
            "mode_name": {1: "nominal", 2: "hold", 3: "emergency"}[self.mode],
            "switched": bool(switched),
            "mode_steps": int(self.mode_steps),
            "dmin_local": float(ind["dmin_local"]),
            "local_diameter": float(ind["local_diameter"]),
            "n_warn_neighbors": int(ind["n_warn_neighbors"]),
            "anchor_norm": float(self._norm(self.anchor_offset)) if self.anchor_valid else 0.0,
        }
        return HybridStepResult(u_cmd=u_cmd, diag=diag)