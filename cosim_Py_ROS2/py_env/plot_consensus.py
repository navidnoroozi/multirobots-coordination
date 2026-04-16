from __future__ import annotations

from pathlib import Path
import csv

import numpy as np
import matplotlib.pyplot as plt

from consensus_config import NetConfig


def _load_csv_float(path: Path) -> np.ndarray:
    return np.genfromtxt(path, delimiter=",", skip_header=1, dtype=float)


def _load_csv_rows(path: Path):
    with path.open("r", newline="", encoding="utf-8") as f:
        rd = csv.reader(f)
        rows = list(rd)
    if len(rows) <= 1:
        return [], []
    return rows[0], rows[1:]


def _pairwise_diameter(pts: np.ndarray) -> float:
    if pts.shape[0] <= 1:
        return 0.0
    dmax = 0.0
    for i in range(pts.shape[0]):
        for j in range(i + 1, pts.shape[0]):
            d = float(np.linalg.norm(pts[i] - pts[j]))
            dmax = max(dmax, d)
    return dmax


def _pairwise_min_distance(pts: np.ndarray) -> float:
    if pts.shape[0] <= 1:
        return float("inf")
    dmin = float("inf")
    for i in range(pts.shape[0]):
        for j in range(i + 1, pts.shape[0]):
            d = float(np.linalg.norm(pts[i] - pts[j]))
            dmin = min(dmin, d)
    return dmin


def _paper_rcparams():
    plt.rcParams.update(
        {
            "figure.dpi": 120,
            "savefig.dpi": 300,
            "font.size": 12,
            "axes.titlesize": 14,
            "axes.labelsize": 12,
            "legend.fontsize": 10,
            "xtick.labelsize": 11,
            "ytick.labelsize": 11,
            "lines.linewidth": 2.0,
            "grid.alpha": 0.3,
            "axes.grid": True,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def _savefig(fig, outdir: Path, name: str):
    outpath = outdir / f"{name}.png"
    fig.savefig(outpath, bbox_inches="tight")
    print(f"[plot_consensus] wrote {outpath}")


def _time_axis_from_zero(ax, x):
    if len(x) > 0:
        ax.set_xlim(left=0, right=int(np.max(x)))


def _formation_metrics_for_positions(pos_pts: np.ndarray, offsets: np.ndarray):
    bary = np.mean(pos_pts, axis=0)
    y = pos_pts - offsets
    Vc = _pairwise_diameter(y)
    desired = bary + offsets
    e = np.linalg.norm(pos_pts - desired, axis=1)
    emax = float(np.max(e)) if e.size else 0.0
    return bary, Vc, e, emax


def _parse_outer(outer_rows):
    per_agent = {}
    fallback_per_outer = {}
    for row in outer_rows:
        try:
            j = int(row[0])
            agent = int(row[1])
            fallback = int(float(row[20]))
            obj = float(row[4]) if row[4] != "" else np.nan
            phi = float(row[6]) if row[6] != "" else np.nan
            diamC = float(row[10]) if row[10] != "" else np.nan
        except Exception:
            continue

        per_agent.setdefault(agent, {"j": [], "obj": [], "phi": [], "diamC": []})
        per_agent[agent]["j"].append(j)
        per_agent[agent]["obj"].append(obj)
        per_agent[agent]["phi"].append(phi)
        per_agent[agent]["diamC"].append(diamC)
        fallback_per_outer[j] = fallback_per_outer.get(j, 0) + fallback
    return per_agent, fallback_per_outer


def _parse_hybrid_modes(rows):
    counts = {}
    for row in rows:
        try:
            j = int(row[1])
            mode = str(row[4]).strip()
        except Exception:
            continue
        if j not in counts:
            counts[j] = {"F": 0, "C": 0, "O": 0, "CO": 0, "T": 0}
        if mode in counts[j]:
            counts[j][mode] += 1
    return counts


def main(show: bool = True) -> None:
    _paper_rcparams()

    traj_path = Path("consensus_traj.csv")
    metrics_path = Path("consensus_metrics.csv")
    outer_path = Path("consensus_outer.csv")
    hybrid_modes_path = Path("hybrid_modes.csv")

    if not traj_path.exists() or not metrics_path.exists() or not outer_path.exists() or not hybrid_modes_path.exists():
        raise FileNotFoundError("Missing one of consensus_traj.csv, consensus_metrics.csv, consensus_outer.csv, hybrid_modes.csv")

    traj = _load_csv_float(traj_path)
    metrics = _load_csv_float(metrics_path)
    _, outer_rows = _load_csv_rows(outer_path)
    _, hybrid_rows = _load_csv_rows(hybrid_modes_path)

    cfg = NetConfig()
    offsets = cfg.formation_offsets()

    outdir = Path("figures_png")
    outdir.mkdir(parents=True, exist_ok=True)

    k = traj[:, 0].astype(int)
    outer_j = traj[:, 1].astype(int)
    inner_t = traj[:, 2].astype(int)
    agent = traj[:, 3].astype(int)

    r0, r1 = traj[:, 4], traj[:, 5]
    v0, v1 = traj[:, 6], traj[:, 7]
    u0, u1 = traj[:, 8], traj[:, 9]

    agents = np.unique(agent).astype(int)
    n_agents = len(agents)
    is_double = np.nanmax(np.abs(v0) + np.abs(v1)) > 1e-12

    M = int(np.nanmax(inner_t)) + 1 if traj.size else 1
    outer_mask = inner_t == (M - 1)

    cmap = plt.get_cmap("tab10")
    colors = {a: cmap((i % 10) / 10) for i, a in enumerate(agents)}

    def _series(a: int, col: np.ndarray) -> np.ndarray:
        return col[agent == a]

    def _k(a: int) -> np.ndarray:
        return k[agent == a]

    def _outer_points(a: int, col: np.ndarray):
        kk = k[(agent == a) & outer_mask]
        yy = col[(agent == a) & outer_mask]
        return kk, yy

    # 1) Positions
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    fig.suptitle(f"Positions over inner steps (Solution 2, M={M})")
    for idx, (col, lab) in enumerate([(r0, "r[0]"), (r1, "r[1]")]):
        ax = axs[idx]
        for a in agents:
            ax.plot(_k(a), _series(a, col), label=f"agent {a}", color=colors[a])
            kk, yy = _outer_points(a, col)
            ax.plot(kk, yy, linestyle="None", marker="o", markersize=4, color=colors[a])
        ax.set_ylabel(lab)
        _time_axis_from_zero(ax, k)
    axs[-1].set_xlabel("Inner step k")
    axs[0].legend(ncol=2, frameon=False)
    fig.tight_layout()
    _savefig(fig, outdir, f"01_positions_solution2_M{M}")

    # 2) Velocities
    if is_double:
        fig, axs = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        fig.suptitle(f"Velocities over inner steps (Solution 2, M={M})")
        for idx, (col, lab) in enumerate([(v0, "v[0]"), (v1, "v[1]")]):
            ax = axs[idx]
            for a in agents:
                ax.plot(_k(a), _series(a, col), label=f"agent {a}", color=colors[a])
                kk, yy = _outer_points(a, col)
                ax.plot(kk, yy, linestyle="None", marker="o", markersize=4, color=colors[a])
            ax.set_ylabel(lab)
            _time_axis_from_zero(ax, k)
        axs[-1].set_xlabel("Inner step k")
        axs[0].legend(ncol=2, frameon=False)
        fig.tight_layout()
        _savefig(fig, outdir, f"02_velocities_solution2_M{M}")

    # 3) Controls
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    fig.suptitle(f"Controls over inner steps (Solution 2, M={M})")
    for idx, (col, lab) in enumerate([(u0, "u[0]"), (u1, "u[1]")]):
        ax = axs[idx]
        for a in agents:
            ax.plot(_k(a), _series(a, col), label=f"agent {a}", color=colors[a])
            kk, yy = _outer_points(a, col)
            ax.plot(kk, yy, linestyle="None", marker="o", markersize=4, color=colors[a])
        ax.set_ylabel(lab)
        _time_axis_from_zero(ax, k)
    axs[-1].set_xlabel("Inner step k")
    axs[0].legend(ncol=2, frameon=False)
    fig.tight_layout()
    _savefig(fig, outdir, f"03_controls_solution2_M{M}")

    # outer-step metrics from trajectories
    outer_steps = np.unique(outer_j[outer_mask])
    outer_steps.sort()

    Vr = []
    Vc = []
    dmin_agents = []
    emax = []
    e_by_agent = {a: [] for a in agents}

    for j in outer_steps:
        pts = []
        for a in agents:
            mask = (agent == a) & (outer_j == j) & outer_mask
            if np.any(mask):
                pts.append([r0[mask][0], r1[mask][0]])
        pts = np.array(pts, dtype=float)
        if pts.shape[0] != n_agents:
            continue
        _, Vc_j, e_j, emax_j = _formation_metrics_for_positions(pts, offsets)
        Vr.append(_pairwise_diameter(pts))
        Vc.append(Vc_j)
        dmin_agents.append(_pairwise_min_distance(pts))
        emax.append(emax_j)
        for idx_a, a in enumerate(agents):
            e_by_agent[a].append(float(e_j[idx_a]))

    Vr = np.array(Vr, dtype=float)
    Vc = np.array(Vc, dtype=float)
    dmin_agents_arr = np.array(dmin_agents, dtype=float)
    emax = np.array(emax, dtype=float)

    # 4) Vr
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(outer_steps[:len(Vr)], Vr, marker="o")
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"Physical diameter $V_r(j)$")
    ax.set_title("Physical diameter over outer steps")
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"04_Vr_solution2_M{M}")

    # 5) Vc
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(outer_steps[:len(Vc)], Vc, marker="o")
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"Transformed diameter $V_c(j)$")
    ax.set_title("Transformed diameter over outer steps")
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"05_Vc_solution2_M{M}")

    # 6) dmin agents
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(outer_steps[:len(dmin_agents_arr)], dmin_agents_arr, marker="o", label=r"$d_{\min}^{agents}(j)$")
    ax.axhline(cfg.d_safe, linestyle="--", color="k", alpha=0.8, label=r"$d_{\mathrm{safe}}$")
    ax.set_xlabel("Outer step j")
    ax.set_ylabel("Distance")
    ax.set_title("Minimum inter-agent distance over outer steps")
    ax.legend(frameon=False)
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"06_dmin_agents_solution2_M{M}")

    # 7) slot error max
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(outer_steps[:len(emax)], emax, marker="o")
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"$\max_i \|r_i-(\bar r + c_i)\|$")
    ax.set_title("Maximum slot-tracking error over outer steps")
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"07_slot_error_max_solution2_M{M}")

    # 8) per-agent slot error
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    for a in agents:
        y = np.array(e_by_agent[a], dtype=float)
        ax.plot(outer_steps[:len(y)], y, marker="o", label=f"agent {a}", color=colors[a])
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"$\|r_i-(\bar r + c_i)\|$")
    ax.set_title("Per-agent slot-tracking error over outer steps")
    ax.legend(frameon=False, ncol=2)
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"08_slot_error_agents_solution2_M{M}")

    # metrics-based
    k_met = metrics[:, 0].astype(int)
    dmin_obs_met = metrics[:, 8]
    nF = metrics[:, 9]
    nC = metrics[:, 10]
    nO = metrics[:, 11]
    nCO = metrics[:, 12]
    nT = metrics[:, 13]

    # 9) obstacle distance
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(k_met, dmin_obs_met, label=r"$d_{\min}^{obs}(k)$")
    ax.axhline(0.0, linestyle="--", color="r", alpha=0.8, label="inflated obstacle boundary")
    ax.set_xlabel("Inner step k")
    ax.set_ylabel("Distance")
    ax.set_title("Minimum obstacle distance over inner steps")
    ax.legend(frameon=False)
    _time_axis_from_zero(ax, k_met)
    fig.tight_layout()
    _savefig(fig, outdir, f"09_dmin_obstacles_solution2_M{M}")

    # 10) mode counts over inner steps
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(k_met, nF, label="F")
    ax.plot(k_met, nC, label="C")
    ax.plot(k_met, nO, label="O")
    ax.plot(k_met, nCO, label="CO")
    ax.plot(k_met, nT, label="T")
    ax.set_xlabel("Inner step k")
    ax.set_ylabel("Number of agents")
    ax.set_title("Hybrid mode counts over inner steps")
    ax.legend(frameon=False, ncol=3)
    _time_axis_from_zero(ax, k_met)
    fig.tight_layout()
    _savefig(fig, outdir, f"10_mode_counts_inner_solution2_M{M}")

    per_agent_outer, fallback_per_outer = _parse_outer(outer_rows)
    mode_counts_outer = _parse_hybrid_modes(hybrid_rows)

    # 11) nominal phi
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    for a in agents:
        if a not in per_agent_outer:
            continue
        ax.plot(per_agent_outer[a]["j"], per_agent_outer[a]["phi"], marker="o", label=f"agent {a}", color=colors[a])
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"$\phi_i^c(j)$")
    ax.set_title("Nominal interiority measure over outer steps")
    ax.legend(frameon=False, ncol=2)
    ax.set_ylim(bottom=0.0)
    fig.tight_layout()
    _savefig(fig, outdir, f"11_phi_solution2_M{M}")

    # 12) nominal fallback count
    if len(fallback_per_outer) > 0:
        xs = np.array(sorted(fallback_per_outer.keys()), dtype=int)
        ys = np.array([fallback_per_outer[x] for x in xs], dtype=float)
        fig = plt.figure(figsize=(8, 4.8))
        ax = plt.gca()
        ax.step(xs, ys, where="post")
        ax.set_xlabel("Outer step j")
        ax.set_ylabel("Number of agents using nominal fallback")
        ax.set_title("Nominal fallback usage over outer steps")
        _time_axis_from_zero(ax, xs)
        fig.tight_layout()
        _savefig(fig, outdir, f"12_nominal_fallback_solution2_M{M}")

    # 13) mode counts over outer steps
    if len(mode_counts_outer) > 0:
        xs = np.array(sorted(mode_counts_outer.keys()), dtype=int)
        yF = np.array([mode_counts_outer[j]["F"] for j in xs], dtype=float)
        yC = np.array([mode_counts_outer[j]["C"] for j in xs], dtype=float)
        yO = np.array([mode_counts_outer[j]["O"] for j in xs], dtype=float)
        yCO = np.array([mode_counts_outer[j]["CO"] for j in xs], dtype=float)
        yT = np.array([mode_counts_outer[j]["T"] for j in xs], dtype=float)

        fig = plt.figure(figsize=(8, 4.8))
        ax = plt.gca()
        ax.step(xs, yF, where="post", label="F")
        ax.step(xs, yC, where="post", label="C")
        ax.step(xs, yO, where="post", label="O")
        ax.step(xs, yCO, where="post", label="CO")
        ax.step(xs, yT, where="post", label="T")
        ax.set_xlabel("Outer step j")
        ax.set_ylabel("Number of agents")
        ax.set_title("Hybrid mode counts over outer steps")
        ax.legend(frameon=False, ncol=3)
        _time_axis_from_zero(ax, xs)
        fig.tight_layout()
        _savefig(fig, outdir, f"13_mode_counts_outer_solution2_M{M}")

    # 14) trajectories with obstacle circles
    fig = plt.figure(figsize=(7.5, 7.5))
    ax = plt.gca()

    for a in agents:
        mask = agent == a
        ax.plot(r0[mask], r1[mask], label=f"agent {a}", color=colors[a])
        ax.plot(r0[mask][0], r1[mask][0], marker="s", color=colors[a])
        ax.plot(r0[mask][-1], r1[mask][-1], marker="o", color=colors[a])

    if cfg.obstacles_enabled:
        for idx, (cx, cy, rad) in enumerate(cfg.obstacles_circles):
            circ = plt.Circle((cx, cy), rad, fill=True, alpha=0.18, color="tab:red")
            ax.add_patch(circ)
            circ_inf = plt.Circle((cx, cy), rad + cfg.obstacle_margin, fill=False, linestyle="--", color="tab:red", alpha=0.9)
            ax.add_patch(circ_inf)
            ax.text(cx, cy, f"O{idx}", ha="center", va="center")

    last_k = int(np.max(k))
    pts_final = []
    for a in agents:
        mask = (agent == a) & (k == last_k)
        if np.any(mask):
            pts_final.append([r0[mask][0], r1[mask][0]])
    pts_final = np.array(pts_final, dtype=float)

    if pts_final.shape[0] == n_agents:
        bary = np.mean(pts_final, axis=0)
        desired = bary + offsets
        ax.plot(bary[0], bary[1], marker="x", markersize=10, color="k", label="final barycenter")
        for i in range(desired.shape[0]):
            ax.plot(desired[i, 0], desired[i, 1], marker="+", markersize=10, color="k")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("r[0]")
    ax.set_ylabel("r[1]")
    ax.set_title("Robot trajectories with obstacle (Solution 2)")
    ax.legend(frameon=False, ncol=2)
    fig.tight_layout()
    _savefig(fig, outdir, f"14_trajectories_solution2_M{M}")

    # 15) speed
    if is_double:
        fig = plt.figure(figsize=(8, 4.8))
        ax = plt.gca()
        ax.plot(k_met, metrics[:, 5], label=r"$\max_i \|v_i\|$")
        ax.plot(k_met, metrics[:, 6], label=r"mean $\|v_i\|$")
        ax.set_xlabel("Inner step k")
        ax.set_ylabel("Speed")
        ax.set_title("Velocity diagnostics over inner steps")
        ax.legend(frameon=False)
        _time_axis_from_zero(ax, k_met)
        fig.tight_layout()
        _savefig(fig, outdir, f"15_speed_solution2_M{M}")

    if show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main(show=True)