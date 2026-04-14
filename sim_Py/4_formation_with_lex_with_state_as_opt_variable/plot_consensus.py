from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from consensus_config import NetConfig


def _load_csv_float(path: Path) -> np.ndarray:
    return np.genfromtxt(path, delimiter=",", skip_header=1, dtype=float)


def _pairwise_diameter(pts: np.ndarray) -> float:
    if pts.shape[0] <= 1:
        return 0.0
    dmax = 0.0
    for i in range(pts.shape[0]):
        for k in range(i + 1, pts.shape[0]):
            d = float(np.linalg.norm(pts[i] - pts[k]))
            if d > dmax:
                dmax = d
    return dmax


def _pairwise_min_distance(pts: np.ndarray) -> float:
    if pts.shape[0] <= 1:
        return float("inf")
    dmin = float("inf")
    for i in range(pts.shape[0]):
        for k in range(i + 1, pts.shape[0]):
            d = float(np.linalg.norm(pts[i] - pts[k]))
            if d < dmin:
                dmin = d
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


def _time_axis_from_zero(ax, data_x=None):
    if data_x is not None and len(data_x) > 0:
        ax.set_xlim(left=0, right=int(np.max(data_x)))


def _formation_metrics_for_positions(pos_pts: np.ndarray, offsets: np.ndarray):
    """
    pos_pts: shape (n_agents, 2)
    offsets: shape (n_agents, 2)
    Returns:
        barycenter,
        transformed diameter Vc,
        per-agent formation tracking errors,
        max formation tracking error
    """
    bary = np.mean(pos_pts, axis=0)
    y = pos_pts - offsets
    Vc = _pairwise_diameter(y)

    desired = bary + offsets
    e = np.linalg.norm(pos_pts - desired, axis=1)
    emax = float(np.max(e)) if e.size else 0.0
    return bary, Vc, e, emax


def main(show: bool = True) -> None:
    _paper_rcparams()

    traj_path = Path("consensus_traj.csv")
    outer_path = Path("consensus_outer.csv")
    metrics_path = Path("consensus_metrics.csv")

    if not traj_path.exists() or not outer_path.exists():
        raise FileNotFoundError("Missing consensus_traj.csv or consensus_outer.csv. Run consensus_plant.py first.")

    traj = _load_csv_float(traj_path)
    out_raw = _load_csv_float(outer_path)
    metrics = _load_csv_float(metrics_path) if metrics_path.exists() else None

    outdir = Path("figures_png")
    outdir.mkdir(parents=True, exist_ok=True)

    cfg = NetConfig()
    offsets = cfg.formation_offsets()

    k = traj[:, 0].astype(int)
    outer_j = traj[:, 1].astype(int)
    inner_t = traj[:, 2].astype(int)
    agent = traj[:, 3].astype(int)

    r0, r1 = traj[:, 4], traj[:, 5]
    v0, v1 = traj[:, 6], traj[:, 7]
    u0, u1 = traj[:, 8], traj[:, 9]

    agents = np.unique(agent).astype(int)
    n_agents = len(agents)

    finite_outer = np.isfinite(out_raw[:, 0]) & np.isfinite(out_raw[:, 1]) & np.isfinite(out_raw[:, 3])
    out = out_raw[finite_outer, :]

    outer_j2 = out[:, 0].astype(int)
    agent2 = out[:, 1].astype(int)
    M = int(np.nanmax(out[:, 3])) if out.shape[0] else 1

    is_double = np.nanmax(np.abs(v0) + np.abs(v1)) > 1e-12

    cmap = plt.get_cmap("tab10")
    colors = {a: cmap((i % 10) / 10) for i, a in enumerate(agents)}

    outer_mask = inner_t == (M - 1)

    def _series(a: int, col: np.ndarray) -> np.ndarray:
        return col[agent == a]

    def _k(a: int) -> np.ndarray:
        return k[agent == a]

    def _outer_points(a: int, col: np.ndarray):
        kk = k[(agent == a) & outer_mask]
        yy = col[(agent == a) & outer_mask]
        return kk, yy

    # ------------------------------------------------------------------
    # 1) Positions over inner steps
    # ------------------------------------------------------------------
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    fig.suptitle(f"Positions over inner steps (M={M})")
    for idx, (col, lab) in enumerate([(r0, "r[0]"), (r1, "r[1]")]):
        ax = axs[idx]
        for a in agents:
            ax.plot(_k(a), _series(a, col), label=f"agent {a}", color=colors[a])
            kk, yy = _outer_points(a, col)
            ax.plot(kk, yy, linestyle="None", marker="o", markersize=4, color=colors[a])
            _time_axis_from_zero(ax, _k(a))
        ax.set_ylabel(lab)
    axs[-1].set_xlabel("Inner step k")
    axs[0].legend(ncol=2, frameon=False)
    fig.tight_layout()
    _savefig(fig, outdir, f"01_positions_M{M}")

    # ------------------------------------------------------------------
    # 2) Velocities (double integrators only)
    # ------------------------------------------------------------------
    if is_double:
        fig, axs = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        fig.suptitle(f"Velocities over inner steps (M={M})")
        for idx, (col, lab) in enumerate([(v0, "v[0]"), (v1, "v[1]")]):
            ax = axs[idx]
            for a in agents:
                ax.plot(_k(a), _series(a, col), label=f"agent {a}", color=colors[a])
                kk, yy = _outer_points(a, col)
                ax.plot(kk, yy, linestyle="None", marker="o", markersize=4, color=colors[a])
                _time_axis_from_zero(ax, _k(a))
            ax.set_ylabel(lab)
        axs[-1].set_xlabel("Inner step k")
        axs[0].legend(ncol=2, frameon=False)
        fig.tight_layout()
        _savefig(fig, outdir, f"02_velocities_M{M}")

    # ------------------------------------------------------------------
    # 3) Controls over inner steps
    # ------------------------------------------------------------------
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    fig.suptitle(f"Controls over inner steps (M={M})")
    for idx, (col, lab) in enumerate([(u0, "u[0]"), (u1, "u[1]")]):
        ax = axs[idx]
        for a in agents:
            ax.plot(_k(a), _series(a, col), label=f"agent {a}", color=colors[a])
            kk, yy = _outer_points(a, col)
            ax.plot(kk, yy, linestyle="None", marker="o", markersize=4, color=colors[a])
            _time_axis_from_zero(ax, _k(a))
        ax.set_ylabel(lab)
    axs[-1].set_xlabel("Inner step k")
    axs[0].legend(ncol=2, frameon=False)
    fig.tight_layout()
    _savefig(fig, outdir, f"03_controls_M{M}")

    # ------------------------------------------------------------------
    # Build outer-step formation metrics from the trajectory log
    # ------------------------------------------------------------------
    outer_steps = np.unique(outer_j[outer_mask])
    outer_steps.sort()

    Vr = []
    Vc = []
    dmin = []
    emax = []
    e_by_agent = {a: [] for a in agents}

    for j in outer_steps:
        pts = []
        for a in agents:
            mask = (agent == a) & outer_mask & (outer_j == j)
            if np.any(mask):
                pts.append([r0[mask][0], r1[mask][0]])
        pts = np.array(pts, dtype=float)
        if pts.shape[0] != n_agents:
            continue

        _, Vc_j, e_j, emax_j = _formation_metrics_for_positions(pts, offsets)
        Vr.append(_pairwise_diameter(pts))
        Vc.append(Vc_j)
        dmin.append(_pairwise_min_distance(pts))
        emax.append(emax_j)
        for idx_a, a in enumerate(agents):
            e_by_agent[a].append(float(e_j[idx_a]))

    Vr = np.array(Vr, dtype=float)
    Vc = np.array(Vc, dtype=float)
    dmin = np.array(dmin, dtype=float)
    emax = np.array(emax, dtype=float)

    # ------------------------------------------------------------------
    # 4) Physical position diameter Vr over outer steps
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(outer_steps[: len(Vr)], Vr, marker="o")
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"Physical diameter $V_r(j)$")
    ax.set_title(f"$V_r(j)$ over outer steps (M={M})")
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"04_Vr_outer_M{M}")

    # ------------------------------------------------------------------
    # 5) Transformed diameter Vc over outer steps
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(outer_steps[: len(Vc)], Vc, marker="o")
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"Transformed diameter $V_c(j)$")
    ax.set_title(f"$V_c(j)$ over outer steps (safe formation, M={M})")
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"05_Vc_outer_M{M}")

    # ------------------------------------------------------------------
    # 6) Minimum pairwise distance over outer steps
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(outer_steps[: len(dmin)], dmin, marker="o", label=r"$d_{\min}(j)$")
    ax.axhline(cfg.d_safe, linestyle="--", color="k", alpha=0.8, label=r"$d_{\mathrm{safe}}$")
    ax.set_xlabel("Outer step j")
    ax.set_ylabel("Distance")
    ax.set_title(f"Minimum pairwise distance over outer steps (M={M})")
    ax.legend(frameon=False)
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"06_dmin_outer_M{M}")

    # ------------------------------------------------------------------
    # 7) Max formation tracking error over outer steps
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(outer_steps[: len(emax)], emax, marker="o")
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"$\max_i \|r_i-(\bar r + c_i)\|$")
    ax.set_title(f"Maximum slot-tracking error over outer steps (M={M})")
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"07_slot_error_max_outer_M{M}")

    # ------------------------------------------------------------------
    # 8) Per-agent formation tracking errors
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    for a in agents:
        y = np.array(e_by_agent[a], dtype=float)
        ax.plot(outer_steps[: len(y)], y, marker="o", label=f"agent {a}", color=colors[a])
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"$\|r_i-(\bar r + c_i)\|$")
    ax.set_title(f"Per-agent slot-tracking error over outer steps (M={M})")
    ax.legend(ncol=2, frameon=False)
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"08_slot_error_agents_outer_M{M}")

    # ------------------------------------------------------------------
    # 9) Interiority phi over outer steps
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    for a in agents:
        m = (agent2 == a) & np.isfinite(out[:, 6]) & np.isfinite(out[:, 0])
        if not np.any(m):
            continue
        x = out[m, 0].astype(int)
        y = out[m, 6]
        ax.plot(x, y, marker="o", label=f"agent {a}", color=colors[a])
        _time_axis_from_zero(ax, x)
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"$\phi^c_{i,j}(x_i^\star(M)-c_i)$")
    ax.set_title(f"Interiority measure over outer steps (safe formation, M={M})")
    ax.legend(ncol=2, frameon=False)
    ax.set_ylim(bottom=0.0)
    fig.tight_layout()
    _savefig(fig, outdir, f"09_phi_outer_M{M}")

    # ------------------------------------------------------------------
    # 10) Lexicographic selection activity
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    for a in agents:
        m = (agent2 == a) & np.isfinite(out[:, 5]) & np.isfinite(out[:, 0])
        if not np.any(m):
            continue
        x = out[m, 0].astype(int)
        y = np.clip(out[m, 5], 0.0, 1.0)
        ax.step(x, y, where="post", label=f"agent {a}", color=colors[a])
        _time_axis_from_zero(ax, x)
    ax.set_xlabel("Outer step j")
    ax.set_ylabel("Lex stage used (0/1)")
    ax.set_title(f"Lexicographic selection activity (M={M})")
    ax.legend(ncol=2, frameon=False)
    ax.set_yticks([0, 1])
    ax.set_ylim(-0.1, 1.1)
    fig.tight_layout()
    _savefig(fig, outdir, f"10_lex_used_outer_M{M}")

    # ------------------------------------------------------------------
    # 11) Primary cost over outer steps
    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    for a in agents:
        m = (agent2 == a) & np.isfinite(out[:, 4]) & np.isfinite(out[:, 0])
        if not np.any(m):
            continue
        x = out[m, 0].astype(int)
        y = out[m, 4]
        ax.plot(x, y, marker="o", label=f"agent {a}", color=colors[a])
        _time_axis_from_zero(ax, x)
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"Primary cost $J_i^{c,\star}(j)$")
    ax.set_title(f"Primary cost over outer steps (safe formation, M={M})")
    ax.legend(ncol=2, frameon=False)
    fig.tight_layout()
    _savefig(fig, outdir, f"11_cost_outer_M{M}")

    # ------------------------------------------------------------------
    # 12) Hull diameter in transformed coordinates
    # ------------------------------------------------------------------
    if out.shape[1] >= 11:
        fig = plt.figure(figsize=(8, 4.8))
        ax = plt.gca()
        diam_col = out[:, 10]
        for a in agents:
            m = (agent2 == a) & np.isfinite(diam_col) & np.isfinite(out[:, 0])
            if not np.any(m):
                continue
            x = out[m, 0].astype(int)
            y = diam_col[m]
            ax.plot(x, y, marker="o", label=f"agent {a}", color=colors[a])
            _time_axis_from_zero(ax, x)
        ax.set_xlabel("Outer step j")
        ax.set_ylabel(r"$\mathrm{diam}(\mathcal{C}_i^c(j))$")
        ax.set_title(f"Transformed hull diameter over outer steps (M={M})")
        ax.legend(ncol=2, frameon=False)
        ax.set_ylim(bottom=0.0)
        fig.tight_layout()
        _savefig(fig, outdir, f"12_diamCc_outer_M{M}")

    # ------------------------------------------------------------------
    # 13) Velocity diagnostics from metrics for double integrators
    # ------------------------------------------------------------------
    if is_double and metrics is not None and metrics.ndim == 2 and metrics.shape[1] >= 7:
        fig = plt.figure(figsize=(8, 4.8))
        ax = plt.gca()
        x = metrics[:, 0]
        ax.plot(x, metrics[:, 5], label=r"$\max_i \|v_i\|$")
        ax.plot(x, metrics[:, 6], label=r"mean $\|v_i\|$")
        ax.set_xlabel("Inner step k")
        ax.set_ylabel("Speed")
        ax.set_title(f"Velocity diagnostics over inner steps (M={M})")
        ax.legend(frameon=False)
        fig.tight_layout()
        _savefig(fig, outdir, f"13_speed_inner_M{M}")

    if show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main(show=True)