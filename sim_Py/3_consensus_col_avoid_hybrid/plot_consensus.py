from __future__ import annotations

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


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


def _paper_rcparams():
    plt.rcParams.update({
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
    })


def _savefig(fig, outdir: Path, name: str):
    outpath = outdir / f"{name}.png"
    fig.savefig(outpath, bbox_inches="tight")
    print(f"[plot_consensus] wrote {outpath}")


def main(show: bool = True) -> None:
    _paper_rcparams()

    traj_path = Path("consensus_traj.csv")
    outer_path = Path("consensus_outer.csv")
    metrics_path = Path("consensus_metrics.csv")
    modes_path = Path("hybrid_modes.csv")
    if not traj_path.exists() or not outer_path.exists():
        raise FileNotFoundError("Missing consensus_traj.csv or consensus_outer.csv. Run consensus_plant.py first.")

    traj = _load_csv_float(traj_path)
    out_raw = _load_csv_float(outer_path)
    metrics = _load_csv_float(metrics_path) if metrics_path.exists() else None
    modes = _load_csv_float(modes_path) if modes_path.exists() else None

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

    finite_outer = np.isfinite(out_raw[:, 0]) & np.isfinite(out_raw[:, 1]) & np.isfinite(out_raw[:, 3])
    out = out_raw[finite_outer, :]

    outer_j2 = out[:, 0].astype(int)
    agent2 = out[:, 1].astype(int)
    M = int(np.nanmax(out[:, 3])) if out.shape[0] else 1

    is_double = (np.nanmax(np.abs(v0) + np.abs(v1)) > 1e-12)

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

    def _time_axis_from_zero(ax, data_x=None):
        ax.set_xlim(left=0, right=len(data_x) - 1 if data_x is not None and len(data_x) > 0 else None)

    # 1) Positions
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

    # 2) Velocities
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

    # 3) Controls
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

    # 4) Vr over outer steps
    outer_steps = np.unique(outer_j[outer_mask])
    outer_steps.sort()
    Vr = []
    for j in outer_steps:
        pts = []
        for a in agents:
            mask = (agent == a) & outer_mask & (outer_j == j)
            if np.any(mask):
                pts.append([r0[mask][0], r1[mask][0]])
        pts = np.array(pts, dtype=float)
        Vr.append(_pairwise_diameter(pts))
    Vr = np.array(Vr, dtype=float)

    fig = plt.figure(figsize=(8, 4.8))
    ax = plt.gca()
    ax.plot(outer_steps, Vr, marker="o")
    ax.set_xlabel("Outer step j")
    ax.set_ylabel(r"Position diameter $V_r(j)$")
    ax.set_title(f"$V_r(j)$ over outer steps (M={M})")
    _time_axis_from_zero(ax, outer_steps)
    fig.tight_layout()
    _savefig(fig, outdir, f"04_Vr_outer_M{M}")

    # 5) Interiority phi
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
    ax.set_ylabel(r"$\phi_{i,j}(x_i^\star(M))$")
    ax.set_title(f"Interiority measure over outer steps (M={M})")
    ax.legend(ncol=2, frameon=False)
    ax.set_ylim(bottom=0.0)
    fig.tight_layout()
    _savefig(fig, outdir, f"05_phi_outer_M{M}")

    # 6) Lex-used
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
    _savefig(fig, outdir, f"06_lex_used_outer_M{M}")

    # 7) Primary cost
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
    ax.set_ylabel(r"Primary cost $J_i^\star(j)$")
    ax.set_title(f"Primary cost over outer steps (M={M})")
    ax.legend(ncol=2, frameon=False)
    fig.tight_layout()
    _savefig(fig, outdir, f"07_cost_outer_M{M}")

    # 8) Velocity decay and mode counts from metrics
    if metrics is not None and metrics.ndim == 2 and metrics.shape[1] >= 12:
        fig, axs = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
        x = metrics[:, 0]
        axs[0].plot(x, metrics[:, 3], label=r"$V_r$")
        axs[0].plot(x, metrics[:, 7], label=r"$d_{\min}$")
        axs[0].set_ylabel("Distance metric")
        axs[0].legend(frameon=False)

        axs[1].step(x, metrics[:, 8], where="post", label="mode 1 count")
        axs[1].step(x, metrics[:, 9], where="post", label="mode 2 count")
        axs[1].step(x, metrics[:, 10], where="post", label="mode 3 count")
        axs[1].step(x, metrics[:, 11], where="post", label="switches")
        axs[1].set_xlabel("Inner step k")
        axs[1].set_ylabel("Count")
        axs[1].legend(ncol=2, frameon=False)

        fig.suptitle(f"Metrics and mode counts over inner steps (M={M})")
        fig.tight_layout()
        _savefig(fig, outdir, f"08_metrics_modes_inner_M{M}")

    # 9) Hull diameter diam_C
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
        ax.set_ylabel(r"$\mathrm{diam}(C_i(j))$")
        ax.set_title(f"Hull diameter over outer steps (M={M})")
        ax.legend(ncol=2, frameon=False)
        ax.set_ylim(bottom=0.0)
        fig.tight_layout()
        _savefig(fig, outdir, f"09_diamC_outer_M{M}")

    # 10) Per-agent mode traces from hybrid_modes.csv
    if modes is not None and modes.ndim == 2 and modes.shape[1] >= 12:
        fig = plt.figure(figsize=(9, 5.2))
        ax = plt.gca()
        for a in agents:
            m = (modes[:, 3].astype(int) == a)
            if not np.any(m):
                continue
            x = modes[m, 0]
            y = modes[m, 4]
            ax.step(x, y + 0.03 * (a - 1), where="post", label=f"agent {a}", color=colors[a])
        ax.set_xlabel("Inner step k")
        ax.set_ylabel("Mode")
        ax.set_title(f"Hybrid mode history over inner steps (M={M})")
        ax.set_yticks([1, 2, 3])
        ax.set_yticklabels(["nominal", "hold", "emergency"])
        ax.legend(ncol=2, frameon=False)
        fig.tight_layout()
        _savefig(fig, outdir, f"10_hybrid_modes_inner_M{M}")

    # 11) Double-integrator speed diagnostics
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
        _savefig(fig, outdir, f"11_speed_inner_M{M}")

    if show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main(show=True)