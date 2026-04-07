from __future__ import annotations

import argparse
import csv
import time
import io
from PIL import Image
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.widgets import Button, Slider
from matplotlib.patches import Circle, FancyArrowPatch, RegularPolygon

from consensus_config import NetConfig 


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Animate agent motion together with the hybrid-mode state machine."
    )
    parser.add_argument("--traj-csv", type=str, default="planning.csv")
    parser.add_argument("--metrics-csv", type=str, default="consensus_metrics.csv")
    parser.add_argument("--hybrid-modes-csv", type=str, default="hybrid_modes.csv")
    parser.add_argument(
        "--save-gif",
        type=str,
        default=False,
        required=True,
        help="Whether to export a GIF of the animation instead of showing an interactive window. If set, the output path can be configured via --gif-path. Note: the GIF export is designed for clean appearance and does not include the interactive UI elements.",
    )
    parser.add_argument(
        "--gif-path", type=str, default="consensus_animation_with_state_machine.gif"
    )
    parser.add_argument("--gif-speed", type=float, default=1.0)
    parser.add_argument("--gif-fps-base", type=float, default=2.0)
    parser.add_argument("--gif-step", type=int, default=1)
    parser.add_argument("--live-fps", type=float, default=0.75)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def _load_traj(path: Path):
    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=float)
    if data.ndim != 2 or data.shape[1] < 10:
        raise ValueError("Unexpected trajectory CSV format.")

    k = data[:, 0].astype(int)
    outer_j = data[:, 1].astype(int)
    inner_t = data[:, 2].astype(int)
    agent = data[:, 3].astype(int)
    r0, r1 = data[:, 4], data[:, 5]
    v0, v1 = data[:, 6], data[:, 7]
    u0, u1 = data[:, 8], data[:, 9]

    agents = np.unique(agent).astype(int)
    n = len(agents)

    k_unique = np.unique(k)
    k_unique.sort()
    T = len(k_unique)
    if T == 0:
        raise ValueError("Trajectory log contains no frames.")

    is_double = np.max(np.abs(v0) + np.abs(v1)) > 1e-12
    k_max = int(k_unique[-1])

    a2i = {a: i for i, a in enumerate(agents)}
    k2t = {kv: ti for ti, kv in enumerate(k_unique)}

    pos = np.zeros((T, n, 2), dtype=float)
    vel = np.zeros((T, n, 2), dtype=float)
    u = np.zeros((T, n, 2), dtype=float)
    outer_for_t = np.zeros((T,), dtype=int)
    inner_for_t = np.zeros((T,), dtype=int)

    for row in range(data.shape[0]):
        ti = k2t[int(k[row])]
        ai = a2i[int(agent[row])]
        pos[ti, ai, :] = [r0[row], r1[row]]
        vel[ti, ai, :] = [v0[row], v1[row]]
        u[ti, ai, :] = [u0[row], u1[row]]
        outer_for_t[ti] = int(outer_j[row])
        inner_for_t[ti] = int(inner_t[row])

    return {
        "pos": pos,
        "vel": vel,
        "u": u,
        "agents": agents,
        "k_unique": k_unique,
        "k_max": k_max,
        "outer_for_t": outer_for_t,
        "inner_for_t": inner_for_t,
        "is_double": is_double,
    }


def _load_metrics(path: Path):
    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=float)
    
    out = {}
    cols = data.shape[1] if data.ndim == 2 else 0
    
    if cols >= 14:
        # Legacy Pure-Python format (14-16 columns)
        has_pair_num = cols >= 15
        has_circle_num = cols >= 16
        for row in data:
            k = int(row[0])
            out[k] = {
                "Vr": float(row[3]),
                "Vv": float(row[4]),
                "max_speed": float(row[5]),
                "mean_speed": float(row[6]),
                "dmin_agents": float(row[7]),
                "dmin_obstacles": float(row[8]),
                "nF": int(row[9]),
                "nC": int(row[10]),
                "nO": int(row[11]),
                "nCO": int(row[12]),
                "nT": int(row[13]),
                "_h_pair_num": int(row[14]) if has_pair_num else 0,
                "_circle_barrier_num": int(row[15]) if has_circle_num else 0,
            }
    elif cols >= 10:
        # New Co-Simulation format (10 columns)
        for row in data:
            k = int(row[0])
            out[k] = {
                "Vr": float("nan"),
                "Vv": float("nan"),
                "max_speed": float("nan"),
                "mean_speed": float("nan"),
                "dmin_agents": float("nan"),  # Logged in health.csv instead
                "dmin_obstacles": float("nan"),
                "nF": int(row[3]),
                "nC": int(row[4]),
                "nO": int(row[5]),
                "nCO": int(row[6]),
                "nT": int(row[7]),
                "_h_pair_num": int(row[8]),
                "_circle_barrier_num": int(row[9]),
            }
    else:
        raise ValueError(f"Unexpected metrics CSV format. Expected >= 10 columns, got {cols}.")
        
    return out


def _load_modes(path: Path):
    per_k: Dict[int, Dict[str, object]] = {}
    with path.open("r", newline="", encoding="utf-8") as f:
        rd = csv.DictReader(f)
        for row in rd:
            try:
                k = int(row["k_global"])
                agent = int(row["agent"])
            except Exception:
                continue
            mode = str(row.get("mode", "F")).strip() or "F"
            desired = str(row.get("desired_mode", mode)).strip() or mode
            effective = str(row.get("effective_mode", mode)).strip() or mode
            rec = per_k.setdefault(
                k,
                {
                    "counts": {"F": 0, "C": 0, "O": 0, "CO": 0, "T": 0},
                    "agent_modes": {},
                    "desired_modes": {},
                    "effective_modes": {},
                },
            )
            if mode in rec["counts"]:
                rec["counts"][mode] += 1
            rec["agent_modes"][agent] = mode
            rec["desired_modes"][agent] = desired
            rec["effective_modes"][agent] = effective

    prev_effective_by_agent: Dict[int, str] = {}
    for k in sorted(per_k.keys()):
        rec = per_k[k]
        counts = rec["counts"]
        dom = max(counts.keys(), key=lambda m: (counts[m], m))
        rec["dominant_mode"] = dom

        agent_transitions: Dict[int, Tuple[str, str]] = {}
        effective_modes = rec["effective_modes"]
        for agent, eff in effective_modes.items():
            prev = prev_effective_by_agent.get(agent)
            if prev is not None and prev != eff:
                agent_transitions[agent] = (prev, eff)
            prev_effective_by_agent[agent] = eff
        rec["agent_transitions"] = agent_transitions
    return per_k


def _pairwise_diameter(pts: np.ndarray) -> float:
    if pts.shape[0] <= 1:
        return 0.0
    dmax = 0.0
    for i in range(pts.shape[0]):
        for j in range(i + 1, pts.shape[0]):
            d = float(np.linalg.norm(pts[i] - pts[j]))
            dmax = max(dmax, d)
    return dmax


def _formation_metrics(pts: np.ndarray, offsets: np.ndarray):
    bary = np.mean(pts, axis=0)
    y = pts - offsets
    Vc = _pairwise_diameter(y)
    desired = bary + offsets
    e = np.linalg.norm(pts - desired, axis=1)
    emax = float(np.max(e)) if e.size else 0.0
    return bary, Vc, emax


def _base_node_color(mode: str):
    return {
        "F": "#d9edf7",
        "C": "#f2dede",
        "O": "#fcf8e3",
        "CO": "#eadcf8",
        "T": "#e2f0d9",
    }[mode]


def _token_offsets(count: int, radius: float = 0.17) -> List[np.ndarray]:
    if count <= 0:
        return []
    if count == 1:
        return [np.array([0.0, 0.0])]
    angles = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
    return [radius * np.array([np.cos(a), np.sin(a)]) for a in angles]


def main() -> None:
    args = parse_args()

    traj = _load_traj(Path(args.traj_csv))
    metrics = _load_metrics(Path(args.metrics_csv))
    modes = _load_modes(Path(args.hybrid_modes_csv))

    pos = traj["pos"]
    vel = traj["vel"]
    u = traj["u"]
    agents = traj["agents"]
    k_unique = traj["k_unique"]
    k_max = traj["k_max"]
    outer_for_t = traj["outer_for_t"]
    inner_for_t = traj["inner_for_t"]
    is_double = traj["is_double"]
    T, n, _ = pos.shape

    cfg = NetConfig()
    d_safe = float(cfg.d_safe)
    offsets = cfg.formation_offsets()

    xmin = np.min(pos[:, :, 0]) - (1.5 + d_safe)
    xmax = np.max(pos[:, :, 0]) + (1.5 + d_safe)
    ymin = np.min(pos[:, :, 1]) - (1.5 + d_safe)
    ymax = np.max(pos[:, :, 1]) + (1.5 + d_safe)

    fig = plt.figure(figsize=(14, 7))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.65, 1.1])
    ax = fig.add_subplot(gs[0, 0])
    ax_sm = fig.add_subplot(gs[0, 1])
    plt.subplots_adjust(bottom=(0.08 if args.save_gif else 0.23), wspace=0.18)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel(r"$x$")
    ax.set_ylabel(r"$y$")
    ax.set_aspect("equal", adjustable="box")
    ax.set_axisbelow(True)
    ax.grid(True)

    if cfg.obstacles_enabled:
        for idx, (cx, cy, rad) in enumerate(cfg.obstacles_circles):
            occ = Circle((cx, cy), radius=rad, fill=True, alpha=0.18, color="tab:red")
            occ_inf = Circle(
                (cx, cy),
                radius=rad + cfg.obstacle_margin,
                fill=False,
                linestyle="--",
                linewidth=1.2,
                color="tab:red",
            )
            ax.add_patch(occ)
            ax.add_patch(occ_inf)
            ax.text(cx, cy, f"O{idx}", ha="center", va="center", fontsize=10, color="tab:red")

    scat = ax.scatter(pos[0, :, 0], pos[0, :, 1])

    trail_lines = []
    for i in range(n):
        (ln,) = ax.plot([pos[0, i, 0]], [pos[0, i, 1]])
        trail_lines.append(ln)

    triangles = []
    for i in range(n):
        tri = RegularPolygon(
            (pos[0, i, 0], pos[0, i, 1]),
            numVertices=3,
            radius=0.20,
            orientation=0.0,
        )
        ax.add_patch(tri)
        triangles.append(tri)

    safety_circles = []
    for i in range(n):
        c = Circle(
            (pos[0, i, 0], pos[0, i, 1]),
            radius=d_safe,
            fill=False,
            linestyle="--",
            linewidth=1.0,
            alpha=0.55,
        )
        ax.add_patch(c)
        safety_circles.append(c)

    overlay_text = ax.text(
        0.98,
        0.98,
        "",
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=11,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.7"),
    )

    # --- State machine axis ---
    ax_sm.set_xlim(-1.9, 1.9)
    ax_sm.set_ylim(-1.75, 1.75)
    ax_sm.set_aspect("equal", adjustable="box")
    ax_sm.axis("off")
    ax_sm.set_title("Hybrid controller state machine (all agents)")

    node_pos = {
        "F": np.array([0.0, 1.12]),
        "C": np.array([-1.10, 0.22]),
        "O": np.array([1.10, 0.22]),
        "CO": np.array([0.0, -1.02]),
        "T": np.array([0.0, 0.05]),
    }
    node_radius = 0.31
    node_patches: Dict[str, Circle] = {}

    edges = [("F", "T"), ("C", "T"), ("O", "T"), ("CO", "T")]
    edge_patches: List[Tuple[Tuple[str, str], FancyArrowPatch]] = []
    for a_name, b_name in edges:
        pa = node_pos[a_name]
        pb = node_pos[b_name]
        arrow1 = FancyArrowPatch(
            pa,
            pb,
            arrowstyle="-|>",
            mutation_scale=12,
            linewidth=1.2,
            color="0.7",
            connectionstyle="arc3,rad=0.05",
        )
        arrow2 = FancyArrowPatch(
            pb,
            pa,
            arrowstyle="-|>",
            mutation_scale=12,
            linewidth=1.2,
            color="0.7",
            connectionstyle="arc3,rad=0.05",
        )
        ax_sm.add_patch(arrow1)
        ax_sm.add_patch(arrow2)
        edge_patches.append(((a_name, b_name), arrow1))
        edge_patches.append(((b_name, a_name), arrow2))

    for mode in ["F", "C", "O", "CO", "T"]:
        circ = Circle(
            tuple(node_pos[mode]),
            radius=node_radius,
            facecolor=_base_node_color(mode),
            edgecolor="0.4",
            linewidth=1.5,
        )
        ax_sm.add_patch(circ)
        node_patches[mode] = circ
        ax_sm.text(
            node_pos[mode][0],
            node_pos[mode][1] + 0.05,
            mode,
            ha="center",
            va="center",
            fontsize=14,
            fontweight="bold",
        )

    # Per-agent colored tokens on the state machine
    cmap = plt.get_cmap("tab10")
    agent_colors = [cmap(i % 10) for i in range(n)]
    token_patches = []
    token_labels = []
    for i in range(n):
        tok = Circle((0.0, 0.0), radius=0.07, facecolor=agent_colors[i], edgecolor="black", linewidth=0.8, zorder=5)
        ax_sm.add_patch(tok)
        token_patches.append(tok)
        lbl = ax_sm.text(0.0, 0.0, f"{agents[i]}", ha="center", va="center", fontsize=8, color="white", zorder=6)
        token_labels.append(lbl)

    transition_patches = []
    for i in range(n):
        arr = FancyArrowPatch(
            (0.0, 0.0),
            (0.0, 0.0),
            arrowstyle="simple",
            mutation_scale=16,
            linewidth=0.0,
            color=agent_colors[i],
            alpha=0.0,
            zorder=4,
        )
        ax_sm.add_patch(arr)
        transition_patches.append(arr)

    legend_lines = []
    legend_y = 1.54
    for i in range(n):
        txt = ax_sm.text(
            -1.72 + 0.84 * (i % 2),
            legend_y - 0.16 * (i // 2),
            f"A{agents[i]}",
            color=agent_colors[i],
            fontsize=10,
            ha="left",
            va="center",
            fontweight="bold",
        )
        legend_lines.append(txt)

    sm_info = ax_sm.text(
        0.0,
        -1.55,
        "",
        ha="center",
        va="center",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.9, edgecolor="0.75"),
    )

    playing = {"on": True}
    frame = {"t": 0}
    fps = {"val": float(np.clip(args.live_fps, 0.1, 10.0))}
    ani_holder = {"ani": None}

    play_clock = {
        "last_wall": time.perf_counter(),
        "accum": 0.0,
    }

    def _set_fps(new_fps: float) -> None:
        fps["val"] = float(np.clip(new_fps, 0.1, 10.0))

    def _heading_vec(ti: int, i: int) -> np.ndarray:
        h = vel[ti, i, :] if is_double else u[ti, i, :]
        nrm = float(np.linalg.norm(h))
        if nrm < 1e-9:
            return np.array([1.0, 0.0])
        return h / nrm

    def _overlay_string(ti: int) -> str:
        pts = pos[ti, :, :]
        _, Vc, emax = _formation_metrics(pts, offsets)
        kk = int(k_unique[ti])
        j = int(outer_for_t[ti])
        met = metrics.get(
            kk,
            {
                "dmin_agents": np.nan,
                "dmin_obstacles": np.nan,
                "nF": 0,
                "nC": 0,
                "nO": 0,
                "nCO": 0,
                "nT": 0,
                "_h_pair_num": 0,
                "_circle_barrier_num": 0,
            },
        )
        return (
            f"outer j={j}, inner={int(inner_for_t[ti])}\n"
            f"$V_c$: {Vc:.3f}\n"
            f"$d_{{min}}^{{agents}}$: {met['dmin_agents']:.3f}\n"
            f"$d_{{min}}^{{obs}}$: {met['dmin_obstacles']:.3f}\n"
            f"$e_{{max}}$: {emax:.3f}"
        )

    def _agent_mode_layout(agent_ids_here: List[int], mode: str) -> Dict[int, np.ndarray]:
        offsets_local = _token_offsets(len(agent_ids_here), radius=0.17)
        base = node_pos[mode]
        out = {}
        for aid, off in zip(agent_ids_here, offsets_local):
            out[aid] = base + off
        return out

    def _apply_frame(ti: int) -> None:
        scat.set_offsets(pos[ti, :, :])
        for i in range(n):
            trail_lines[i].set_data(pos[: ti + 1, i, 0], pos[: ti + 1, i, 1])
            hv = _heading_vec(ti, i)
            ang = float(np.arctan2(hv[1], hv[0]))
            triangles[i].xy = (pos[ti, i, 0], pos[ti, i, 1])
            triangles[i].orientation = ang
            safety_circles[i].center = (pos[ti, i, 0], pos[ti, i, 1])

        kk = int(k_unique[ti])
        ax.set_title(
            f"Inner step k={kk}/{k_max} | Unicycle Vehicle Model",
        )
        overlay_text.set_text(_overlay_string(ti))

        mode_rec = modes.get(
            kk,
            {
                "counts": {"F": 0, "C": 0, "O": 0, "CO": 0, "T": 0},
                "dominant_mode": "F",
                "agent_modes": {},
                "effective_modes": {},
                "agent_transitions": {},
            },
        )
        counts = mode_rec["counts"]
        dominant_mode = mode_rec["dominant_mode"]
        effective_modes = dict(mode_rec.get("effective_modes", {}))
        agent_transitions = dict(mode_rec.get("agent_transitions", {}))

        # Use actual token occupancy, not aggregated CSV counts, to decide which states are active.
        actual_counts = {m: 0 for m in ["F", "C", "O", "CO", "T"]}
        for aid in agents.tolist():
            eff_mode = effective_modes.get(int(aid), "F")
            if eff_mode not in actual_counts:
                eff_mode = "F"
            actual_counts[eff_mode] += 1

        for mode, patch in node_patches.items():
            patch.set_facecolor(_base_node_color(mode))
            patch.set_linewidth(1.5)
            patch.set_edgecolor("0.4")
            patch.set_alpha(0.32 if actual_counts.get(mode, 0) == 0 else 0.98)
            # A state is active iff at least one agent token is currently inside it.
            if actual_counts.get(mode, 0) > 0:
                patch.set_linewidth(2.4)
                patch.set_edgecolor("tab:red")

        edge_map = {}
        for (src_mode, dst_mode), edge in edge_patches:
            edge.set_color("0.75")
            edge.set_linewidth(1.2)
            edge.set_alpha(0.35)
            edge_map[(src_mode, dst_mode)] = edge

        # Layout tokens around each active mode node
        per_mode_agents: Dict[str, List[int]] = {m: [] for m in ["F", "C", "O", "CO", "T"]}
        for aid in agents.tolist():
            eff_mode = effective_modes.get(int(aid), "F")
            if eff_mode not in per_mode_agents:
                eff_mode = "F"
            per_mode_agents[eff_mode].append(int(aid))

        token_targets: Dict[int, np.ndarray] = {}
        for mode, aid_list in per_mode_agents.items():
            layout = _agent_mode_layout(aid_list, mode)
            token_targets.update(layout)

        agent_to_index = {int(a): idx for idx, a in enumerate(agents.tolist())}
        for aid, idx in agent_to_index.items():
            target = token_targets.get(aid, node_pos["F"])
            token_patches[idx].center = tuple(target)
            token_labels[idx].set_position(tuple(target))

        # Per-agent transition arrows
        transition_summaries = []
        for aid, idx in agent_to_index.items():
            arr = transition_patches[idx]
            if aid in agent_transitions:
                src_mode, dst_mode = agent_transitions[aid]
                arr.set_positions(tuple(node_pos[src_mode]), tuple(node_pos[dst_mode]))
                arr.set_alpha(0.75)
                transition_summaries.append(f"A{aid}:{src_mode}->{dst_mode}")
                if (src_mode, dst_mode) in edge_map:
                    edge_map[(src_mode, dst_mode)].set_color(agent_colors[idx])
                    edge_map[(src_mode, dst_mode)].set_linewidth(2.8)
                    edge_map[(src_mode, dst_mode)].set_alpha(0.95)
            else:
                arr.set_positions((0.0, 0.0), (0.0, 0.0))
                arr.set_alpha(0.0)

        if transition_summaries:
            tr_text = " | ".join(transition_summaries)
        else:
            tr_text = "no agent transition at this frame"

        sm_info.set_text(
            f"dominant mode: {dominant_mode}\n"
            f"{tr_text}"
        )

    def _advance_frame_by_time() -> None:
        now = time.perf_counter()
        dt = now - play_clock["last_wall"]
        play_clock["last_wall"] = now
        if not playing["on"]:
            return
        play_clock["accum"] += dt
        seconds_per_frame = 1.0 / max(fps["val"], 1e-9)
        steps = int(play_clock["accum"] / seconds_per_frame)
        if steps > 0:
            frame["t"] = (frame["t"] + steps) % T
            play_clock["accum"] -= steps * seconds_per_frame

    def update(_frame_idx):
        _advance_frame_by_time()
        _apply_frame(frame["t"])
        return [
            scat,
            *trail_lines,
            *triangles,
            *safety_circles,
            overlay_text,
            *node_patches.values(),
            *token_patches,
            *token_labels,
            *transition_patches,
            sm_info,
        ]

    # Controls
    ax_play = None
    btn_play = None
    ax_restart = None
    btn_restart = None
    ax_minus = None
    btn_minus = None
    ax_plus = None
    btn_plus = None
    ax_slider = None
    s_fps = None
    if not args.save_gif:
    # --- UI ELEMENTS (interactive mode only) ---
        ax_play = plt.axes([0.06, 0.06, 0.09, 0.06])
        btn_play = Button(ax_play, "Pause")
        ax_restart = plt.axes([0.16, 0.06, 0.09, 0.06])
        btn_restart = Button(ax_restart, "Restart")
        ax_minus = plt.axes([0.27, 0.06, 0.05, 0.06])
        btn_minus = Button(ax_minus, "-")
        ax_plus = plt.axes([0.33, 0.06, 0.05, 0.06])
        btn_plus = Button(ax_plus, "+")
        ax_slider = plt.axes([0.46, 0.07, 0.45, 0.03])
        s_fps = Slider(
            ax_slider,
            "Live FPS",
            valmin=0.10,
            valmax=10.0,
            valinit=fps["val"],
            valstep=0.05,
        )

    def _reset_play_clock() -> None:
        play_clock["last_wall"] = time.perf_counter()
        play_clock["accum"] = 0.0

    def on_play(_evt):
        playing["on"] = not playing["on"]
        btn_play.label.set_text("Pause" if playing["on"] else "Play")
        _reset_play_clock()

    def on_restart(_evt):
        frame["t"] = 0
        _reset_play_clock()
        _apply_frame(0)
        fig.canvas.draw_idle()

    def on_minus(_evt):
        _set_fps(fps["val"] / 1.5)
        s_fps.set_val(fps["val"])

    def on_plus(_evt):
        _set_fps(fps["val"] * 1.5)
        s_fps.set_val(fps["val"])

    def on_slider(val):
        _set_fps(float(val))
        _reset_play_clock()
    
    if not args.save_gif:
        btn_play.on_clicked(on_play)
        btn_restart.on_clicked(on_restart)
        btn_minus.on_clicked(on_minus)
        btn_plus.on_clicked(on_plus)
        s_fps.on_changed(on_slider)

    _apply_frame(0)
    _reset_play_clock()

    # Interactive animation only when not exporting a GIF.
    ani = None
    if not args.save_gif:
        # Use a fast timer and time-based frame stepping so the slider strongly affects the visible pace.
        ani = FuncAnimation(fig, update, interval=30, blit=False)
        ani_holder["ani"] = ani

    if args.save_gif:
        gif_fps = max(0.2, float(args.gif_fps_base) * float(args.gif_speed))
        step = max(1, int(args.gif_step))

        # Clean export-only appearance: no UI widgets, no tick labels, no title clutter.
        # Keep tick locations so the grid has positions to render against in the GIF.
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_xlabel(r"$x$")
        ax.set_ylabel(r"$y$")
        ax.set_aspect("equal", adjustable="box")
        ax.set_axisbelow(True)
        ax.grid(True)
        ax.set_title("")
        ax.tick_params(axis="both", which="both", labelbottom=False, labelleft=False)
        ax_sm.set_title("")

        overlay_text.set_fontsize(10)
        sm_info.set_fontsize(9)

        # Freeze playback state during export.
        playing["on"] = False
        frame["t"] = 0

        # 1. Apply the first frame to force the layout
        _apply_frame(0)
        fig.canvas.draw()
        
        # 2. Calculate a tight bounding box to strictly crop out white margins
        bbox = fig.get_tightbbox(fig.canvas.get_renderer())

        gif_path = args.gif_path or "consensus_animation_with_state_machine.gif"
        total_frames = int(np.ceil(T / step))
        print(f"[plot_consensus_animation] Rendering {total_frames} frames to {gif_path} ...")
        
        images = []
        
        # 3. Lowering the DPI from 100 to 72 scales down dimensions and drastically drops file size
        export_dpi = 72 
        
        for frame_idx in range(total_frames):
            ti = int(np.clip(frame_idx * step, 0, T - 1))
            frame["t"] = ti
            _apply_frame(ti)
            
            # Render the frame to memory using the pre-calculated cropped bounding box
            buf = io.BytesIO()
            fig.savefig(buf, format='png', dpi=export_dpi, bbox_inches=bbox)
            buf.seek(0)
            
            # Load the image into PIL
            img = Image.open(buf)
            img.load()
            images.append(img)
            
        # 4. Save the compiled GIF using optimization for inter-frame compression
        duration_ms = int(1000.0 / gif_fps)
        if images:
            images[0].save(
                gif_path,
                save_all=True,
                append_images=images[1:],
                duration=duration_ms,
                loop=0,
                optimize=True  # Shrinks the GIF by dropping unchanged background pixels
            )
        print(f"[plot_consensus_animation] wrote {gif_path}")

    if not args.no_show and not args.save_gif:
        plt.show()


if __name__ == "__main__":
    main()
