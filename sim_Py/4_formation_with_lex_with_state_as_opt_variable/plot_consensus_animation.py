from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.widgets import Button, Slider
from matplotlib.patches import RegularPolygon, Circle

from consensus_config import NetConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Animate safe-formation trajectories and optionally export GIF.")
    parser.add_argument("--traj-csv", type=str, default="consensus_traj.csv", help="Path to consensus_traj.csv")
    parser.add_argument("--save-gif", action="store_true", help="Save the animation as a GIF")
    parser.add_argument("--gif-path", type=str, default="consensus_animation.gif", help="Output GIF filename")
    parser.add_argument(
        "--gif-speed",
        type=float,
        default=1.0,
        help="GIF playback speed multiplier. >1 faster, <1 slower.",
    )
    parser.add_argument(
        "--gif-fps-base",
        type=float,
        default=12.0,
        help="Base FPS used for GIF export before applying --gif-speed",
    )
    parser.add_argument(
        "--gif-step",
        type=int,
        default=1,
        help="Record every N-th frame into the GIF. Use >1 to reduce GIF size.",
    )
    parser.add_argument(
        "--live-fps",
        type=float,
        default=12.0,
        help="Initial live playback speed in frames per second (display only).",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open the interactive window. Useful together with --save-gif.",
    )
    return parser.parse_args()


def _load_traj(path: Path):
    if not path.exists():
        raise FileNotFoundError(f"Missing trajectory file: {path}")

    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=float)
    if data.ndim != 2 or data.shape[1] < 10:
        raise ValueError(
            f"Unexpected trajectory CSV format in {path}. "
            "Expected at least 10 columns: "
            "[k_global, outer_j, inner_t, agent, r0, r1, v0, v1, u0, u1]."
        )

    k = data[:, 0].astype(int)
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

    k_max = int(k_unique[-1])
    is_double = np.max(np.abs(v0) + np.abs(v1)) > 1e-12

    a2i = {a: i for i, a in enumerate(agents)}
    k2t = {kv: ti for ti, kv in enumerate(k_unique)}

    pos = np.zeros((T, n, 2), dtype=float)
    vel = np.zeros((T, n, 2), dtype=float)
    u = np.zeros((T, n, 2), dtype=float)

    for row in range(data.shape[0]):
        ti = k2t[int(k[row])]
        ai = a2i[int(agent[row])]
        pos[ti, ai, :] = [r0[row], r1[row]]
        vel[ti, ai, :] = [v0[row], v1[row]]
        u[ti, ai, :] = [u0[row], u1[row]]

    return {
        "pos": pos,
        "vel": vel,
        "u": u,
        "agents": agents,
        "k_unique": k_unique,
        "k_max": k_max,
        "is_double": is_double,
    }


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


def _formation_metrics(pts: np.ndarray, offsets: np.ndarray):
    bary = np.mean(pts, axis=0)
    y = pts - offsets
    Vc = _pairwise_diameter(y)
    desired = bary + offsets
    e = np.linalg.norm(pts - desired, axis=1)
    emax = float(np.max(e)) if e.size else 0.0
    return bary, Vc, emax


def main() -> None:
    args = parse_args()
    traj_path = Path(args.traj_csv)
    traj = _load_traj(traj_path)

    pos = traj["pos"]
    vel = traj["vel"]
    u = traj["u"]
    agents = traj["agents"]
    k_unique = traj["k_unique"]
    k_max = traj["k_max"]
    is_double = traj["is_double"]

    T, n, _ = pos.shape

    cfg = NetConfig()
    d_safe = float(cfg.d_safe)
    offsets = cfg.formation_offsets()

    xmin = np.min(pos[:, :, 0]) - (1.5 + d_safe)
    xmax = np.max(pos[:, :, 0]) + (1.5 + d_safe)
    ymin = np.min(pos[:, :, 1]) - (1.5 + d_safe)
    ymax = np.max(pos[:, :, 1]) + (1.5 + d_safe)

    # ------------------------------------------------------------------
    # Interactive figure
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 8))
    plt.subplots_adjust(bottom=0.28)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel("r[0]")
    ax.set_ylabel("r[1]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

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

    formation_text = ax.text(
        0.98,
        0.98,
        "",
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=11,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.7"),
    )

    playing = {"on": True}
    frame = {"t": 0}
    fps = {"val": float(np.clip(args.live_fps, 1.0, 60.0))}
    ani_holder = {"ani": None}

    def _fps_to_interval_ms(fps_value: float) -> int:
        return int(round(1000.0 / max(fps_value, 1e-6)))

    def _set_fps(new_fps: float) -> None:
        new_fps = float(np.clip(new_fps, 1.0, 60.0))
        fps["val"] = new_fps
        ani = ani_holder["ani"]
        if ani is not None and ani.event_source is not None:
            was_running = playing["on"]
            ani.event_source.stop()
            ani.event_source.interval = _fps_to_interval_ms(new_fps)
            if was_running:
                ani.event_source.start()

    def _heading_vec(ti: int, i: int) -> np.ndarray:
        h = vel[ti, i, :] if is_double else u[ti, i, :]
        nrm = float(np.linalg.norm(h))
        if nrm < 1e-9:
            return np.array([1.0, 0.0])
        return h / nrm

    def _formation_overlay_string(ti: int) -> str:
        pts = pos[ti, :, :]
        _, Vc, emax = _formation_metrics(pts, offsets)
        dmin = _pairwise_min_distance(pts)
        return (
            f"safe formation\n"
            f"$V_c$: {Vc:.3f}\n"
            f"$d_{{min}}$: {dmin:.3f}\n"
            f"$e_{{max}}$: {emax:.3f}"
        )

    def _apply_frame(ti: int, target_ax, target_scat, target_lines, target_tris, target_circles, target_text, set_title: bool):
        target_scat.set_offsets(pos[ti, :, :])

        for i in range(n):
            target_lines[i].set_data(pos[: ti + 1, i, 0], pos[: ti + 1, i, 1])

            hv = _heading_vec(ti, i)
            ang = float(np.arctan2(hv[1], hv[0]))
            target_tris[i].xy = (pos[ti, i, 0], pos[ti, i, 1])
            target_tris[i].orientation = ang

            target_circles[i].center = (pos[ti, i, 0], pos[ti, i, 1])

        target_text.set_text(_formation_overlay_string(ti))

        if set_title:
            k_now = int(k_unique[ti])
            target_ax.set_title(
                f"Inner step k={k_now}/{k_max} | "
                f"{'double' if is_double else 'single'} integrator | "
                f"d_safe={d_safe:.2f} | playback={fps['val']:.1f} fps"
            )

    def update(_):
        if playing["on"]:
            frame["t"] = (frame["t"] + 1) % T
        ti = frame["t"]
        _apply_frame(ti, ax, scat, trail_lines, triangles, safety_circles, formation_text, set_title=True)
        return [scat, *trail_lines, *triangles, *safety_circles, formation_text]

    ax_play = plt.axes([0.08, 0.08, 0.12, 0.08])
    btn_play = Button(ax_play, "Pause")

    ax_restart = plt.axes([0.21, 0.08, 0.12, 0.08])
    btn_restart = Button(ax_restart, "Restart")

    ax_minus = plt.axes([0.35, 0.08, 0.06, 0.08])
    btn_minus = Button(ax_minus, "-")

    ax_plus = plt.axes([0.42, 0.08, 0.06, 0.08])
    btn_plus = Button(ax_plus, "+")

    ax_slider = plt.axes([0.52, 0.10, 0.40, 0.03])
    s_speed = Slider(
        ax_slider,
        "Playback FPS",
        valmin=1.0,
        valmax=60.0,
        valinit=fps["val"],
    )

    _slider_updating = {"busy": False}

    def on_play(_):
        playing["on"] = not playing["on"]
        btn_play.label.set_text("Pause" if playing["on"] else "Play")
        ani = ani_holder["ani"]
        if ani is not None and ani.event_source is not None:
            if playing["on"]:
                ani.event_source.start()
            else:
                ani.event_source.stop()

    def on_restart(_):
        frame["t"] = 0
        _apply_frame(0, ax, scat, trail_lines, triangles, safety_circles, formation_text, set_title=True)
        fig.canvas.draw_idle()

    def on_minus(_):
        _set_fps(fps["val"] / 1.25)
        if not _slider_updating["busy"]:
            _slider_updating["busy"] = True
            s_speed.set_val(fps["val"])
            _slider_updating["busy"] = False

    def on_plus(_):
        _set_fps(fps["val"] * 1.25)
        if not _slider_updating["busy"]:
            _slider_updating["busy"] = True
            s_speed.set_val(fps["val"])
            _slider_updating["busy"] = False

    def on_slider(val):
        if _slider_updating["busy"]:
            return
        _set_fps(float(val))

    btn_play.on_clicked(on_play)
    btn_restart.on_clicked(on_restart)
    btn_minus.on_clicked(on_minus)
    btn_plus.on_clicked(on_plus)
    s_speed.on_changed(on_slider)

    ani = FuncAnimation(fig, update, interval=_fps_to_interval_ms(fps["val"]), blit=False)
    ani_holder["ani"] = ani
    _apply_frame(0, ax, scat, trail_lines, triangles, safety_circles, formation_text, set_title=True)

    # ------------------------------------------------------------------
    # GIF export using a dedicated figure containing only the plot area
    # ------------------------------------------------------------------
    if args.save_gif:
        gif_speed = max(float(args.gif_speed), 1e-6)
        gif_fps = max(1.0, float(args.gif_fps_base) * gif_speed)
        gif_step = max(1, int(args.gif_step))
        gif_path = Path(args.gif_path)

        print(
            f"[plot_consensus_animation] Saving GIF to {gif_path} "
            f"with fps={gif_fps:.2f}, step={gif_step} "
            f"(base_fps={args.gif_fps_base:.2f}, speed_multiplier={gif_speed:.2f})"
        )

        # stop live timer before export
        ani.event_source.stop()
        playing["on"] = False
        btn_play.label.set_text("Play")

        fig_gif, ax_gif = plt.subplots(figsize=(8, 8))
        ax_gif.set_xlim(xmin, xmax)
        ax_gif.set_ylim(ymin, ymax)
        ax_gif.set_xlabel("r[0]")
        ax_gif.set_ylabel("r[1]")
        ax_gif.set_aspect("equal", adjustable="box")
        ax_gif.grid(True)

        scat_gif = ax_gif.scatter(pos[0, :, 0], pos[0, :, 1])

        trail_lines_gif = []
        for i in range(n):
            (ln,) = ax_gif.plot([pos[0, i, 0]], [pos[0, i, 1]])
            trail_lines_gif.append(ln)

        triangles_gif = []
        for i in range(n):
            tri = RegularPolygon(
                (pos[0, i, 0], pos[0, i, 1]),
                numVertices=3,
                radius=0.20,
                orientation=0.0,
            )
            ax_gif.add_patch(tri)
            triangles_gif.append(tri)

        safety_circles_gif = []
        for i in range(n):
            c = Circle(
                (pos[0, i, 0], pos[0, i, 1]),
                radius=d_safe,
                fill=False,
                linestyle="--",
                linewidth=1.0,
                alpha=0.55,
            )
            ax_gif.add_patch(c)
            safety_circles_gif.append(c)

        formation_text_gif = ax_gif.text(
            0.98,
            0.98,
            "",
            transform=ax_gif.transAxes,
            ha="right",
            va="top",
            fontsize=11,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.7"),
        )

        writer = PillowWriter(fps=gif_fps)
        with writer.saving(fig_gif, str(gif_path), dpi=100):
            for ti in range(0, T, gif_step):
                _apply_frame(
                    ti,
                    ax_gif,
                    scat_gif,
                    trail_lines_gif,
                    triangles_gif,
                    safety_circles_gif,
                    formation_text_gif,
                    set_title=False,
                )
                fig_gif.canvas.draw()
                writer.grab_frame()

        plt.close(fig_gif)
        print(f"[plot_consensus_animation] Wrote {gif_path}")

    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()