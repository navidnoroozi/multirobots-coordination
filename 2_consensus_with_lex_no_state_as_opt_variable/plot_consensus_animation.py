# plot_consensus_animation_plus.py
#
# Animated 2D trajectory replay with:
# 1) Speed slider (Matplotlib Slider)
# 2) Frame-by-frame stepping with arrow keys
# 3) Export to GIF (PillowWriter) via:
#    - UI button "Export GIF", or
#    - CLI: python plot_consensus_animation_plus.py --gif out.gif
#
# Controls:
# - Space: Pause/Resume
# - Left/Right: Step -1 / +1 frame (best while paused; also works while playing)
# - Up/Down: Speed x1.25 / /1.25
# - Home: Reset to frame 0
# - End: Jump near end
# - Slider: continuous speed control (0.02x .. 10x)
#
# Notes:
# - Loops forever while the figure is open (interactive mode).
# - For GIF export, the output is one full pass through the timeline (T frames).
#
# Requirements:
#   pip install pandas numpy matplotlib pillow
#
# Usage:
#   python plot_consensus_animation_plus.py
#   python plot_consensus_animation_plus.py --log consensus_log.csv
#   python plot_consensus_animation_plus.py --gif consensus.gif
#
import argparse
import time
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.widgets import Slider, Button


def _rot2(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)


def _triangle_xy(center: np.ndarray, heading: np.ndarray, size: float) -> np.ndarray:
    """
    Oriented triangle polygon. heading determines orientation.
    """
    cx, cy = float(center[0]), float(center[1])
    h = np.array(heading, dtype=float)
    n = float(np.linalg.norm(h))
    theta = 0.0 if n < 1e-12 else float(np.arctan2(h[1], h[0]))

    # Triangle in local frame pointing along +x (nose)
    local = np.array([
        [ 1.25,  0.0],   # nose
        [-0.85,  0.62],  # rear-left
        [-0.85, -0.62],  # rear-right
    ], dtype=float) * float(size)

    R = _rot2(theta)
    world = local @ R.T
    world[:, 0] += cx
    world[:, 1] += cy
    return world


def load_trajectories(log_csv: str):
    df = pd.read_csv(log_csv).sort_values(["inner_k", "agent"])
    agents = sorted(df["agent"].unique())
    traj = {}
    for a in agents:
        dfa = df[df["agent"] == a].sort_values(["inner_k"])
        traj[a] = np.vstack([dfa["x0"].values, dfa["x1"].values]).T
    T = min(traj[a].shape[0] for a in agents)
    for a in agents:
        traj[a] = traj[a][:T, :]
    return traj, agents, T


def export_gif(
    traj,
    agents,
    T: int,
    out_gif: str,
    fps: int = 30,
    trail_len: int = 45,
    show_global_barycenter: bool = True,
):
    """
    Export a single full pass (T frames) as a GIF.
    """
    out_path = Path(out_gif)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Compute bounds
    all_xy = np.vstack([traj[a] for a in agents])
    xmin, ymin = all_xy.min(axis=0)
    xmax, ymax = all_xy.max(axis=0)
    pad = 0.15 * max(xmax - xmin, ymax - ymin, 1.0)
    xmin -= pad; xmax += pad; ymin -= pad; ymax += pad

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel("x0")
    ax.set_ylabel("x1")
    ax.set_title("Distributed MPC Consensus — Animated Trajectories (GIF export)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    bary_scatter = None
    if show_global_barycenter:
        bary_scatter = ax.scatter([], [], s=120, marker="*", linewidths=1.5, zorder=6)

    trail_lines = {}
    glow_points = {}
    tri_patches = {}

    for a in agents:
        (line,) = ax.plot([], [], lw=2.0, alpha=0.55, label=f"agent {a}", zorder=2)
        trail_lines[a] = line
        glow = ax.scatter([], [], s=240, alpha=0.15, zorder=4)
        dot = ax.scatter([], [], s=50, zorder=5)
        glow_points[a] = (glow, dot)
        patch = plt.Polygon(np.zeros((3, 2)), closed=True, alpha=0.90, zorder=7)
        ax.add_patch(patch)
        tri_patches[a] = patch

    ax.legend(loc="upper right")

    time_text = ax.text(
        0.02, 0.98, "",
        transform=ax.transAxes,
        ha="left", va="top",
        fontsize=11,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.65),
        zorder=10
    )

    # Size scaling for triangles
    tri_size = 0.25 * max(xmax - xmin, ymax - ymin) / 10.0

    def init():
        for a in agents:
            trail_lines[a].set_data([], [])
            glow, dot = glow_points[a]
            glow.set_offsets(np.empty((0, 2)))
            dot.set_offsets(np.empty((0, 2)))
            tri_patches[a].set_xy(np.zeros((3, 2)))
        if bary_scatter is not None:
            bary_scatter.set_offsets(np.empty((0, 2)))
        time_text.set_text("")
        return []

    def update_by_k(k: int):
        for a in agents:
            pts = traj[a]
            k0 = max(0, k - trail_len)
            trail = pts[k0:k + 1, :]
            trail_lines[a].set_data(trail[:, 0], trail[:, 1])

            p = pts[k, :]
            glow, dot = glow_points[a]
            glow.set_offsets(p.reshape(1, 2))
            dot.set_offsets(p.reshape(1, 2))

            if k >= 1:
                heading = pts[k, :] - pts[k - 1, :]
            else:
                heading = np.array([1.0, 0.0], dtype=float)

            tri = _triangle_xy(p, heading, size=tri_size)
            tri_patches[a].set_xy(tri)

        if bary_scatter is not None:
            bary = np.mean(np.vstack([traj[a][k, :] for a in agents]), axis=0)
            bary_scatter.set_offsets(bary.reshape(1, 2))

        time_text.set_text(f"frame k={k}/{T-1}")
        return []

    ani = FuncAnimation(fig, lambda k: update_by_k(int(k)), init_func=init, frames=T, interval=1000/fps, blit=False)

    writer = PillowWriter(fps=fps)
    print(f"[GIF] Saving {T} frames to: {out_path.resolve()}")
    ani.save(str(out_path), writer=writer)
    plt.close(fig)
    print("[GIF] Done.")


def interactive_player(
    traj,
    agents,
    T: int,
    fps: int = 30,
    trail_len: int = 45,
    show_global_barycenter: bool = True,
):
    # Compute bounds
    all_xy = np.vstack([traj[a] for a in agents])
    xmin, ymin = all_xy.min(axis=0)
    xmax, ymax = all_xy.max(axis=0)
    pad = 0.15 * max(xmax - xmin, ymax - ymin, 1.0)
    xmin -= pad; xmax += pad; ymin -= pad; ymax += pad

    plt.rcParams["figure.figsize"] = (10, 8)

    fig, ax = plt.subplots()
    # Leave room for widgets
    plt.subplots_adjust(bottom=0.20)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel("x0")
    ax.set_ylabel("x1")
    ax.set_title("Distributed MPC Consensus — Animated Trajectories (looping replay)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    # Optional global barycenter
    bary_scatter = None
    if show_global_barycenter:
        bary_scatter = ax.scatter([], [], s=140, marker="*", linewidths=1.5, zorder=6)

    trail_lines = {}
    glow_points = {}
    tri_patches = {}

    for a in agents:
        (line,) = ax.plot([], [], lw=2.0, alpha=0.55, label=f"agent {a}", zorder=2)
        trail_lines[a] = line
        glow = ax.scatter([], [], s=260, alpha=0.15, zorder=4)
        dot = ax.scatter([], [], s=55, zorder=5)
        glow_points[a] = (glow, dot)
        patch = plt.Polygon(np.zeros((3, 2)), closed=True, alpha=0.90, zorder=7)
        ax.add_patch(patch)
        tri_patches[a] = patch

    ax.legend(loc="upper right")

    time_text = ax.text(
        0.02, 0.98, "",
        transform=ax.transAxes,
        ha="left", va="top",
        fontsize=11,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.65),
        zorder=10
    )

    # Widgets: speed slider + export button
    ax_speed = fig.add_axes([0.12, 0.08, 0.62, 0.03])
    speed_slider = Slider(
        ax=ax_speed,
        label="Speed",
        valmin=0.02,
        valmax=10.0,
        valinit=1.0,
        valfmt="%.2fx",
    )

    ax_btn = fig.add_axes([0.78, 0.06, 0.18, 0.07])
    btn_export = Button(ax_btn, "Export GIF", hovercolor="0.9")

    # Playback state
    interval_ms = int(1000 / fps)
    playback_rate = {"rate": 1.0}      # frames per update; can be < 1 for slow motion
    time_acc = {"t": 0.0}              # floating accumulator (frame units)
    paused = {"flag": False}

    # triangle size scaling
    tri_size = 0.25 * max(xmax - xmin, ymax - ymin) / 10.0

    def set_speed(val):
        playback_rate["rate"] = float(val)

    speed_slider.on_changed(set_speed)

    def init():
        for a in agents:
            trail_lines[a].set_data([], [])
            glow, dot = glow_points[a]
            glow.set_offsets(np.empty((0, 2)))
            dot.set_offsets(np.empty((0, 2)))
            tri_patches[a].set_xy(np.zeros((3, 2)))
        if bary_scatter is not None:
            bary_scatter.set_offsets(np.empty((0, 2)))
        time_text.set_text("")
        return []

    def draw_at_k(k: int):
        k = int(k) % T
        for a in agents:
            pts = traj[a]
            k0 = max(0, k - trail_len)
            trail = pts[k0:k + 1, :]
            trail_lines[a].set_data(trail[:, 0], trail[:, 1])

            p = pts[k, :]
            glow, dot = glow_points[a]
            glow.set_offsets(p.reshape(1, 2))
            dot.set_offsets(p.reshape(1, 2))

            if k >= 1:
                heading = pts[k, :] - pts[k - 1, :]
            else:
                heading = np.array([1.0, 0.0], dtype=float)

            tri = _triangle_xy(p, heading, size=tri_size)
            tri_patches[a].set_xy(tri)

        if bary_scatter is not None:
            bary = np.mean(np.vstack([traj[a][k, :] for a in agents]), axis=0)
            bary_scatter.set_offsets(bary.reshape(1, 2))

        time_text.set_text(f"k={k}/{T-1} | speed={playback_rate['rate']:.2f}x | "
                           f"{'PAUSED' if paused['flag'] else 'PLAY'}")
        fig.canvas.draw_idle()

    def update(_frame_idx):
        # Continuous-time playback in "frame units"
        time_acc["t"] += playback_rate["rate"]
        k = int(time_acc["t"]) % T
        # Draw
        draw_at_k(k)
        return []

    ani = FuncAnimation(fig, update, init_func=init, interval=interval_ms, blit=False)

    def export_from_ui(_event=None):
        # Export one full pass of frames to a timestamped gif
        out_name = f"consensus_anim_{int(time.time())}.gif"
        out_path = Path(out_name).resolve()

        # Temporarily pause animation for a clean export
        was_paused = paused["flag"]
        if not paused["flag"]:
            paused["flag"] = True
            ani.event_source.stop()

        # Build a fresh animation object for exporting (avoids UI state issues)
        export_gif(
            traj=traj,
            agents=agents,
            T=T,
            out_gif=str(out_path),
            fps=fps,
            trail_len=trail_len,
            show_global_barycenter=show_global_barycenter,
        )
        print(f"[UI] Exported GIF to: {out_path}")

        # Resume if it was playing
        if not was_paused:
            paused["flag"] = False
            ani.event_source.start()

    btn_export.on_clicked(export_from_ui)

    def on_key(event):
        # Space: pause/resume
        if event.key == " ":
            paused["flag"] = not paused["flag"]
            if paused["flag"]:
                ani.event_source.stop()
            else:
                ani.event_source.start()

        # Frame stepping (best while paused, still works while playing)
        elif event.key == "left":
            time_acc["t"] -= 1.0
            draw_at_k(int(time_acc["t"]))

        elif event.key == "right":
            time_acc["t"] += 1.0
            draw_at_k(int(time_acc["t"]))

        # Speed adjustments
        elif event.key == "up":
            new_rate = min(playback_rate["rate"] * 1.25, 10.0)
            playback_rate["rate"] = new_rate
            speed_slider.set_val(new_rate)

        elif event.key == "down":
            new_rate = max(playback_rate["rate"] / 1.25, 0.02)
            playback_rate["rate"] = new_rate
            speed_slider.set_val(new_rate)

        # Jump controls
        elif event.key == "home":
            time_acc["t"] = 0.0
            draw_at_k(0)

        elif event.key == "end":
            time_acc["t"] = float(T - 2)
            draw_at_k(T - 2)

        # Quick export hotkey
        elif event.key in ["g", "G"]:
            export_from_ui()

    fig.canvas.mpl_connect("key_press_event", on_key)

    # Start display
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", type=str, default="consensus_log.csv", help="CSV log (from plant simulation)")
    ap.add_argument("--gif", type=str, default="", help="If set, export GIF to this path and exit")
    ap.add_argument("--fps", type=int, default=30, help="FPS for animation and GIF export")
    ap.add_argument("--trail", type=int, default=45, help="Trail length (frames)")
    ap.add_argument("--no-bary", action="store_true", help="Disable global barycenter marker")
    args = ap.parse_args()

    traj, agents, T = load_trajectories(args.log)
    show_bary = not args.no_bary

    if args.gif:
        export_gif(
            traj=traj,
            agents=agents,
            T=T,
            out_gif=args.gif,
            fps=args.fps,
            trail_len=args.trail,
            show_global_barycenter=show_bary,
        )
    else:
        interactive_player(
            traj=traj,
            agents=agents,
            T=T,
            fps=args.fps,
            trail_len=args.trail,
            show_global_barycenter=show_bary,
        )


if __name__ == "__main__":
    main()
