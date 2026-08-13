#!/usr/bin/env python3
"""
render_demo.py -- renders a static PNG (final trajectories) and an animated
GIF (robots moving along Gamma*) from a UGV-Coordination-System run, reading
the same map YAML given to `ugv_coordination -i` and the solution YAML it
wrote via `-o`. No dependency on tools/visualize.py's Qt backend -- this
uses Agg so it runs headless.
"""
import argparse
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib import animation
import yaml

CAR_WIDTH = 2.0
LF = 2.0
LB = 1.0
OBS_RADIUS = 0.8
COLORS = plt.cm.tab10.colors


def load(map_path, solution_path):
    with open(map_path) as f:
        m = yaml.safe_load(f)
    with open(solution_path) as f:
        s = yaml.safe_load(f)
    return m, s


def car_patch(x, y, yaw, color, alpha=1.0):
    # Rectangle anchored at rear axle (x, y), extending LB behind and LF ahead.
    rect = Rectangle(
        (x - LB, y - CAR_WIDTH / 2),
        LB + LF,
        CAR_WIDTH,
        angle=math.degrees(yaw),
        rotation_point=(x, y),
        facecolor=color,
        edgecolor="black",
        linewidth=0.6,
        alpha=alpha,
        zorder=3,
    )
    return rect


def setup_axes(ax, m):
    dimx, dimy = m["map"]["dimensions"]
    ax.set_xlim(-2, dimx + 2)
    ax.set_ylim(-2, dimy + 2)
    ax.set_aspect("equal")
    for ox, oy in m["map"]["obstacles"]:
        ax.add_patch(Circle((ox, oy), OBS_RADIUS, facecolor="dimgray", edgecolor="none", zorder=2))
    ax.set_facecolor("#f7f7f7")
    ax.grid(alpha=0.25, zorder=0)


def render_static(map_path, solution_path, title, out_png):
    m, s = load(map_path, solution_path)
    agents = sorted(s["schedule"].keys(), key=lambda k: int(k.replace("agent", "")))

    fig, ax = plt.subplots(figsize=(7, 7))
    setup_axes(ax, m)

    for idx, name in enumerate(agents):
        color = COLORS[idx % len(COLORS)]
        traj = s["schedule"][name]
        xs = [pt["x"] for pt in traj]
        ys = [pt["y"] for pt in traj]
        ax.plot(xs, ys, "-", color=color, linewidth=1.6, alpha=0.85, zorder=2, label=name)
        # start (hollow) / goal (filled) markers + a couple of car silhouettes
        ax.add_patch(car_patch(xs[0], ys[0], traj[0]["yaw"], color, alpha=0.55))
        ax.add_patch(car_patch(xs[-1], ys[-1], traj[-1]["yaw"], color, alpha=1.0))
        ax.annotate(name, (xs[0], ys[0]), textcoords="offset points", xytext=(4, 4), fontsize=7)

    stats = s.get("statistics", {})
    subtitle = f"cost={stats.get('cost', 0):.1f}  makespan={stats.get('makespan', 0):.1f}  runtime={stats.get('runtime', 0):.3f}s"
    ax.set_title(f"{title}\n{subtitle}", fontsize=11)
    ax.legend(loc="upper left", fontsize=7, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    print(f"wrote {out_png}")


def interp_state(traj, t):
    """Piecewise-linear interpolation of a trajectory's (x, y, yaw) at
    continuous time t (T_s-step grid -> smooth playback)."""
    if t <= traj[0]["t"]:
        p = traj[0]
        return p["x"], p["y"], p["yaw"]
    if t >= traj[-1]["t"]:
        p = traj[-1]
        return p["x"], p["y"], p["yaw"]
    for k in range(len(traj) - 1):
        a, b = traj[k], traj[k + 1]
        if a["t"] <= t <= b["t"]:
            span = (b["t"] - a["t"]) or 1
            r = (t - a["t"]) / span
            dyaw = (b["yaw"] - a["yaw"] + math.pi) % (2 * math.pi) - math.pi
            return a["x"] + r * (b["x"] - a["x"]), a["y"] + r * (b["y"] - a["y"]), a["yaw"] + r * dyaw
    p = traj[-1]
    return p["x"], p["y"], p["yaw"]


def render_gif(map_path, solution_path, title, out_gif, fps=12, speedup=1.0, trail=True):
    m, s = load(map_path, solution_path)
    agents = sorted(s["schedule"].keys(), key=lambda k: int(k.replace("agent", "")))
    trajs = {name: s["schedule"][name] for name in agents}
    tmax = max(traj[-1]["t"] for traj in trajs.values())

    n_frames = max(2, int(tmax / speedup) + 1)
    frame_times = [i * speedup for i in range(n_frames)]

    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    setup_axes(ax, m)
    stats = s.get("statistics", {})
    ax.set_title(f"{title}  (T_s steps, makespan={stats.get('makespan', 0):.1f})", fontsize=10)

    car_patches = []
    trail_lines = []
    trail_xy = {name: ([], []) for name in agents}
    for idx, name in enumerate(agents):
        color = COLORS[idx % len(COLORS)]
        patch = car_patch(trajs[name][0]["x"], trajs[name][0]["y"], trajs[name][0]["yaw"], color)
        ax.add_patch(patch)
        car_patches.append(patch)
        if trail:
            (line,) = ax.plot([], [], "-", color=color, linewidth=1.0, alpha=0.5, zorder=1)
            trail_lines.append(line)

    def update(frame_idx):
        t = frame_times[frame_idx]
        for idx, name in enumerate(agents):
            x, y, yaw = interp_state(trajs[name], t)
            car_patches[idx].remove()
            color = COLORS[idx % len(COLORS)]
            new_patch = car_patch(x, y, yaw, color)
            ax.add_patch(new_patch)
            car_patches[idx] = new_patch
            if trail:
                trail_xy[name][0].append(x)
                trail_xy[name][1].append(y)
                trail_lines[idx].set_data(trail_xy[name][0], trail_xy[name][1])
        return car_patches + trail_lines

    anim = animation.FuncAnimation(fig, update, frames=n_frames, blit=False, interval=1000 / fps)
    anim.save(out_gif, writer=animation.PillowWriter(fps=fps))
    plt.close(fig)
    print(f"wrote {out_gif}  ({n_frames} frames, tmax={tmax} steps)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--solution", required=True)
    ap.add_argument("--title", required=True)
    ap.add_argument("--out-png", required=True)
    ap.add_argument("--out-gif", required=True)
    ap.add_argument("--speedup", type=float, default=1.0)
    args = ap.parse_args()

    render_static(args.map, args.solution, args.title, args.out_png)
    render_gif(args.map, args.solution, args.title, args.out_gif, speedup=args.speedup)
