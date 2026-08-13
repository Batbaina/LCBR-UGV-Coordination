#!/usr/bin/env python3
"""
visualize_v2.py -- improved replacement for src/visualize.py.

What changed vs. the original, and why:
  - Car shape: a real oriented body (rounded rectangle) + a filled
    triangular "nose" pointing in the heading direction, so you can read
    forward/backward and turning at a glance. The original was a plain
    rectangle -- at a distance, every car looked like every other car and
    you could not tell which way it was facing without zooming in.
  - Color palette: tab20 (20 maximally distinct colors, print-friendly)
    instead of hsv (hsv puts near-identical shades next to each other for
    consecutive agent ids, which is exactly when you most need them to be
    distinguishable).
  - Legend, title (instance name + agent count), light grid, and axis
    tick labels in world units -- none of that existed before, so a
    reader had no scale reference and no idea which run they were
    looking at.
  - Start markers (translucent filled square) in addition to the existing
    dashed goal outlines, so start and goal are both visible even on a
    static frame.
  - Headless-safe: uses the non-interactive "Agg" backend by default and
    only switches to an interactive backend if you explicitly ask to
    --show. The original hard-coded TkAgg, which throws on any machine
    without Tk/X11 (a common source of "nothing happens" reports).
  - New --static mode: renders a single PNG with every agent's full path
    traced from start to goal (no animation, no ffmpeg needed). This is
    almost always what you actually want for a quick sanity check or for
    a figure in the report -- the video is for a demo, the static plot is
    for actually reading trajectories.
  - New --compare mode: puts two solutions (e.g. CL-CBS vs. LCBR) for the
    SAME instance side by side in one static PNG, with per-agent path
    color kept consistent across both panels, and each panel's cost /
    makespan printed in its title -- this is the figure you want for the
    report's Section 6 comparison.
  - GIF export added (via pillow, no ffmpeg dependency) alongside mp4.

Usage:
  # Static, single solution (fast, no video encoder needed):
  python3 visualize_v2.py -m instance.yaml -s solution.yaml --static -o traj.png

  # Animated video:
  python3 visualize_v2.py -m instance.yaml -s solution.yaml -v out.mp4 --speed 2

  # Animated GIF (no ffmpeg needed):
  python3 visualize_v2.py -m instance.yaml -s solution.yaml -v out.gif --speed 2

  # Interactive window (only if you actually have a display):
  python3 visualize_v2.py -m instance.yaml -s solution.yaml --show

  # Side-by-side static comparison of two solvers on the same instance:
  python3 visualize_v2.py -m instance.yaml --compare clcbs.yaml:CL-CBS lcbr.yaml:LCBR \
      -o compare.png
"""
import argparse
import math
import os
import sys

import matplotlib
if "--show" not in sys.argv:
    matplotlib.use("Agg")
else:
    matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyBboxPatch, Circle, Polygon, Rectangle
from matplotlib.transforms import Affine2D
import numpy as np
import yaml

FRAMES_PER_MOVE = 3


def load_car_config(script_dir):
    car_width, lf, lb, obs_radius = 2.0, 2.0, 1.0, 1.0
    for candidate in (os.path.join(script_dir, "config.yaml"),
                     os.path.join(script_dir, "..", "src", "config.yaml"),
                     os.path.join(script_dir, "..", "config", "vehicle_config.yaml")):
        if os.path.isfile(candidate):
            with open(candidate) as f:
                cfg = yaml.safe_load(f)
            car_width = cfg.get("carWidth", car_width)
            lf = cfg.get("LF", lf)
            lb = cfg.get("LB", lb)
            obs_radius = max(cfg.get("obsRadius", obs_radius) - 0.1, 0.1)
            break
    return car_width, lf, lb, obs_radius


def get_state(t, states):
    """Linearly interpolate (x, y, yaw) at continuous time t from the
    discrete `states` list (each {x, y, yaw, t})."""
    idx = 0
    while idx < len(states) and states[idx]["t"] < t:
        idx += 1
    if idx == 0:
        d = states[0]
        return np.array([float(d["x"]), float(d["y"]), float(d["yaw"])])
    if idx >= len(states):
        d = states[-1]
        return np.array([float(d["x"]), float(d["y"]), float(d["yaw"])])
    a, b = states[idx - 1], states[idx]
    yaw_a, yaw_b = float(a["yaw"]), float(b["yaw"])
    if yaw_a - yaw_b > math.pi:
        yaw_a -= 2 * math.pi
    elif yaw_b - yaw_a > math.pi:
        yaw_a += 2 * math.pi
    dt = b["t"] - a["t"]
    frac = (t - a["t"]) / dt if dt else 0.0
    pos_a = np.array([float(a["x"]), float(a["y"]), yaw_a])
    pos_b = np.array([float(b["x"]), float(b["y"]), yaw_b])
    return pos_a + (pos_b - pos_a) * frac


def car_patches(x, y, yaw, color, lf, lb, car_width, alpha=0.9, zorder=5):
    """Return (body_patch, nose_patch): a rounded body rectangle plus a
    small triangular nose so heading is readable at a glance."""
    length = lf + lb
    body = FancyBboxPatch(
        (-lb, -car_width / 2), length, car_width,
        boxstyle=f"round,pad=0,rounding_size={car_width * 0.18}",
        facecolor=color, edgecolor="black", linewidth=0.6,
        alpha=alpha, zorder=zorder)
    nose_len = lf * 0.55
    nose = Polygon(
        [(lf - nose_len * 0.15, -car_width * 0.32),
         (lf + nose_len * 0.55, 0.0),
         (lf - nose_len * 0.15, car_width * 0.32)],
        closed=True, facecolor="black", edgecolor="none",
        alpha=alpha, zorder=zorder + 1)
    tr = Affine2D().rotate(yaw).translate(x, y) + plt.gca().transData
    body.set_transform(tr)
    nose.set_transform(tr)
    return body, nose


class Scene:
    """One map + one schedule, rendered into a given matplotlib Axes."""

    def __init__(self, ax, map_data, schedule, car_width, lf, lb, obs_radius,
                title=None):
        self.ax = ax
        self.map = map_data
        self.schedule = schedule["schedule"]
        self.car_width, self.lf, self.lb = car_width, lf, lb
        self.agents = sorted(self.schedule.keys(),
                             key=lambda n: int(n.replace("agent", "")))
        n_agents = len(self.agents)
        cmap = plt.get_cmap("tab20" if n_agents > 10 else "tab10")
        self.colors = {a: cmap(i % 20) for i, a in enumerate(self.agents)}

        dimx, dimy = map_data["map"]["dimensions"][0], map_data["map"]["dimensions"][1]
        ax.set_xlim(-1.5, dimx + 1.5)
        ax.set_ylim(-1.5, dimy + 1.5)
        ax.set_aspect("equal")
        ax.grid(True, linewidth=0.3, alpha=0.4)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.add_patch(Rectangle((-0.5, -0.5), dimx + 1, dimy + 1,
                               facecolor="none", edgecolor="#555555", lw=1))

        for o in map_data["map"].get("obstacles") or []:
            if o[0] < 0 or o[1] < 0:
                continue  # sentinel entries used by the CL-CBS batching code
            ax.add_patch(Circle((o[0], o[1]), obs_radius,
                                facecolor="#888888", edgecolor="#666666",
                                zorder=1))

        # Two supported instance schemas:
        #   (a) original CL-CBS "agents:" schema -- each agent has a fixed
        #       start AND goal, both known ahead of any solve.
        #   (b) this project's "robots:"/"pois:" schema -- free one-to-one
        #       assignment (Sec. 3.2/4.2 of the paper), so there is no
        #       fixed per-robot goal in the MAP file at all; which POI a
        #       robot actually reaches is only known from the SOLVED
        #       schedule. For (b), the "goal" marker drawn below is
        #       therefore the trajectory's own final state, not a
        #       pre-declared goal field -- this is the only correct
        #       reading given free assignment, and it degrades gracefully
        #       to the same visual (dashed outline at the true endpoint)
        #       either way.
        if "agents" in map_data:
            agents_by_name = {d["name"]: d for d in map_data["agents"]}
        else:
            agents_by_name = {}
            robots = map_data.get("robots", [])
            for i, r in enumerate(robots):
                name = f"agent{i}"
                final = self.schedule[name][-1] if name in self.schedule else None
                goal = [final["x"], final["y"], final["yaw"]] if final else r["start"]
                agents_by_name[name] = {"name": name, "start": r["start"], "goal": goal}
        self.body_patches, self.nose_patches, self.labels = {}, {}, {}
        self.path_x = {a: [] for a in self.agents}
        self.path_y = {a: [] for a in self.agents}
        self.path_lines = {}

        for name in self.agents:
            d = agents_by_name[name]
            color = self.colors[name]
            # start marker
            ax.add_patch(Circle((d["start"][0], d["start"][1]), 0.5,
                                facecolor=color, edgecolor="black",
                                alpha=0.35, zorder=2))
            # goal marker (dashed outline body silhouette)
            gb, gn = car_patches(d["goal"][0], d["goal"][1], d["goal"][2],
                                 "none", lf, lb, car_width, alpha=0.9, zorder=2)
            gb.set_edgecolor(color)
            gb.set_linestyle((0, (3, 2)))
            gb.set_linewidth(1.3)
            gn.set_facecolor("none")
            gn.set_edgecolor(color)
            ax.add_patch(gb)
            ax.add_patch(gn)

            line, = ax.plot([], [], color=color, lw=1.3, alpha=0.7, zorder=3)
            self.path_lines[name] = line

            body, nose = car_patches(d["start"][0], d["start"][1],
                                     d["start"][2], color, lf, lb, car_width)
            ax.add_patch(body)
            ax.add_patch(nose)
            self.body_patches[name] = body
            self.nose_patches[name] = nose
            label = ax.text(d["start"][0], d["start"][1],
                            name.replace("agent", ""), fontsize=6,
                            ha="center", va="center", zorder=6)
            self.labels[name] = label

        # legend: one line-color swatch per agent, capped so it stays readable
        handles = [plt.Line2D([0], [0], color=self.colors[a], lw=3, label=a)
                  for a in self.agents[:20]]
        ax.legend(handles=handles, loc="upper right", fontsize=6,
                 ncol=max(1, len(handles) // 10), framealpha=0.85)

        if title:
            ax.set_title(title, fontsize=10)

        self.T = max(s[-1]["t"] for s in self.schedule.values())

    def draw_static_full_paths(self):
        """Trace every agent's complete path (no animation)."""
        for name in self.agents:
            states = self.schedule[name]
            xs = [s["x"] for s in states]
            ys = [s["y"] for s in states]
            self.path_lines[name].set_data(xs, ys)
            last = states[-1]
            body, nose = self.body_patches[name], self.nose_patches[name]
            tr = Affine2D().rotate(last["yaw"]).translate(
                last["x"], last["y"]) + self.ax.transData
            body.set_transform(tr)
            nose.set_transform(tr)
            self.labels[name].set_position((last["x"], last["y"]))

    def artists(self):
        out = []
        out += list(self.body_patches.values())
        out += list(self.nose_patches.values())
        out += list(self.labels.values())
        out += list(self.path_lines.values())
        return out

    def update_frame(self, t):
        for name in self.agents:
            pos = get_state(t, self.schedule[name])
            tr = Affine2D().rotate(pos[2]).translate(
                pos[0], pos[1]) + self.ax.transData
            self.body_patches[name].set_transform(tr)
            self.nose_patches[name].set_transform(tr)
            self.labels[name].set_position((pos[0], pos[1]))
            self.path_x[name].append(pos[0])
            self.path_y[name].append(pos[1])
            self.path_lines[name].set_data(self.path_x[name], self.path_y[name])
        return self.artists()


def build_title(stats, label=None):
    parts = []
    if label:
        parts.append(label)
    if stats:
        if "cost" in stats:
            parts.append(f"cost={float(stats['cost']):.1f}")
        if "makespan" in stats:
            parts.append(f"makespan={float(stats['makespan']):.1f}")
        if "runtime" in stats:
            parts.append(f"runtime={float(stats['runtime']):.3f}s")
    return "  |  ".join(parts) if parts else None


def render_static(map_data, schedule, car_width, lf, lb, obs_radius, out_path,
                  title=None, dpi=160):
    fig, ax = plt.subplots(figsize=(8, 8))
    scene = Scene(ax, map_data, schedule, car_width, lf, lb, obs_radius, title)
    scene.draw_static_full_paths()
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)
    print(f"Wrote {out_path}")


def render_compare(map_data, solutions, car_width, lf, lb, obs_radius, out_path,
                   dpi=160):
    """solutions: list of (schedule_dict, label)."""
    n = len(solutions)
    fig, axes = plt.subplots(1, n, figsize=(8 * n, 8))
    if n == 1:
        axes = [axes]
    for ax, (schedule, label) in zip(axes, solutions):
        stats = schedule.get("statistics", {})
        title = build_title(stats, label)
        scene = Scene(ax, map_data, schedule, car_width, lf, lb, obs_radius, title)
        scene.draw_static_full_paths()
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)
    print(f"Wrote {out_path}")


def render_animation(map_data, schedule, car_width, lf, lb, obs_radius,
                     video_path, speed, show, title=None, dpi=150):
    fig, ax = plt.subplots(figsize=(8, 8))
    scene = Scene(ax, map_data, schedule, car_width, lf, lb, obs_radius, title)
    n_frames = int(scene.T + 1) * FRAMES_PER_MOVE

    def init():
        return scene.artists()

    def animate(i):
        return scene.update_frame(i / FRAMES_PER_MOVE)

    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=n_frames, interval=100,
                                   blit=True, repeat=False)
    if video_path:
        if video_path.lower().endswith(".gif"):
            anim.save(video_path, writer="pillow", fps=10 * speed, dpi=dpi)
        else:
            anim.save(video_path, writer="ffmpeg", fps=10 * speed, dpi=dpi)
        print(f"Wrote {video_path}")
    elif show:
        plt.show()
    plt.close(fig)


def parse_stats_from_schedule(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return data


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-m", "--map", required=True, help="instance YAML (map + agents)")
    p.add_argument("-s", "--schedule", help="solution YAML (single-solution modes)")
    p.add_argument("-v", "--video", default=None, help="output video/gif path")
    p.add_argument("--speed", type=int, default=1, help="playback speedup factor")
    p.add_argument("--static", action="store_true",
                  help="render one static PNG with full paths instead of animating")
    p.add_argument("--show", action="store_true",
                  help="open an interactive window (needs a display)")
    p.add_argument("-o", "--output", default="trajectories.png",
                  help="output PNG path for --static / --compare")
    p.add_argument("--dpi", type=int, default=160)
    p.add_argument("--compare", nargs="+", default=None,
                  help="one or more solution.yaml:Label pairs to plot side by side, "
                       "e.g. --compare clcbs.yaml:CL-CBS lcbr.yaml:LCBR")
    args = p.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    car_width, lf, lb, obs_radius = load_car_config(script_dir)

    with open(args.map) as f:
        map_data = yaml.safe_load(f)

    if args.compare:
        solutions = []
        for item in args.compare:
            if ":" in item:
                path, label = item.split(":", 1)
            else:
                path, label = item, os.path.basename(item)
            solutions.append((parse_stats_from_schedule(path), label))
        render_compare(map_data, solutions, car_width, lf, lb, obs_radius,
                      args.output, dpi=args.dpi)
        return

    if not args.schedule:
        sys.exit("ERROR: -s/--schedule is required unless --compare is used.")
    schedule = parse_stats_from_schedule(args.schedule)
    title = build_title(schedule.get("statistics", {}),
                        os.path.basename(args.schedule))

    if args.static:
        render_static(map_data, schedule, car_width, lf, lb, obs_radius,
                     args.output, title=title, dpi=args.dpi)
    else:
        render_animation(map_data, schedule, car_width, lf, lb, obs_radius,
                         args.video, args.speed, args.show, title=title,
                         dpi=args.dpi)


if __name__ == "__main__":
    main()