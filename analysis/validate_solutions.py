#!/usr/bin/env python3
"""Batch version of the independent C3b/C4/C5 audit -- Sec. 5.2's "0
violation" sentence, checked, not assumed. Reimplemented from scratch
(exact rotated-rectangle obstacle test, not an approximation), does not
import anything from include/ -- the point is to catch bugs the solver
itself might share between its own construction and its own validation.

Iterates every (solution, instance) pair recorded in a run's manifest.csv
(written by experiments/*/run_*.py), which is the only place that maps a
solution YAML back to the map YAML it came from.

Usage:
    python3 validate_solutions.py --run-dir ../runs/main_comparison/<run_id> \
        --vehicle-config ../config/vehicle_config.yaml
"""
import argparse
import csv
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _load as _io  # noqa: E402


def load_vehicle_config(path):
    try:
        d = yaml.safe_load(open(path)) or {}
    except FileNotFoundError:
        d = {}
    return {
        "r": d.get("r", 3.0), "deltat": d.get("deltat", 0.706),
        "carWidth": d.get("carWidth", 2.0), "LF": d.get("LF", 2.0),
        "LB": d.get("LB", 1.0), "obsRadius": d.get("obsRadius", 0.8),
    }


def agent_collision(s1, s2, LF, carWidth):
    d2 = (s1["x"] - s2["x"]) ** 2 + (s1["y"] - s2["y"]) ** 2
    return d2 < (2 * LF) ** 2 + carWidth ** 2


def obs_collision_exact(s, obstacle, LF, LB, carWidth, obsRadius):
    # Exact replica of src/ugv_coordination.cpp State::obsCollision: rotate
    # the obstacle into the vehicle's body frame and test an axis-aligned
    # box, not a circle proxy.
    dx = obstacle[0] - s["x"]
    dy = obstacle[1] - s["y"]
    yaw = s["yaw"]
    c, sN = math.cos(-yaw), math.sin(-yaw)
    rx = dx * c + dy * sN
    ry = dx * (-sN) + dy * c
    return (-LB - obsRadius < rx < LF + obsRadius) and \
           (-carWidth / 2.0 - obsRadius < ry < carWidth / 2.0 + obsRadius)


def validate_one(map_path, solution_path, cfg):
    r, deltat = cfg["r"], cfg["deltat"]
    LF, LB, carWidth, obsRadius = cfg["LF"], cfg["LB"], cfg["carWidth"], cfg["obsRadius"]

    m = yaml.safe_load(open(map_path))
    s = yaml.safe_load(open(solution_path))
    # A failed/infeasible run still gets its output file OPENED (creating
    # an empty file) by ugv_coordination even though nothing is written to
    # it on failure -- yaml.safe_load() on an empty file returns None, not
    # a dict. Skip these explicitly (None schedule) rather than crashing;
    # the caller already has success/failure from instance_metrics.csv.
    if not s or "schedule" not in s:
        return None, 0, 0
    obstacles = m["map"]["obstacles"]
    dimx, dimy = m["map"]["dimensions"]

    agents = sorted(s["schedule"].keys(), key=lambda k: int(k.replace("agent", "")))
    trajs = {name: s["schedule"][name] for name in agents}

    violations = {"C4_obstacle": 0, "C4_bounds": 0, "C5_interrobot": 0, "C3b_kinematic": 0}

    for name, traj in trajs.items():
        for st in traj:
            if not (0 <= st["x"] <= dimx and 0 <= st["y"] <= dimy):
                violations["C4_bounds"] += 1
            for obs in obstacles:
                if obs_collision_exact(st, obs, LF, LB, carWidth, obsRadius):
                    violations["C4_obstacle"] += 1

    by_time = {}
    for name, traj in trajs.items():
        for st in traj:
            by_time.setdefault(st["t"], {})[name] = st
    for t, states_at_t in by_time.items():
        names = list(states_at_t.keys())
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                if agent_collision(states_at_t[names[i]], states_at_t[names[j]], LF, carWidth):
                    violations["C5_interrobot"] += 1

    max_step = r * deltat * 1.15
    for name, traj in trajs.items():
        for k in range(1, len(traj)):
            a, b = traj[k - 1], traj[k]
            step = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
            if step > max_step:
                violations["C3b_kinematic"] += 1

    n_states = sum(len(t) for t in trajs.values())
    return violations, len(agents), n_states


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--vehicle-config", default=None)
    args = ap.parse_args()

    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    cfg = load_vehicle_config(args.vehicle_config) if args.vehicle_config else load_vehicle_config("")

    manifest_path = os.path.join(args.run_dir, "manifest.csv")
    if not os.path.exists(manifest_path):
        sys.exit(f"{manifest_path} not found -- this run predates manifest.csv, or is not "
                 f"from experiments/*/run_*.py")

    sol_dir = os.path.join(args.run_dir, "solutions")
    rows_out = []
    total_violations = 0
    with open(manifest_path) as f:
        for row in csv.DictReader(f):
            sol_path = os.path.join(sol_dir, row["solution_filename"])
            map_path = row["instance_path"]
            if not (os.path.exists(sol_path) and os.path.exists(map_path)):
                continue
            violations, n_agents, n_states = validate_one(map_path, sol_path, cfg)
            if violations is None:
                # Empty solution file (failed/infeasible run) -- nothing to
                # validate, and correctly NOT counted as "0 violations"
                # (that would misleadingly look like a validated success).
                continue
            v_total = sum(violations.values())
            total_violations += v_total
            rows_out.append({
                "solution_filename": row["solution_filename"], "method": row["method"],
                "delta_w": row["delta_w"], "n_agents": n_agents, "n_states": n_states,
                **violations, "total_violations": v_total,
            })
            flag = "OK" if v_total == 0 else "VIOLATIONS"
            print(f"[{flag}] {row['solution_filename']}  ({row['method']}"
                  f"{' dw='+row['delta_w'] if row['delta_w'] else ''})  "
                  f"C4_obs={violations['C4_obstacle']} C4_bounds={violations['C4_bounds']} "
                  f"C5={violations['C5_interrobot']} C3b={violations['C3b_kinematic']}")

    import pandas as pd
    table = pd.DataFrame(rows_out)
    tables_dir = _io.results_dir(root, "tables")
    out_path = os.path.join(tables_dir, "validation_report.csv")
    table.to_csv(out_path, index=False)

    print(f"\n{len(rows_out)} solutions checked, {total_violations} total flagged states/pairs.")
    print(f"wrote {out_path}")
    sys.exit(0 if total_violations == 0 else 1)


if __name__ == "__main__":
    main()
