#!/usr/bin/env python3
"""
euclidean_ablation.py -- Sec. 5.6 "mini-ablation": how much does Stage 2
(assignment by SHA* trajectory length, Eq. 6-Eq. 7) buy you over the naive
alternative of assigning by straight-line Euclidean distance, ignoring
obstacles and kinematics entirely?

Rigorous version: reads the REAL M x Q cost matrix L_ij the pipeline
computed (dumped via `ugv_coordination --cost-matrix-csv`), and compares
two permutations evaluated on that SAME matrix:
  (a) the SHA*-optimal assignment (Hungarian on L_ij) -- this is exactly
      what Stage 2 already produces (SoC_nominal in instance_log.csv);
  (b) the Euclidean-optimal assignment (Hungarian on straight-line
      distance), then LOOKED UP in L_ij to see what it actually costs to
      drive with the real kinematics and obstacles.

This is the fair comparison: both permutations are priced with the same
real trajectory lengths, so the gap is attributable purely to the
assignment DECISION, not to Euclidean distance being a lower bound.

Usage:
    # 1. dump the cost matrix once (re-run just for this, or add the flag
    #    to your normal run):
    ./ugv_coordination -i instance.yaml -o /dev/null --mode lcbr \
        --cost-matrix-csv cost_matrix.csv --log-dir /dev/null
    # 2. compare:
    python3 euclidean_ablation.py --instance instance.yaml --cost-matrix cost_matrix.csv
"""
import argparse
import csv
import math

import numpy as np
import yaml
from scipy.optimize import linear_sum_assignment


def load_positions(instance_path):
    d = yaml.safe_load(open(instance_path))
    if "robots" in d and "pois" in d:
        starts = [(r["start"][0], r["start"][1]) for r in d["robots"]]
        goals = [(p["goal"][0], p["goal"][1]) for p in d["pois"]]
    elif "agents" in d:
        starts = [(a["start"][0], a["start"][1]) for a in d["agents"]]
        goals = [(a["goal"][0], a["goal"][1]) for a in d["agents"]]
    else:
        raise ValueError("Unrecognized instance schema (need 'robots'+'pois' or 'agents')")
    return starts, goals


def load_cost_matrix(path, n):
    L = np.full((n, n), np.inf)
    with open(path) as f:
        for row in csv.DictReader(f):
            i, j, v = int(row["robot_id"]), int(row["poi_id"]), float(row["L_ij"])
            L[i, j] = np.inf if v < 0 else v
    return L


def hungarian_cost(cost, big_m=1e9):
    filled = np.where(np.isinf(cost), big_m, cost)
    row, col = linear_sum_assignment(filled)
    return filled[row, col].sum(), list(zip(row.tolist(), col.tolist()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", required=True)
    ap.add_argument("--cost-matrix", required=True,
                     help="CSV dumped via `ugv_coordination --cost-matrix-csv` for this instance")
    args = ap.parse_args()

    starts, goals = load_positions(args.instance)
    n = len(starts)
    if len(goals) != n:
        raise ValueError(f"M != Q ({n} starts, {len(goals)} goals)")

    L = load_cost_matrix(args.cost_matrix, n)

    # (a) SHA*-optimal assignment (what Stage 2 actually does)
    sha_cost, sha_assignment = hungarian_cost(L)

    # (b) Euclidean-optimal assignment, priced with the REAL matrix L
    euclid_matrix = np.array([[math.hypot(s[0] - g[0], s[1] - g[1]) for g in goals] for s in starts])
    _, euclid_assignment = hungarian_cost(euclid_matrix)
    euclid_priced_with_L = sum(L[i, j] for i, j in euclid_assignment)
    infeasible_pairs = [(i, j) for i, j in euclid_assignment if not np.isfinite(L[i, j])]

    print(f"Instance: {args.instance}  (M=Q={n})")
    print(f"\n(a) SHA*-optimal assignment (Stage 2, Eq. 6-7):")
    print(f"    assignment (robot -> POI): {sha_assignment}")
    print(f"    SoC_nominal = {sha_cost:.3f}")
    print(f"\n(b) Euclidean-optimal assignment, driven with the REAL SHA* lengths:")
    print(f"    assignment (robot -> POI): {euclid_assignment}")
    if infeasible_pairs:
        print(f"    contains {len(infeasible_pairs)} INFEASIBLE pair(s) under real kinematics/obstacles: "
              f"{infeasible_pairs} -- no path exists at all for at least one Euclidean-preferred pairing.")
    else:
        print(f"    cost when actually driven = {euclid_priced_with_L:.3f}")
        delta_pct = 100.0 * (euclid_priced_with_L - sha_cost) / sha_cost
        print(f"\n    Delta vs SHA*-optimal: {delta_pct:+.1f}%")
        print("    (Positive = the Euclidean-preferred pairing costs more real travel; "
              "this is the number that justifies the M^2 SHA* queries of Stage 1.)")


if __name__ == "__main__":
    main()

