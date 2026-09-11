#!/usr/bin/env python3
"""Alternative to select_instances.py for the delta_w ablation: instead
of picking the first N instances alphabetically (which can land almost
entirely on trivial 0-conflict cases -- exactly what happened on the
100x100/300x300 ablation, where E_tot_median=0.0 at every delta_w),
this reads an EXISTING runs/main_comparison/ campaign and picks only
instances where CL-CBS itself already needed more than one BCT node
(bct_nodes_expanded > 1, i.e. a real conflict was resolved).

Fewer, denser-in-signal instances -> the same informative E_tot-vs-delta_w
curve as the 50x50 ablation, without multiplying wall-clock time by
re-running mostly-trivial cases.

Usage:
    cd experiments/delta_w_ablation
    python3 select_conflict_instances.py --map-size 100by100 --scenario obstacle \\
        --top-n 15 --tag 100conf
    python3 select_conflict_instances.py --map-size 300by300 --scenario obstacle \\
        --top-n 10 --tag 300conf

Then run the ablation directly against this selection (skip
select_instances.py entirely for this tag -- this script already wrote
the same selected_instances_<tag>.txt format it expects):
    python3 run_ablation.py --config ../../config/experiments/delta_w_ablation_100x100.yaml \\
        --tag 100conf --parallel 3
"""
import argparse
import os
import sys

import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _common as common  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def instance_id_to_path(root, instance_id, scenario):
    """Reverse-engineers the benchmark file path from instance_id
    (e.g. 'map_100by100_obst50_agents30_ex1' ->
    benchmark/map100by100/agents30/obstacle/map_100by100_obst50_agents30_ex1.yaml).
    Relies on Wen et al.'s own naming convention -- the same one
    map_size/scenario extraction in analysis.ipynb depends on."""
    parts = instance_id.split("_")
    # map_100by100_obst50_agents30_ex1 -> ["map","100by100","obst50","agents30","ex1"]
    map_size = parts[1]
    agents_part = next(p for p in parts if p.startswith("agents"))
    return os.path.join(root, "benchmark", f"map{map_size}", agents_part, scenario,
                        f"{instance_id}.yaml")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-size", required=True, help="e.g. 100by100, 300by300")
    ap.add_argument("--scenario", default="obstacle", choices=["obstacle", "empty"])
    ap.add_argument("--top-n", type=int, default=15,
                    help="how many conflict-having instances to select, per (map_size, scenario)")
    ap.add_argument("--min-bct-nodes", type=int, default=2,
                    help="CL-CBS bct_nodes_expanded must exceed this (>1 = at least one real conflict)")
    ap.add_argument("--tag", required=True,
                    help="writes selected_instances_<tag>.txt, same format run_ablation.py expects")
    args = ap.parse_args()

    root = common.repo_root()
    from importlib import import_module
    sys.path.insert(0, os.path.join(root, "analysis"))
    _load = import_module("_load")

    run_dirs = _load.discover_runs("main_comparison", root)
    if not run_dirs:
        sys.exit("No runs/main_comparison/ found -- run the main comparison campaign first, "
                 "this script selects FROM its results.")
    instances, _ = _load.merge_runs(run_dirs)

    mask = (
        instances.instance_id.str.contains(f"map_{args.map_size}_") &
        (instances.method == "CL-CBS") &
        (instances.success == 1) &
        (instances.bct_nodes_expanded >= args.min_bct_nodes)
    )
    if args.scenario == "empty":
        mask &= instances.instance_id.str.contains("_obst0_")
    else:
        mask &= ~instances.instance_id.str.contains("_obst0_")

    candidates = instances[mask].sort_values("bct_nodes_expanded", ascending=False)
    selected = candidates.head(args.top_n)

    if selected.empty:
        sys.exit(f"No instances found with map_size={args.map_size}, scenario={args.scenario}, "
                 f"CL-CBS bct_nodes_expanded>={args.min_bct_nodes} -- lower --min-bct-nodes, or "
                 f"this map size genuinely has few/no conflicts in your main_comparison data.")

    print(f"Selected {len(selected)} instance(s), bct_nodes_expanded range: "
         f"{selected.bct_nodes_expanded.min()}-{selected.bct_nodes_expanded.max()}")

    paths = [instance_id_to_path(root, iid, args.scenario) for iid in selected.instance_id]
    missing = [p for p in paths if not os.path.exists(p)]
    if missing:
        print(f"WARNING: {len(missing)} selected instance file(s) not found on disk "
             f"(benchmark/ folder incomplete?):")
        for p in missing[:5]:
            print("  ", p)
        paths = [p for p in paths if os.path.exists(p)]

    out_path = os.path.join(HERE, f"selected_instances_{args.tag}.txt")
    common.write_selected_instances(paths, out_path)
    print(f"-> {out_path} ({len(paths)} usable instance(s))")


if __name__ == "__main__":
    main()
