#!/usr/bin/env python3
"""Runs the main CL-CBS vs LCBR comparison (Sec. 5.2) over a
selected_instances[_<tag>].txt file (run select_instances.py first with
the matching --config/--tag, and after any config change). Writes one new
run_id under runs/main_comparison/ with a frozen config.yaml +
metadata.json snapshot, instance_metrics.csv, and low_level_queries.csv.

Run this once per map-size config (or any other split you want -- each
call gets its own run_id, they never collide), then combine the resulting
run directories in analysis/ via _load.merge_runs([...]). See
docs/EXPERIMENTS.md "Splitting the benchmark by map size" for the full
worked example.

Usage:
    python3 run_comparison.py                                # main_comparison.yaml / selected_instances.txt
    python3 run_comparison.py --config ../../config/experiments/main_comparison_100x100.yaml --tag 100x100
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _common as common  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None,
                     help="path to a config/experiments/*.yaml (default: main_comparison.yaml)")
    ap.add_argument("--tag", default=None,
                     help="must match the --tag given to select_instances.py for this batch")
    args = ap.parse_args()

    root = common.repo_root()
    config_path = args.config or os.path.join(root, "config/experiments/main_comparison.yaml")
    cfg = common.load_config(config_path)

    fname = f"selected_instances_{args.tag}.txt" if args.tag else "selected_instances.txt"
    selected_path = os.path.join(HERE, fname)
    if not os.path.exists(selected_path):
        sys.exit(f"{selected_path} not found -- run select_instances.py "
                 f"(with the same --config/--tag) first")
    instances = common.read_selected_instances(selected_path)
    if not instances:
        sys.exit(f"{selected_path} is empty")

    run_id, run_dir = common.new_run_dir("main_comparison", root)
    common.snapshot_config(config_path, run_dir)
    scratch = os.path.join(run_dir, "_scratch_logs")
    sol_dir = os.path.join(run_dir, "solutions")
    os.makedirs(scratch, exist_ok=True)
    os.makedirs(sol_dir, exist_ok=True)
    manifest = common.ManifestWriter(run_dir)

    binary = os.path.join(root, cfg["binary"])
    vehicle_config = os.path.join(root, cfg["vehicle_config"])
    timeout = cfg.get("timeout", 180)
    max_exp = cfg.get("max_low_level_expansions")

    n_ok, n_total = 0, 0
    for inst_path in instances:
        base = os.path.splitext(os.path.basename(inst_path))[0]
        for method in cfg.get("methods", ["clcbs", "lcbr"]):
            if method == "clcbs":
                out_yaml = os.path.join(sol_dir, f"{base}_clcbs.yaml")
                n_total += 1
                n_ok += common.run_binary(binary, inst_path, out_yaml, "clcbs", base,
                                          scratch, vehicle_config, timeout,
                                          max_low_level_expansions=max_exp)
                manifest.add(out_yaml, inst_path, "clcbs")
            elif method == "lcbr":
                for dw in cfg.get("delta_w", [10]):
                    out_yaml = os.path.join(sol_dir, f"{base}_lcbr_dw{dw}.yaml")
                    n_total += 1
                    n_ok += common.run_binary(binary, inst_path, out_yaml, "lcbr", base,
                                              scratch, vehicle_config, timeout, delta_w=dw,
                                              max_low_level_expansions=max_exp)
                    manifest.add(out_yaml, inst_path, "lcbr", dw)

    manifest.close()
    common.finalize_run_csvs(scratch, run_dir)
    print(f"\n[main_comparison{'/'+args.tag if args.tag else ''}] {n_ok}/{n_total} runs OK.")
    print(f"  runs/main_comparison/{run_id}/instance_metrics.csv")
    print(f"  runs/main_comparison/{run_id}/low_level_queries.csv")
    print(f"  runs/main_comparison/{run_id}/config.yaml + metadata.json")


if __name__ == "__main__":
    main()
