#!/usr/bin/env python3
"""Convenience orchestrator: runs every analysis/*.py script against a set
of run directories, so 'regenerate everything for the paper' is one
command instead of six. Each individual script remains independently
runnable (and is what you want when only one table/figure changed).

Usage:
    python3 generate_figures.py \
        --main-comparison-run ../runs/main_comparison/<run_id> \
        --delta-w-ablation-run ../runs/delta_w_ablation/<run_id> \
        [--delta-w 10] [--vehicle-config ../config/vehicle_config.yaml]
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def run(script, args_list):
    cmd = ["python3", os.path.join(HERE, script)] + args_list
    print(f"\n=== {script} ===")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"({script} exited {result.returncode} -- continuing with the rest)", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--main-comparison-run", default=None)
    ap.add_argument("--delta-w-ablation-run", default=None)
    ap.add_argument("--delta-w", type=int, default=None,
                     help="LCBR delta_w to use for Tables/Figures that need a single value")
    ap.add_argument("--vehicle-config", default=None)
    args = ap.parse_args()

    if args.main_comparison_run:
        dw_args = ["--delta-w", str(args.delta_w)] if args.delta_w else []
        run("main_comparison.py", ["--run-dir", args.main_comparison_run])
        run("paired_statistics.py", ["--run-dir", args.main_comparison_run] + dw_args)
        run("mechanism_analysis.py", ["--run-dir", args.main_comparison_run])
        run("conflict_density.py", ["--run-dir", args.main_comparison_run] + dw_args)
        if args.vehicle_config:
            run("validate_solutions.py", ["--run-dir", args.main_comparison_run,
                                          "--vehicle-config", args.vehicle_config])
    else:
        print("(--main-comparison-run not given -- skipping Tables I-IV, Figure 2, Figure 4)")

    if args.delta_w_ablation_run:
        baseline_args = ["--baseline-run-dir", args.main_comparison_run] if args.main_comparison_run else []
        run("delta_w_analysis.py", ["--run-dir", args.delta_w_ablation_run] + baseline_args)
    else:
        print("(--delta-w-ablation-run not given -- skipping Table V, Figure 3)")

    print("\nAll requested tables/figures are under results/tables/ and results/figures/.")


if __name__ == "__main__":
    main()
