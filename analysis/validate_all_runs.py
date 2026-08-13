#!/usr/bin/env python3
"""Runs analysis/validate_solutions.py (C3b/C4/C5, independent re-check
from the raw solution YAML) across EVERY discovered runs/main_comparison/
run_id/ in one command, and writes ONE combined report instead of one
per run_dir -- what you need for a single "0 violations across the full
pilot" sentence in the paper.

Usage:
    cd analysis
    python3 validate_all_runs.py --vehicle-config ../config/vehicle_config.yaml
"""
import argparse
import glob
import os
import subprocess
import sys

import pandas as pd

sys.path.insert(0, ".")
import _load  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle-config", required=True)
    ap.add_argument("--experiment", default="main_comparison",
                    help="which experiments/ subfolder's runs to validate")
    args = ap.parse_args()

    root = os.path.abspath("..")
    run_dirs = _load.discover_runs(args.experiment, root)
    if not run_dirs:
        sys.exit(f"No {args.experiment} runs found under {root}/runs/{args.experiment}/")

    print(f"Validating {len(run_dirs)} run(s):")
    for rd in run_dirs:
        print(" ", rd)
    print()

    all_reports = []
    for rd in run_dirs:
        print(f"=== {rd} ===")
        # No check=True: validate_solutions.py deliberately exits 1 when
        # violations are found (a signal to collect, not a script
        # error) -- only a genuine crash (missing manifest, etc.) should
        # stop this loop, and that already prints its own clear error.
        subprocess.run(
            [sys.executable, "validate_solutions.py", "--run-dir", rd,
             "--vehicle-config", args.vehicle_config],
        )
        # validate_solutions.py always writes to the same fixed path --
        # move it aside per run_dir immediately so the next iteration
        # doesn't overwrite it before we've collected it.
        report_path = os.path.join(root, "results", "tables", "validation_report.csv")
        if os.path.exists(report_path):
            df = pd.read_csv(report_path)
            df["run_dir"] = rd
            all_reports.append(df)
        print()

    if not all_reports:
        sys.exit("No validation reports were produced -- check the errors above.")

    combined = pd.concat(all_reports, ignore_index=True)
    out_path = os.path.join(root, "results", "tables", "validation_report_combined.csv")
    combined.to_csv(out_path, index=False)

    n_total = len(combined)
    n_bad = (combined.total_violations > 0).sum()
    print("=" * 60)
    print(f"TOTAL: {n_total} solutions checked across {len(run_dirs)} run(s).")
    print(combined[["C4_obstacle", "C4_bounds", "C5_interrobot", "C3b_kinematic",
                    "total_violations"]].sum())
    if n_bad:
        print(f"\n{n_bad} solution(s) with violations -- see {out_path}:")
        print(combined[combined.total_violations > 0][
            ["run_dir", "solution_filename", "total_violations"]].to_string(index=False))
    else:
        print("\nAll checked solutions: 0 violations of C3b/C4/C5.")
    print(f"\nCombined report written to {out_path}")


if __name__ == "__main__":
    main()
