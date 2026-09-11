#!/usr/bin/env python3
"""Table I (Sec. 5.2): per-method summary (success rate, runtime, N_LL,
E-bar, E_tot, BCT nodes, SoC, L_max), grouped by (method, delta_w_steps).

Usage:
    python3 main_comparison.py --run-dir ../runs/main_comparison/<run_id>
"""
import argparse
import os
import sys

import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _load as _io  # noqa: E402


def build_table(instances):
    clcbs = instances[instances.method == "CL-CBS"].copy()
    clcbs["N_LL"] = clcbs.N_full_baseline
    lcbr = instances[instances.method == "LCBR"].copy()
    lcbr["N_LL"] = lcbr.N_success + lcbr.N_fail + lcbr.N_full_fallback
    both = pd.concat([clcbs, lcbr], ignore_index=True)
    both["E_bar"] = both.E_tot / both.N_LL.replace(0, float("nan"))

    g = both.groupby(["method", "delta_w_steps"]).agg(
        n=("instance_id", "count"),
        success_pct=("success", lambda s: 100.0 * s.mean()),
        runtime_median=("conflict_resolution_runtime_s", "median"),
        runtime_q1=("conflict_resolution_runtime_s", lambda s: s.quantile(0.25)),
        runtime_q3=("conflict_resolution_runtime_s", lambda s: s.quantile(0.75)),
        N_LL_median=("N_LL", "median"),
        E_bar_median=("E_bar", "median"),
        E_tot_median=("E_tot", "median"),
        bct_nodes_median=("bct_nodes_expanded", "median"),
        SoC_median=("SoC_final", "median"),
        L_max_median=("L_max_final", "median"),
    ).reset_index()
    return g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    args = ap.parse_args()

    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    instances, _ = _io.load_run(args.run_dir)

    table = build_table(instances)
    print(table.to_string(index=False))

    out_path = os.path.join(_io.results_dir(root, "tables"), "main_comparison.csv")
    table.to_csv(out_path, index=False)
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    main()
