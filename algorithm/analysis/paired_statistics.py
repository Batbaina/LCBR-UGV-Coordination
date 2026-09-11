#!/usr/bin/env python3
"""Table II (paired CL-CBS vs LCBR decomposition, with Wilcoxon signed-rank
test) and Table III (cross-resolution matrix) -- Sec. 5.2. Logic carried
over unchanged from the validated scripts/analyze_logs.py of the previous
iteration; only the I/O layer (reads runs/<run_id>/, writes
results/tables/) changed.

Usage:
    python3 paired_statistics.py --run-dir ../runs/main_comparison/<run_id> \
        [--delta-w 10]   # required if the run has more than one delta_w value
"""
import argparse
import os
import sys

import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _load as _io  # noqa: E402

try:
    from scipy.stats import wilcoxon
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False


def select_delta_w(instances, delta_w):
    lcbr = instances[instances.method == "LCBR"]
    dws = sorted(lcbr.delta_w_steps.unique())
    if len(dws) <= 1:
        return instances
    if delta_w is None:
        sys.exit(f"This run contains a delta_w sweep {dws} -- pass --delta-w <value>")
    if delta_w not in dws:
        sys.exit(f"--delta-w {delta_w} not found (available: {dws})")
    return instances[(instances.method != "LCBR") | (instances.delta_w_steps == delta_w)]


def paired_comparison(instances):
    clcbs = instances[instances.method == "CL-CBS"].set_index("instance_id")
    lcbr = instances[instances.method == "LCBR"].set_index("instance_id")
    both_success = clcbs.index[clcbs.success == 1].intersection(lcbr.index[lcbr.success == 1])
    n = len(both_success)
    if n == 0:
        print("No instance solved by both methods -- nothing to pair.")
        return pd.DataFrame()

    c = clcbs.loc[both_success]
    l = lcbr.loc[both_success]

    n_ll_c = c.N_full_baseline
    n_ll_l = l.N_success + l.N_fail + l.N_full_fallback
    ebar_c = c.E_tot / n_ll_c.replace(0, pd.NA)
    ebar_l = l.E_tot / n_ll_l.replace(0, pd.NA)

    def wilcoxon_p(a, b):
        if not HAVE_SCIPY or n < 2:
            return float("nan")
        diff = (a - b).values
        if (diff == 0).all():
            return 1.0
        try:
            return wilcoxon(a, b).pvalue
        except ValueError:
            return float("nan")

    rows = [
        ("n (resolved by both)", n, "", ""),
        ("runtime (conflict-resolution only) median",
         c.conflict_resolution_runtime_s.median(), l.conflict_resolution_runtime_s.median(),
         (c.conflict_resolution_runtime_s / l.conflict_resolution_runtime_s).median()),
        ("runtime Wilcoxon p-value", "", "",
         wilcoxon_p(c.conflict_resolution_runtime_s, l.conflict_resolution_runtime_s)),
        ("N_LL median (branch replans actually issued)",
         n_ll_c.median(), n_ll_l.median(), (n_ll_l / n_ll_c.replace(0, pd.NA)).median()),
        ("E-bar median (expansions per low-level call)",
         ebar_c.median(), ebar_l.median(), (ebar_l / ebar_c.replace(0, pd.NA)).median()),
        ("E_tot median", c.E_tot.median(), l.E_tot.median(),
         (l.E_tot / c.E_tot.replace(0, pd.NA)).median()),
        ("E_tot Wilcoxon p-value", "", "", wilcoxon_p(c.E_tot, l.E_tot)),
        ("BCT nodes median", c.bct_nodes_expanded.median(), l.bct_nodes_expanded.median(),
         (l.bct_nodes_expanded / c.bct_nodes_expanded.replace(0, pd.NA)).median()),
        ("SoC_final median", c.SoC_final.median(), l.SoC_final.median(), ""),
        ("Delta SoC % (LCBR vs CL-CBS), median", "", "",
         (100.0 * (l.SoC_final - c.SoC_final) / c.SoC_final).median()),
        ("SoC Wilcoxon p-value", "", "", wilcoxon_p(c.SoC_final, l.SoC_final)),
        ("L_max_final median", c.L_max_final.median(), l.L_max_final.median(), ""),
        ("N_switch median", c.N_switch.median(), l.N_switch.median(), ""),
        ("D_reverse median", c.D_reverse.median(), l.D_reverse.median(), ""),
    ]
    return pd.DataFrame(rows, columns=["metric", "CL-CBS", "LCBR", "ratio_or_delta"])


def resolution_matrix(instances):
    clcbs = instances[instances.method == "CL-CBS"].set_index("instance_id").success
    lcbr = instances[instances.method == "LCBR"].set_index("instance_id").success
    common = clcbs.index.intersection(lcbr.index)
    c, l = clcbs.loc[common], lcbr.loc[common]
    n1 = int(((c == 1) & (l == 1)).sum())
    n2 = int(((c == 1) & (l == 0)).sum())
    n3 = int(((c == 0) & (l == 1)).sum())
    n4 = int(((c == 0) & (l == 0)).sum())
    return pd.DataFrame([[n1, n2], [n3, n4]],
                        index=["CL-CBS solves", "CL-CBS fails"],
                        columns=["LCBR solves", "LCBR fails"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--delta-w", type=int, default=None)
    args = ap.parse_args()

    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    instances, _ = _io.load_run(args.run_dir)
    filtered = select_delta_w(instances, args.delta_w)

    print("=== Table II: paired CL-CBS vs LCBR ===")
    table2 = paired_comparison(filtered)
    print(table2.to_string(index=False))
    print("\n=== Table III: cross-resolution matrix ===")
    table3 = resolution_matrix(filtered)
    print(table3)

    tables_dir = _io.results_dir(root, "tables")
    table2.to_csv(os.path.join(tables_dir, "paired_comparison.csv"), index=False)
    table3.to_csv(os.path.join(tables_dir, "resolution_matrix.csv"))
    print(f"\nwrote {tables_dir}/paired_comparison.csv")
    print(f"wrote {tables_dir}/resolution_matrix.csv")


if __name__ == "__main__":
    main()
