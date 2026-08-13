#!/usr/bin/env python3
"""Table IV (LCBR mechanism summary, incl. E-bar_full sanity check) and
Figure 2 (r_w / DeltaT histograms) -- Sec. 5.3.

Usage:
    python3 mechanism_analysis.py --run-dir ../runs/main_comparison/<run_id>
"""
import argparse
import os
import sys

import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _load as _io  # noqa: E402


def mechanism_summary(instances):
    lcbr = instances[instances.method == "LCBR"]
    clcbs = instances[instances.method == "CL-CBS"]
    if lcbr.empty:
        return pd.DataFrame()
    n_local_query = lcbr.N_success + lcbr.N_fail
    r_success = (lcbr.N_success / n_local_query.replace(0, pd.NA)).median()
    n_branch = n_local_query + lcbr.N_inapplicable
    r_inapp = (lcbr.N_inapplicable / n_branch.replace(0, pd.NA)).median()
    r_wasted = (lcbr.E_local_fail / lcbr.E_tot.replace(0, pd.NA)).median()

    ebar_full_lcbr = (lcbr.E_full / lcbr.N_full_fallback.replace(0, pd.NA)).median()
    ebar_full_clcbs = ((clcbs.E_full / clcbs.N_full_baseline.replace(0, pd.NA)).median()
                        if not clcbs.empty else float("nan"))
    sanity_ratio = (ebar_full_lcbr / ebar_full_clcbs
                     if ebar_full_clcbs == ebar_full_clcbs and ebar_full_clcbs else float("nan"))

    return pd.DataFrame([{
        "median_r_success": r_success,
        "median_r_inapplicable": r_inapp,
        "median_r_wasted": r_wasted,
        "median_E_local_success": lcbr.E_local_success.median(),
        "median_E_bar_full_LCBR_fallback": ebar_full_lcbr,
        "median_E_bar_full_CLCBS_baseline": ebar_full_clcbs,
        "sanity_ratio": sanity_ratio,
    }])


def make_figure2(queries, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if queries is None:
        print("(no low_level_queries.csv -- skipping Figure 2)")
        return
    local_success = queries[(queries.method == "LCBR") & (queries.query_type == "local") &
                            (queries.local_outcome == "success")]
    if local_success.empty:
        print("(no successful local repairs -- skipping Figure 2)")
        return

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].hist(local_success.window_fraction, bins=20, color="tab:blue", alpha=0.8)
    axes[0].set_xlabel(r"$r_w = T_w / T_i^N$")
    axes[0].set_ylabel("count")
    axes[0].set_title("Fraction of horizon replanned")
    axes[0].axvline(local_success.window_fraction.median(), color="black", linestyle="--",
                    label=f"median={local_success.window_fraction.median():.2f}")
    axes[0].legend(fontsize=8)

    axes[1].hist(local_success.delta_T_repair, bins=20, color="tab:orange", alpha=0.8)
    axes[1].set_xlabel(r"$\Delta T = T^\star - T_w$ (T_s steps)")
    axes[1].set_title("Repaired-segment duration change")
    axes[1].axvline(0, color="black", linestyle=":", linewidth=1)
    axes[1].axvline(local_success.delta_T_repair.median(), color="black", linestyle="--",
                    label=f"median={local_success.delta_T_repair.median():.1f}")
    axes[1].legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print(f"wrote {out_path}")

    frac_neg = (local_success.delta_T_repair < 0).mean()
    frac_zero = (local_success.delta_T_repair == 0).mean()
    frac_pos = (local_success.delta_T_repair > 0).mean()
    print(f"delta_T: {100*frac_neg:.1f}% <0, {100*frac_zero:.1f}% =0, {100*frac_pos:.1f}% >0")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    args = ap.parse_args()

    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    instances, queries = _io.load_run(args.run_dir)

    table = mechanism_summary(instances)
    print(table.to_string(index=False))
    tables_dir = _io.results_dir(root, "tables")
    table.to_csv(os.path.join(tables_dir, "mechanism_analysis.csv"), index=False)
    print(f"\nwrote {tables_dir}/mechanism_analysis.csv")

    figures_dir = _io.results_dir(root, "figures")
    make_figure2(queries, os.path.join(figures_dir, "mechanism_analysis.pdf"))


if __name__ == "__main__":
    main()
