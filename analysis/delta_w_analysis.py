#!/usr/bin/env python3
"""Table V and Figure 3 (Sec. 5.4): delta_w ablation. Reads the LCBR sweep
from a delta_w_ablation run; optionally merges in a CL-CBS baseline run
(same instances, from main_comparison) to draw it as a reference line.

Usage:
    python3 delta_w_analysis.py --run-dir ../runs/delta_w_ablation/<run_id> \
        [--baseline-run-dir ../runs/main_comparison/<run_id>]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _load as _io  # noqa: E402


def delta_w_table(lcbr):
    n_ll = lcbr.N_success + lcbr.N_fail + lcbr.N_full_fallback
    n_branch = n_ll + lcbr.N_inapplicable
    g = lcbr.assign(
        r_success=(lcbr.N_success / (lcbr.N_success + lcbr.N_fail).replace(0, float("nan"))),
        r_fallback=(lcbr.N_full_fallback / n_branch.replace(0, float("nan"))),
        E_wasted=lcbr.E_local_fail,
        delta_SoC_pct=100.0 * (lcbr.SoC_final - lcbr.SoC_nominal) / lcbr.SoC_nominal,
    ).groupby("delta_w_steps").agg(
        n=("instance_id", "count"),
        runtime_median=("conflict_resolution_runtime_s", "median"),
        E_tot_median=("E_tot", "median"),
        r_success_median=("r_success", "median"),
        r_fallback_median=("r_fallback", "median"),
        E_wasted_median=("E_wasted", "median"),
        delta_SoC_pct_median=("delta_SoC_pct", "median"),
    ).reset_index()
    return g


def make_figure3(table, baseline_runtime, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    ax = axes[0]
    ax.plot(table.delta_w_steps, table.runtime_median, "o-", color="tab:blue", label="LCBR runtime")
    if baseline_runtime is not None:
        ax.axhline(baseline_runtime, color="tab:red", linestyle="--", label="CL-CBS baseline")
    ax.set_xlabel(r"$\delta_w$ (T_s steps)")
    ax.set_ylabel("runtime (s)")
    ax.set_yscale("log")
    ax.set_title("(a) Runtime vs $\\delta_w$")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax2 = axes[1]
    ax2.plot(table.delta_w_steps, table.r_success_median, "o-", color="tab:green", label="$r_{success}$")
    ax2.plot(table.delta_w_steps, table.r_fallback_median, "s-", color="tab:orange", label="$r_{fallback}$")
    ax2.set_xlabel(r"$\delta_w$ (T_s steps)")
    ax2.set_ylabel("rate")
    ax2.set_title("(b) Local-repair behavior vs $\\delta_w$")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print(f"wrote {out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True, help="delta_w_ablation run directory")
    ap.add_argument("--baseline-run-dir", default=None,
                     help="optional main_comparison run directory, for the CL-CBS reference line")
    args = ap.parse_args()

    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    instances, _ = _io.load_run(args.run_dir)
    lcbr = instances[instances.method == "LCBR"]
    if lcbr.delta_w_steps.nunique() <= 1:
        sys.exit("This run has only one delta_w value -- nothing to ablate. "
                 "Check config/experiments/delta_w_ablation.yaml's delta_w list.")

    table = delta_w_table(lcbr)
    print(table.to_string(index=False))
    tables_dir = _io.results_dir(root, "tables")
    table.to_csv(os.path.join(tables_dir, "delta_w_ablation.csv"), index=False)
    print(f"\nwrote {tables_dir}/delta_w_ablation.csv")

    baseline_runtime = None
    if args.baseline_run_dir:
        base_instances, _ = _io.load_run(args.baseline_run_dir)
        base_clcbs = base_instances[(base_instances.method == "CL-CBS") & (base_instances.success == 1)]
        common_ids = set(lcbr.instance_id) & set(base_clcbs.instance_id)
        if common_ids:
            baseline_runtime = base_clcbs[base_clcbs.instance_id.isin(common_ids)].conflict_resolution_runtime_s.median()
        else:
            print("(no overlapping instance_id between the two runs -- skipping baseline reference line)")

    figures_dir = _io.results_dir(root, "figures")
    make_figure3(table, baseline_runtime, os.path.join(figures_dir, "delta_w_ablation.pdf"))


if __name__ == "__main__":
    main()
