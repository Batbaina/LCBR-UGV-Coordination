#!/usr/bin/env python3
"""Figure 4 (Sec. 5.5): speedup / expansion-ratio vs conflict density.
N_conflicts^CL-CBS is a baseline-derived difficulty indicator
(bct_nodes_expanded - 1 on CL-CBS's own successful rows), not an
intrinsic property of the instance -- see docs/EXPERIMENTS.md.

Usage:
    python3 conflict_density.py --run-dir ../runs/main_comparison/<run_id> [--delta-w 10]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _load as _io  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--delta-w", type=int, default=None)
    args = ap.parse_args()

    root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    instances, _ = _io.load_run(args.run_dir)

    lcbr_all = instances[instances.method == "LCBR"]
    dws = sorted(lcbr_all.delta_w_steps.unique())
    delta_w = args.delta_w
    if len(dws) > 1 and delta_w is None:
        sys.exit(f"Multiple delta_w values present {dws} -- pass --delta-w <value>")
    delta_w = delta_w if delta_w is not None else dws[0]

    clcbs = instances[(instances.method == "CL-CBS") & (instances.success == 1)].set_index("instance_id")
    lcbr = instances[(instances.method == "LCBR") & (instances.delta_w_steps == delta_w) &
                     (instances.success == 1)].set_index("instance_id")
    common = clcbs.index.intersection(lcbr.index)
    if len(common) == 0:
        sys.exit("No paired successful instances between CL-CBS and LCBR in this run.")

    c, l = clcbs.loc[common], lcbr.loc[common]
    n_conflicts = c.bct_nodes_expanded - 1
    speedup = c.conflict_resolution_runtime_s / l.conflict_resolution_runtime_s
    e_ratio = c.E_tot / l.E_tot.replace(0, 1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].scatter(n_conflicts, speedup, alpha=0.7, color="tab:blue")
    axes[0].axhline(1.0, color="black", linestyle=":", linewidth=1)
    axes[0].set_xlabel(r"$N_{conflicts}^{CL-CBS}$ (bct_nodes - 1)")
    axes[0].set_ylabel(r"Speedup $S = t_{CL-CBS} / t_{LCBR}$")
    axes[0].set_title("Runtime speedup vs conflict density")
    axes[0].grid(alpha=0.3)

    axes[1].scatter(n_conflicts, e_ratio, alpha=0.7, color="tab:green")
    axes[1].axhline(1.0, color="black", linestyle=":", linewidth=1)
    axes[1].set_xlabel(r"$N_{conflicts}^{CL-CBS}$ (bct_nodes - 1)")
    axes[1].set_ylabel(r"$E_{CL-CBS} / E_{LCBR}$")
    axes[1].set_title("Expansion-count ratio vs conflict density")
    axes[1].grid(alpha=0.3)

    fig.suptitle(f"delta_w={delta_w}, n={len(common)} paired instances")
    fig.tight_layout()

    figures_dir = _io.results_dir(root, "figures")
    out_path = os.path.join(figures_dir, "conflict_density.pdf")
    fig.savefig(out_path)
    plt.close(fig)
    print(f"wrote {out_path}")

    table = pd.DataFrame({
        "instance_id": common, "n_conflicts_clcbs": n_conflicts.values,
        "speedup": speedup.values, "E_ratio": e_ratio.values,
    })
    tables_dir = _io.results_dir(root, "tables")
    table.to_csv(os.path.join(tables_dir, "conflict_density.csv"), index=False)
    print(f"wrote {tables_dir}/conflict_density.csv")


if __name__ == "__main__":
    main()
