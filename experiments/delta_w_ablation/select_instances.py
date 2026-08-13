#!/usr/bin/env python3
"""Same role as main_comparison/select_instances.py, for any
config/experiments/*delta_w*.yaml. Supports --config/--tag for the same
reason (split by map size, or by anything else) -- see
docs/EXPERIMENTS.md "Splitting the benchmark by map size"."""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _common as common  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None,
                     help="path to a config/experiments/*.yaml (default: delta_w_ablation.yaml)")
    ap.add_argument("--tag", default=None,
                     help="suffix for the output file: selected_instances_<tag>.txt")
    args = ap.parse_args()

    root = common.repo_root()
    config_path = args.config or os.path.join(root, "config/experiments/delta_w_ablation.yaml")
    cfg = common.load_config(config_path)
    selected = common.select_instances(cfg, root)
    if not selected:
        sys.exit(f"No instances matched -- check {config_path} and that "
                 f"benchmark/ is populated (see benchmark/README.md)")
    fname = f"selected_instances_{args.tag}.txt" if args.tag else "selected_instances.txt"
    out_path = os.path.join(HERE, fname)
    common.write_selected_instances(selected, out_path)
    print(f"Selected {len(selected)} instances (from {os.path.basename(config_path)}) -> {out_path}")


if __name__ == "__main__":
    main()
