#!/usr/bin/env python3
"""Resolves a config/experiments/*.yaml instance selection into
experiments/main_comparison/selected_instances[_<tag>].txt (one path per
line), deterministically. Re-run whenever the config changes;
run_comparison.py always reads this file rather than re-globbing, so
"which instances did this run actually use" is answerable from a
committed text file, not from re-running glob() against a benchmark
folder that could have changed.

Default config is main_comparison.yaml, but any config/experiments/*.yaml
following the same schema works -- this is how you run separate batches
per map size (see config/experiments/main_comparison_100x100.yaml etc.)
without one overwriting another's selection: pass --tag to give each its
own selected_instances_<tag>.txt.

Usage:
    python3 select_instances.py                                   # main_comparison.yaml -> selected_instances.txt
    python3 select_instances.py --config ../../config/experiments/main_comparison_100x100.yaml --tag 100x100
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
                     help="suffix for the output file: selected_instances_<tag>.txt "
                         "(default: selected_instances.txt, no suffix)")
    args = ap.parse_args()

    root = common.repo_root()
    config_path = args.config or os.path.join(root, "config/experiments/main_comparison.yaml")
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
