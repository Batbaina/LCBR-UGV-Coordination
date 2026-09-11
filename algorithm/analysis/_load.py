"""
_io.py -- shared loading helpers for analysis/*.py. Every analysis script
reads from a runs/<experiment>/<run_id>/ folder (instance_metrics.csv +
low_level_queries.csv, written by experiments/*/run_*.py) and nothing
else -- no analysis script re-invokes the binary.
"""
import os

import pandas as pd


def load_run(run_dir):
    inst_path = os.path.join(run_dir, "instance_metrics.csv")
    query_path = os.path.join(run_dir, "low_level_queries.csv")
    if not os.path.exists(inst_path):
        raise FileNotFoundError(
            f"{inst_path} not found -- did the corresponding experiments/*/run_*.py finish?")
    instances = pd.read_csv(inst_path)
    queries = pd.read_csv(query_path) if os.path.exists(query_path) else None
    return instances, queries


def merge_runs(run_dirs, dedupe=True):
    """Concatenates instance_metrics.csv / low_level_queries.csv across
    several run_id folders -- e.g. joining main_comparison's CL-CBS
    baseline with delta_w_ablation's LCBR sweep over the same instances,
    so analysis scripts can plot both on one figure without re-running
    the CL-CBS baseline redundantly for every ablation.

    If `dedupe` (default): if the same (instance_id, method,
    delta_w_steps) combination appears in more than one run_dir (e.g. you
    re-ran the same config twice after tweaking something), only the row
    from the LAST run_dir in the given order is kept -- list your
    run_dirs oldest-first for this to mean "most recent wins".
    """
    all_inst, all_query = [], []
    for rd in run_dirs:
        inst, query = load_run(rd)
        inst = inst.copy()
        inst["_source_run_dir"] = rd
        all_inst.append(inst)
        if query is not None:
            query = query.copy()
            query["_source_run_dir"] = rd
            all_query.append(query)
    instances = pd.concat(all_inst, ignore_index=True)
    queries = pd.concat(all_query, ignore_index=True) if all_query else None

    if dedupe:
        key = ["instance_id", "method", "delta_w_steps"]
        n_before = len(instances)
        instances = instances.drop_duplicates(subset=key, keep="last").reset_index(drop=True)
        n_dropped = n_before - len(instances)
        if n_dropped:
            print(f"[merge_runs] dropped {n_dropped} duplicate (instance_id, method, "
                 f"delta_w_steps) row(s) -- kept the most recently listed run_dir "
                 f"for each; pass dedupe=False to keep every row instead.")

    return instances, queries


def discover_runs(experiment, root):
    """Returns every runs/<experiment>/<run_id>/ folder that has a
    complete instance_metrics.csv, sorted oldest-first by run_id
    (run_ids are timestamps 'YYYYMMDDTHHMMSS', so lexicographic order is
    chronological order). Combine with merge_runs(..., dedupe=True) to
    always load 'every batch you've ever run for this experiment, most
    recent wins on overlap' without hand-editing a list of timestamps."""
    base = os.path.join(root, "runs", experiment)
    if not os.path.isdir(base):
        return []
    out = []
    for run_id in sorted(os.listdir(base)):
        run_dir = os.path.join(base, run_id)
        if os.path.isfile(os.path.join(run_dir, "instance_metrics.csv")):
            out.append(run_dir)
    return out


def results_dir(root, kind):
    """kind: 'tables' or 'figures'."""
    d = os.path.join(root, "results", kind)
    os.makedirs(d, exist_ok=True)
    return d
