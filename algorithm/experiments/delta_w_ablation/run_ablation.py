#!/usr/bin/env python3
"""Runs the delta_w sweep (Sec. 5.4) over selected_instances[_<tag>].txt --
LCBR only, at every delta_w in the given config. Supports --config/--tag
for the same reason as main_comparison's scripts (split by map size, or
by anything else). The CL-CBS baseline for these same instances is
expected to come from a main_comparison run over the same instances
(avoids running the full-horizon baseline redundantly once per ablation)
-- see docs/EXPERIMENTS.md for how analysis/delta_w_analysis.py joins the
two.

--parallel N runs N (instance, delta_w) tasks concurrently via a thread
pool (subprocess.run releases the GIL while the child C++ process runs,
so this gives real OS-level parallelism, not just Python-level
overlap). Each worker slot writes to its OWN scratch subdirectory
(_scratch_logs/worker_<i>/) -- ugv_coordination's CsvLogWriter uses a
buffered ofstream with no cross-process write locking, so two processes
appending to the SAME instance_log.csv/query_log.csv concurrently could
interleave/corrupt rows; per-worker isolation avoids that risk entirely,
then all worker CSVs are concatenated into the run's final combined
instance_metrics.csv/low_level_queries.csv at the end.

Usage:
    python3 run_ablation.py                                     # sequential
    python3 run_ablation.py --config ../../config/experiments/delta_w_ablation_50x50.yaml --tag 50
    python3 run_ablation.py --config ../../config/experiments/delta_w_ablation_300x300.yaml --tag 300 --parallel 4
"""
import argparse
import os
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _common as common  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None,
                    help="path to a config/experiments/*.yaml (default: delta_w_ablation.yaml)")
    ap.add_argument("--tag", default=None,
                    help="must match the --tag given to select_instances.py for this batch")
    ap.add_argument("--parallel", type=int, default=1,
                    help="number of concurrent runs (default: 1, sequential -- matches the "
                         "original behavior exactly). A safe starting point is your machine's "
                         "core count minus 1, e.g. --parallel 3 on a 4-core laptop.")
    args = ap.parse_args()

    root = common.repo_root()
    config_path = args.config or os.path.join(root, "config/experiments/delta_w_ablation.yaml")
    cfg = common.load_config(config_path)

    fname = f"selected_instances_{args.tag}.txt" if args.tag else "selected_instances.txt"
    selected_path = os.path.join(HERE, fname)
    if not os.path.exists(selected_path):
        sys.exit(f"{selected_path} not found -- run select_instances.py "
                 f"(with the same --config/--tag) first")
    instances = common.read_selected_instances(selected_path)
    if not instances:
        sys.exit(f"{selected_path} is empty")

    run_id, run_dir = common.new_run_dir("delta_w_ablation", root)
    common.snapshot_config(config_path, run_dir)
    scratch_root = os.path.join(run_dir, "_scratch_logs")
    sol_dir = os.path.join(run_dir, "solutions")
    os.makedirs(sol_dir, exist_ok=True)
    manifest = common.ManifestWriter(run_dir)

    binary = os.path.join(root, cfg["binary"])
    vehicle_config = os.path.join(root, cfg["vehicle_config"])
    timeout = cfg.get("timeout", 180)
    max_exp = cfg.get("max_low_level_expansions")
    n_workers = max(1, args.parallel)

    worker_scratch_dirs = [os.path.join(scratch_root, f"worker_{i}") for i in range(n_workers)]
    for d in worker_scratch_dirs:
        os.makedirs(d, exist_ok=True)

    tasks = []
    for inst_path in instances:
        base = os.path.splitext(os.path.basename(inst_path))[0]
        for dw in cfg.get("delta_w", [3, 5, 7, 12, 22]):
            out_yaml = os.path.join(sol_dir, f"{base}_lcbr_dw{dw}.yaml")
            tasks.append((inst_path, base, dw, out_yaml))

    def run_one(task_idx, task):
        inst_path, base, dw, out_yaml = task
        worker_scratch = worker_scratch_dirs[task_idx % n_workers]
        ok = common.run_binary(binary, inst_path, out_yaml, "lcbr", base,
                               worker_scratch, vehicle_config, timeout, delta_w=dw,
                               max_low_level_expansions=max_exp)
        return task, ok

    n_ok, n_total = 0, len(tasks)
    if n_workers == 1:
        for i, task in enumerate(tasks):
            task, ok = run_one(i, task)
            n_ok += ok
            manifest.add(task[3], task[0], "lcbr", task[2])
    else:
        print(f"[delta_w_ablation] running {n_total} tasks across {n_workers} parallel workers...")
        with ThreadPoolExecutor(max_workers=n_workers) as pool:
            futures = {pool.submit(run_one, i, task): task for i, task in enumerate(tasks)}
            for fut in as_completed(futures):
                task, ok = fut.result()
                n_ok += ok
                manifest.add(task[3], task[0], "lcbr", task[2])

    manifest.close()
    common.finalize_run_csvs_multi(worker_scratch_dirs, run_dir)
    print(f"\n[delta_w_ablation{'/'+args.tag if args.tag else ''}] {n_ok}/{n_total} runs OK.")
    print(f"  runs/delta_w_ablation/{run_id}/instance_metrics.csv")
    print(f"  runs/delta_w_ablation/{run_id}/low_level_queries.csv")


if __name__ == "__main__":
    main()
