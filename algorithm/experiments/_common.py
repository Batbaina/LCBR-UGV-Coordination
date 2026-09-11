"""
_common.py -- shared helpers for experiments/*/*.py. Single place that
knows how to: read a config/experiments/*.yaml, select instances
deterministically, create a runs/<experiment>/<run_id>/ directory with a
frozen config.yaml + metadata.json snapshot, invoke the C++ binary, and
normalize its output filenames to instance_metrics.csv / low_level_queries.csv.

No changes to the C++ binary: it still writes instance_log.csv /
query_log.csv (see include/experiment_logger.hpp) into whatever --log-dir
it is given -- that path is already built, tested, and left untouched.
This module runs the binary with --log-dir pointed at a scratch directory
inside the run folder, then copies the two CSVs into the run's canonical
names. Purely a post-processing step; zero risk to the validated C++ path.
"""
import datetime
import glob
import json
import os
import shutil
import subprocess
import sys

try:
    import yaml
except ImportError:
    sys.exit("This script requires pyyaml: pip install pyyaml")


def repo_root():
    # experiments/<name>/foo.py -> repo root is two levels up from this file
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))


def load_config(config_path):
    with open(config_path) as f:
        return yaml.safe_load(f)


def select_instances(cfg, root):
    """Deterministic instance selection: sorted filename order, first
    max_instances_per_dir entries per directory (or all if null/omitted).
    Deterministic on purpose -- the same config always yields the same
    selection, so selected_instances.txt is reproducible, not a random
    sample that changes between runs."""
    selected = []
    cap = cfg.get("max_instances_per_dir")
    default_pattern = cfg.get("pattern", "*.yaml")
    for entry in cfg["instances"]:
        d = os.path.join(root, entry["dir"])
        pat = entry.get("pattern", default_pattern)
        files = sorted(glob.glob(os.path.join(d, pat)))
        if cap:
            files = files[:cap]
        selected.extend(files)
    return selected


def write_selected_instances(paths, out_path):
    with open(out_path, "w") as f:
        for p in paths:
            f.write(p + "\n")


def read_selected_instances(path):
    with open(path) as f:
        return [line.strip() for line in f if line.strip()]


def new_run_dir(experiment, root):
    run_id = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
    d = os.path.join(root, "runs", experiment, run_id)
    os.makedirs(d, exist_ok=True)
    return run_id, d


def snapshot_config(cfg_path, run_dir):
    """Freezes the exact config that drove this run, plus enough metadata
    to answer 'what produced this CSV' months later (Sec. 5.1 'same
    everything' protocol, made verifiable instead of assumed)."""
    shutil.copy(cfg_path, os.path.join(run_dir, "config.yaml"))
    meta = {
        "timestamp_utc": datetime.datetime.utcnow().isoformat() + "Z",
        "config_source": os.path.abspath(cfg_path),
        "git_commit": _git_hash(os.path.dirname(cfg_path)),
        "hostname": _hostname(),
        "python_version": sys.version.split()[0],
    }
    with open(os.path.join(run_dir, "metadata.json"), "w") as f:
        json.dump(meta, f, indent=2)


def _git_hash(cwd):
    try:
        out = subprocess.run(["git", "rev-parse", "HEAD"], cwd=cwd,
                             capture_output=True, text=True, timeout=5)
        return out.stdout.strip() if out.returncode == 0 else None
    except Exception:
        return None


def _hostname():
    try:
        import socket
        return socket.gethostname()
    except Exception:
        return None


def run_binary(binary, instance_path, out_yaml, mode, instance_id,
               scratch_log_dir, vehicle_config, timeout, delta_w=None,
               max_low_level_expansions=None, cost_matrix_csv=None,
               paired_probe=False):

    cmd = [
        binary,
        "-i", instance_path,
        "-o", out_yaml,
        "--mode", mode,
        "--instance-id", instance_id,
        "--log-dir", scratch_log_dir,
        "--vehicle-config", vehicle_config,
        "--timeout", str(timeout)
    ]

    if delta_w is not None:
        cmd += ["--delta_w_steps", str(delta_w)]

    if max_low_level_expansions is not None:
        cmd += [
            "--max-low-level-expansions",
            str(max_low_level_expansions)
        ]

    if cost_matrix_csv is not None:
        cmd += [
            "--cost-matrix-csv",
            cost_matrix_csv
        ]

    if paired_probe:
        cmd += ["--paired-probe"]

    label = (
        f"{os.path.basename(instance_path)} {mode}"
        + (f" dw={delta_w}" if delta_w is not None else "")
        + (" paired-probe" if paired_probe else "")
    )

    print(
        f"[run] {label} ...",
        flush=True
    )

    proc = subprocess.run(
        cmd,
        capture_output=True,
        text=True
    )

    ok = proc.returncode == 0

    print(
        f"[run]   -> {'OK' if ok else 'FAIL'}",
        flush=True
    )

    if not ok and proc.stderr:
        print(
            "  stderr:",
            proc.stderr.strip()[-400:],
            file=sys.stderr
        )

    return ok

def finalize_run_csvs(scratch_log_dir, run_dir):
    """Copies the binary's own instance_log.csv / query_log.csv (written
    throughout the run into the scratch dir) into this run's canonical
    instance_metrics.csv / low_level_queries.csv names."""
    src_inst = os.path.join(scratch_log_dir, "instance_log.csv")
    src_query = os.path.join(scratch_log_dir, "query_log.csv")
    if os.path.exists(src_inst):
        shutil.copy(src_inst, os.path.join(run_dir, "instance_metrics.csv"))
    if os.path.exists(src_query):
        shutil.copy(src_query, os.path.join(run_dir, "low_level_queries.csv"))


def finalize_run_csvs_multi(scratch_log_dirs, run_dir):
    """Same as finalize_run_csvs, but concatenates several scratch dirs
    (one per parallel worker -- see run_binary's note on why concurrent
    writers must never share a single scratch dir) into one combined
    instance_metrics.csv / low_level_queries.csv, header written once."""
    import csv as _csv
    for name, out_name in [("instance_log.csv", "instance_metrics.csv"),
                           ("query_log.csv", "low_level_queries.csv")]:
        out_path = os.path.join(run_dir, out_name)
        header = None
        rows = []
        for d in scratch_log_dirs:
            src = os.path.join(d, name)
            if not os.path.exists(src):
                continue
            with open(src, newline="") as f:
                reader = _csv.reader(f)
                file_rows = list(reader)
            if not file_rows:
                continue
            if header is None:
                header = file_rows[0]
            rows.extend(file_rows[1:])
        if header is not None:
            with open(out_path, "w", newline="") as f:
                writer = _csv.writer(f)
                writer.writerow(header)
                writer.writerows(rows)


class ManifestWriter:
    """Records (solution_filename, source_instance_path, method, delta_w)
    per run -- needed by analysis/validate_solutions.py to know which map
    YAML each solution YAML in solutions/ came from (the solution itself
    does not carry that back-reference)."""

    def __init__(self, run_dir):
        self.path = os.path.join(run_dir, "manifest.csv")
        self._wrote_header = os.path.exists(self.path)
        self._f = open(self.path, "a")
        if not self._wrote_header:
            self._f.write("solution_filename,instance_path,method,delta_w\n")

    def add(self, out_yaml, instance_path, method, delta_w=None):
        self._f.write(f"{os.path.basename(out_yaml)},{os.path.abspath(instance_path)},"
                      f"{method},{delta_w if delta_w is not None else ''}\n")
        self._f.flush()

    def close(self):
        self._f.close()
