#!/usr/bin/env python3
"""
smoke_test.py -- black-box integration test, registered with CTest (see
tests/CMakeLists.txt). Runs the real binary end-to-end on the fast toy
instances and checks observable invariants (exit code, output files,
CSV log rows) rather than internals -- this is what currently backs
tests/ until the white-box stubs (test_local_repair.cpp etc.) are wired
up, see docs/REPRODUCIBILITY.md.

Usage:
    python3 smoke_test.py --binary <path to ugv_coordination> \
        --instances-dir <path to experiments/full_pipeline/instances> \
        --vehicle-config <path to config/vehicle_config.yaml>
"""
import argparse
import csv
import os
import subprocess
import sys
import tempfile


def check(cond, msg, failures):
    status = "ok" if cond else "FAIL"
    print(f"  [{status}] {msg}")
    if not cond:
        failures.append(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", required=True)
    ap.add_argument("--instances-dir", required=True)
    ap.add_argument("--vehicle-config", required=True)
    args = ap.parse_args()

    failures = []

    with tempfile.TemporaryDirectory() as tmp:
        log_dir = os.path.join(tmp, "logs")
        os.makedirs(log_dir, exist_ok=True)

        # --- toy_A_simple, LCBR: must solve, and log a success row ---
        inst = os.path.join(args.instances_dir, "instance1_grouped_scattered.yaml")
        out_yaml = os.path.join(tmp, "out_simple.yaml")
        print("smoke test: instance1_grouped_scattered.yaml, LCBR")
        proc = subprocess.run(
            [args.binary, "-i", inst, "-o", out_yaml, "--mode", "lcbr",
             "--instance-id", "smoke_simple", "--log-dir", log_dir,
             "--vehicle-config", args.vehicle_config, "--timeout", "30"],
            capture_output=True, text=True, timeout=60)
        check(proc.returncode == 0, "exit code 0", failures)
        check(os.path.exists(out_yaml), "output solution YAML written", failures)
        check("Successfully found a solution" in proc.stdout, "reports success", failures)

        inst_log = os.path.join(log_dir, "instance_log.csv")
        check(os.path.exists(inst_log), "instance_log.csv written", failures)
        if os.path.exists(inst_log):
            rows = list(csv.DictReader(open(inst_log)))
            matching = [r for r in rows if r["instance_id"] == "smoke_simple"]
            check(len(matching) == 1, "exactly one instance_log row for this run", failures)
            if matching:
                check(matching[0]["success"] == "1", "logged row has success=1", failures)
                check(matching[0]["method"] == "LCBR", "logged row has method=LCBR", failures)

        # --- instance2_grouped_wall.yaml, CL-CBS vs LCBR: both must solve,
        # and SoC_nominal (Stage 1+2, deterministic) must match exactly
        # between the two methods -- this is the "same Gamma^0" guarantee
        # the whole paired-comparison protocol (Sec. 5.1) depends on.
        inst2 = os.path.join(args.instances_dir, "instance2_grouped_wall.yaml")
        soc_nominal = {}
        for mode in ("clcbs", "lcbr"):
            out_yaml2 = os.path.join(tmp, f"out_wall_{mode}.yaml")
            print(f"smoke test: instance2_grouped_wall.yaml, {mode}")
            proc2 = subprocess.run(
                [args.binary, "-i", inst2, "-o", out_yaml2, "--mode", mode,
                 "--instance-id", f"smoke_wall_{mode}", "--log-dir", log_dir,
                 "--vehicle-config", args.vehicle_config, "--timeout", "60"],
                capture_output=True, text=True, timeout=90)
            check(proc2.returncode == 0, f"{mode}: exit code 0", failures)
            rows = list(csv.DictReader(open(inst_log)))
            matching = [r for r in rows if r["instance_id"] == f"smoke_wall_{mode}"]
            if matching:
                soc_nominal[mode] = matching[0]["SoC_nominal"]

        if "clcbs" in soc_nominal and "lcbr" in soc_nominal:
            check(soc_nominal["clcbs"] == soc_nominal["lcbr"],
                 f"SoC_nominal identical between CL-CBS and LCBR "
                 f"({soc_nominal['clcbs']} == {soc_nominal['lcbr']}) -- same Gamma^0 guarantee",
                 failures)

    if failures:
        print(f"\n{len(failures)} check(s) FAILED:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nAll smoke checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
