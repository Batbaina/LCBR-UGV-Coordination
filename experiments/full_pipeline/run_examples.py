#!/usr/bin/env python3
"""Sec. 5.6 "End-to-End Coordination Examples". Runs the full pipeline
(Stage 1 -> 2 -> 3, CL-CBS and LCBR) on the 2-3 curated instances under
experiments/full_pipeline/instances/, no statistical evaluation intended
-- this is illustrative, not the main comparison (that is
main_comparison/). Writes one run_id under runs/full_pipeline/ with:

  solutions/*.yaml                       raw solver output
  stage1_stage2/<instance>_cost_matrix.csv    Stage 1: M x Q SHA*-length matrix L_ij (Eq. 6)
  stage1_stage2/<instance>_assignment.csv     Stage 2: resolved robot->POI assignment (Eq. 7)
  instance_metrics.csv, low_level_queries.csv  Stage 3 + everything else (per-query
                                                 rows already carry bct_node_id, so
                                                 filtering low_level_queries.csv by
                                                 instance_id IS the per-instance
                                                 "conflict module" raw data -- no
                                                 separate file needed for it)
  figures/ (if render: true)             rendered via tools/visualize_v2.py:
                                          <instance>_<method>.png (static, one method)
                                          <instance>_compare.png (CL-CBS vs LCBR side by side)
                                          <instance>_<method>.gif (animated)
"""
import glob
import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import _common as common  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    root = common.repo_root()
    config_path = os.path.join(root, "config/experiments/full_pipeline.yaml")
    cfg = common.load_config(config_path)

    instances_dir = os.path.join(root, cfg["instances_dir"])
    pattern = cfg.get("pattern", "*.yaml")
    instances = sorted(glob.glob(os.path.join(instances_dir, pattern)))
    if not instances:
        sys.exit(f"No instances found in {instances_dir}")

    run_id, run_dir = common.new_run_dir("full_pipeline", root)
    common.snapshot_config(config_path, run_dir)
    scratch = os.path.join(run_dir, "_scratch_logs")
    sol_dir = os.path.join(run_dir, "solutions")
    stage12_dir = os.path.join(run_dir, "stage1_stage2")
    fig_dir = os.path.join(run_dir, "figures")
    os.makedirs(scratch, exist_ok=True)
    os.makedirs(sol_dir, exist_ok=True)
    os.makedirs(stage12_dir, exist_ok=True)
    manifest = common.ManifestWriter(run_dir)

    binary = os.path.join(root, cfg["binary"])
    vehicle_config = os.path.join(root, cfg["vehicle_config"])
    timeout = cfg.get("timeout", 300)
    max_exp = cfg.get("max_low_level_expansions")
    render = cfg.get("render", True)
    if render:
        os.makedirs(fig_dir, exist_ok=True)
    visualize_script = os.path.join(root, "tools", "visualize_v2.py")

    n_ok, n_total = 0, 0
    for inst_path in instances:
        base = os.path.splitext(os.path.basename(inst_path))[0]
        # Stage 1 (cost matrix) and Stage 2 (assignment) are identical
        # regardless of --mode (both methods share the exact same
        # deterministic Stage 1+2) -- dumped once per instance, during
        # whichever method runs first below, not once per method.
        cost_matrix_csv = os.path.join(stage12_dir, f"{base}_cost_matrix.csv")
        assignment_csv = os.path.join(stage12_dir, f"{base}_assignment.csv")
        dumped_stage12 = False

        method_solutions = {}  # method -> out_yaml, for --compare after the loop
        for method in cfg.get("methods", ["clcbs", "lcbr"]):
            if method == "clcbs":
                out_yaml = os.path.join(sol_dir, f"{base}_clcbs.yaml")
                n_total += 1
                ok = common.run_binary(binary, inst_path, out_yaml, "clcbs", base,
                                       scratch, vehicle_config, timeout,
                                       max_low_level_expansions=max_exp,
                                       cost_matrix_csv=None if dumped_stage12 else cost_matrix_csv)
                n_ok += ok
                manifest.add(out_yaml, inst_path, "clcbs")
                if ok:
                    method_solutions["CL-CBS"] = out_yaml
                    if render:
                        _render_static(visualize_script, inst_path, out_yaml,
                                       fig_dir, f"{base}_clcbs")
            elif method == "lcbr":
                for dw in cfg.get("delta_w", [10]):
                    out_yaml = os.path.join(sol_dir, f"{base}_lcbr_dw{dw}.yaml")
                    n_total += 1
                    ok = common.run_binary(binary, inst_path, out_yaml, "lcbr", base,
                                           scratch, vehicle_config, timeout, delta_w=dw,
                                           max_low_level_expansions=max_exp,
                                           cost_matrix_csv=None if dumped_stage12 else cost_matrix_csv)
                    n_ok += ok
                    manifest.add(out_yaml, inst_path, "lcbr", dw)
                    if ok:
                        method_solutions[f"LCBR (dw={dw})"] = out_yaml
                        if render:
                            _render_static(visualize_script, inst_path, out_yaml,
                                          fig_dir, f"{base}_lcbr_dw{dw}")
                            _render_gif(visualize_script, inst_path, out_yaml,
                                       fig_dir, f"{base}_lcbr_dw{dw}")
            dumped_stage12 = True  # cost matrix written by whichever ran first above

        # Assignment CSV: a cheap, separate run (--timeout 1s -- we only
        # need Stage 1+2 to finish, Stage 3's outcome is discarded here)
        # specifically to capture --assignment-csv, kept out of the main
        # loop above so a failed/timed-out Stage 3 never prevents the
        # assignment itself from being dumped.
        _dump_assignment_only(binary, inst_path, vehicle_config, assignment_csv,
                              max_exp, scratch)

        if render and len(method_solutions) >= 2:
            compare_args = [f"{p}:{label}" for label, p in method_solutions.items()]
            _render_compare(visualize_script, inst_path, compare_args, fig_dir, f"{base}_compare")

    manifest.close()
    common.finalize_run_csvs(scratch, run_dir)
    print(f"\n[full_pipeline] {n_ok}/{n_total} runs OK.")
    print(f"  runs/full_pipeline/{run_id}/instance_metrics.csv")
    print(f"  runs/full_pipeline/{run_id}/low_level_queries.csv  (filter by instance_id for per-instance conflict/Module C data)")
    print(f"  runs/full_pipeline/{run_id}/stage1_stage2/  (Stage 1 cost matrices + Stage 2 assignments)")
    print(f"  runs/full_pipeline/{run_id}/solutions/")
    if render:
        print(f"  runs/full_pipeline/{run_id}/figures/  (rendered via tools/visualize_v2.py)")


def _dump_assignment_only(binary, inst_path, vehicle_config, assignment_csv, max_exp, scratch):
    cmd = [binary, "-i", inst_path, "-o", os.path.join(scratch, "_assign_dump_out.yaml"),
           "--mode", "clcbs", "--timeout", "1", "--assignment-csv", assignment_csv,
           "--vehicle-config", vehicle_config, "--log-dir", ""]
    if max_exp is not None:
        cmd += ["--max-low-level-expansions", str(max_exp)]
    try:
        subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired:
        print(f"[full_pipeline]   assignment dump timed out for {inst_path}", file=sys.stderr)


def _render_static(script, map_path, solution_path, fig_dir, tag):
    cmd = ["python3", script, "-m", map_path, "-s", solution_path,
           "--static", "-o", os.path.join(fig_dir, f"{tag}.png")]
    _run_render(cmd, tag)


def _render_gif(script, map_path, solution_path, fig_dir, tag):
    cmd = ["python3", script, "-m", map_path, "-s", solution_path,
           "-v", os.path.join(fig_dir, f"{tag}.gif"), "--speed", "3"]
    _run_render(cmd, tag)


def _render_compare(script, map_path, compare_args, fig_dir, tag):
    cmd = ["python3", script, "-m", map_path, "--compare"] + compare_args + \
          ["-o", os.path.join(fig_dir, f"{tag}.png")]
    _run_render(cmd, tag)


def _run_render(cmd, tag):
    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True, timeout=120)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        print(f"[full_pipeline]   render failed for {tag}: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
