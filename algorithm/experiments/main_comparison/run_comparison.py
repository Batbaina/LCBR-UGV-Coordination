#!/usr/bin/env python3

"""
Main CL-CBS vs LCBR validation experiment.

For every selected instance, this script executes:

    1. CL-CBS normal
    2. LCBR normal
    3. LCBR paired-probe (optional)

The paired-probe run is used ONLY for the same-conflict
Local-vs-Full analysis (Q3/Q4).

IMPORTANT
---------
The paired-probe execution performs additional counterfactual
full-horizon SHA* calls.

Therefore:

    paired-probe runtime MUST NOT be used for the global
    CL-CBS vs LCBR runtime comparison.

Normal solver-level results:
    instance_metrics.csv
    low_level_queries.csv

Paired same-conflict results:
    paired_probe_instances.csv
    paired_conflicts.csv
"""

import argparse
import os
import shutil
import sys


# ============================================================================
# Import common experiment utilities
# ============================================================================

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)
        ),
        ".."
    )
)

import _common as common  # noqa: E402


HERE = os.path.dirname(
    os.path.abspath(__file__)
)


# ============================================================================
# MAIN
# ============================================================================

def main():

    # ------------------------------------------------------------------------
    # Command-line arguments
    # ------------------------------------------------------------------------

    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--config",
        default=None,
        help=(
            "path to config/experiments/*.yaml "
            "(default: main_comparison.yaml)"
        )
    )

    ap.add_argument(
        "--tag",
        default=None,
        help=(
            "must match the --tag given to "
            "select_instances.py for this batch"
        )
    )

    args = ap.parse_args()


    # ------------------------------------------------------------------------
    # Repository / configuration
    # ------------------------------------------------------------------------

    root = common.repo_root()

    config_path = (
        args.config
        or os.path.join(
            root,
            "config/experiments/main_comparison.yaml"
        )
    )

    cfg = common.load_config(
        config_path
    )


    # ------------------------------------------------------------------------
    # Selected instances
    # ------------------------------------------------------------------------

    fname = (
        f"selected_instances_{args.tag}.txt"
        if args.tag
        else "selected_instances.txt"
    )

    selected_path = os.path.join(
        HERE,
        fname
    )


    if not os.path.exists(
        selected_path
    ):

        sys.exit(
            f"{selected_path} not found -- "
            "run select_instances.py "
            "with the same --config/--tag first"
        )


    instances = (
        common.read_selected_instances(
            selected_path
        )
    )


    if not instances:

        sys.exit(
            f"{selected_path} is empty"
        )


    # ------------------------------------------------------------------------
    # Create run directory
    # ------------------------------------------------------------------------

    run_id, run_dir = (
        common.new_run_dir(
            "main_comparison",
            root
        )
    )


    common.snapshot_config(
        config_path,
        run_dir
    )


    # ------------------------------------------------------------------------
    # Separate scratch directories
    # ------------------------------------------------------------------------
    #
    # IMPORTANT:
    #
    # normal_scratch:
    #
    #     CL-CBS normal
    #     LCBR normal
    #
    # probe_scratch:
    #
    #     LCBR paired-probe only
    #
    # This guarantees that paired-probe rows can never contaminate
    # instance_metrics.csv / low_level_queries.csv.
    # ------------------------------------------------------------------------

    normal_scratch = os.path.join(
        run_dir,
        "_scratch_logs"
    )

    probe_scratch = os.path.join(
        run_dir,
        "_scratch_probe_logs"
    )


    sol_dir = os.path.join(
        run_dir,
        "solutions"
    )


    os.makedirs(
        normal_scratch,
        exist_ok=True
    )

    os.makedirs(
        probe_scratch,
        exist_ok=True
    )

    os.makedirs(
        sol_dir,
        exist_ok=True
    )


    # ------------------------------------------------------------------------
    # Manifest
    # ------------------------------------------------------------------------

    manifest = common.ManifestWriter(
        run_dir
    )


    # ------------------------------------------------------------------------
    # Experiment parameters
    # ------------------------------------------------------------------------

    binary = os.path.join(
        root,
        cfg["binary"]
    )


    vehicle_config = os.path.join(
        root,
        cfg["vehicle_config"]
    )


    timeout = cfg.get(
        "timeout",
        180
    )


    max_exp = cfg.get(
        "max_low_level_expansions"
    )


    methods = cfg.get(
        "methods",
        ["clcbs", "lcbr"]
    )


    delta_values = cfg.get(
        "delta_w",
        [10]
    )


    paired_probe_enabled = cfg.get(
        "paired_probe",
        False
    )


    # ------------------------------------------------------------------------
    # Counters
    # ------------------------------------------------------------------------

    normal_ok = 0
    normal_total = 0

    probe_ok = 0
    probe_total = 0


    print()
    print("=" * 72)
    print("LCBR / CL-CBS VALIDATION")
    print("=" * 72)

    print(
        f"Instances: {len(instances)}"
    )

    print(
        f"Methods: {methods}"
    )

    print(
        f"delta_w: {delta_values}"
    )

    print(
        f"Paired probe: "
        f"{'ENABLED' if paired_probe_enabled else 'DISABLED'}"
    )

    print("=" * 72)
    print()


    # ========================================================================
    # EXPERIMENT LOOP
    # ========================================================================

    for index, inst_path in enumerate(
        instances,
        start=1
    ):

        base = os.path.splitext(
            os.path.basename(
                inst_path
            )
        )[0]


        print()
        print(
            f"[instance "
            f"{index}/{len(instances)}] "
            f"{base}"
        )


        # ====================================================================
        # CL-CBS NORMAL
        # ====================================================================

        if "clcbs" in methods:

            out_yaml = os.path.join(
                sol_dir,
                f"{base}_clcbs.yaml"
            )


            normal_total += 1


            ok = common.run_binary(
                binary= binary,
                instance_path=inst_path,
                out_yaml=out_yaml,
                mode="clcbs",
                instance_id=base,
                scratch_log_dir=normal_scratch,
                vehicle_config=vehicle_config,
                timeout=timeout,
                max_low_level_expansions=max_exp,
                paired_probe=False
            )


            normal_ok += int(ok)


            manifest.add(
                out_yaml,
                inst_path,
                "clcbs"
            )


        # ====================================================================
        # LCBR
        # ====================================================================

        if "lcbr" in methods:

            for dw in delta_values:

                # ------------------------------------------------------------
                # A. NORMAL LCBR
                # ------------------------------------------------------------
                #
                # This execution is used for:
                #
                # Q1 solver success
                # Q2 local repair / fallback
                # Q5 BCT interaction
                # Q6 total computational performance
                # Q7 final solution quality
                #
                # ------------------------------------------------------------

                out_yaml = os.path.join(
                    sol_dir,
                    f"{base}_lcbr_dw{dw}.yaml"
                )


                normal_total += 1


                ok = common.run_binary(
                    binary=binary,
                    instance_path=inst_path,
                    out_yaml=out_yaml,
                    mode="lcbr",
                    instance_id=base,
                    scratch_log_dir=normal_scratch,
                    vehicle_config=vehicle_config,
                    timeout=timeout,
                    delta_w=dw,
                    max_low_level_expansions=max_exp,
                    paired_probe=False
                )


                normal_ok += int(ok)


                manifest.add(
                    out_yaml,
                    inst_path,
                    "lcbr",
                    dw
                )


                # ------------------------------------------------------------
                # B. PAIRED-PROBE LCBR
                # ------------------------------------------------------------
                #
                # Used ONLY for:
                #
                # Q3:
                #   same-conflict Local vs Full expansions/runtime
                #
                # Q4:
                #   same-conflict Local vs Full trajectory quality
                #
                # Never use this execution's total runtime for Q6.
                #
                # ------------------------------------------------------------

                if paired_probe_enabled:

                    probe_yaml = os.path.join(
                        sol_dir,
                        f"{base}_lcbr_probe_dw{dw}.yaml"
                    )


                    probe_total += 1


                    probe_ok_this = (
                        common.run_binary(
                            binary=binary,
                            instance_path=inst_path,
                            out_yaml=probe_yaml,
                            mode="lcbr",
                            instance_id=base,
                            scratch_log_dir=probe_scratch,
                            vehicle_config=vehicle_config,
                            timeout=timeout,
                            delta_w=dw,
                            max_low_level_expansions=max_exp,
                            paired_probe=True
                        )
                    )


                    probe_ok += int(
                        probe_ok_this
                    )


                    manifest.add(
                        probe_yaml,
                        inst_path,
                        "lcbr_probe",
                        dw
                    )


    # ========================================================================
    # CLOSE MANIFEST
    # ========================================================================

    manifest.close()


    # ========================================================================
    # NORMAL RESULTS
    # ========================================================================
    #
    # _common.py converts:
    #
    #   _scratch_logs/instance_log.csv
    #       -> instance_metrics.csv
    #
    #   _scratch_logs/query_log.csv
    #       -> low_level_queries.csv
    #
    # ========================================================================

    common.finalize_run_csvs(
        normal_scratch,
        run_dir
    )


    # ========================================================================
    # PAIRED-PROBE RESULTS
    # ========================================================================

    if paired_probe_enabled:

        probe_instance_source = os.path.join(
            probe_scratch,
            "instance_log.csv"
        )


        probe_query_source = os.path.join(
            probe_scratch,
            "query_log.csv"
        )


        # --------------------------------------------------------------------
        # Instance-level diagnostic file
        # --------------------------------------------------------------------

        if os.path.exists(
            probe_instance_source
        ):

            shutil.copy(
                probe_instance_source,
                os.path.join(
                    run_dir,
                    "paired_probe_instances.csv"
                )
            )


        # --------------------------------------------------------------------
        # Conflict/query-level paired file
        # --------------------------------------------------------------------

        if os.path.exists(
            probe_query_source
        ):

            shutil.copy(
                probe_query_source,
                os.path.join(
                    run_dir,
                    "paired_conflicts.csv"
                )
            )


    # ========================================================================
    # SUMMARY
    # ========================================================================

    print()
    print("=" * 72)
    print("EXPERIMENT COMPLETE")
    print("=" * 72)


    print(
        f"Normal solver runs: "
        f"{normal_ok}/{normal_total} OK"
    )


    if paired_probe_enabled:

        print(
            f"Paired-probe runs: "
            f"{probe_ok}/{probe_total} OK"
        )


    print()
    print("Run directory:")
    print(
        f"  runs/main_comparison/{run_id}"
    )


    print()
    print("Normal solver-level data:")
    print(
        "  instance_metrics.csv"
    )

    print(
        "  low_level_queries.csv"
    )


    if paired_probe_enabled:

        print()
        print(
            "Same-conflict paired data:"
        )

        print(
            "  paired_conflicts.csv"
        )

        print(
            "  paired_probe_instances.csv"
        )


    print()
    print(
        "Reproducibility:"
    )

    print(
        "  config.yaml"
    )

    print(
        "  metadata.json"
    )

    print(
        "  manifest.csv"
    )


    print()
    print(
        "IMPORTANT:"
    )

    print(
        "  Use normal LCBR runtime from "
        "instance_metrics.csv for Q6."
    )

    print(
        "  Never use paired-probe total runtime "
        "as LCBR solver runtime."
    )

    print("=" * 72)


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":

    main()