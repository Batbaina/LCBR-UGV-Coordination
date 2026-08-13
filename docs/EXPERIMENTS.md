# Running experiments

Three experiments, each with its own config, run script(s), and analysis
script(s). All read from `benchmark/` (populate it first -- see
`benchmark/README.md`) or from the curated instances already committed
under `experiments/full_pipeline/instances/` and `experiments/real_instances/`.

## 1. Main comparison (Sec. 5.2)

**What it answers:** what do you gain by replacing full-horizon
replanning with LCBR, on the same instances, same delta_w?

```bash
cd experiments/main_comparison
python3 select_instances.py      # writes selected_instances.txt from
                                  # config/experiments/main_comparison.yaml
python3 run_comparison.py        # writes runs/main_comparison/<run_id>/
```

```bash
cd ../../analysis
python3 main_comparison.py    --run-dir ../runs/main_comparison/<run_id>   # Table I
python3 paired_statistics.py  --run-dir ../runs/main_comparison/<run_id>   # Table II + III
python3 mechanism_analysis.py --run-dir ../runs/main_comparison/<run_id>   # Table IV + Figure 2
python3 conflict_density.py   --run-dir ../runs/main_comparison/<run_id>   # Figure 4
python3 validate_solutions.py --run-dir ../runs/main_comparison/<run_id> \
    --vehicle-config ../config/vehicle_config.yaml                        # "0 violations" check
```

Re-run `select_instances.py` any time you edit
`config/experiments/main_comparison.yaml` -- `run_comparison.py` always
reads the frozen `selected_instances.txt`, never re-globs, so what ran is
always exactly what is committed to that file.

## 2. delta_w ablation (Sec. 5.4)

**What it answers:** how sensitive is LCBR to its one new parameter?

```bash
cd experiments/delta_w_ablation
python3 select_instances.py
python3 run_ablation.py          # LCBR only, sweeps delta_w
```

```bash
cd ../../analysis
python3 delta_w_analysis.py --run-dir ../runs/delta_w_ablation/<run_id> \
    --baseline-run-dir ../runs/main_comparison/<run_id>   # Table V + Figure 3
```

`--baseline-run-dir` is optional but recommended: it draws the CL-CBS
runtime as a horizontal reference line on Figure 3(a), pulled from a
`main_comparison` run over the same instances (the ablation config
deliberately does not re-run the CL-CBS baseline once per delta_w value
-- it is delta_w-independent, so re-running it would be wasted compute).
If the two runs' instance sets do not overlap, the script proceeds
without the reference line and says so.

## 3. Full pipeline examples (Sec. 5.6)

**What it answers:** does the whole pipeline (Stage 1 -> 2 -> 3) work
end-to-end, illustrated, not statistically evaluated?

```bash
cd experiments/full_pipeline
python3 run_examples.py          # both methods, on the 3 curated instances
                                  # (simple / obstacles / conflict_rich)
```

Produces `runs/full_pipeline/<run_id>/`:
```
instance_metrics.csv, low_level_queries.csv   Stage 3 + everything else --
                                               filter low_level_queries.csv
                                               by instance_id for the raw
                                               per-conflict data (Module C)
stage1_stage2/<instance>_cost_matrix.csv      Stage 1: full M x Q SHA*-length
                                               matrix L_ij (Eq. 6)
stage1_stage2/<instance>_assignment.csv       Stage 2: resolved robot->POI
                                               assignment (Eq. 7)
solutions/*.yaml
figures/<instance>_<method>.png               static, one method
figures/<instance>_compare.png                CL-CBS vs LCBR side by side
figures/<instance>_lcbr_dw<K>.gif             animated
```
Rendered via `tools/visualize_v2.py` (`--static`/`--compare`/`-v ...gif`).
`analysis/analysis.ipynb` Sec. 13 loads and displays all of the above --
cost matrix as a heatmap, assignment as a table, Stage 3 as a grouped
summary, plus the rendered images inline.

## Splitting the benchmark by map size (or any other split you want)

`select_instances.py` and `run_comparison.py` both accept `--config` (any
`config/experiments/*.yaml` following the same schema) and `--tag` (keeps
each batch's `selected_instances_<tag>.txt` from overwriting another's).
Each invocation of `run_comparison.py` gets its own timestamped `run_id`
under `runs/main_comparison/` -- they never collide, so you can run one
batch per map size (or per anything else) and combine them afterward.

```bash
cd experiments/main_comparison

python3 select_instances.py --config ../../config/experiments/main_comparison_50x50.yaml   --tag 50x50
python3 run_comparison.py   --config ../../config/experiments/main_comparison_50x50.yaml    --tag 50x50

python3 select_instances.py --config ../../config/experiments/main_comparison_100x100.yaml --tag 100x100
python3 run_comparison.py   --config ../../config/experiments/main_comparison_100x100.yaml  --tag 100x100

python3 select_instances.py --config ../../config/experiments/main_comparison_300x300.yaml --tag 300x300
python3 run_comparison.py   --config ../../config/experiments/main_comparison_300x300.yaml  --tag 300x300
```

Note the increasing `timeout` across the three shipped configs (180s /
300s / 600s) and shrinking `max_instances_per_dir` (10 / 5 / 3) -- larger
maps cost more per instance, pilot before widening either number (see
`docs/REPRODUCIBILITY.md`).

Then combine them in `analysis/analysis.ipynb` (or any script) with
`_load.merge_runs()`:

```python
import _load
instances, queries = _load.merge_runs([
    "../runs/main_comparison/<run_id_50x50>",
    "../runs/main_comparison/<run_id_100x100>",
    "../runs/main_comparison/<run_id_300x300>",
])
```

**Watch out:** `num_agents` alone does NOT distinguish map size -- a
50x50 instance and a 100x100 instance can both have `num_agents == 10`.
Map size is only encoded in `instance_id` (e.g. `map_50by50_obst25_...`
vs `map_100by100_obst50_...`). Derive an explicit column before grouping
by map size:

```python
instances["map_size"] = instances.instance_id.str.extract(r"map_(\d+by\d+)_")
instances.groupby(["map_size", "num_agents"]).size()
```

## Everything at once

```bash
cd analysis
python3 generate_figures.py \
    --main-comparison-run ../runs/main_comparison/<run_id> \
    --delta-w-ablation-run ../runs/delta_w_ablation/<run_id> \
    --vehicle-config ../config/vehicle_config.yaml
```

## Mini-ablation: does the M^2 SHA* cost matrix pay off? (Sec. 5.6)

```bash
cd build
./ugv_coordination -i ../experiments/real_instances/instance3_grouped_croprows.yaml -o /dev/null \
    --mode lcbr --cost-matrix-csv /tmp/cm.csv --log-dir /tmp/scratch \
    --vehicle-config ../config/vehicle_config.yaml

cd ../analysis
python3 euclidean_ablation.py \
    --instance ../experiments/real_instances/instance3_grouped_croprows.yaml \
    --cost-matrix /tmp/cm.csv
```

Compares the SHA*-optimal assignment (what Stage 2 actually produces)
against the Euclidean-optimal assignment, **both priced on the same real
`L_ij` matrix** -- the fair comparison, since a straight-line distance is
otherwise just a lower bound that ignores obstacles and kinematics. On
`instance3_grouped_croprows.yaml` this shows the Euclidean-preferred
pairing costs **+10% more real travel** than Stage 2's actual assignment.
On instances with less geometric asymmetry between robot-goal pairings
the gap can be close to 0% -- run it per instance, don't assume a fixed
number transfers.

## Mapping to paper deliverables

| Paper item | Script | Output |
|---|---|---|
| Table I (main comparison) | `analysis/main_comparison.py` | `results/tables/main_comparison.csv` |
| Table II (paired decomposition) | `analysis/paired_statistics.py` | `results/tables/paired_comparison.csv` |
| Table III (cross-resolution matrix) | `analysis/paired_statistics.py` | `results/tables/resolution_matrix.csv` |
| Table IV (LCBR mechanism) | `analysis/mechanism_analysis.py` | `results/tables/mechanism_analysis.csv` |
| Table V (delta_w ablation) | `analysis/delta_w_analysis.py` | `results/tables/delta_w_ablation.csv` |
| Table VI (end-to-end examples) | -- | `runs/full_pipeline/<run_id>/instance_metrics.csv` directly |
| Mini-ablation (assignment justification, Sec. 5.6) | `analysis/euclidean_ablation.py` | printed to stdout, no CSV (single-instance, run per instance of interest) |
| Figure 1 (runtime vs #agents) | not yet generated -- needs a multi-agent-count sweep, see docs/REPRODUCIBILITY.md "Not done yet" |
| Figure 2 (r_w / DeltaT histograms) | `analysis/mechanism_analysis.py` | `results/figures/mechanism_analysis.pdf` |
| Figure 3 (delta_w ablation) | `analysis/delta_w_analysis.py` | `results/figures/delta_w_ablation.pdf` |
| Figure 4 (speedup vs conflict density) | `analysis/conflict_density.py` | `results/figures/conflict_density.pdf` |
| Figure 5 (Gamma^0 -> Gamma^star, zoomed) | not yet generated -- `tools/visualize_v2.py` plots only the final solved trajectory, not the nominal (pre-repair) one for overlay comparison, see docs/REPRODUCIBILITY.md "Not done yet" |

## Conflict-density indicator, precisely

`analysis/conflict_density.py` uses `bct_nodes_expanded - 1` on CL-CBS's
own successful rows as `N_conflicts^CL-CBS`. This is a **baseline-derived
difficulty indicator**, not an intrinsic property of the instance -- the
count depends on CL-CBS's own conflict-selection policy and BCT expansion
order. State this explicitly wherever the figure is used.
