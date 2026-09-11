# Reproducibility notes

## Determinism

The whole pipeline is deterministic: no `rand()`/`<random>` anywhere in
`include/` or `src/`. SHA*, the Hungarian assignment, and the BCT search
are all deterministic given the same inputs. Confirmed empirically by
`tests/smoke_test.py` (`test_timeout_field...` aside, the smoke test
explicitly asserts `SoC_nominal` is bit-identical between a CL-CBS run
and an LCBR run on the same instance -- the "same Gamma^0" guarantee
Sec. 5.1's protocol depends on). There is no `--seed` flag because there
is nothing for it to seed.

## Build environment this was last validated against

- g++ 13, C++14
- Boost 1.83.0 (program_options; header-only geometry/heap)
- OMPL 1.5.2
- yaml-cpp 0.8.0
- CMake 3.x, `-DCMAKE_BUILD_TYPE=Release`

`ctest` (2 tests: `test_task_allocation`, `smoke_test`) passes clean on
this configuration as of the last restructuring pass.

## Robustness fix already applied (not a paper-affecting change)

`include/hybrid_astar.hpp`'s low-level search (unmodified from CL-CBS
otherwise) has no built-in cap on expansions. On a genuinely hard or
practically-infeasible single robot-POI query -- this occurs on Wen et
al.'s own benchmark, not just synthetic instances -- the open set grows
without bound until the process is OOM-killed, since search time (and
therefore state-space size) is otherwise unbounded.

Fix: an optional per-call expansion cap
(`Constants::maxLowLevelExpansions`, CLI: `--max-low-level-expansions`,
default 150000). A query that exceeds the cap is treated as infeasible
(Eq. 6), exactly like a query that fails for any other reason. Default
0 restores the original CL-CBS unbounded behavior. This is a robustness
fix, not an algorithmic change -- it does not alter search ordering,
constraints, or trajectory results for any query that completes within
budget.

## Known instrumentation gaps -- FIXED

Both items below were audited, documented, and left unfixed pending
approval in the previous pass. Approved and fixed:

1. **`T_star` was hardcoded to 0 for full-horizon query log rows.**
   Fixed in `include/body_conflict_tree.hpp`: both `replanFullHorizon()`
   and the rarely-exercised `root_init` fallback path now compute the
   actual returned trajectory duration (`newNode.solution[i].states.back().first.time`
   / `start.solution[i].states.back().first.time`) instead of a literal
   `0.0`. The `root_init` path additionally had `runtimeS`, `gAfter`, and
   `pathAfter` hardcoded to 0 even on success -- fixed at the same time,
   same bug pattern. Verified on a real run: `query_log.csv`'s `T_star`
   column for `query_type=="full"` rows now shows real values (e.g. `30.0,
   9.0, 24.0, ...`), not a column of zeros.

2. **Stage 3 timeout was not distinguished from generic search exhaustion.**
   Fixed: `InstanceLogRecord` gained a `timeout` boolean column (right
   after `success` in `instance_metrics.csv`). `body_conflict_tree.hpp`'s
   `search()` now passes its already-computed `timedOut` flag into
   `fillInstanceLog()` instead of discarding it. Verified on a real forced
   timeout: the logged row shows `success=0,timeout=1`, distinguishable
   from a genuine search-exhaustion failure (`success=0,timeout=0`).

3. **`N_LL`'s ambiguous definition** (the in-code comment only summed
   local queries, omitting LCBR's full-horizon fallback calls) --
   corrected to an explicit per-method formula in
   `experiment_logger.hpp`'s comment. No data was ever wrong (all
   analysis scripts already used the correct inclusive formula); only the
   comment was misleading.

**If you already have `instance_metrics.csv` files from before this fix**:
they lack the `timeout` column entirely (different column count) and
their `T_star` values on `query_type=="full"` rows in the matching
`low_level_queries.csv` are all 0 regardless of the true duration. Old
data is not corrupted for any OTHER field -- `success`, `SoC_final`,
`E_tot`, etc. were never affected by either bug. Re-run only if you
specifically need `timeout` or full-horizon `T_star`.

## `tests/` coverage, honestly stated

- `test_task_allocation.cpp` -- real, compiling, 12/12 checks passing.
  Covers the two pure functions in `task_allocation.hpp` that don't need
  OMPL (Hungarian assignment, physical trajectory length).
- `smoke_test.py` -- real, black-box, registered with CTest. Runs the
  actual binary on the toy instances and checks exit codes, output files,
  logged CSV invariants, and the same-Gamma^0 cross-method guarantee.
- `test_local_repair.cpp`, `test_splice.cpp`, `test_fallback.cpp` --
  **stubs, not wired into the build.** White-box testing of
  `LocalRepair`/`BodyConflictTree` needs their concrete
  State/Action/Constraint/... types extracted from
  `src/ugv_coordination.cpp` into a reusable header first (a real
  refactor, out of scope for this pass). Each file's header comment
  documents exactly which test cases should exist once that's done,
  concretely motivated by the two gaps above.
- `test_logging.cpp` -- **stub**, but *not* blocked by the same issue
  (`experiment_logger.hpp` has no OMPL dependency). Left as a stub for
  lack of time in this pass, not a structural blocker -- the easiest of
  the four to make real next.

## User-provided tools, adopted and fixed

`tools/visualize_v2.py` and `tools/generate_instance.py` were provided
ready-made and are now the project's primary visualization and
instance-generation tools (see README.md's file table). Both needed a
small compatibility patch before they worked in this project, neither
affecting their own logic:

- **`visualize_v2.py`** assumed the original CL-CBS `agents:` schema
  (fixed per-robot goal, known ahead of any solve). This project's
  `robots:`/`pois:` schema has no such field under free one-to-one
  assignment (Sec. 3.2/4.2) -- patched `Scene.__init__` to derive each
  agent's goal marker from the SOLVED trajectory's final state when the
  `agents:` key is absent, which is the only semantically correct reading
  given free assignment (which POI a robot reaches is not knowable from
  the map file alone). Verified: renders correctly on a `robots:`/`pois:`
  solution (`--static` and `--compare` both tested).
- **`generate_instance.py`**'s optional real-planner validator
  (`--build-dir`) looked for a binary literally named `CL-CBS` and
  invoked it with a `-b 1` flag -- neither exists in this project (the
  binary is `ugv_coordination`, no `-b` flag in its CLI). Fixed to use
  the correct binary name and pass `--vehicle-config`/`--timeout`/
  `--log-dir` instead. Verified: generated a `crop_rows` instance with
  `--build-dir ../build --vehicle-config ../config/vehicle_config.yaml`
  and no validation-disabled warning, confirming every sampled point was
  actually checked against the real planner (not silently skipped).

See `runs/README.md`'s `real_instances_visualized/` entry for a
CL-CBS-vs-LCBR comparison rendered end-to-end with `visualize_v2.py`.

## Solution validation (0 violations expected... with one documented exception)

`analysis/validate_solutions.py --run-dir ... --vehicle-config ...`
independently re-checks C3b/C4/C5 from the raw solution YAML, not trusting
the solver's own internal checks. Verified on `main_comparison/20260811T145258/`
(73 solutions): **72/73 solutions have 0 violations.**

The one exception (`map_50by50_obst25_agents20_ex13`, both CL-CBS and
LCBR) is **not a solver bug**: the single flagged state is `t=0` -- the
robot's START configuration, taken directly from Wen et al.'s original
benchmark file and never validated by the solver itself (per
`Environment::isSolution`/`HybridAStar::search`, the seed state is
accepted unconditionally; only states GENERATED during search go through
`stateValid()`, matching the paper's own stated assumption in Sec. 3.2
that "initial and goal configurations are assumed to be obstacle-free").
Both methods flag the identical state because it's the same unmodified
input, not independently-arrived-at errors -- strong evidence this is a
narrow pre-existing edge case in the upstream benchmark data (with this
project's specific vehicle footprint) rather than anything introduced by
CL-CBS or LCBR. Worth knowing before treating "0 violations" as an
unconditional guarantee across the full benchmark; it is not one for the
handful of instances whose original start/goal happens to sit this close
to an obstacle.

Also fixed while producing this: `validate_solutions.py` crashed
(`TypeError: 'NoneType' object is not subscriptable`) on failed/infeasible
runs -- `ugv_coordination` opens the `-o` output file even on failure, so
it exists but is empty, and `yaml.safe_load()` on an empty file returns
`None` rather than raising. Patched to skip these (correctly -- an empty
solution is not "0 violations", it's "nothing to check") instead of
crashing the whole batch.

## Empirical findings worth knowing before a large benchmark run

Found during earlier development/testing passes, worth keeping in mind
when interpreting results rather than re-discovering them:

- **delta_w's sweet spot is instance-dependent, not universal.** On a
  synthetic bottleneck instance (narrow single-gap crossing, several
  robots), delta_w near the theoretical minimum (`delta_T + Ts`) won
  decisively (~100x runtime reduction, <1% quality loss). On a real Wen
  benchmark instance (100x100, 20 agents), delta_w=3 (near-minimum) made
  the BCT explode (596 nodes, did not converge in 60s) while delta_w=10
  converged cleanly and roughly matched the CL-CBS baseline's runtime.
  Always run the delta_w sweep per instance class; never assume one
  value transfers.
- **Free one-to-one assignment (Stage 2) can turn an originally-feasible
  Wen instance into an infeasible one.** Wen's benchmark instances use a
  *fixed* agent-goal pairing, individually guaranteed reachable. This
  pipeline's Stage 2 computes the full `M x Q` cost matrix and may
  require *every* pairing to be at least checked; if a POI is
  unreachable from every robot within the search budget (rare, but
  observed on at least one real `map50by50/agents5/obstacle` instance),
  the whole instance becomes infeasible under free assignment even
  though the original fixed pairing was fine. This is a consequence of
  the reformulation (Sec. 4.2), not a solver bug -- worth reporting as a
  measured statistic ("X% of instances remain feasible under free
  assignment") rather than silently filtering such instances out.
- **`map300by300/agents90-100/obstacle` is the heaviest benchmark cell**
  (up to 100x100 = 10000 Stage-1 SHA* queries per instance). Pilot on
  `map50by50` first; budget accordingly.

## Not done yet (tracked, not silently skipped)

- **Figure 1** (runtime vs number of agents): needs a sweep across
  multiple agent-count directories in one run, aggregated by
  `num_agents` -- `config/experiments/main_comparison.yaml` currently
  lists 4 agent-count cells for `map50by50`, which is enough to produce
  this once a full (non-pilot) run completes; no new code needed, just a
  wider run plus a new `analysis/agents_scaling.py` (not yet written) or
  extending `analysis/main_comparison.py` to plot `E_bar_median` /
  `runtime_median` vs `num_agents` from the already-produced table.
- **Figure 5** (Gamma^0 -> Gamma^star, spatial zoom on one repaired
  segment): `tools/visualize_v2.py` currently only plots the final
  solution, not the nominal (pre-repair) one for overlay comparison.
  `--assignment-csv` (added alongside this pass, see README.md's CLI
  table) gives the resolved robot->POI pairing, but not the nominal
  trajectory geometry itself -- an analogous `--gamma0-yaml` flag would
  need adding to `src/ugv_coordination.cpp` to export Gamma^0's states,
  plus a new `visualize_v2.py` rendering mode that overlays it against
  the final solution.
