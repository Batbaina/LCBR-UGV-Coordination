# LCBR: Local Conflict-Based Trajectory Repair

This document is the algorithm-level companion to the paper ("Stage 1 --
UGV Coordination and Task Allocation"). It maps paper sections to code so
a reader can go from an equation to the exact line that implements it.
For "how do I run an experiment", see `docs/EXPERIMENTS.md`. For "how do I
reproduce a specific number", see `docs/REPRODUCIBILITY.md`.

## Relationship to CL-CBS

LCBR is built directly on top of CL-CBS (Wen et al., 2022,
[github.com/APRIL-ZJU/CL-CBS](https://github.com/APRIL-ZJU/CL-CBS)). The
low-level search engine (`include/hybrid_astar.hpp`) is **unmodified**
except for one addition: an optional per-call expansion cap (see
`docs/REPRODUCIBILITY.md`, "robustness fix"). Everything else --
motion primitives, cost function `g(.)`, heuristic, analytic expansion,
Body Conflict Tree structure, conflict detection, branch-constraint
generation -- is identical between the CL-CBS baseline and LCBR. This is
deliberate and load-bearing: it is what makes the experimental comparison
an ablation of one specific mechanism, not a comparison of two different
codebases.

## Pipeline (Algorithm 3)

Three stages, all in `src/ugv_coordination.cpp::main()`:

| Stage | Paper | Code |
|---|---|---|
| 1. Nominal trajectory generation | Sec. 4.1 | `main()`, the `M x Q` loop calling `HybridAStar::search()` per robot-POI pair |
| 2. Task allocation | Sec. 4.2, Eq. 6-7 | `task_allocation.hpp::solveAssignment()` (Hungarian/Kuhn-Munkres) |
| 3. Conflict resolution | Sec. 4.3 | `body_conflict_tree.hpp::BodyConflictTree::search()` |

Stage 3 is a single class parametrized by `LowLevelStrategy::{FULL_HORIZON,
LCBR}` -- both the CL-CBS baseline and LCBR run through the exact same BCT
loop; only the function called to replan a constrained robot differs
(`replanFullHorizon()` vs `LocalRepair::repair()`).

## LCBR (Algorithm 1)

`include/local_repair.hpp::LocalRepair::repair()`, called once per branch
constraint when `LowLevelStrategy::LCBR` is active.

| Step | Paper | Code |
|---|---|---|
| Inapplicability check | Sec. 4.3.8, `t >= T_i^N` | `repair()`, lines ~87-92 |
| Repair window extraction | Eq. 10 | `repair()`, lines ~94-106 |
| Local replanning query | Eq. 11-12 | `repair()`, `localSearch.search(xs, ...)` |
| Junction cost correction | Sec. 4.3.6 remark | `correctJunctionAction()` |
| Splice + suffix shift | Eq. 12 | `repair()`, lines ~174-198 |
| Post-splice validation | Sec. 4.3.6 | `repair()`, lines ~200-218 (checks the shifted suffix only -- prefix and local segment are unaffected by construction) |
| Fallback | Sec. 4.3.8 | `body_conflict_tree.hpp::search()`, the `else { childSuccess = replanFullHorizon(...) }` branch |

## Three LCBR outcomes (Sec. 4)

Never merged in the code (`local_outcome` column in `low_level_queries.csv`):

- `success` -- local segment found, spliced trajectory passes validation
- `fail` -- local search returned nothing (`no_local_path`) or the spliced
  trajectory violates a constraint after the suffix shift
  (`constraint_violation_after_shift`)
- `inapplicable` -- conflict at or past the robot's current arrival time;
  no SHA* call is made at all

## Documented limitations (not hidden, not silently worked around)

- **`g(.)` is not ordered between LCBR and full-horizon.** SHA* is
  complete but not optimal (Wen et al.); LCBR further restricts the
  search domain by fixing prefix, suffix, and boundary configurations.
  `g(Gamma_LCBR)` and `g(Gamma_full)` are not ordered in general -- this
  is measured empirically (Delta SoC), not assumed.
- **Completeness is argued, not proven.** The full-horizon query is
  always retained as fallback; `analysis/paired_statistics.py`'s
  cross-resolution matrix (Table III) is the empirical proxy for this
  claim.
- **Junction cost correction is approximate** when a splice boundary
  lands inside the Reeds-Shepp analytic-expansion tail (geometry-dependent,
  non-fixed-length transitions). Flagged per-query via
  `junction_correction_skipped`, not silently misapplied.
- **`E_wasted` is a lower bound** on LCBR's search overhead: it captures
  expansions spent on outright-failed local queries, not the additional
  BCT branching a *successful* but lower-quality local repair can induce
  further down the tree. The observable proxy for that second effect is
  the `N_LL(LCBR) / N_LL(CL-CBS)` ratio (Table II).
- **delta_w's sweet spot is instance-dependent**, not universal (see
  `docs/REPRODUCIBILITY.md` for the two contradicting empirical examples
  found during development: a bottleneck-heavy synthetic instance where
  small delta_w wins big, and a real Wen benchmark instance where small
  delta_w loses badly).
