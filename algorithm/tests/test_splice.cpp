/**
 * @file test_splice.cpp
 * @brief ASPIRATIONAL STUB -- see test_fallback.cpp for why this isn't
 * wired into the build yet (same LocalRepair extraction blocker).
 *
 * What SHOULD be tested here once LocalRepair's concrete types are
 * extractable:
 *
 *   1. test_splice_preserves_grid_invariant
 *      After repair(), assert states[k].first.time == k +
 *      states[0].first.time for every k in the spliced trajectory (the
 *      invariant local_repair.hpp's own file header documents and
 *      depends on). Construct cases with delta_T > 0, < 0, and == 0.
 *
 *   2. test_splice_junction_states_match_exactly
 *      Assert the local segment's first state equals x_s (the prefix's
 *      last state) and the local segment's last state equals x_g within
 *      floating-point tolerance (the "reconnection_residual" the outcome
 *      struct already reports -- this test would assert it stays small
 *      across a battery of window positions, not just log it).
 *
 *   3. test_junction_cost_correction_matches_manual_recomputation
 *      For a spliced trajectory, manually recompute total cost by walking
 *      every action with Constants::stepCost() at the correct prevAct
 *      context, and assert it equals spliced.cost -- catches drift
 *      between the "corrected junction" fast path and the true cost if
 *      correctJunctionAction()'s logic is ever touched.
 *
 *   4. test_junction_correction_skipped_flag_is_conservative
 *      Force a case where the junction action's original cost does NOT
 *      match the fixed-primitive formula (i.e. it landed in the RS
 *      analytic-expansion tail) and assert junction_correction_skipped
 *      is set rather than silently applying a wrong correction.
 */
int main() { return 0; }  // placeholder: intentionally does nothing yet
