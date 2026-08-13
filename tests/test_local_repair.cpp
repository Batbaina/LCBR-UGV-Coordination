/**
 * @file test_local_repair.cpp
 * @brief ASPIRATIONAL STUB -- see test_fallback.cpp for why this isn't
 * wired into the build yet (same LocalRepair extraction blocker).
 *
 * What SHOULD be tested here once LocalRepair's concrete types are
 * extractable -- the three-way outcome classification from paper Sec.
 * 4.3.8, each currently only exercised indirectly via full pipeline runs:
 *
 *   1. test_inapplicable_when_conflict_at_or_past_goal
 *      conflictTimeStep >= T_i^N must return outcome.inapplicable==true,
 *      outcome.success==false, and must NOT invoke SHA* at all (assert
 *      local_expansions == 0 and local_runtime_s == 0 -- confirms no
 *      wasted search work on an inapplicable branch).
 *
 *   2. test_local_fail_no_local_path
 *      A window with no feasible connecting path under the active
 *      constraints must return failureReason=="no_local_path", with
 *      local_expansions > 0 (search WAS attempted, just failed -- must
 *      not be conflated with the inapplicable case, per Sec. 4's
 *      explicit "do not merge fail and inapplicable" requirement).
 *
 *   3. test_local_fail_post_shift_constraint_violation
 *      A window whose local search succeeds, but whose temporally-shifted
 *      suffix collides with an active constraint, must return
 *      failureReason=="constraint_violation_after_shift" and
 *      outcome.success==false, even though a local path WAS found.
 *
 *   4. test_local_success_matches_window_extraction_equations
 *      For a clean success case, assert t_minus/t_plus/T_w exactly match
 *      Eq. 10 (t^-=max(0,t-dw), t^+=min(T_i^N,t+dw)) and
 *      window_fraction == T_w/T_i_current.
 */
int main() { return 0; }  // placeholder: intentionally does nothing yet
