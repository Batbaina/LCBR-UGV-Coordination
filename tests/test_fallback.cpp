/**
 * @file test_fallback.cpp
 * @brief ASPIRATIONAL STUB -- not yet wired into the build (see
 * tests/CMakeLists.txt, where this file is listed but excluded pending
 * the refactor noted below). Documents the white-box tests that should
 * exist around BodyConflictTree's fallback path.
 *
 * Why this isn't real yet: LocalRepair and BodyConflictTree are templated
 * over State/Action/Constraint/... types that currently live INLINE in
 * src/ugv_coordination.cpp rather than in a reusable header. Testing them
 * directly means either (a) extracting those concrete types into
 * include/concrete_types.hpp (a real refactor, explicitly out of scope
 * for the instrumentation-audit pass this test skeleton was requested
 * in), or (b) linking this test against ugv_coordination.cpp's object
 * file and using a friend/test-only accessor. Neither has been done.
 *
 * What SHOULD be tested here once that refactor happens -- concretely
 * motivated by two real bugs the manual audit already found (see
 * docs/REPRODUCIBILITY.md "Known instrumentation gaps"):
 *
 *   1. test_fallback_returns_correct_duration
 *      Construct a BCT node where local repair fails and triggers
 *      replanFullHorizon(). Assert the resulting QueryLogRecord's T_star
 *      field equals the ACTUAL duration of the returned trajectory
 *      (newNode.solution[i].states.back().first.time), not 0. This is
 *      the exact bug found by manual audit: replanFullHorizon() currently
 *      hardcodes Tstar=0 in its logQuery() call
 *      (include/body_conflict_tree.hpp, around line 276) regardless of
 *      what the full-horizon query actually returned.
 *
 *   2. test_every_local_failure_triggers_exactly_one_fallback
 *      Force a local repair failure (e.g. a constraint that blocks the
 *      only feasible local segment) and assert exactly one subsequent
 *      QueryLogRecord row with query_type=="full" and
 *      fallback_triggered==true appears for the same (bct_node_id,
 *      robot_id), with sha_expansions > 0 on both rows (i.e. E_wasted is
 *      never silently zero when a real search happened).
 *
 *   3. test_fallback_uses_true_initial_state
 *      Assert replanFullHorizon() is called with the robot's TRUE x_i^0
 *      (the original start passed into BodyConflictTree::search()), not
 *      the local repair window's x_s -- per paper Sec. 4.3.8.
 */
int main() { return 0; }  // placeholder: intentionally does nothing yet
