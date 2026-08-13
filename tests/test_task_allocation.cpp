/**
 * @file test_task_allocation.cpp
 * @brief Real white-box unit tests for the two pure functions in
 * task_allocation.hpp (Hungarian assignment, physical trajectory length).
 * No OMPL/boost::geometry/yaml-cpp dependency -- these two functions
 * don't need them, so this test binary links against nothing but the
 * header itself. Registered with CTest (see tests/CMakeLists.txt).
 *
 * This is the one file in tests/ that is a genuine, compiling,
 * currently-passing unit test rather than an aspirational stub -- see
 * test_local_repair.cpp / test_splice.cpp / test_fallback.cpp for what
 * would be needed to extend white-box testing to LocalRepair/
 * BodyConflictTree (they need the concrete State/Constraint/... types,
 * which currently live inline in src/ugv_coordination.cpp rather than a
 * reusable header -- extracting them is a refactor, out of scope for this
 * pass; see docs/REPRODUCIBILITY.md).
 */
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "task_allocation.hpp"

// Minimal State/Action/Cost stand-ins satisfying exactly what
// trajectoryPhysicalLength needs (a .x, .y pair per state) -- deliberately
// NOT the real State from ugv_coordination.cpp, to keep this test free of
// the OMPL/boost::geometry dependency chain.
struct TestState {
  double x, y;
};

static int g_failures = 0;

#define CHECK(cond, msg)                                                   \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::cerr << "FAIL: " << msg << " (" << #cond << ")\n";              \
      ++g_failures;                                                        \
    } else {                                                               \
      std::cout << "  ok: " << msg << "\n";                                \
    }                                                                      \
  } while (0)

static bool approxEqual(double a, double b, double eps = 1e-9) {
  return std::fabs(a - b) < eps;
}

void test_trajectory_physical_length() {
  std::cout << "test_trajectory_physical_length\n";
  libMultiRobotPlanning::PlanResult<TestState, int, double> traj;
  // A 3-4-5 right triangle path: (0,0) -> (3,0) -> (3,4). Length = 3 + 4 = 7.
  traj.states.push_back({TestState{0, 0}, 0.0});
  traj.states.push_back({TestState{3, 0}, 0.0});
  traj.states.push_back({TestState{3, 4}, 0.0});
  double len = ugv::trajectoryPhysicalLength(traj);
  CHECK(approxEqual(len, 7.0), "3-4-5 triangle path length == 7.0");

  // Single-state trajectory: zero length.
  libMultiRobotPlanning::PlanResult<TestState, int, double> single;
  single.states.push_back({TestState{5, 5}, 0.0});
  CHECK(approxEqual(ugv::trajectoryPhysicalLength(single), 0.0),
       "single-state trajectory has zero length");

  // Empty trajectory: zero length (must not crash on states.size()==0).
  libMultiRobotPlanning::PlanResult<TestState, int, double> empty;
  CHECK(approxEqual(ugv::trajectoryPhysicalLength(empty), 0.0),
       "empty trajectory has zero length, no crash");
}

void test_hungarian_trivial_identity() {
  std::cout << "test_hungarian_trivial_identity\n";
  // Diagonal-optimal 3x3: robot i's cheapest goal is POI i by a wide margin.
  std::vector<std::vector<double>> cost = {
      {1.0, 100.0, 100.0},
      {100.0, 1.0, 100.0},
      {100.0, 100.0, 1.0},
  };
  auto assignment = ugv::solveAssignment(cost);
  CHECK(assignment.size() == 3, "assignment has 3 entries");
  CHECK(assignment[0] == 0, "robot 0 -> POI 0");
  CHECK(assignment[1] == 1, "robot 1 -> POI 1");
  CHECK(assignment[2] == 2, "robot 2 -> POI 2");
}

void test_hungarian_forces_swap() {
  std::cout << "test_hungarian_forces_swap\n";
  // 2x2 where the identity pairing is NOT optimal: swapping is cheaper.
  // Identity: 10 + 10 = 20. Swap: 1 + 1 = 2.
  std::vector<std::vector<double>> cost = {
      {10.0, 1.0},
      {1.0, 10.0},
  };
  auto assignment = ugv::solveAssignment(cost);
  double total = cost[0][assignment[0]] + cost[1][assignment[1]];
  CHECK(approxEqual(total, 2.0), "Hungarian finds the cheaper swapped pairing (total cost 2.0)");
  CHECK(assignment[0] == 1 && assignment[1] == 0, "assignment is the swap (0->1, 1->0)");
}

void test_hungarian_infeasible_pair_avoided() {
  std::cout << "test_hungarian_infeasible_pair_avoided\n";
  // Robot 0 cannot reach POI 0 (kInfeasibleCost sentinel). A feasible
  // perfect matching still exists via the off-diagonal, and Hungarian
  // must find it rather than being dragged into the infeasible cell.
  std::vector<std::vector<double>> cost = {
      {ugv::kInfeasibleCost, 5.0},
      {5.0, ugv::kInfeasibleCost},
  };
  auto assignment = ugv::solveAssignment(cost);
  CHECK(cost[0][assignment[0]] < ugv::kInfeasibleCost / 2,
       "robot 0 is NOT assigned to the infeasible pair");
  CHECK(cost[1][assignment[1]] < ugv::kInfeasibleCost / 2,
       "robot 1 is NOT assigned to the infeasible pair");
}

int main() {
  test_trajectory_physical_length();
  test_hungarian_trivial_identity();
  test_hungarian_forces_swap();
  test_hungarian_infeasible_pair_avoided();

  if (g_failures > 0) {
    std::cerr << "\n" << g_failures << " check(s) FAILED\n";
    return 1;
  }
  std::cout << "\nAll checks passed.\n";
  return 0;
}
