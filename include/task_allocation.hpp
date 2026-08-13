/**
 * @file task_allocation.hpp
 * @brief Stage 1 (nominal trajectory generation) and Stage 2 (task
 * allocation) of the paper's pipeline (Sec. 4.1 / 4.2, Algorithm 3 lines
 * 1-12). This module has no equivalent in the original CL-CBS repository:
 * the original assumes a fixed robot<->goal pairing (agent i always drives
 * to goal i). Building the M x Q cost matrix from SHA* trajectory lengths
 * and solving the resulting Linear Assignment Problem is new.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "hybrid_astar.hpp"
#include "neighbor.hpp"
#include "planresult.hpp"

namespace ugv {

// Sentinel cost for an infeasible robot-POI pair (Gamma_ij == empty set,
// paper Eq. 6). Large but finite so the Hungarian algorithm's arithmetic
// stays well-defined; any assignment that uses it is rejected afterwards.
constexpr double kInfeasibleCost = 1e12;

/**
 * @brief Physical trajectory length L(Gamma_i), Eq. (4): sum of Euclidean
 * distances between consecutive sampled poses. This is distinct from
 * PlanResult::cost, which is the SHA* search cost g(.) including the
 * turning/reversing/direction-change penalties (Eq. 9) -- L(.) is used for
 * task allocation, g(.) is used to order the BCT (see body_conflict_tree.hpp).
 */
template <typename State, typename Action, typename Cost>
double trajectoryPhysicalLength(
    const libMultiRobotPlanning::PlanResult<State, Action, Cost> &traj) {
  double length = 0.0;
  for (size_t k = 1; k < traj.states.size(); ++k) {
    const auto &a = traj.states[k - 1].first;
    const auto &b = traj.states[k].first;
    length += std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
  }
  return length;
}

/**
 * @brief Square assignment problem solver (Kuhn-Munkres / Hungarian
 * algorithm, O(n^3)), minimizing total cost. `cost` must be an n x n matrix
 * (rows = robots, columns = POIs, M = Q per the paper's problem definition,
 * Sec. 3.2). Returns assignment[i] = j (robot i assigned to POI j).
 *
 * Standard potential-based implementation (e.g. Kuhn 1955 / e-maxx.ru
 * "Hungarian algorithm" folklore version), 0-indexed here.
 */
inline std::vector<int> solveAssignment(
    const std::vector<std::vector<double>> &cost) {
  const int n = static_cast<int>(cost.size());
  const double INF = std::numeric_limits<double>::infinity();

  // 1-indexed internal arrays (standard trick for this formulation).
  std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
  std::vector<int> p(n + 1, 0), way(n + 1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(n + 1, INF);
    std::vector<bool> used(n + 1, false);
    do {
      used[j0] = true;
      int i0 = p[j0];
      double delta = INF;
      int j1 = -1;
      for (int j = 1; j <= n; ++j) {
        if (!used[j]) {
          double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  std::vector<int> assignment(n, -1);
  for (int j = 1; j <= n; ++j) {
    if (p[j] > 0) {
      assignment[p[j] - 1] = j - 1;
    }
  }
  return assignment;
}

/**
 * @brief Result of Stage 1 + Stage 2: the retained nominal trajectory set
 * Gamma^0 (Eq. matching Algorithm 3, "Gamma^0 <- {Gamma_ij : a*_ij = 1}"),
 * the assignment, and diagnostics for the instance-level log.
 */
template <typename State, typename Action, typename Cost>
struct AllocationResult {
  bool feasible = false;
  std::vector<int> assignment;  // assignment[i] = j
  std::vector<libMultiRobotPlanning::PlanResult<State, Action, Cost>> gamma0;
  std::vector<std::vector<double>> costMatrix;  // L_ij, kInfeasibleCost if infeasible
  double nominalGenRuntimeSeconds = 0.0;
  double lapRuntimeSeconds = 0.0;
  int feasiblePairs = 0;
  int totalPairs = 0;
};

}  // namespace ugv
