/**
 * @file planresult.hpp
 * @brief Result container for a single-agent trajectory.
 *
 * Reused unmodified from CL-CBS (Wen et al.). states[k].first has a .time
 * field equal to k on the T_s grid; this invariant is relied upon
 * throughout local_repair.hpp when indexing into a trajectory by time step.
 */
#pragma once

#include <vector>

namespace libMultiRobotPlanning {

template <typename State, typename Action, typename Cost>
struct PlanResult {
  //! states and their gScore
  std::vector<std::pair<State, Cost> > states;
  //! actions and their cost
  std::vector<std::pair<Action, Cost> > actions;
  //! actual cost of the result (SHA* cost g(.), distinct from physical
  //! path length L(.) used for task allocation, cf. paper Sec. 4.3.1)
  Cost cost;
  //! lower bound of the cost (for suboptimal solvers)
  Cost fmin;
};

}  // namespace libMultiRobotPlanning
