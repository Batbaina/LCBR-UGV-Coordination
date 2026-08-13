/**
 * @file neighbor.hpp
 * @brief State-transition record used by the low-level search.
 *
 * Reused unmodified from CL-CBS (Wen et al., "CL-MAPF: Multi-Agent Path
 * Finding for Car-Like robots with kinematic and spatiotemporal
 * constraints"). Part of the fixed low-level search machinery that this
 * project's contribution (LCBR) deliberately leaves untouched: only the
 * *scope* of the low-level query changes, never its mechanics.
 */
#pragma once

namespace libMultiRobotPlanning {

template <typename State, typename Action, typename Cost>
struct Neighbor {
  Neighbor(const State& state, const Action& action, Cost cost)
      : state(state), action(action), cost(cost) {}

  //! neighboring state
  State state;
  //! action to get to the neighboring state
  Action action;
  //! cost to get to the neighboring state
  Cost cost;
};

}  // namespace libMultiRobotPlanning
