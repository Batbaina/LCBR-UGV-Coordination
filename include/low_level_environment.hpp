/**
 * @file low_level_environment.hpp
 * @brief Adapter presenting an Environment (+ agent index + constraint set)
 * to HybridAStar's expected template interface.
 *
 * This is exactly the private nested `LowLevelEnvironment` struct of
 * CL-CBS's original `cl_cbs.hpp`, factored out into its own header so it
 * can be shared verbatim by both the full-horizon low-level query
 * (body_conflict_tree.hpp) and the bounded local query (local_repair.hpp).
 * Sharing this one adapter is what guarantees "same everything except the
 * scope of the low-level query" between CL-CBS and LCBR: both call
 * HybridAStar through the identical wrapper.
 */
#pragma once

#include "hybrid_astar.hpp"
#include "neighbor.hpp"

namespace ugv {

template <typename State, typename Action, typename Cost, typename Environment,
          typename Constraints>
struct LowLevelEnvironment {
  LowLevelEnvironment(Environment &env, size_t agentIdx,
                      const Constraints &constraints)
      : m_env(env) {
    m_env.setLowLevelContext(agentIdx, &constraints);
  }

  Cost admissibleHeuristic(const State &s) {
    return m_env.admissibleHeuristic(s);
  }

  bool isSolution(
      const State &s, Cost g,
      std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                         std::hash<State>> &camefrom) {
    return m_env.isSolution(s, g, camefrom);
  }

  void getNeighbors(const State &s, Action act,
                    std::vector<libMultiRobotPlanning::Neighbor<State, Action, Cost>>
                        &neighbors) {
    m_env.getNeighbors(s, act, neighbors);
  }

  State getGoal() { return m_env.getGoal(); }

  int calcIndex(const State &s) { return m_env.calcIndex(s); }

  void onExpandNode(const State &s, Cost fScore, Cost gScore) {
    m_env.onExpandLowLevelNode(s, fScore, gScore);
  }

  void onDiscover(const State & /*s*/, Cost /*fScore*/, Cost /*gScore*/) {}

 private:
  Environment &m_env;
};

}  // namespace ugv
