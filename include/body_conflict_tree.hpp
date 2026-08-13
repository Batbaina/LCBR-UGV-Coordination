/**
 * @file body_conflict_tree.hpp
 * @brief Body Conflict Tree (BCT) search -- Algorithm 2 of the paper
 * ("CL-CBS with Local Trajectory Repair (LCBR)"), which is deliberately
 * the SAME class for both the CL-CBS baseline and the LCBR contribution.
 *
 * This mirrors CL-CBS's original `cl_cbs.hpp` (Wen et al.) almost exactly
 * -- best-first BCT expansion by Cost(N) = sum_i g(Gamma_i^N), conflict
 * detection, two-child branching -- with one deliberate structural choice:
 * the scope of the low-level replanning call is a runtime strategy
 * (`LowLevelStrategy::FULL_HORIZON` or `LowLevelStrategy::LCBR`) rather
 * than being hardwired. This is what lets the experimental comparison
 * satisfy "same everything except the low-level query" (see the paper's
 * Experimental Setup, Sec. 5.1): both strategies run through this exact
 * same BCT loop, the exact same conflict detector, and the exact same
 * SHA* engine (hybrid_astar.hpp) -- only the FUNCTION called to replan a
 * constrained robot differs.
 *
 * Every low-level call this class makes is logged as one QueryLogRecord
 * row; on termination (success, failure, or timeout) it fills in the
 * conflict-resolution-stage fields of one InstanceLogRecord (the
 * nominal-generation and assignment fields are filled by the caller, see
 * src/ugv_coordination.cpp, Stage 1 / Stage 2).
 */
#pragma once

#include <algorithm>
#include <boost/heap/d_ary_heap.hpp>
#include <cassert>
#include <chrono>
#include <map>
#include <string>
#include <vector>

#include "experiment_logger.hpp"
#include "hybrid_astar.hpp"
#include "local_repair.hpp"
#include "low_level_environment.hpp"
#include "task_allocation.hpp"
#include "timer.hpp"

// Fallback default if the caller doesn't pass an explicit timeout to the
// BodyConflictTree constructor. Prefer --timeout on the command line
// (src/ugv_coordination.cpp) over editing this macro.
#define UGV_DEFAULT_MAX_RUNTIME 120

namespace ugv {

using libMultiRobotPlanning::HybridAStar;
using libMultiRobotPlanning::PlanResult;

enum class LowLevelStrategy { FULL_HORIZON, LCBR };

inline std::string toString(LowLevelStrategy s) {
  return s == LowLevelStrategy::FULL_HORIZON ? "CL-CBS" : "LCBR";
}

template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraint, typename Constraints, typename Environment>
class BodyConflictTree {
 public:
  BodyConflictTree(Environment &environment, LowLevelStrategy strategy,
                   int deltaWSteps, int deltaTSteps, std::string instanceId,
                   CsvLogWriter *queryLogger = nullptr,
                   int maxRuntimeSeconds = UGV_DEFAULT_MAX_RUNTIME)
      : m_env(environment),
        m_strategy(strategy),
        m_deltaW(deltaWSteps),
        m_deltaT(deltaTSteps),
        m_instanceId(std::move(instanceId)),
        m_methodLabel(toString(strategy)),
        m_queryLogger(queryLogger),
        m_maxRuntimeSeconds(maxRuntimeSeconds) {}

  /// Algorithm 2 / Algorithm 3 (lines 13-16). `gamma0` is the nominal
  /// trajectory set from Stage 1+2 (task_allocation.hpp); if a given
  /// agent's entry has fewer than 2 states it is treated as "not seeded"
  /// and solved from scratch via a full-horizon query (defensive
  /// fallback, should not trigger when Stage 1+2 always run first).
  bool search(const std::vector<State> &initialStates,
             const std::vector<PlanResult<State, Action, Cost>> &gamma0,
             std::vector<PlanResult<State, Action, Cost>> &solution,
             InstanceLogRecord &instanceLog) {
    HighLevelNode start;
    const size_t M = initialStates.size();
    start.solution.resize(M);
    start.constraints.resize(M);
    start.cost = 0;
    start.id = 0;

    for (size_t i = 0; i < M; ++i) {
      if (i < gamma0.size() && gamma0[i].states.size() > 1) {
        start.solution[i] = gamma0[i];
      } else {
        // Defensive fallback: Stage 1+2 did not seed this agent. Solve it
        // from scratch. Not counted in the conflict-resolution-stage
        // instance-log fields (those measure BCT branch effort only).
        LowLevelEnvironment<State, Action, Cost, Environment, Constraints>
            llenv(m_env, i, start.constraints[i]);
        HybridAStar<State, Action, Cost,
                   LowLevelEnvironment<State, Action, Cost, Environment,
                                       Constraints>>
            lowLevel(llenv, Constants::maxLowLevelExpansions);
        m_env.resetLowLevelExpansionCounter();
        Timer rootInitTimer;
        bool ok = lowLevel.search(initialStates[i], start.solution[i]);
        rootInitTimer.stop();
        // [FIX] Tstar/runtimeS/gAfter/pathAfter were previously hardcoded
        // to 0 here even on success, the same bug pattern fixed in
        // replanFullHorizon() below -- this rarely-exercised branch had
        // it too.
        double rootTstar = ok ? static_cast<double>(
                                    start.solution[i].states.back().first.time)
                              : 0.0;
        double rootPathAfter = ok ? trajectoryPhysicalLength(start.solution[i]) : 0.0;
        logQuery(/*nodeId=*/0, /*robotId=*/i, /*queryType=*/"root_init",
                /*outcome=*/"n/a", /*failureReason=*/"", /*tConflict=*/-1,
                /*tMinus=*/-1, /*tPlus=*/-1, /*Tw=*/0.0, /*Ticur=*/0.0,
                /*windowFraction=*/0.0, /*Tstar=*/rootTstar, /*deltaT=*/0.0,
                /*expansions=*/m_env.lowLevelExpandedThisCall(),
                /*runtimeS=*/rootInitTimer.elapsedSeconds(), /*fallback=*/false,
                /*gBefore=*/0.0,
                /*gAfter=*/ok ? start.solution[i].cost : 0.0,
                /*pathBefore=*/0.0, /*pathAfter=*/rootPathAfter,
                /*reconResidual=*/0.0, /*junctionSkipped=*/false,
                /*clipL=*/false, /*clipR=*/false);
        if (!ok) return false;
      }
      start.cost += start.solution[i].cost;
    }

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>
        open;
    auto handle = open.push(start);
    (*handle).handle = handle;

    long long E_tot = 0, E_local_success = 0, E_local_fail = 0, E_full = 0;
    long long N_success = 0, N_fail = 0, N_inapplicable = 0;
    long long N_full_baseline = 0, N_full_fallback = 0;
    int bctNodesExpanded = 0;

    Timer conflictTimer;
    std::chrono::high_resolution_clock::time_point wallClockStart =
        std::chrono::high_resolution_clock::now();
    int id = 1;
    bool timedOut = false;

    while (!open.empty()) {
      auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - wallClockStart);
      if (elapsed.count() > m_maxRuntimeSeconds) {
        timedOut = true;
        break;
      }

      HighLevelNode P = open.top();
      m_env.onExpandHighLevelNode(P.cost);
      open.pop();
      bctNodesExpanded++;

      Conflict conflict;
      if (!m_env.getFirstConflict(P.solution, conflict)) {
        conflictTimer.stop();
        solution = P.solution;
        fillInstanceLog(instanceLog, true, conflictTimer.elapsedSeconds(),
                        bctNodesExpanded, N_success, N_fail, N_inapplicable,
                        N_full_baseline, N_full_fallback, E_tot,
                        E_local_success, E_local_fail, E_full, solution);
        return true;
      }

      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);

      for (const auto &c : constraints) {
        size_t i = c.first;
        HighLevelNode newNode = P;
        newNode.id = id;
        assert(!newNode.constraints[i].overlap(c.second));
        newNode.constraints[i].add(c.second);
        newNode.cost -= newNode.solution[i].cost;

        int conflictTimeStep = c.second.constraints.begin()->time;
        bool childSuccess = false;

        if (m_strategy == LowLevelStrategy::FULL_HORIZON) {
          childSuccess = replanFullHorizon(
              i, newNode, initialStates[i], id, /*fallback=*/false, P.solution[i],
              conflictTimeStep, E_tot, E_full, N_full_baseline);
        } else {
          LocalRepair<State, Action, Cost, Environment, Constraint, Constraints>
              repairer(m_env, m_deltaW, m_deltaT);
          auto outcome =
              repairer.repair(P.solution[i], conflictTimeStep, i, newNode.constraints[i]);

          logQuery(id, i, "local",
                  outcome.inapplicable ? "inapplicable"
                                        : (outcome.success ? "success" : "fail"),
                  outcome.failureReason, conflictTimeStep, outcome.t_minus,
                  outcome.t_plus, outcome.T_w, outcome.T_i_current,
                  outcome.T_i_current > 0 ? outcome.T_w / outcome.T_i_current : 0.0,
                  outcome.T_star, outcome.delta_T, outcome.local_expansions,
                  outcome.local_runtime_s, false, outcome.g_before,
                  outcome.success ? outcome.g_after : outcome.g_local_attempted,
                  outcome.path_length_before, outcome.path_length_after,
                  outcome.reconnection_residual, outcome.junction_correction_skipped,
                  outcome.clipped_left, outcome.clipped_right);

          E_tot += outcome.local_expansions;
          if (outcome.inapplicable) {
            N_inapplicable++;
          } else if (outcome.success) {
            N_success++;
            E_local_success += outcome.local_expansions;
          } else {
            N_fail++;
            E_local_fail += outcome.local_expansions;  // == E_wasted contribution
          }

          if (outcome.success) {
            newNode.solution[i] = outcome.trajectory;
            childSuccess = true;
          } else {
            childSuccess = replanFullHorizon(
                i, newNode, initialStates[i], id, /*fallback=*/true, P.solution[i],
                conflictTimeStep, E_tot, E_full, N_full_fallback);
          }
        }

        newNode.cost += newNode.solution[i].cost;
        if (childSuccess) {
          auto h = open.push(newNode);
          (*h).handle = h;
        }
        ++id;
      }
    }

    conflictTimer.stop();
    fillInstanceLog(instanceLog, false, conflictTimer.elapsedSeconds(),
                    bctNodesExpanded, N_success, N_fail, N_inapplicable,
                    N_full_baseline, N_full_fallback, E_tot, E_local_success,
                    E_local_fail, E_full, {}, timedOut);
    return false;
  }

 private:
  struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost>> solution;
    std::vector<Constraints> constraints;
    Cost cost;
    int id;
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;
    bool operator<(const HighLevelNode &n) const { return cost > n.cost; }
  };

  /// Full-horizon low-level query: replans robot `i` from its TRUE initial
  /// configuration under the complete accumulated constraint set of the
  /// child node. Used directly by the CL-CBS baseline, and as LCBR's
  /// fallback (paper Sec. 4.3.8) when local repair fails or is
  /// inapplicable.
  bool replanFullHorizon(size_t i, HighLevelNode &newNode,
                        const State &agentInitialState, int nodeId,
                        bool fallback, const PlanResult<State, Action, Cost> &before,
                        int conflictTimeStep, long long &E_tot, long long &E_full,
                        long long &counterToBump) {
    LowLevelEnvironment<State, Action, Cost, Environment, Constraints> llenv(
        m_env, i, newNode.constraints[i]);
    HybridAStar<State, Action, Cost,
               LowLevelEnvironment<State, Action, Cost, Environment, Constraints>>
        lowLevel(llenv, Constants::maxLowLevelExpansions);
    m_env.resetLowLevelExpansionCounter();
    Timer t;
    bool success = lowLevel.search(agentInitialState, newNode.solution[i]);
    t.stop();
    int expansions = m_env.lowLevelExpandedThisCall();
    E_tot += expansions;
    E_full += expansions;
    counterToBump++;

    double pathBefore = trajectoryPhysicalLength(before);
    double pathAfter = success ? trajectoryPhysicalLength(newNode.solution[i]) : 0.0;
    // [FIX] Previously hardcoded to 0 regardless of the actual returned
    // trajectory duration -- every "full" query_type row silently lost
    // this field. Now reflects the true duration (in T_s steps) of the
    // trajectory this call returned, matching what local queries already
    // report via outcome.T_star.
    double tstar = success
                      ? static_cast<double>(newNode.solution[i].states.back().first.time)
                      : 0.0;
    logQuery(nodeId, i, "full", "n/a", "", conflictTimeStep, -1, -1, 0,
            before.states.empty() ? 0 : before.states.back().first.time, 0, tstar,
            0, expansions, t.elapsedSeconds(), fallback, before.cost,
            success ? newNode.solution[i].cost : 0.0, pathBefore, pathAfter, 0.0,
            false, false, false);
    return success;
  }

  void logQuery(int nodeId, size_t robotId, const std::string &queryType,
               const std::string &outcome, const std::string &failureReason,
               int tConflict, int tMinus, int tPlus, double Tw, double Ticur,
               double windowFraction, double Tstar, double deltaT,
               int expansions, double runtimeS, bool fallback, double gBefore,
               double gAfter, double pathBefore, double pathAfter,
               double reconResidual, bool junctionSkipped, bool clipL,
               bool clipR) {
    if (!m_queryLogger) return;
    QueryLogRecord r;
    r.instance_id = m_instanceId;
    r.method = m_methodLabel;
    r.bct_node_id = nodeId;
    r.robot_id = robotId;
    r.query_type = queryType;
    r.local_outcome = outcome;
    r.failure_reason = failureReason;
    r.t_conflict = tConflict;
    r.t_minus = tMinus;
    r.t_plus = tPlus;
    r.T_w = Tw;
    r.T_i_current = Ticur;
    r.window_fraction = windowFraction;
    r.T_star = Tstar;
    r.delta_T_repair = deltaT;
    r.sha_expansions = expansions;
    r.sha_runtime_s = runtimeS;
    r.fallback_triggered = fallback;
    r.g_before = gBefore;
    r.g_after = gAfter;
    r.path_length_before = pathBefore;
    r.path_length_after = pathAfter;
    r.reconnection_residual = reconResidual;
    r.junction_correction_skipped = junctionSkipped;
    r.clipped_left = clipL;
    r.clipped_right = clipR;
    m_queryLogger->writeRow(r.toCsvRow());
  }

  void fillInstanceLog(InstanceLogRecord &log, bool success,
                       double conflictRuntimeS, int bctNodes, long long Ns,
                       long long Nf, long long Ninapp, long long Nfb,
                       long long Nff, long long Etot, long long Els,
                       long long Elf, long long Efull,
                       const std::vector<PlanResult<State, Action, Cost>> &finalSolution,
                       bool timedOut = false) {
    log.method = m_methodLabel;
    log.delta_w_steps = m_deltaW;
    log.delta_T_steps = m_deltaT;
    log.success = success;
    log.timeout = timedOut;  // [FIX] previously computed and discarded
    log.conflict_resolution_runtime_s = conflictRuntimeS;
    log.bct_nodes_expanded = bctNodes;
    log.N_success = Ns;
    log.N_fail = Nf;
    log.N_inapplicable = Ninapp;
    log.N_full_baseline = Nfb;
    log.N_full_fallback = Nff;
    log.E_tot = Etot;
    log.E_local_success = Els;
    log.E_local_fail = Elf;
    log.E_full = Efull;

    if (success) {
      double soc = 0.0, lmax = 0.0, tmax = 0.0;
      long long nSwitch = 0;
      double dReverse = 0.0;
      for (const auto &traj : finalSolution) {
        double len = trajectoryPhysicalLength(traj);
        soc += len;
        lmax = std::max(lmax, len);
        if (!traj.states.empty())
          tmax = std::max(tmax, static_cast<double>(traj.states.back().first.time));

        // Direction switches and reverse-travel distance (Sec. 5.2
        // secondary metrics). Action encoding: 0-2 forward, 3-5 backward,
        // 6 wait (excluded from the switch test, matching the exact
        // condition Environment::getNeighbors uses for penaltyCOD).
        for (size_t k = 0; k < traj.actions.size(); ++k) {
          int act = traj.actions[k].first;
          bool isBackward = (act >= 3 && act != 6);
          if (isBackward && k + 1 < traj.states.size()) {
            const auto &a = traj.states[k].first;
            const auto &b = traj.states[k + 1].first;
            dReverse += std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
          }
          if (k > 0) {
            int prevAct = traj.actions[k - 1].first;
            bool prevBackward = (prevAct >= 3 && prevAct != 6);
            if (act != 6 && prevAct != 6 && isBackward != prevBackward) {
              nSwitch++;
            }
          }
        }
      }
      log.SoC_final = soc;
      log.L_max_final = lmax;
      log.mission_completion_time = tmax;
      log.N_switch = nSwitch;
      log.D_reverse = dReverse;
    }
  }

  Environment &m_env;
  LowLevelStrategy m_strategy;
  int m_deltaW;
  int m_deltaT;
  std::string m_instanceId;
  std::string m_methodLabel;
  CsvLogWriter *m_queryLogger;
  int m_maxRuntimeSeconds;
};

}  // namespace ugv
