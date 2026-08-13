/**
 * @file local_repair.hpp
 * @brief Local Conflict-Based Trajectory Repair (LCBR) -- this project's
 * main contribution. Implements Algorithm 1 of the paper ("Local Trajectory
 * Repair"): given a branch constraint on robot R_i at BCT node N, replan
 * only a bounded segment around the conflict instead of the complete
 * start-to-goal trajectory (paper Sec. 4.3.3-4.3.7).
 *
 * Design invariant this file relies on: every PlanResult produced anywhere
 * in this project satisfies states[k].first.time == k + states[0].first.time
 * for all k (the T_s grid invariant). It is established by HybridAStar
 * (states are appended one T_s step at a time) and preserved here by
 * construction of the spliced trajectory (Eq. 12).
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "environment.hpp"
#include "hybrid_astar.hpp"
#include "low_level_environment.hpp"
#include "planresult.hpp"
#include "task_allocation.hpp"
#include "timer.hpp"

namespace ugv {

using libMultiRobotPlanning::HybridAStar;
using libMultiRobotPlanning::PlanResult;

template <typename State, typename Action, typename Cost, typename Environment,
          typename Constraint, typename Constraints>
class LocalRepair {
 public:
  /// Outcome of a single call to repair(), carrying every diagnostic field
  /// needed to populate one QueryLogRecord row (Sec. "LCBR Mechanism
  /// Analysis" of the experimental plan).
  struct Outcome {
    bool success = false;
    bool inapplicable = false;  // t_conflict >= T_i^N (paper Sec. 4.3.8)
    std::string failureReason;  // "" | "no_local_path" | "constraint_violation_after_shift"

    PlanResult<State, Action, Cost> trajectory;  // valid only if success

    int t_minus = -1;
    int t_plus = -1;
    double T_w = 0.0;
    double T_i_current = 0.0;
    double T_star = 0.0;
    double delta_T = 0.0;
    bool clipped_left = false;
    bool clipped_right = false;

    int local_expansions = 0;
    double local_runtime_s = 0.0;

    double g_before = 0.0;
    double g_local_attempted = 0.0;  // g(.) of the attempted local segment alone
    double g_after = 0.0;            // g(.) of the full spliced trajectory (if success)
    double path_length_before = 0.0;
    double path_length_after = 0.0;
    double reconnection_residual = 0.0;
    bool junction_correction_skipped = false;
  };

  LocalRepair(Environment &env, int deltaWSteps, int deltaTSteps)
      : m_env(env), m_deltaW(deltaWSteps), m_deltaT(deltaTSteps) {}

  /// Algorithm 1. `currentTraj` is Gamma_i^N, `conflictTimeStep` is the
  /// time index t of the detected body conflict (== the .time field of the
  /// newly-added branch Constraint), `agentIdx` is the constrained robot,
  /// `constraints` is the COMPLETE accumulated constraint set for that
  /// robot at the child node (i.e. C_i^N union {c_new}).
  Outcome repair(const PlanResult<State, Action, Cost> &currentTraj,
                int conflictTimeStep, size_t agentIdx,
                const Constraints &constraints) {
    Outcome out;
    out.g_before = currentTraj.cost;
    out.path_length_before = trajectoryPhysicalLength(currentTraj);

    const int T_i_N = currentTraj.states.back().first.time;
    out.T_i_current = T_i_N;

    // --- Inapplicability check (paper Sec. 4.3.8 / Algorithm 1, line 1) ---
    if (conflictTimeStep >= T_i_N) {
      out.inapplicable = true;
      out.success = false;
      return out;
    }

    // --- Repair window extraction (Eq. 10) ---
    int tMinus = std::max(0, conflictTimeStep - m_deltaW);
    int tPlus = std::min(T_i_N, conflictTimeStep + m_deltaW);
    out.t_minus = tMinus;
    out.t_plus = tPlus;
    out.T_w = tPlus - tMinus;
    out.clipped_left = (conflictTimeStep - m_deltaW) < 0;
    out.clipped_right = (conflictTimeStep + m_deltaW) > T_i_N;

    // Grid invariant: states[k].first.time == k for a trajectory that
    // started at time 0 (always true for a robot's own trajectory).
    const State xs = currentTraj.states[tMinus].first;
    const State xg = currentTraj.states[tPlus].first;

    // --- Local replanning problem (Eq. 11-12 / Sec. 4.3.5-4.3.6) ---
    ugv::LowLevelEnvironment<State, Action, Cost, Environment, Constraints>
        llenv(m_env, agentIdx, constraints);
    m_env.setLocalGoal(xg);
    m_env.resetLowLevelExpansionCounter();

    HybridAStar<State, Action, Cost,
               ugv::LowLevelEnvironment<State, Action, Cost, Environment,
                                        Constraints>>
        localSearch(llenv, Constants::maxLowLevelExpansions);
    PlanResult<State, Action, Cost> localResult;
    Timer localTimer;
    bool found = localSearch.search(xs, localResult, /*initialCost=*/0.0);
    localTimer.stop();

    out.local_expansions = m_env.lowLevelExpandedThisCall();
    out.local_runtime_s = localTimer.elapsedSeconds();
    m_env.clearLocalGoal();

    if (!found) {
      out.success = false;
      out.failureReason = "no_local_path";
      return out;
    }

    out.g_local_attempted = localResult.cost;
    out.T_star = localResult.states.back().first.time - tMinus;
    out.delta_T = out.T_star - out.T_w;
    {
      const State &reached = localResult.states.back().first;
      out.reconnection_residual = std::sqrt((reached.x - xg.x) * (reached.x - xg.x) +
                                            (reached.y - xg.y) * (reached.y - xg.y));
    }
    const int deltaTSteps = static_cast<int>(std::lround(out.delta_T));

    // --- Junction cost correction (Sec. 4.3.6 / Remark: direction-change
    // penalty at the splice boundaries). See Constants::stepCost() in
    // environment.hpp for the documented scope/limitation of this
    // correction (fixed-length primitive steps only). ---
    bool correctionSkipped = false;

    if (!localResult.actions.empty()) {
      int prevActOriginal = 0;  // HybridAStar always seeds the start node
                                 // with Action() == 0 internally.
      int prevActTrue =
          (tMinus > 0) ? currentTraj.actions[tMinus - 1].first : 0;
      correctJunctionAction(localResult.actions.front(), prevActOriginal,
                            prevActTrue, localResult.cost, correctionSkipped);
    }

    // Copy the suffix action slice so we can correct its first entry
    // in-place without mutating currentTraj.
    std::vector<std::pair<Action, Cost>> suffixActions(
        currentTraj.actions.begin() + tPlus, currentTraj.actions.end());
    if (!suffixActions.empty()) {
      int prevActOriginal =
          (tPlus > 0) ? currentTraj.actions[tPlus - 1].first : 0;
      int prevActTrue = !localResult.actions.empty()
                            ? localResult.actions.back().first
                            : prevActOriginal;
      Cost dummyCostAccumulator = 0.0;  // suffix total tracked via loop sum below
      correctJunctionAction(suffixActions.front(), prevActOriginal,
                            prevActTrue, dummyCostAccumulator, correctionSkipped);
    }
    out.junction_correction_skipped = correctionSkipped;

    // --- Splice + temporal shift of the suffix (Eq. 12) ---
    PlanResult<State, Action, Cost> spliced;
    spliced.states.reserve(currentTraj.states.size() + 8);
    spliced.actions.reserve(currentTraj.actions.size() + 8);

    // Prefix: states strictly before t^-, actions[0 .. tMinus-1]
    for (int k = 0; k < tMinus; ++k) spliced.states.push_back(currentTraj.states[k]);
    for (int k = 0; k < tMinus; ++k) spliced.actions.push_back(currentTraj.actions[k]);

    // Local segment: covers [t^-, t^- + T*]
    for (const auto &s : localResult.states) spliced.states.push_back(s);
    for (const auto &a : localResult.actions) spliced.actions.push_back(a);

    // Suffix: originally the interval after t^+ up to T_i^N, shifted by
    // delta_T. We keep the state at index tPlus out of the *new* sequence
    // since it is now provided by the local segment's own last state; the
    // suffix proper starts at the state that followed it.
    std::vector<std::pair<State, Cost>> shiftedSuffixStates;
    for (size_t k = static_cast<size_t>(tPlus) + 1; k < currentTraj.states.size(); ++k) {
      State shifted = currentTraj.states[k].first;
      shifted.time += deltaTSteps;
      shiftedSuffixStates.push_back(std::make_pair(shifted, currentTraj.states[k].second));
    }
    for (const auto &s : shiftedSuffixStates) spliced.states.push_back(s);
    for (const auto &a : suffixActions) spliced.actions.push_back(a);

    // --- Re-validation of the shifted suffix (Sec. 4.3.6): the temporal
    // shift can cause a previously-safe suffix state to violate the active
    // constraint set (or newly collide with static/dynamic obstacles at
    // its new time index). Re-run the same feasibility check the search
    // itself uses. The low-level context (agent + constraints) set at the
    // top of this function via the LowLevelEnvironment constructor is
    // still active on m_env at this point. ---
    bool suffixValid = true;
    for (const auto &s : shiftedSuffixStates) {
      if (!m_env.checkStateValid(s.first)) {
        suffixValid = false;
        break;
      }
    }
    if (!suffixValid) {
      out.success = false;
      out.failureReason = "constraint_violation_after_shift";
      return out;
    }

    // --- Recompute total cost over the complete spliced trajectory (Sec.
    // 4.3.6: "g(Gamma_i^{N'}) is recomputed over the complete trajectory
    // using the same transition costs and penalties as SHA*"). Interior
    // transitions keep their original per-step cost (unaffected by the
    // splice); only the two junction transitions were corrected above. ---
    double totalCost = 0.0;
    for (const auto &a : spliced.actions) totalCost += a.second;
    spliced.cost = totalCost;
    spliced.fmin = totalCost;

    out.success = true;
    out.trajectory = spliced;
    out.g_after = totalCost;
    out.path_length_after = trajectoryPhysicalLength(spliced);
    return out;
  }

 private:
  /// Recomputes the cost of a single junction action if (and only if) its
  /// original cost matches the fixed-length primitive-grid formula under
  /// the ORIGINAL (wrong) previous-action context -- i.e. it is safe to
  /// re-derive under the TRUE previous-action context. If the original
  /// cost does not match (most likely because the action belongs to the
  /// Reeds-Shepp analytic expansion tail, whose transitions have
  /// geometry-dependent, non-fixed-length costs -- see the documented
  /// limitation on Constants::stepCost() in environment.hpp), the
  /// correction is skipped and `skippedFlag` is set so the caller can log
  /// it.
  static void correctJunctionAction(std::pair<Action, Cost> &junctionAction,
                                    int prevActOriginal, int prevActTrue,
                                    Cost &costAccumulatorToAdjust,
                                    bool &skippedFlag) {
    if (prevActOriginal == prevActTrue) return;  // nothing to correct
    double expectedOriginal =
        Constants::stepCost(junctionAction.first, prevActOriginal);
    const double eps = 1e-6;
    if (std::fabs(expectedOriginal - junctionAction.second) > eps) {
      // Not a fixed-length primitive step (almost certainly an RS
      // analytic-expansion transition near the trajectory's true
      // endpoint) -- leave its cost untouched, flag the approximation.
      skippedFlag = true;
      return;
    }
    double corrected = Constants::stepCost(junctionAction.first, prevActTrue);
    costAccumulatorToAdjust += (corrected - junctionAction.second);
    junctionAction.second = corrected;
  }

  Environment &m_env;
  int m_deltaW;
  int m_deltaT;  // delta_T (constraintWaitTime), kept for reference/logging
};

}  // namespace ugv
