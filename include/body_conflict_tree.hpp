/**
 * @file body_conflict_tree.hpp
 * @brief Body Conflict Tree (BCT) search for CL-CBS and LCBR.
 *
 * Both methods use exactly the same BCT:
 *
 *   CL-CBS -> full-horizon low-level replanning
 *   LCBR   -> local repair + full-horizon fallback
 *
 * Experimental extension:
 *
 * When pairedProbe=true and LCBR is used, every applicable local-repair
 * branch is additionally evaluated with a COUNTERFACTUAL full-horizon
 * SHA* query under the same BCT-child constraint set.
 *
 * IMPORTANT:
 *
 * The paired full-horizon query:
 *
 *   - does NOT replace the LCBR trajectory;
 *   - does NOT modify the BCT node;
 *   - does NOT modify the BCT cost;
 *   - does NOT enter E_tot;
 *   - does NOT enter N_LL;
 *   - does NOT determine fallback;
 *   - does NOT enter the normal LCBR computational metrics.
 *
 * It is used only for Q3/Q4:
 *
 *   same conflict + same robot + same child constraints
 *       Local SHA* versus Full-horizon SHA*
 */

#pragma once

#include <algorithm>
#include <boost/heap/d_ary_heap.hpp>
#include <cassert>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "experiment_logger.hpp"
#include "hybrid_astar.hpp"
#include "local_repair.hpp"
#include "low_level_environment.hpp"
#include "task_allocation.hpp"
#include "timer.hpp"

#define UGV_DEFAULT_MAX_RUNTIME 120

namespace ugv {

using libMultiRobotPlanning::HybridAStar;
using libMultiRobotPlanning::PlanResult;


// ============================================================================
// LOW-LEVEL STRATEGY
// ============================================================================

enum class LowLevelStrategy
{
  FULL_HORIZON,
  LCBR
};


inline std::string toString(LowLevelStrategy s)
{
  return
      s == LowLevelStrategy::FULL_HORIZON
          ? "CL-CBS"
          : "LCBR";
}


// ============================================================================
// BODY CONFLICT TREE
// ============================================================================

template <
    typename State,
    typename Action,
    typename Cost,
    typename Conflict,
    typename Constraint,
    typename Constraints,
    typename Environment>
class BodyConflictTree
{

 public:

  // ==========================================================================
  // CONSTRUCTOR
  // ==========================================================================

  BodyConflictTree(
      Environment &environment,
      LowLevelStrategy strategy,
      int deltaWSteps,
      int deltaTSteps,
      std::string instanceId,
      CsvLogWriter *queryLogger = nullptr,
      int maxRuntimeSeconds = UGV_DEFAULT_MAX_RUNTIME,
      bool pairedProbe = false)
      :
        m_env(environment),
        m_strategy(strategy),
        m_deltaW(deltaWSteps),
        m_deltaT(deltaTSteps),
        m_instanceId(std::move(instanceId)),
        m_methodLabel(toString(strategy)),
        m_queryLogger(queryLogger),
        m_maxRuntimeSeconds(maxRuntimeSeconds),
        m_pairedProbe(pairedProbe)
  {
  }


  // ==========================================================================
  // BCT SEARCH
  // ==========================================================================

  bool search(
      const std::vector<State> &initialStates,
      const std::vector<PlanResult<State, Action, Cost>> &gamma0,
      std::vector<PlanResult<State, Action, Cost>> &solution,
      InstanceLogRecord &instanceLog)
  {

    HighLevelNode start;

    const size_t M = initialStates.size();

    start.solution.resize(M);
    start.constraints.resize(M);

    start.cost = 0;
    start.id = 0;


    // ========================================================================
    // ROOT INITIALIZATION
    // ========================================================================

    for (size_t i = 0; i < M; ++i)
    {

      if (
          i < gamma0.size() &&
          gamma0[i].states.size() > 1)
      {

        start.solution[i] = gamma0[i];
      }

      else
      {

        LowLevelEnvironment<
            State,
            Action,
            Cost,
            Environment,
            Constraints>
            llenv(
                m_env,
                i,
                start.constraints[i]);


        HybridAStar<
            State,
            Action,
            Cost,
            LowLevelEnvironment<
                State,
                Action,
                Cost,
                Environment,
                Constraints>>
            lowLevel(
                llenv,
                Constants::maxLowLevelExpansions);


        m_env.resetLowLevelExpansionCounter();


        Timer rootInitTimer;


        bool ok =
            lowLevel.search(
                initialStates[i],
                start.solution[i]);


        rootInitTimer.stop();


        double rootTstar =
            ok
                ? static_cast<double>(
                      start.solution[i]
                          .states
                          .back()
                          .first
                          .time)
                : 0.0;


        double rootPathAfter =
            ok
                ? trajectoryPhysicalLength(
                      start.solution[i])
                : 0.0;


        logQuery(
            0,
            i,
            "root_init",
            "n/a",
            "",
            -1,
            -1,
            -1,
            0.0,
            0.0,
            0.0,
            rootTstar,
            0.0,
            m_env.lowLevelExpandedThisCall(),
            rootInitTimer.elapsedSeconds(),
            false,
            0.0,
            ok
                ? start.solution[i].cost
                : 0.0,
            0.0,
            rootPathAfter,
            0.0,
            false,
            false,
            false);


        if (!ok)
        {
          // Exceptional Stage-3 initialization failure.
          //
          // Normally gamma0 is already valid and this branch should
          // therefore almost never be reached.
          instanceLog.method = m_methodLabel;
          instanceLog.delta_w_steps = m_deltaW;
          instanceLog.delta_T_steps = m_deltaT;
          instanceLog.success = false;
          instanceLog.timeout = false;
          instanceLog.status = "STAGE3_FAILURE";

          return false;
        }
      }


      start.cost +=
          start.solution[i].cost;
    }


    // ========================================================================
    // OPEN LIST
    // ========================================================================

    typename boost::heap::d_ary_heap<
        HighLevelNode,
        boost::heap::arity<2>,
        boost::heap::mutable_<true>>
        open;


    auto handle =
        open.push(start);


    (*handle).handle =
        handle;


    // ========================================================================
    // REAL SOLVER COUNTERS
    // ========================================================================
    //
    // IMPORTANT:
    //
    // These counters describe ONLY the actual CL-CBS / LCBR algorithm.
    //
    // Counterfactual paired probes are deliberately excluded.
    // ========================================================================

    long long E_tot = 0;

    long long E_local_success = 0;

    long long E_local_fail = 0;

    long long E_full = 0;


    long long N_success = 0;

    long long N_fail = 0;

    long long N_inapplicable = 0;

    long long N_full_baseline = 0;

    long long N_full_fallback = 0;


    int bctNodesExpanded = 0;


    // ========================================================================
    // PAIRED-PROBE DIAGNOSTIC COUNTERS
    // ========================================================================

    double pairedProbeRuntimeS = 0.0;

    long long pairedProbeCalls = 0;


    // ========================================================================
    // BCT TIMER
    // ========================================================================

    Timer conflictTimer;


    auto wallClockStart =
        std::chrono::high_resolution_clock::now();


    int id = 1;

    bool timedOut = false;


    // ========================================================================
    // MAIN BCT LOOP
    // ========================================================================

    while (!open.empty())
    {

      auto elapsed =
          std::chrono::duration_cast<
              std::chrono::duration<double>>(
              std::chrono::
                  high_resolution_clock::now() -
              wallClockStart);


      if (
          elapsed.count() >
          m_maxRuntimeSeconds)
      {

        timedOut = true;

        break;
      }


      // ----------------------------------------------------------------------
      // Expand BCT node
      // ----------------------------------------------------------------------

      HighLevelNode P =
          open.top();


      m_env.onExpandHighLevelNode(
          P.cost);


      open.pop();


      bctNodesExpanded++;


      // ----------------------------------------------------------------------
      // Conflict detection
      // ----------------------------------------------------------------------

      Conflict conflict;


      if (
          !m_env.getFirstConflict(
              P.solution,
              conflict))
      {

        conflictTimer.stop();


        solution =
            P.solution;


        // --------------------------------------------------------------------
        // Final Stage-3 status
        // --------------------------------------------------------------------

        instanceLog.status =
            "SUCCESS";


        instanceLog.paired_probe_runtime_s =
            pairedProbeRuntimeS;


        instanceLog.paired_probe_calls =
            pairedProbeCalls;


        fillInstanceLog(
            instanceLog,
            true,
            conflictTimer.elapsedSeconds(),
            bctNodesExpanded,
            N_success,
            N_fail,
            N_inapplicable,
            N_full_baseline,
            N_full_fallback,
            E_tot,
            E_local_success,
            E_local_fail,
            E_full,
            solution);


        return true;
      }


      // ----------------------------------------------------------------------
      // Generate branch constraints
      // ----------------------------------------------------------------------

      std::map<size_t, Constraints>
          constraints;


      m_env.createConstraintsFromConflict(
          conflict,
          constraints);


      // ======================================================================
      // CREATE BCT CHILDREN
      // ======================================================================

      for (
          const auto &c :
          constraints)
      {

        const size_t i =
            c.first;


        HighLevelNode newNode =
            P;


        newNode.id =
            id;


        assert(
            !newNode
                 .constraints[i]
                 .overlap(c.second));


        newNode
            .constraints[i]
            .add(c.second);


        newNode.cost -=
            newNode.solution[i].cost;


        int conflictTimeStep =
            c.second
                .constraints
                .begin()
                ->time;


        bool childSuccess =
            false;


        // ====================================================================
        // CL-CBS
        // ====================================================================

        if (
            m_strategy ==
            LowLevelStrategy::
                FULL_HORIZON)
        {

          childSuccess =
              replanFullHorizon(
                  i,
                  newNode,
                  initialStates[i],
                  id,
                  false,
                  P.solution[i],
                  conflictTimeStep,
                  E_tot,
                  E_full,
                  N_full_baseline);
        }


        // ====================================================================
        // LCBR
        // ====================================================================

        else
        {

          LocalRepair<
              State,
              Action,
              Cost,
              Environment,
              Constraint,
              Constraints>
              repairer(
                  m_env,
                  m_deltaW,
                  m_deltaT);


          // ------------------------------------------------------------------
          // ACTUAL LCBR LOCAL REPAIR
          // ------------------------------------------------------------------

          auto outcome =
              repairer.repair(
                  P.solution[i],
                  conflictTimeStep,
                  i,
                  newNode.constraints[i]);


          // ==================================================================
          // Q3/Q4 PAIRED COUNTERFACTUAL FULL-HORIZON QUERY
          // ==================================================================
          //
          // SAME:
          //
          //   - instance
          //   - BCT child
          //   - constrained robot
          //   - accumulated constraint set
          //
          // DIFFERENCE:
          //
          //   Local:
          //       x_s -> x_g
          //
          //   Full:
          //       initial state -> assigned goal
          //
          // The full query below is DIAGNOSTIC ONLY.
          // ==================================================================

          bool pairedFullSuccess =
              false;


          int pairedFullExpansions =
              0;


          double pairedFullRuntime =
              0.0;


          double pairedFullGAfter =
              0.0;


          double pairedFullPathAfter =
              0.0;


          double pairedFullTStar =
              0.0;


          if (
              m_pairedProbe &&
              !outcome.inapplicable)
          {

            LowLevelEnvironment<
                State,
                Action,
                Cost,
                Environment,
                Constraints>
                probeEnv(
                    m_env,
                    i,
                    newNode.constraints[i]);


            HybridAStar<
                State,
                Action,
                Cost,
                LowLevelEnvironment<
                    State,
                    Action,
                    Cost,
                    Environment,
                    Constraints>>
                probePlanner(
                    probeEnv,
                    Constants::
                        maxLowLevelExpansions);


            PlanResult<
                State,
                Action,
                Cost>
                probeResult;


            m_env
                .resetLowLevelExpansionCounter();


            Timer probeTimer;


            pairedFullSuccess =
                probePlanner.search(
                    initialStates[i],
                    probeResult);


            probeTimer.stop();


            pairedFullRuntime =
                probeTimer.elapsedSeconds();


            pairedFullExpansions =
                m_env
                    .lowLevelExpandedThisCall();


            pairedProbeRuntimeS +=
                pairedFullRuntime;


            pairedProbeCalls++;


            if (pairedFullSuccess)
            {

              pairedFullGAfter =
                  probeResult.cost;


              pairedFullPathAfter =
                  trajectoryPhysicalLength(
                      probeResult);


              if (
                  !probeResult
                       .states
                       .empty())
              {

                pairedFullTStar =
                    static_cast<double>(
                        probeResult
                            .states
                            .back()
                            .first
                            .time);
              }
            }
          }


          // ==================================================================
          // LOG ACTUAL LOCAL QUERY + COUNTERFACTUAL FULL QUERY
          // ==================================================================

          logQuery(
              id,
              i,
              "local",

              outcome.inapplicable
                  ? "inapplicable"
                  : (
                        outcome.success
                            ? "success"
                            : "fail"),

              outcome.failureReason,

              conflictTimeStep,

              outcome.t_minus,

              outcome.t_plus,

              outcome.T_w,

              outcome.T_i_current,

              outcome.T_i_current > 0
                  ? outcome.T_w /
                        outcome.T_i_current
                  : 0.0,

              outcome.T_star,

              outcome.delta_T,

              outcome.local_expansions,

              outcome.local_runtime_s,

              false,

              outcome.g_before,

              outcome.success
                  ? outcome.g_after
                  : outcome.g_local_attempted,

              outcome.path_length_before,

              outcome.path_length_after,

              outcome.reconnection_residual,

              outcome.junction_correction_skipped,

              outcome.clipped_left,

              outcome.clipped_right,

              // --------------------------------------------------------------
              // Paired probe
              // --------------------------------------------------------------

              m_pairedProbe &&
                  !outcome.inapplicable,

              pairedFullSuccess,

              pairedFullExpansions,

              pairedFullRuntime,

              pairedFullGAfter,

              pairedFullPathAfter,

              pairedFullTStar);


          // ==================================================================
          // UPDATE REAL LCBR COUNTERS
          // ==================================================================

          E_tot +=
              outcome.local_expansions;


          if (
              outcome.inapplicable)
          {

            N_inapplicable++;
          }

          else if (
              outcome.success)
          {

            N_success++;


            E_local_success +=
                outcome.local_expansions;
          }

          else
          {

            N_fail++;


            E_local_fail +=
                outcome.local_expansions;
          }


          // ==================================================================
          // REAL LCBR DECISION
          // ==================================================================
          //
          // CRITICAL:
          //
          // pairedFullSuccess is NEVER consulted here.
          //
          // The paired probe therefore cannot alter the actual LCBR search.
          // ==================================================================

          if (
              outcome.success)
          {

            newNode.solution[i] =
                outcome.trajectory;


            childSuccess =
                true;
          }

          else
          {

            childSuccess =
                replanFullHorizon(
                    i,
                    newNode,
                    initialStates[i],
                    id,
                    true,
                    P.solution[i],
                    conflictTimeStep,
                    E_tot,
                    E_full,
                    N_full_fallback);
          }
        }


        // ====================================================================
        // INSERT SUCCESSFUL CHILD
        // ====================================================================

        newNode.cost +=
            newNode.solution[i].cost;


        if (childSuccess)
        {

          auto h =
              open.push(
                  newNode);


          (*h).handle =
              h;
        }


        ++id;
      }
    }


    // ========================================================================
    // FAILURE / TIMEOUT
    // ========================================================================

    conflictTimer.stop();


    // ------------------------------------------------------------------------
    // Distinguish a genuine BCT failure from the Stage-3 wall-clock timeout.
    // ------------------------------------------------------------------------

    instanceLog.status =
        timedOut
            ? "STAGE3_TIMEOUT"
            : "STAGE3_FAILURE";


    instanceLog.paired_probe_runtime_s =
        pairedProbeRuntimeS;


    instanceLog.paired_probe_calls =
        pairedProbeCalls;


    fillInstanceLog(
        instanceLog,
        false,
        conflictTimer.elapsedSeconds(),
        bctNodesExpanded,
        N_success,
        N_fail,
        N_inapplicable,
        N_full_baseline,
        N_full_fallback,
        E_tot,
        E_local_success,
        E_local_fail,
        E_full,
        {},
        timedOut);


    return false;
  }


 private:

  // ==========================================================================
  // HIGH-LEVEL NODE
  // ==========================================================================

  struct HighLevelNode
  {

    std::vector<
        PlanResult<
            State,
            Action,
            Cost>>
        solution;


    std::vector<
        Constraints>
        constraints;


    Cost cost;


    int id;


    typename boost::heap::
        d_ary_heap<
            HighLevelNode,
            boost::heap::arity<2>,
            boost::heap::mutable_<true>>
            ::handle_type
        handle;


    bool operator<(
        const HighLevelNode &n)
        const
    {

      return
          cost >
          n.cost;
    }
  };


  // ==========================================================================
  // FULL-HORIZON REPLANNING
  // ==========================================================================

  bool replanFullHorizon(
      size_t i,
      HighLevelNode &newNode,
      const State &agentInitialState,
      int nodeId,
      bool fallback,
      const PlanResult<
          State,
          Action,
          Cost> &before,
      int conflictTimeStep,
      long long &E_tot,
      long long &E_full,
      long long &counterToBump)
  {

    LowLevelEnvironment<
        State,
        Action,
        Cost,
        Environment,
        Constraints>
        llenv(
            m_env,
            i,
            newNode.constraints[i]);


    HybridAStar<
        State,
        Action,
        Cost,
        LowLevelEnvironment<
            State,
            Action,
            Cost,
            Environment,
            Constraints>>
        lowLevel(
            llenv,
            Constants::
                maxLowLevelExpansions);


    m_env
        .resetLowLevelExpansionCounter();


    Timer t;


    bool success =
        lowLevel.search(
            agentInitialState,
            newNode.solution[i]);


    t.stop();


    int expansions =
        m_env
            .lowLevelExpandedThisCall();


    E_tot +=
        expansions;


    E_full +=
        expansions;


    counterToBump++;


    double pathBefore =
        trajectoryPhysicalLength(
            before);


    double pathAfter =
        success
            ? trajectoryPhysicalLength(
                  newNode.solution[i])
            : 0.0;


    double tstar =
        success
            ? static_cast<double>(
                  newNode
                      .solution[i]
                      .states
                      .back()
                      .first
                      .time)
            : 0.0;


    logQuery(
        nodeId,
        i,
        "full",
        "n/a",
        "",
        conflictTimeStep,
        -1,
        -1,
        0.0,

        before.states.empty()
            ? 0.0
            : before
                  .states
                  .back()
                  .first
                  .time,

        0.0,
        tstar,
        0.0,
        expansions,
        t.elapsedSeconds(),
        fallback,
        before.cost,

        success
            ? newNode.solution[i].cost
            : 0.0,

        pathBefore,
        pathAfter,
        0.0,
        false,
        false,
        false);


    return success;
  }


  // ==========================================================================
  // QUERY LOGGER
  // ==========================================================================

  void logQuery(
      int nodeId,
      size_t robotId,
      const std::string &queryType,
      const std::string &outcome,
      const std::string &failureReason,
      int tConflict,
      int tMinus,
      int tPlus,
      double Tw,
      double Ticur,
      double windowFraction,
      double Tstar,
      double deltaT,
      int expansions,
      double runtimeS,
      bool fallback,
      double gBefore,
      double gAfter,
      double pathBefore,
      double pathAfter,
      double reconResidual,
      bool junctionSkipped,
      bool clipL,
      bool clipR,

      // Paired-probe fields
      bool pairedProbeEnabled = false,
      bool pairedFullSuccess = false,
      int pairedFullExpansions = 0,
      double pairedFullRuntimeS = 0.0,
      double pairedFullGAfter = 0.0,
      double pairedFullPathAfter = 0.0,
      double pairedFullTStar = 0.0)
  {

    if (!m_queryLogger)
    {
      return;
    }


    QueryLogRecord r;


    r.instance_id =
        m_instanceId;


    r.method =
        m_methodLabel;


    r.bct_node_id =
        nodeId;


    r.robot_id =
        robotId;


    r.query_type =
        queryType;


    r.local_outcome =
        outcome;


    r.failure_reason =
        failureReason;


    r.t_conflict =
        tConflict;


    r.t_minus =
        tMinus;


    r.t_plus =
        tPlus;


    r.T_w =
        Tw;


    r.T_i_current =
        Ticur;


    r.window_fraction =
        windowFraction;


    r.T_star =
        Tstar;


    r.delta_T_repair =
        deltaT;


    r.sha_expansions =
        expansions;


    r.sha_runtime_s =
        runtimeS;


    r.fallback_triggered =
        fallback;


    r.g_before =
        gBefore;


    r.g_after =
        gAfter;


    r.path_length_before =
        pathBefore;


    r.path_length_after =
        pathAfter;


    r.reconnection_residual =
        reconResidual;


    r.junction_correction_skipped =
        junctionSkipped;


    r.clipped_left =
        clipL;


    r.clipped_right =
        clipR;


    // ------------------------------------------------------------------------
    // Paired full-horizon probe
    // ------------------------------------------------------------------------

    r.paired_probe_enabled =
        pairedProbeEnabled;


    r.paired_full_success =
        pairedFullSuccess;


    r.paired_full_expansions =
        pairedFullExpansions;


    r.paired_full_runtime_s =
        pairedFullRuntimeS;


    r.paired_full_g_after =
        pairedFullGAfter;


    r.paired_full_path_length_after =
        pairedFullPathAfter;


    r.paired_full_T_star =
        pairedFullTStar;


    m_queryLogger->writeRow(
        r.toCsvRow());
  }


  // ==========================================================================
  // INSTANCE LOGGER
  // ==========================================================================

  void fillInstanceLog(
      InstanceLogRecord &log,
      bool success,
      double conflictRuntimeS,
      int bctNodes,
      long long Ns,
      long long Nf,
      long long Ninapp,
      long long Nfb,
      long long Nff,
      long long Etot,
      long long Els,
      long long Elf,
      long long Efull,
      const std::vector<
          PlanResult<
              State,
              Action,
              Cost>> &finalSolution,
      bool timedOut = false)
  {

    log.method =
        m_methodLabel;


    log.delta_w_steps =
        m_deltaW;


    log.delta_T_steps =
        m_deltaT;


    log.success =
        success;


    log.timeout =
        timedOut;


    log.conflict_resolution_runtime_s =
        conflictRuntimeS;


    log.bct_nodes_expanded =
        bctNodes;


    log.N_success =
        Ns;


    log.N_fail =
        Nf;


    log.N_inapplicable =
        Ninapp;


    log.N_full_baseline =
        Nfb;


    log.N_full_fallback =
        Nff;


    log.E_tot =
        Etot;


    log.E_local_success =
        Els;


    log.E_local_fail =
        Elf;


    log.E_full =
        Efull;


    // ========================================================================
    // FINAL SOLUTION QUALITY
    // ========================================================================

    if (success)
    {

      double soc =
          0.0;


      double lmax =
          0.0;


      double tmax =
          0.0;


      long long nSwitch =
          0;


      double dReverse =
          0.0;


      for (
          const auto &traj :
          finalSolution)
      {

        double len =
            trajectoryPhysicalLength(
                traj);


        soc +=
            len;


        lmax =
            std::max(
                lmax,
                len);


        if (
            !traj
                 .states
                 .empty())
        {

          tmax =
              std::max(
                  tmax,
                  static_cast<double>(
                      traj
                          .states
                          .back()
                          .first
                          .time));
        }


        // --------------------------------------------------------------------
        // Direction switches / reverse distance
        // --------------------------------------------------------------------

        for (
            size_t k = 0;
            k < traj.actions.size();
            ++k)
        {

          int act =
              traj.actions[k].first;


          bool isBackward =
              act >= 3 &&
              act != 6;


          if (
              isBackward &&
              k + 1 <
                  traj.states.size())
          {

            const auto &a =
                traj.states[k].first;


            const auto &b =
                traj.states[k + 1].first;


            dReverse +=
                std::sqrt(
                    (a.x - b.x) *
                        (a.x - b.x) +
                    (a.y - b.y) *
                        (a.y - b.y));
          }


          if (k > 0)
          {

            int prevAct =
                traj.actions[k - 1]
                    .first;


            bool prevBackward =
                prevAct >= 3 &&
                prevAct != 6;


            if (
                act != 6 &&
                prevAct != 6 &&
                isBackward !=
                    prevBackward)
            {

              nSwitch++;
            }
          }
        }
      }


      log.SoC_final =
          soc;


      log.L_max_final =
          lmax;


      log.mission_completion_time =
          tmax;


      log.N_switch =
          nSwitch;


      log.D_reverse =
          dReverse;
    }
  }


  // ==========================================================================
  // MEMBERS
  // ==========================================================================

  Environment &m_env;


  LowLevelStrategy m_strategy;


  int m_deltaW;


  int m_deltaT;


  std::string m_instanceId;


  std::string m_methodLabel;


  CsvLogWriter *m_queryLogger;


  int m_maxRuntimeSeconds;


  // --------------------------------------------------------------------------
  // Diagnostic experiment switch
  // --------------------------------------------------------------------------

  bool m_pairedProbe =
      false;
};


}  // namespace ugv