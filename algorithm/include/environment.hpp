/**
 * @file environment.hpp
 * @brief Search environment: robot footprint, kinematics, obstacles,
 * constraints, and the SHA* callback surface (heuristic, isSolution,
 * getNeighbors, calcIndex).
 *
 * Derived from CL-CBS (Wen et al., 2022), `include/environment.hpp`. Every
 * function below is byte-for-byte identical to the original EXCEPT where
 * marked "[LCBR]" or "[LOG]". The two functional additions are:
 *
 *   1. A local-goal override (setLocalGoal / clearLocalGoal / getGoal).
 *      LCBR's bounded query (Sec. 4.3.5 of the paper) must search towards
 *      a boundary configuration x_g instead of the robot's final assigned
 *      POI. getGoal() already doubles as "the exact endpoint the analytic
 *      expansion snapped onto" (isSolution overwrites it) because that is
 *      how the original HybridAStar retrieves the terminal state from
 *      cameFrom. We preserve that mechanic exactly, just redirecting it at
 *      m_localGoal instead of m_goals[m_agentIdx] when a local goal is
 *      active, so nothing about the search's control flow changes.
 *
 *   2. Per-call low-level expansion counters, needed to log E_local and
 *      E_full separately per query (Sec. 5.3 of the experimental plan)
 *      instead of only the cumulative total the original code tracked.
 *
 * The holonomic (twoDCost) heuristic term is disabled when a local goal is
 * active, because that costmap is precomputed towards the *final* POI, not
 * towards x_g -- using it for a windowed query would be actively
 * misleading, not just merely inadmissible. See admissibleHeuristic().
 */

#pragma once
#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <boost/functional/hash.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "neighbor.hpp"
#include "planresult.hpp"

namespace Constants {
// [m] --- The minimum turning radius of the vehicle
static float r = 3;
static float deltat = 6.75 * 6 / 180.0 * M_PI;
// [#] --- A movement cost penalty for turning (choosing non straight motion
// primitives)
static float penaltyTurning = 1.5;
// [#] --- A movement cost penalty for reversing (choosing motion primitives >
// 2)
static float penaltyReversing = 2.0;
// [#] --- A movement cost penalty for change of direction (changing from
// primitives < 3 to primitives > 2)
static float penaltyCOD = 2.0;
// map resolution
static float mapResolution = 2.0;
// change to set calcIndex resolution
static float xyResolution = r * deltat;
static float yawResolution = deltat;

// width of car
static float carWidth = 2.0;
// distance from rear to vehicle front end
static float LF = 2.0;
// distance from rear to vehicle back end
static float LB = 1.0;
// obstacle default radius
static float obsRadius = 1;
// least time to wait for constraint -- this IS delta_T (temporal safety
// margin) from the paper's Eq. (7)/(8): a constraint blocks
// [t - delta_T, t + delta_T] on the T_s grid.
static int constraintWaitTime = 2;

// [LCBR] Repair margin delta_w, expressed in T_s steps (paper Eq. 10-11).
// Must satisfy delta_w >= constraintWaitTime + 1 (Eq. 11, with T_s = 1
// step). Set from the command line / config; defaulted here so the
// environment still compiles standalone.
static int repairMarginSteps = 10;

// [robustness] Cap on SHA* expansions for a SINGLE low-level call (Stage
// 1 nominal query, LCBR local query, or full-horizon query/fallback).
// The original CL-CBS low-level search has no such cap; on a genuinely
// hard or practically-infeasible robot-POI pair (this occurs on Wen et
// al.'s own benchmark, e.g. a goal only reachable via a maneuver tighter
// than the vehicle's minimum turning radius allows), the open set grows
// without bound until the process is OOM-killed, since search time (and
// therefore state-space size) is otherwise unbounded. A query that
// exceeds this cap is treated as infeasible (Gamma_ij = empty set,
// Eq. 6), consistent with any other reason a query might fail. Default
// is comfortably above every successful single-call expansion count
// observed in this project's own test runs (max ~11k); set to 0 to
// restore the original CL-CBS unbounded behavior.
static size_t maxLowLevelExpansions = 150000;

// R = 3, 6.75 DEG
std::vector<double> dyaw = {0, deltat, -deltat, 0, -deltat, deltat};
std::vector<double> dx = {r * deltat, r *sin(deltat),  r *sin(deltat),
                          -r *deltat, -r *sin(deltat), -r *sin(deltat)};
std::vector<double> dy = {0, -r *(1 - cos(deltat)), r *(1 - cos(deltat)),
                          0, -r *(1 - cos(deltat)), r *(1 - cos(deltat))};

static inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
}

// [LCBR] Per-transition SHA* cost, reproducing exactly the penalty formula
// used inside Environment::getNeighbors() for the fixed-length primitive
// steps (base length dx[0], scaled by turning/reversing/change-of-direction
// penalties). `prevAct` is the action used on the PRECEDING transition
// (Action() == 0, i.e. "forward straight", is the convention HybridAStar
// itself uses to seed the very first node of a search -- see
// hybrid_astar.hpp, `openSet.push(Node(startState, Action(), ...))`).
//
// This is used by local_repair.hpp to recompute the cost of the two
// transitions that sit exactly at a splice boundary (prefix->local segment,
// local segment->suffix), because those transitions were originally
// evaluated with the wrong `prevAct` context (Sec. 4.3.6 / Remark on
// direction-change penalties at the junction).
//
// NOTE (documented approximation, see project README "Limitations"): this
// formula assumes a fixed-length primitive step of length dx[0]. It does
// NOT hold for a transition that belongs to the Reeds-Shepp analytic
// expansion appended by isSolution() near a trajectory's true endpoint
// (those have geometry-dependent, non-fixed lengths). local_repair.hpp
// guards against this by comparing the junction action's *original* cost
// against the primitive-grid cost family before applying this correction,
// and skips the correction (flagging it in the log) otherwise. In practice
// this only matters when the repair window happens to end inside the last
// 1-3 states of a trajectory, i.e. inside the RS analytic tail.
static inline double stepCost(int act, int prevAct) {
  if (act == 6) return dx[0];  // wait
  double g = dx[0];
  if (act % 3 != 0) {  // turning
    g = g * penaltyTurning;
  }
  if ((act < 3 && prevAct >= 3) || (prevAct < 3 && act >= 3)) {
    g = g * penaltyCOD;  // change of direction
  }
  if (act >= 3) {  // backwards
    g = g * penaltyReversing;
  }
  return g;
}

}  // namespace Constants

namespace libMultiRobotPlanning {

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace libMultiRobotPlanning;
typedef ompl::base::SE2StateSpace::StateType OmplState;
typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::segment<Point> Segment;

/**
 * @brief Environment class: obstacles, kinematics, footprints, constraints,
 * and the SHA* callback surface.
 */
template <typename Location, typename State, typename Action, typename Cost,
          typename Conflict, typename Constraint, typename Constraints>
class Environment {
 public:
  Environment(size_t maxx, size_t maxy, std::unordered_set<Location> obstacles,
              std::multimap<int, State> dynamic_obstacles,
              std::vector<State> goals)
      : m_obstacles(std::move(obstacles)),
        m_dynamic_obstacles(std::move(dynamic_obstacles)),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        m_lowLevelExpandedThisCall(0),
        m_localGoalActive(false) {
    m_dimx = static_cast<int>(maxx / Constants::mapResolution);
    m_dimy = static_cast<int>(maxy / Constants::mapResolution);
    holonomic_cost_maps = std::vector<std::vector<std::vector<double>>>(
        goals.size(), std::vector<std::vector<double>>(
                          m_dimx, std::vector<double>(m_dimy, 0)));
    m_goals.clear();
    for (const auto &g : goals) {
      if (g.x < 0 || g.x > maxx || g.y < 0 || g.y > maxy) {
        std::cout << "\033[1m\033[31m Goal out of boundary, Fail to build "
                     "environment \033[0m\n";
        return;
      }
      m_goals.emplace_back(
          State(g.x, g.y, Constants::normalizeHeadingRad(g.yaw)));
    }
    updateCostmap();
  }

  Environment(const Environment &) = delete;
  Environment &operator=(const Environment &) = delete;

  /// High Level Environment functions
  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, double>> &solution,
      Conflict &result) {
    int max_t = 0;
    for (const auto &sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }
    for (int t = 0; t < max_t; ++t) {
      // check drive-drive collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.agentCollision(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.s1 = state1;
            result.s2 = state2;
            return true;
          }
        }
      }
    }
    return false;
  }

  void createConstraintsFromConflict(
      const Conflict &conflict, std::map<size_t, Constraints> &constraints) {
    Constraints c1;
    c1.constraints.emplace(
        Constraint(conflict.time, conflict.s2, conflict.agent2));
    constraints[conflict.agent1] = c1;
    Constraints c2;
    c2.constraints.emplace(
        Constraint(conflict.time, conflict.s1, conflict.agent1));
    constraints[conflict.agent2] = c2;
  }

  void onExpandHighLevelNode(int /*cost*/) {
    m_highLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  /// Low Level Environment functions
  void setLowLevelContext(size_t agentIdx, const Constraints *constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto &c : constraints->constraints) {
      if (currentGoal().agentCollision(c.s)) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, c.time);
      }
    }
  }

  // [LCBR] ---------------------------------------------------------------
  // Local-goal override used by LCBR's bounded query (paper Eq. 10-12).
  // While active, getGoal()/currentGoal() return m_localGoal instead of
  // m_goals[m_agentIdx], and isSolution() snaps m_localGoal (not
  // m_goals[m_agentIdx]) onto the exact analytic-expansion endpoint, so the
  // permanent per-agent goal table is never mutated by a local query.
  void setLocalGoal(const State &g) {
    m_localGoal = g;
    m_localGoalActive = true;
  }
  void clearLocalGoal() { m_localGoalActive = false; }
  bool hasLocalGoal() const { return m_localGoalActive; }

  // [LOG] ------------------------------------------------------------------
  // Per-call low-level expansion counter (E_local / E_full in the
  // experimental logs). Must be reset immediately before every SHA* call.
  void resetLowLevelExpansionCounter() { m_lowLevelExpandedThisCall = 0; }
  int lowLevelExpandedThisCall() const { return m_lowLevelExpandedThisCall; }

  // [LCBR] Exposes the private feasibility check (static obstacles, map
  // bounds, dynamic obstacles from earlier batches, and the *currently
  // active* constraint set set via setLowLevelContext) so that
  // local_repair.hpp can re-validate the temporally-shifted suffix after
  // splicing (paper Sec. 4.3.6: "Gamma_i^{N'} is re-validated against the
  // complete active constraint set before the child node is accepted").
  bool checkStateValid(const State &s) { return stateValid(s); }

  double admissibleHeuristic(const State &s) {
    State goal = currentGoal();
    // non-holonomic-without-obstacles heuristic: use a Reeds-Shepp
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    OmplState *rsStart = (OmplState *)reedsSheppPath.allocState();
    OmplState *rsEnd = (OmplState *)reedsSheppPath.allocState();
    rsStart->setXY(s.x, s.y);
    rsStart->setYaw(s.yaw);
    rsEnd->setXY(goal.x, goal.y);
    rsEnd->setYaw(goal.yaw);
    double reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    // Euclidean distance
    double euclideanCost =
        sqrt(pow(goal.x - s.x, 2) + pow(goal.y - s.y, 2));

    // [LCBR] The precomputed holonomic-with-obstacles costmap
    // (holonomic_cost_maps[m_agentIdx]) encodes grid distance towards the
    // agent's FINAL POI, not towards an arbitrary local goal x_g. Reusing
    // it for a windowed query would silently corrupt the heuristic (it is
    // not just inadmissible, its magnitude has nothing to do with the
    // local query). Skip it whenever a local goal is active; Reeds-Shepp +
    // Euclidean remain valid lower bounds towards x_g in open space.
    if (m_localGoalActive) {
      return std::max({reedsSheppCost, euclideanCost});
    }

    double twoDoffset =
        sqrt(pow((s.x - static_cast<int>(s.x)) -
                     (goal.x - static_cast<int>(goal.x)),
                 2) +
             pow((s.y - static_cast<int>(s.y)) -
                     (goal.y - static_cast<int>(goal.y)),
                 2));
    double twoDCost =
        holonomic_cost_maps[m_agentIdx]
                           [static_cast<int>(s.x / Constants::mapResolution)]
                           [static_cast<int>(s.y / Constants::mapResolution)] -
        twoDoffset;

    return std::max({reedsSheppCost, euclideanCost, twoDCost});
  }

  bool isSolution(
      const State &state, double gscore,
      std::unordered_map<State, std::tuple<State, Action, double, double>,
                         std::hash<State>> &_camefrom) {
    State goal = currentGoal();
    double goal_distance =
        sqrt(pow(state.x - goal.x, 2) + pow(state.y - goal.y, 2));
    if (goal_distance > 3 * (Constants::LB + Constants::LF)) return false;
    ompl::base::ReedsSheppStateSpace reedsSheppSpace(Constants::r);
    OmplState *rsStart = (OmplState *)reedsSheppSpace.allocState();
    OmplState *rsEnd = (OmplState *)reedsSheppSpace.allocState();
    rsStart->setXY(state.x, state.y);
    rsStart->setYaw(-state.yaw);
    rsEnd->setXY(goal.x, goal.y);
    rsEnd->setYaw(-goal.yaw);
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath reedsShepppath =
        reedsSheppSpace.reedsShepp(rsStart, rsEnd);

    std::vector<State> path;
    std::unordered_map<State, std::tuple<State, Action, double, double>,
                       std::hash<State>>
        cameFrom;
    cameFrom.clear();
    path.emplace_back(state);
    for (auto pathidx = 0; pathidx < 5; pathidx++) {
      if (fabs(reedsShepppath.length_[pathidx]) < 1e-6) continue;
      double deltat = 0, dx = 0, act = 0, cost = 0;
      switch (reedsShepppath.type_[pathidx]) {
        case 0:  // RS_NOP
          continue;
          break;
        case 1:  // RS_LEFT
          deltat = -reedsShepppath.length_[pathidx];
          dx = Constants::r * sin(-deltat);
          act = 2;
          cost = reedsShepppath.length_[pathidx] * Constants::r *
                 Constants::penaltyTurning;
          break;
        case 2:  // RS_STRAIGHT
          deltat = 0;
          dx = reedsShepppath.length_[pathidx] * Constants::r;
          act = 0;
          cost = dx;
          break;
        case 3:  // RS_RIGHT
          deltat = reedsShepppath.length_[pathidx];
          dx = Constants::r * sin(deltat);
          act = 1;
          cost = reedsShepppath.length_[pathidx] * Constants::r *
                 Constants::penaltyTurning;
          break;
        default:
          std::cout << "\033[1m\033[31m"
                    << "Warning: Receive unknown ReedsSheppPath type"
                    << "\033[0m\n";
          break;
      }
      if (cost < 0) {
        cost = -cost * Constants::penaltyReversing;
        act = act + 3;
      }
      State s = path.back();
      std::vector<std::pair<State, double>> next_path;
      if (generatePath(s, act, deltat, dx, next_path)) {
        for (auto iter = next_path.begin(); iter != next_path.end(); iter++) {
          State next_s = iter->first;
          gscore += iter->second;
          if (!(next_s == path.back())) {
            cameFrom.insert(std::make_pair<>(
                next_s,
                std::make_tuple<>(path.back(), act, iter->second, gscore)));
          }
          path.emplace_back(next_s);
        }
      } else {
        return false;
      }
    }

    if (path.back().time <= m_lastGoalConstraint) {
      return false;
    }

    // [LCBR] Snap the exact analytic-expansion endpoint onto the *active*
    // goal slot (local goal if a windowed query is in progress, otherwise
    // the agent's permanent final-POI slot) -- this preserves the original
    // mechanic HybridAStar relies on (cameFrom.find(m_env.getGoal())) while
    // never mutating m_goals[m_agentIdx] during a local query.
    if (m_localGoalActive) {
      m_localGoal = path.back();
    } else {
      m_goals[m_agentIdx] = path.back();
    }

    _camefrom.insert(cameFrom.begin(), cameFrom.end());
    return true;
  }

  void getNeighbors(const State &s, Action action,
                    std::vector<Neighbor<State, Action, double>> &neighbors) {
    neighbors.clear();
    double g = Constants::dx[0];
    for (Action act = 0; act < 6; act++) {  // has 6 directions for Reeds-Shepp
      double xSucc, ySucc, yawSucc;
      g = Constants::dx[0];
      xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
              Constants::dy[act] * sin(-s.yaw);
      ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
              Constants::dy[act] * cos(-s.yaw);
      yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
      if (act % 3 != 0) {  // penalize turning
        g = g * Constants::penaltyTurning;
      }
      if ((act < 3 && action >= 3) || (action < 3 && act >= 3)) {
        // penalize change of direction
        g = g * Constants::penaltyCOD;
      }
      if (act >= 3) {  // backwards
        g = g * Constants::penaltyReversing;
      }
      State tempState(xSucc, ySucc, yawSucc, s.time + 1);
      if (stateValid(tempState)) {
        neighbors.emplace_back(
            Neighbor<State, Action, double>(tempState, act, g));
      }
    }
    // wait
    g = Constants::dx[0];
    State tempState(s.x, s.y, s.yaw, s.time + 1);
    if (stateValid(tempState)) {
      neighbors.emplace_back(Neighbor<State, Action, double>(tempState, 6, g));
    }
  }

  // [LCBR] Returns whichever goal is currently active (local or final POI).
  State getGoal() { return currentGoal(); }

  uint64_t calcIndex(const State &s) {
    return (uint64_t)s.time * (2 * M_PI / Constants::deltat) *
               (m_dimx / Constants::xyResolution) *
               (m_dimy / Constants::xyResolution) +
           (uint64_t)(Constants::normalizeHeadingRad(s.yaw) /
                      Constants::yawResolution) *
               (m_dimx / Constants::xyResolution) *
               (m_dimy / Constants::xyResolution) +
           (uint64_t)(s.y / Constants::xyResolution) *
               (m_dimx / Constants::xyResolution) +
           (uint64_t)(s.x / Constants::xyResolution);
  }

  void onExpandLowLevelNode(const State & /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
    m_lowLevelExpandedThisCall++;
  }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

  bool startAndGoalValid(const std::vector<State> &m_starts, const size_t iter,
                         const int batchsize) {
    assert(m_goals.size() == m_starts.size());
    for (size_t i = 0; i < m_goals.size(); i++)
      for (size_t j = i + 1; j < m_goals.size(); j++) {
        if (m_goals[i].agentCollision(m_goals[j])) {
          std::cout << "ERROR: Goal point of " << i + iter * batchsize << " & "
                    << j + iter * batchsize << " collide!\n";
          return false;
        }
        if (m_starts[i].agentCollision(m_starts[j])) {
          std::cout << "ERROR: Start point of " << i + iter * batchsize << " & "
                    << j + iter * batchsize << " collide!\n";
          return false;
        }
      }
    return true;
  }

 private:
  // [LCBR] Single accessor used everywhere a "current goal" was previously
  // read directly from m_goals[m_agentIdx]; redirects to the local goal
  // when a windowed query is active.
  State currentGoal() {
    return m_localGoalActive ? m_localGoal : m_goals[m_agentIdx];
  }

  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, double>> &solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State &s) {
    double x_ind = s.x / Constants::mapResolution;
    double y_ind = s.y / Constants::mapResolution;
    if (x_ind < 0 || x_ind >= m_dimx || y_ind < 0 || y_ind >= m_dimy)
      return false;

    for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++) {
      if (s.obsCollision(*it)) return false;
    }

    auto it = m_dynamic_obstacles.equal_range(s.time);
    for (auto itr = it.first; itr != it.second; ++itr) {
      if (s.agentCollision(itr->second)) return false;
    }
    auto itlow = m_dynamic_obstacles.lower_bound(-s.time);
    auto itup = m_dynamic_obstacles.upper_bound(-1);
    for (auto it = itlow; it != itup; ++it)
      if (s.agentCollision(it->second)) return false;

    for (auto it = m_constraints->constraints.begin();
         it != m_constraints->constraints.end(); it++) {
      if (!it->satisfyConstraint(s)) return false;
    }

    return true;
  }

 private:
  struct compare_node {
    bool operator()(const std::pair<State, double> &n1,
                    const std::pair<State, double> &n2) const {
      return (n1.second > n2.second);
    }
  };
  void updateCostmap() {
    boost::heap::fibonacci_heap<std::pair<State, double>,
                                boost::heap::compare<compare_node>>
        heap;

    std::set<std::pair<int, int>> temp_obs_set;
    for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++) {
      temp_obs_set.insert(
          std::make_pair(static_cast<int>(it->x / Constants::mapResolution),
                         static_cast<int>(it->y / Constants::mapResolution)));
    }

    for (size_t idx = 0; idx < m_goals.size(); idx++) {
      heap.clear();
      int goal_x = static_cast<int>(m_goals[idx].x / Constants::mapResolution);
      int goal_y = static_cast<int>(m_goals[idx].y / Constants::mapResolution);
      heap.push(std::make_pair(State(goal_x, goal_y, 0), 0));

      while (!heap.empty()) {
        std::pair<State, double> node = heap.top();
        heap.pop();

        int x = node.first.x;
        int y = node.first.y;
        for (int dx = -1; dx <= 1; dx++)
          for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            int new_x = x + dx;
            int new_y = y + dy;
            if (new_x == goal_x && new_y == goal_y) continue;
            if (new_x >= 0 && new_x < m_dimx && new_y >= 0 && new_y < m_dimy &&
                holonomic_cost_maps[idx][new_x][new_y] == 0 &&
                temp_obs_set.find(std::make_pair(new_x, new_y)) ==
                    temp_obs_set.end()) {
              holonomic_cost_maps[idx][new_x][new_y] =
                  holonomic_cost_maps[idx][x][y] +
                  sqrt(pow(dx * Constants::mapResolution, 2) +
                       pow(dy * Constants::mapResolution, 2));
              heap.push(std::make_pair(State(new_x, new_y, 0),
                                       holonomic_cost_maps[idx][new_x][new_y]));
            }
          }
      }
    }
  }

  bool generatePath(State startState, int act, double deltaSteer,
                    double deltaLength,
                    std::vector<std::pair<State, double>> &result) {
    double xSucc, ySucc, yawSucc, dx, dy, dyaw, ratio;
    result.emplace_back(std::make_pair<>(startState, 0));
    if (act == 0 || act == 3) {
      for (size_t i = 0; i < (size_t)(deltaLength / Constants::dx[act]); i++) {
        State s = result.back().first;
        xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                Constants::dy[act] * sin(-s.yaw);
        ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                Constants::dy[act] * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
        State nextState(xSucc, ySucc, yawSucc, result.back().first.time + 1);
        if (!stateValid(nextState)) return false;
        result.emplace_back(std::make_pair<>(nextState, Constants::dx[0]));
      }
      ratio =
          (deltaLength - static_cast<int>(deltaLength / Constants::dx[act]) *
                             Constants::dx[act]) /
          Constants::dx[act];
      dyaw = 0;
      dx = ratio * Constants::dx[act];
      dy = 0;
    } else {
      for (size_t i = 0; i < (size_t)(deltaSteer / Constants::dyaw[act]); i++) {
        State s = result.back().first;
        xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                Constants::dy[act] * sin(-s.yaw);
        ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                Constants::dy[act] * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
        State nextState(xSucc, ySucc, yawSucc, result.back().first.time + 1);
        if (!stateValid(nextState)) return false;
        result.emplace_back(std::make_pair<>(
            nextState, Constants::dx[0] * Constants::penaltyTurning));
      }
      ratio =
          (deltaSteer - static_cast<int>(deltaSteer / Constants::dyaw[act]) *
                            Constants::dyaw[act]) /
          Constants::dyaw[act];
      dyaw = ratio * Constants::dyaw[act];
      dx = Constants::r * sin(dyaw);
      dy = -Constants::r * (1 - cos(dyaw));
      if (act == 2 || act == 5) {
        dx = -dx;
        dy = -dy;
      }
    }
    State s = result.back().first;
    xSucc = s.x + dx * cos(-s.yaw) - dy * sin(-s.yaw);
    ySucc = s.y + dx * sin(-s.yaw) + dy * cos(-s.yaw);
    yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw);
    State nextState(xSucc, ySucc, yawSucc, result.back().first.time + 1);
    if (!stateValid(nextState)) return false;
    result.emplace_back(std::make_pair<>(nextState, ratio * Constants::dx[0]));

    return true;
  }

 private:
  int m_dimx;
  int m_dimy;
  std::vector<std::vector<std::vector<double>>> holonomic_cost_maps;
  std::unordered_set<Location> m_obstacles;
  std::multimap<int, State> m_dynamic_obstacles;
  std::vector<State> m_goals;
  std::vector<double> m_vel_limit;
  size_t m_agentIdx;
  const Constraints *m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  int m_lowLevelExpandedThisCall;  // [LOG]

  bool m_localGoalActive;  // [LCBR]
  State m_localGoal;       // [LCBR]
};

}  // namespace libMultiRobotPlanning
