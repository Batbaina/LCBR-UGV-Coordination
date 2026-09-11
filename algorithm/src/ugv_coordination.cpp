/**
 * @file ugv_coordination.cpp
 * @brief Main entry point: Overall UGV Coordination Framework, Algorithm 3
 * of the paper. Ties together:
 *   Stage 1 (nominal trajectory generation, task_allocation.hpp)
 *   Stage 2 (task allocation / Hungarian LAP, task_allocation.hpp)
 *   Stage 3 (conflict resolution: CL-CBS baseline or LCBR,
 *            body_conflict_tree.hpp)
 *
 * State/Location/Constraint/Conflict/Constraints are concrete types
 * defined here exactly as in the original CL-CBS `src/cl_cbs.cpp` (Wen et
 * al.) -- the generic headers (environment.hpp, hybrid_astar.hpp,
 * body_conflict_tree.hpp, local_repair.hpp) are templated over them and do
 * not need to change when these types are defined here rather than there.
 *
 * Usage:
 *   ./ugv_coordination -i map.yaml -o solution.yaml \
 *       --mode {clcbs|lcbr} --delta_w_steps 10 \
 *       --instance-id my_instance --log-dir ./logs
 */
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <tuple>
#include <unordered_set>

#include "body_conflict_tree.hpp"
#include "environment.hpp"
#include "experiment_logger.hpp"
#include "hybrid_astar.hpp"
#include "low_level_environment.hpp"
#include "task_allocation.hpp"
#include "timer.hpp"

using libMultiRobotPlanning::HybridAStar;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::Point;    // only used when PRCISE_COLLISION is defined
using libMultiRobotPlanning::Segment;  // only used when PRCISE_COLLISION is defined

// ============================================================================
// Concrete problem types -- identical in spirit to CL-CBS's src/cl_cbs.cpp.
// Kept in the .cpp (not the generic headers) exactly like the original, so
// that environment.hpp / body_conflict_tree.hpp / local_repair.hpp stay
// fully generic and reusable.
// ============================================================================

// Toggle for exact rectangle-vs-rectangle collision (boost::geometry
// segment intersection) instead of the default fast circumscribed-circle
// test. Restored verbatim from CL-CBS's src/cl_cbs.cpp so both collision
// models remain available to CL-CBS and LCBR identically -- whichever is
// selected applies to both strategies equally, preserving "same everything
// except the low-level query scope".
// #define PRCISE_COLLISION

struct Location {
  Location(double x, double y) : x(x), y(y) {}
  double x;
  double y;

  bool operator<(const Location &other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }
  bool operator==(const Location &other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }
  friend std::ostream &operator<<(std::ostream &os, const Location &c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct State {
  State(double x, double y, double yaw, int time = 0)
      : time(time), x(x), y(y), yaw(yaw) {
    rot.resize(2, 2);
    rot(0, 0) = cos(-this->yaw);
    rot(0, 1) = -sin(-this->yaw);
    rot(1, 0) = sin(-this->yaw);
    rot(1, 1) = cos(-this->yaw);
#ifdef PRCISE_COLLISION
    corner1 = Point(
        this->x -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LB) - this->yaw),
        this->y -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LB) - this->yaw));
    corner2 = Point(
        this->x -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LB) + this->yaw),
        this->y +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LB * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LB) + this->yaw));
    corner3 = Point(
        this->x +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LF) - this->yaw),
        this->y +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LF) - this->yaw));
    corner4 = Point(
        this->x +
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                cos(atan2(Constants::carWidth / 2, Constants::LF) + this->yaw),
        this->y -
            sqrt(pow(Constants::carWidth / 2 * 1.1, 2) +
                 pow(Constants::LF * 1.1, 2)) *
                sin(atan2(Constants::carWidth / 2, Constants::LF) + this->yaw));
#endif
  }
  State() = default;

  bool operator==(const State &s) const {
    return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
  }

  bool agentCollision(const State &other) const {
#ifndef PRCISE_COLLISION
    if (pow(this->x - other.x, 2) + pow(this->y - other.y, 2) <
        pow(2 * Constants::LF, 2) + pow(Constants::carWidth, 2))
      return true;
    return false;
#else
    std::vector<Segment> rectangle1{Segment(this->corner1, this->corner2),
                                    Segment(this->corner2, this->corner3),
                                    Segment(this->corner3, this->corner4),
                                    Segment(this->corner4, this->corner1)};
    std::vector<Segment> rectangle2{Segment(other.corner1, other.corner2),
                                    Segment(other.corner2, other.corner3),
                                    Segment(other.corner3, other.corner4),
                                    Segment(other.corner4, other.corner1)};
    for (auto seg1 = rectangle1.begin(); seg1 != rectangle1.end(); seg1++)
      for (auto seg2 = rectangle2.begin(); seg2 != rectangle2.end(); seg2++) {
        if (boost::geometry::intersects(*seg1, *seg2)) return true;
      }
    return false;
#endif
  }

  bool obsCollision(const Location &obstacle) const {
    boost::numeric::ublas::matrix<double> obs(1, 2);
    obs(0, 0) = obstacle.x - this->x;
    obs(0, 1) = obstacle.y - this->y;
    auto rotated_obs = boost::numeric::ublas::prod(obs, rot);
    if (rotated_obs(0, 0) > -Constants::LB - Constants::obsRadius &&
        rotated_obs(0, 0) < Constants::LF + Constants::obsRadius &&
        rotated_obs(0, 1) > -Constants::carWidth / 2.0 - Constants::obsRadius &&
        rotated_obs(0, 1) < Constants::carWidth / 2.0 + Constants::obsRadius)
      return true;
    return false;
  }

  friend std::ostream &operator<<(std::ostream &os, const State &s) {
    return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")@" << s.time;
  }

  int time;
  double x;
  double y;
  double yaw;

 private:
  boost::numeric::ublas::matrix<double> rot;
  Point corner1, corner2, corner3, corner4;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.yaw);
    return seed;
  }
};
}  // namespace std

using Action = int;

struct Conflict {
  int time;
  size_t agent1;
  size_t agent2;
  State s1;
  State s2;

  friend std::ostream &operator<<(std::ostream &os, const Conflict &c) {
    os << c.time << ": Collision [ " << c.agent1 << c.s1 << " , " << c.agent2
       << c.s2 << " ]";
    return os;
  }
};

struct Constraint {
  Constraint(int time, State s, size_t agentid)
      : time(time), s(s), agentid(agentid) {}
  Constraint() = default;
  int time;
  State s;
  size_t agentid;

  bool operator<(const Constraint &other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) <
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw, other.agentid);
  }
  bool operator==(const Constraint &other) const {
    return std::tie(time, s.x, s.y, s.yaw, agentid) ==
           std::tie(other.time, other.s.x, other.s.y, other.s.yaw, other.agentid);
  }
  friend std::ostream &operator<<(std::ostream &os, const Constraint &c) {
    return os << "Constraint[" << c.time << "," << c.s << "from " << c.agentid
              << "]";
  }

  bool satisfyConstraint(const State &state) const {
    if (state.time < this->time ||
        state.time > this->time + Constants::constraintWaitTime)
      return true;
    return !this->s.agentCollision(state);
  }
};

namespace std {
template <>
struct hash<Constraint> {
  size_t operator()(const Constraint &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.s.x);
    boost::hash_combine(seed, s.s.y);
    boost::hash_combine(seed, s.s.yaw);
    boost::hash_combine(seed, s.agentid);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<Constraint> constraints;
  void add(const Constraints &other) {
    constraints.insert(other.constraints.begin(), other.constraints.end());
  }
  bool overlap(const Constraints &other) {
    for (const auto &c : constraints) {
      if (other.constraints.count(c) > 0) return true;
    }
    return false;
  }
  friend std::ostream &operator<<(std::ostream &os, const Constraints &cs) {
    for (const auto &c : cs.constraints) os << c << std::endl;
    return os;
  }
};

using EnvironmentT =
    libMultiRobotPlanning::Environment<Location, State, Action, double,
                                       Conflict, Constraint, Constraints>;

// ============================================================================
// Vehicle / algorithm configuration (extends the original config.yaml with
// LCBR's single extra parameter, delta_w).
// ============================================================================
void readAgentConfig(const std::string &configPath, int cliDeltaWSteps) {
  YAML::Node car_config;
  try {
    car_config = YAML::LoadFile(configPath);
  } catch (std::exception &e) {
    std::cerr << "\033[1m\033[33mWARNING: Failed to load vehicle config file: "
              << configPath << "\033[0m , Using default params.\n";
  }
  auto getOr = [&](const char *key, double def) {
    return car_config[key] ? car_config[key].as<double>() : def;
  };

  Constants::r = getOr("r", Constants::r);
  Constants::deltat = getOr("deltat", Constants::deltat);
  Constants::penaltyTurning = getOr("penaltyTurning", Constants::penaltyTurning);
  Constants::penaltyReversing = getOr("penaltyReversing", Constants::penaltyReversing);
  Constants::penaltyCOD = getOr("penaltyCOD", Constants::penaltyCOD);
  Constants::mapResolution = getOr("mapResolution", Constants::mapResolution);
  Constants::xyResolution = Constants::r * Constants::deltat;
  Constants::yawResolution = Constants::deltat;
  Constants::carWidth = getOr("carWidth", Constants::carWidth);
  Constants::LF = getOr("LF", Constants::LF);
  Constants::LB = getOr("LB", Constants::LB);
  Constants::obsRadius = getOr("obsRadius", Constants::obsRadius);
  Constants::constraintWaitTime =
      static_cast<int>(getOr("constraintWaitTime", Constants::constraintWaitTime));

  // delta_w: CLI value wins if provided (>0); otherwise read from config;
  // otherwise fall back to the environment.hpp default. Enforce Eq. (11):
  // delta_w >= delta_T + T_s (T_s == 1 step in this discretization).
  int deltaW = cliDeltaWSteps > 0
                  ? cliDeltaWSteps
                  : static_cast<int>(getOr("repairMarginSteps", Constants::repairMarginSteps));
  int minDeltaW = Constants::constraintWaitTime + 1;
  if (deltaW < minDeltaW) {
    std::cerr << "\033[1m\033[33mWARNING: delta_w=" << deltaW
              << " violates delta_w >= delta_T + T_s (min " << minDeltaW
              << "). Clamping.\033[0m\n";
    deltaW = minDeltaW;
  }
  Constants::repairMarginSteps = deltaW;

  Constants::dx = {Constants::r * Constants::deltat,
                   Constants::r * sin(Constants::deltat),
                   Constants::r * sin(Constants::deltat),
                   -Constants::r * Constants::deltat,
                   -Constants::r * sin(Constants::deltat),
                   -Constants::r * sin(Constants::deltat)};
  Constants::dy = {0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat)),
                   0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat))};
  Constants::dyaw = {0, Constants::deltat,  -Constants::deltat,
                     0, -Constants::deltat, Constants::deltat};
}

// ============================================================================
// YAML solution output, format-compatible with the original CL-CBS
// tools/visualize.py.
// ============================================================================
void writeSolutionYaml(std::ostream &out,
                       const std::vector<PlanResult<State, Action, double>> &solution,
                       double runtimeSeconds) {
  double makespan = 0, flowtime = 0, cost = 0;
  for (const auto &s : solution) cost += s.cost;
  for (const auto &s : solution) {
    double current_makespan = 0;
    for (size_t i = 0; i < s.actions.size(); ++i) {
      if (s.actions[i].second < Constants::dx[0])
        current_makespan += s.actions[i].second;
      else if (s.actions[i].first % 3 == 0)
        current_makespan += Constants::dx[0];
      else
        current_makespan += Constants::r * Constants::deltat;
    }
    flowtime += current_makespan;
    if (current_makespan > makespan) makespan = current_makespan;
  }
  out << "statistics:" << std::endl;
  out << "  cost: " << cost << std::endl;
  out << "  makespan: " << makespan << std::endl;
  out << "  flowtime: " << flowtime << std::endl;
  out << "  runtime: " << runtimeSeconds << std::endl;
  out << "schedule:" << std::endl;
  for (size_t a = 0; a < solution.size(); ++a) {
    out << "  agent" << a << ":" << std::endl;
    for (const auto &state : solution[a].states) {
      out << "    - x: " << state.first.x << std::endl
          << "      y: " << state.first.y << std::endl
          << "      yaw: " << state.first.yaw << std::endl
          << "      t: " << state.first.time << std::endl;
    }
  }
}

int main(int argc, char *argv[]) {
  namespace po = boost::program_options;
  std::string inputFile, outputFile, mode, instanceId, logDir, vehicleConfig;
  std::string costMatrixCsv;
  std::string assignmentCsv;
  size_t maxLowLevelExpansions;
  int deltaWSteps;
  int timeoutSeconds;
  // Experimental diagnostic:
  // when enabled, LCBR additionally evaluates a counterfactual
  // full-horizon SHA* query for each applicable local-repair branch.
  // This probe is used only for the paired Local-vs-Full analysis.
  bool pairedProbe;

  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(), "input map/agents YAML")(
      "output,o", po::value<std::string>(&outputFile)->required(), "output solution YAML")(
      "mode,m", po::value<std::string>(&mode)->default_value("lcbr"),
      "conflict-resolution strategy: 'clcbs' (full-horizon baseline) or 'lcbr' (this project's contribution)")(
      "delta_w_steps", po::value<int>(&deltaWSteps)->default_value(0),
      "LCBR repair margin, in T_s steps (0 => read from vehicle config / default)")(
      "timeout", po::value<int>(&timeoutSeconds)->default_value(120),
      "conflict-resolution wall-clock budget, in seconds (Stage 3 only; Stage 1/2 are unbounded)")(
      "paired-probe",po::bool_switch(&pairedProbe)->default_value(false),
      "LCBR diagnostic only: for every applicable local-repair branch, "
      "also solve the same child constraint with a counterfactual "
      "full-horizon SHA* query. The probe result does not modify the BCT.")(
      "instance-id", po::value<std::string>(&instanceId)->default_value(""),
      "identifier written into the experiment logs (defaults to the input filename)")(
      "log-dir", po::value<std::string>(&logDir)->default_value(""),
      "directory for query_log.csv / instance_log.csv (empty => no logging)")(
      "vehicle-config", po::value<std::string>(&vehicleConfig)->default_value("../config/vehicle_config.yaml"),
      "vehicle/algorithm parameter YAML")(
      "cost-matrix-csv", po::value<std::string>(&costMatrixCsv)->default_value(""),
      "optional path to dump the M x Q SHA*-length cost matrix (Eq. 6) as CSV "
      "(robot_id,poi_id,L_ij ; L_ij=-1 if infeasible) -- used by scripts/euclidean_ablation.py")(
      "assignment-csv", po::value<std::string>(&assignmentCsv)->default_value(""),
      "optional path to dump Stage 2's resolved robot->POI assignment as CSV "
      "(robot_id,poi_id,L_ij,feasible)")(
      "max-low-level-expansions", po::value<size_t>(&maxLowLevelExpansions)->default_value(150000),
      "cap on SHA* expansions for a single low-level call (Stage 1 query, LCBR "
      "local query, or full-horizon query); 0 = unbounded (original CL-CBS "
      "behavior, risks OOM on pathological instances -- see README)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error &e) {
    std::cerr << e.what() << "\n\n" << desc << std::endl;
    return 1;
  }

  if (instanceId.empty()) instanceId = inputFile;
  ugv::LowLevelStrategy strategy = (mode == "clcbs")
                                       ? ugv::LowLevelStrategy::FULL_HORIZON
                                       : ugv::LowLevelStrategy::LCBR;
  // The paired probe is meaningful only for LCBR.
  // Silently disable it for the CL-CBS baseline.
  if (strategy == ugv::LowLevelStrategy::FULL_HORIZON &&
      pairedProbe) {

    std::cerr
        << "WARNING: --paired-probe is only applicable to LCBR. "
        << "Disabling paired probe for CL-CBS.\n";

    pairedProbe = false;
  }

  readAgentConfig(vehicleConfig, deltaWSteps);
  Constants::maxLowLevelExpansions = maxLowLevelExpansions;

  YAML::Node map_config;
  try {
    map_config = YAML::LoadFile(inputFile);
  } catch (std::exception &e) {
    std::cerr << "\033[1m\033[31mERROR: Failed to load map file: " << inputFile
              << "\033[0m\n";
    return 1;
  }
  const auto &dim = map_config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  std::unordered_set<Location> obstacles;
  std::vector<State> goals, startStates;
  for (const auto &node : map_config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<double>(), node[1].as<double>()));
  }

  // Two supported instance schemas:
  //   (a) original CL-CBS "agents" schema: each agent owns both a start AND
  //       a goal (Wen et al.'s benchmark). Kept for backward compatibility.
  //   (b) "robots" / "pois" schema: robots and POIs are listed separately,
  //       under a "start" / "goal" key respectively, with no fixed
  //       pairing. This maps directly onto the paper's Sec. 3.2/4.2
  //       formulation (free one-to-one assignment over ALL robots and ALL
  //       POIs, Eq. C1a-C1c) and is in fact the more natural schema for
  //       this pipeline -- Stage 2 was always going to re-assign freely
  //       regardless of how the input groups start/goal pairs.
  if (map_config["robots"] && map_config["pois"]) {
    for (const auto &node : map_config["robots"]) {
      const auto &start = node["start"];
      startStates.emplace_back(
          State(start[0].as<double>(), start[1].as<double>(), start[2].as<double>()));
    }
    for (const auto &node : map_config["pois"]) {
      const auto &goal = node["goal"];
      goals.emplace_back(
          State(goal[0].as<double>(), goal[1].as<double>(), goal[2].as<double>()));
    }
  } else if (map_config["agents"]) {
    for (const auto &node : map_config["agents"]) {
      const auto &start = node["start"];
      const auto &goal = node["goal"];
      startStates.emplace_back(
          State(start[0].as<double>(), start[1].as<double>(), start[2].as<double>()));
      goals.emplace_back(
          State(goal[0].as<double>(), goal[1].as<double>(), goal[2].as<double>()));
    }
  } else {
    std::cerr << "\033[1m\033[31mERROR: " << inputFile
              << " has neither a 'robots'+'pois' section nor an 'agents' "
                 "section -- unrecognized instance schema.\033[0m\n";
    return 1;
  }

  const size_t M = startStates.size();
  const size_t Q = goals.size();
  if (M != Q) {
    std::cerr << "\033[1m\033[31mERROR: this pipeline assumes M == Q "
                 "(one-to-one assignment, paper Sec. 3.2). Got M="
              << M << ", Q=" << Q << ".\033[0m\n";
    return 1;
  }

  std::multimap<int, State> dynamic_obstacles;  // unused at this pipeline
                                                 // stage (single batch); kept
                                                 // for Environment's ctor
                                                 // signature compatibility.

  ugv::InstanceLogRecord instanceLog;
  instanceLog.instance_id = instanceId;
  instanceLog.num_agents = static_cast<int>(M);

  std::unique_ptr<ugv::CsvLogWriter> queryLogger, instanceLogger;
  if (!logDir.empty()) {
    queryLogger.reset(new ugv::CsvLogWriter(logDir + "/query_log.csv",
                                            ugv::QueryLogRecord::header()));
    instanceLogger.reset(new ugv::CsvLogWriter(logDir + "/instance_log.csv",
                                               ugv::InstanceLogRecord::header()));
  }

  Timer totalTimer;

  // ---------------------------------------------------------------------
  // Stage 1: Nominal Trajectory Generation (paper Sec. 4.1). M x Q
  // independent SHA* queries, kinematics + static obstacles only.
  // ---------------------------------------------------------------------
  std::cout << "Stage 1: nominal trajectory generation (" << M << "x" << Q
            << " SHA* queries)...\n";
  Timer nominalTimer;
  std::vector<std::vector<PlanResult<State, Action, double>>> trajCache(
      M, std::vector<PlanResult<State, Action, double>>(Q));
  std::vector<std::vector<double>> costMatrix(
      M, std::vector<double>(Q, ugv::kInfeasibleCost));

  // A single Environment suffices for the whole M x Q sweep: it does not
  // depend on the start state, only on the map, the obstacles and the POI
  // list (whose holonomic costmaps -- one per POI -- are precomputed once
  // in the constructor and then reused by every (i, j) query).
  EnvironmentT nominalEnv(dimx, dimy, obstacles, dynamic_obstacles, goals);
  Constraints emptyConstraints;  // no inter-robot avoidance at Stage 1 (Sec. 4.1)
  for (size_t i = 0; i < M; ++i) {
    for (size_t j = 0; j < Q; ++j) {
      ugv::LowLevelEnvironment<State, Action, double, EnvironmentT, Constraints>
          llenv(nominalEnv, j, emptyConstraints);  // agentIdx=j selects goals[j]
      HybridAStar<State, Action, double,
                 ugv::LowLevelEnvironment<State, Action, double, EnvironmentT,
                                          Constraints>>
          planner(llenv, Constants::maxLowLevelExpansions);
      bool ok = planner.search(startStates[i], trajCache[i][j]);
      if (ok) {
        costMatrix[i][j] = ugv::trajectoryPhysicalLength(trajCache[i][j]);
      }
    }
  }
  nominalTimer.stop();
  instanceLog.nominal_gen_runtime_s = nominalTimer.elapsedSeconds();
  std::cout << "  done in " << nominalTimer.elapsedSeconds() << "s\n";

  if (!costMatrixCsv.empty()) {
    std::ofstream cmOut(costMatrixCsv);
    cmOut << "robot_id,poi_id,L_ij\n";
    for (size_t i = 0; i < M; ++i) {
      for (size_t j = 0; j < Q; ++j) {
        double v = costMatrix[i][j] >= ugv::kInfeasibleCost ? -1.0 : costMatrix[i][j];
        cmOut << i << ',' << j << ',' << v << '\n';
      }
    }
    std::cout << "  cost matrix written to " << costMatrixCsv << "\n";
  }

  // ---------------------------------------------------------------------
  // Stage 2: Multi-UGV Task Allocation (paper Sec. 4.2). Hungarian LAP on
  // physical trajectory length L_ij.
  // ---------------------------------------------------------------------
  std::cout << "Stage 2: task allocation (Hungarian algorithm)...\n";
  Timer lapTimer;
  std::vector<int> assignment = ugv::solveAssignment(costMatrix);
  lapTimer.stop();
  instanceLog.lap_runtime_s = lapTimer.elapsedSeconds();

  if (!assignmentCsv.empty()) {
    std::ofstream asOut(assignmentCsv);
    asOut << "robot_id,poi_id,L_ij,feasible\n";
    for (size_t i = 0; i < M; ++i) {
      int j = assignment[i];
      bool feasiblePair = j >= 0 && costMatrix[i][j] < ugv::kInfeasibleCost;
      double v = feasiblePair ? costMatrix[i][j] : -1.0;
      asOut << i << ',' << j << ',' << v << ',' << (feasiblePair ? 1 : 0) << '\n';
    }
    std::cout << "  assignment written to " << assignmentCsv << "\n";
  }

  bool feasible = true;
  double socNominal = 0.0;
  std::vector<PlanResult<State, Action, double>> gamma0(M);
  std::vector<State> assignedGoals(M);
  for (size_t i = 0; i < M; ++i) {
    int j = assignment[i];
    if (j < 0 || costMatrix[i][j] >= ugv::kInfeasibleCost) {
      feasible = false;
      break;
    }
    gamma0[i] = trajCache[i][j];
    assignedGoals[i] = goals[j];
    socNominal += costMatrix[i][j];
  }
  instanceLog.SoC_nominal = socNominal;

  if (!feasible) {
    std::cerr << "\033[1m\033[31m No feasible perfect matching -- coordination "
                 "instance declared infeasible (Eq. 6).\033[0m\n";
    instanceLog.method = ugv::toString(strategy);
    instanceLog.delta_w_steps = Constants::repairMarginSteps;
    instanceLog.delta_T_steps = Constants::constraintWaitTime;
    instanceLog.success = false;
    instanceLog.timeout = false;
    instanceLog.status ="PRECHECK_INFEASIBLE";
    totalTimer.stop();
    instanceLog.total_runtime_s = totalTimer.elapsedSeconds();
    if (instanceLogger) instanceLogger->writeRow(instanceLog.toCsvRow());
    return 1;
  }
  std::cout << "  done in " << lapTimer.elapsedSeconds() << "s, SoC_nominal="
            << socNominal << "\n";

  // ---------------------------------------------------------------------
  // Stage 3: Conflict resolution -- CL-CBS baseline or LCBR (Sec. 4.3).
  // Both strategies run through the exact same BodyConflictTree loop; only
  // the low-level replanning call differs.
  // ---------------------------------------------------------------------
  std::cout << "Stage 3: conflict resolution [" << ugv::toString(strategy)
            << ", delta_w=" << Constants::repairMarginSteps
            << " steps, timeout=" << timeoutSeconds << "s]...\n";
  EnvironmentT env(dimx, dimy, obstacles, dynamic_obstacles, assignedGoals);
  ugv::BodyConflictTree<
    State,
    Action,
    double,
    Conflict,
    Constraint,
    Constraints,
    EnvironmentT>
    bct(
        env,
        strategy,
        Constants::repairMarginSteps,
        Constants::constraintWaitTime,
        instanceId,
        queryLogger.get(),
        timeoutSeconds,
        pairedProbe);

  std::vector<PlanResult<State, Action, double>> solution;
  bool success = bct.search(startStates, gamma0, solution, instanceLog);
  totalTimer.stop();
  instanceLog.total_runtime_s = totalTimer.elapsedSeconds();

  if (instanceLogger) instanceLogger->writeRow(instanceLog.toCsvRow());

  std::ofstream out(outputFile);
  if (success) {
    std::cout << "\033[1m\033[32m Successfully found a solution! \033[0m\n"
              << "  Total runtime: " << instanceLog.total_runtime_s << "s\n"
              << "  BCT nodes expanded: " << instanceLog.bct_nodes_expanded << "\n"
              << "  SoC_final: " << instanceLog.SoC_final << "\n"
              << "  L_max_final: " << instanceLog.L_max_final << "\n";
    writeSolutionYaml(out, solution, instanceLog.total_runtime_s);
  } else {
    std::cout << "\033[1m\033[31m Failed to resolve conflicts within the time "
                 "budget. \033[0m\n";
  }

  return success ? 0 : 1;
}
