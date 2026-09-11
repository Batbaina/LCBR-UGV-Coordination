/**
 * @file experiment_logger.hpp
 * @brief Query-level and instance-level logging for CL-CBS / LCBR
 * experimental validation.
 *
 * Every low-level SHA* call produces one QueryLogRecord row.
 * Every attempted instance produces one InstanceLogRecord row.
 *
 * In addition to the original logging scheme, QueryLogRecord contains
 * optional paired-probe fields used to compare:
 *
 *      Local SHA*  vs  Full-horizon SHA*
 *
 * for the SAME BCT child / conflict / constrained robot / constraint set.
 *
 * IMPORTANT:
 * The paired full-horizon query is diagnostic only. It must not modify
 * the actual BCT search. Its runtime must also not be included in the
 * normal LCBR solver runtime used for the global comparison.
 */

#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace ugv {

// ============================================================================
// QUERY-LEVEL LOGGING
// ============================================================================

/**
 * @brief One row per low-level SHA* call.
 *
 * Normal query types:
 *
 *   local       : LCBR local repair query
 *   full        : CL-CBS full query or LCBR fallback
 *   root_init   : defensive initialization query
 *
 * For an LCBR local query, the optional paired_* fields store the result
 * of the counterfactual full-horizon query evaluated under the SAME
 * BCT-child constraint set.
 */
struct QueryLogRecord {

  // --------------------------------------------------------------------------
  // Identification
  // --------------------------------------------------------------------------

  std::string instance_id;

  // "CL-CBS" | "LCBR"
  std::string method;

  int bct_node_id = -1;

  size_t robot_id = 0;

  // "local" | "full" | "root_init"
  std::string query_type;

  // "success" | "fail" | "inapplicable" | "n/a"
  std::string local_outcome;

  // ""
  // "no_local_path"
  // "constraint_violation_after_shift"
  std::string failure_reason;

  // --------------------------------------------------------------------------
  // Conflict / local-window information
  // --------------------------------------------------------------------------

  int t_conflict = -1;

  int t_minus = -1;

  int t_plus = -1;

  // Original repair-window duration
  double T_w = 0.0;

  // Current complete trajectory duration
  double T_i_current = 0.0;

  // r_w = T_w / T_i_current
  double window_fraction = 0.0;

  // Duration of the new local trajectory
  double T_star = 0.0;

  // T_star - T_w
  double delta_T_repair = 0.0;

  // --------------------------------------------------------------------------
  // Actual SHA* query statistics
  // --------------------------------------------------------------------------

  int sha_expansions = 0;

  double sha_runtime_s = 0.0;

  bool fallback_triggered = false;

  // --------------------------------------------------------------------------
  // Cost / trajectory information
  // --------------------------------------------------------------------------

  double g_before = 0.0;

  double g_after = 0.0;

  double path_length_before = 0.0;

  double path_length_after = 0.0;

  // --------------------------------------------------------------------------
  // Local-repair diagnostics
  // --------------------------------------------------------------------------

  double reconnection_residual = 0.0;

  bool junction_correction_skipped = false;

  bool clipped_left = false;

  bool clipped_right = false;

  // ==========================================================================
  // PAIRED SAME-CONFLICT FULL-HORIZON PROBE
  // ==========================================================================
  //
  // These variables are populated ONLY when:
  //
  //      method == "LCBR"
  //      query_type == "local"
  //      paired-probe experiment is enabled
  //
  // The idea is:
  //
  //       SAME BCT child
  //       SAME robot
  //       SAME accumulated constraint set
  //
  //              |
  //              +---- Local SHA*
  //              |
  //              +---- Full-horizon SHA*
  //
  // The full query is COUNTERFACTUAL:
  //
  //   - its trajectory is NOT inserted into the BCT;
  //   - its cost does NOT modify the BCT node;
  //   - its expansions do NOT enter E_tot of normal LCBR;
  //   - its runtime does NOT enter normal LCBR runtime.
  //
  // It exists only to answer Q3/Q4 experimentally.
  // ==========================================================================

  bool paired_probe_enabled = false;

  // Did the counterfactual full-horizon SHA* query find a solution?
  bool paired_full_success = false;

  // Number of SHA* nodes expanded by the paired full query
  int paired_full_expansions = 0;

  // Runtime of the paired full query alone
  double paired_full_runtime_s = 0.0;

  // SHA* objective g(.) of the paired full trajectory
  double paired_full_g_after = 0.0;

  // Physical trajectory length of the paired full trajectory
  double paired_full_path_length_after = 0.0;

  // Final time / duration of the paired full trajectory
  double paired_full_T_star = 0.0;


  // ==========================================================================
  // CSV HEADER
  // ==========================================================================

  static std::string header() {

    return
        "instance_id,"
        "method,"
        "bct_node_id,"
        "robot_id,"
        "query_type,"
        "local_outcome,"
        "failure_reason,"
        "t_conflict,"
        "t_minus,"
        "t_plus,"
        "T_w,"
        "T_i_current,"
        "window_fraction,"
        "T_star,"
        "delta_T_repair,"
        "sha_expansions,"
        "sha_runtime_s,"
        "fallback_triggered,"
        "g_before,"
        "g_after,"
        "path_length_before,"
        "path_length_after,"
        "reconnection_residual,"
        "junction_correction_skipped,"
        "clipped_left,"
        "clipped_right,"
        "paired_probe_enabled,"
        "paired_full_success,"
        "paired_full_expansions,"
        "paired_full_runtime_s,"
        "paired_full_g_after,"
        "paired_full_path_length_after,"
        "paired_full_T_star";
  }


  // ==========================================================================
  // CSV ROW
  // ==========================================================================

  std::string toCsvRow() const {

    std::ostringstream os;

    os
        << instance_id << ','
        << method << ','
        << bct_node_id << ','
        << robot_id << ','
        << query_type << ','
        << local_outcome << ','
        << failure_reason << ','
        << t_conflict << ','
        << t_minus << ','
        << t_plus << ','
        << T_w << ','
        << T_i_current << ','
        << window_fraction << ','
        << T_star << ','
        << delta_T_repair << ','
        << sha_expansions << ','
        << sha_runtime_s << ','
        << (fallback_triggered ? 1 : 0) << ','
        << g_before << ','
        << g_after << ','
        << path_length_before << ','
        << path_length_after << ','
        << reconnection_residual << ','
        << (junction_correction_skipped ? 1 : 0) << ','
        << (clipped_left ? 1 : 0) << ','
        << (clipped_right ? 1 : 0) << ','

        // Paired full-horizon probe
        << (paired_probe_enabled ? 1 : 0) << ','
        << (paired_full_success ? 1 : 0) << ','
        << paired_full_expansions << ','
        << paired_full_runtime_s << ','
        << paired_full_g_after << ','
        << paired_full_path_length_after << ','
        << paired_full_T_star;

    return os.str();
  }
};


// ============================================================================
// INSTANCE-LEVEL LOGGING
// ============================================================================

/**
 * @brief One row per complete solver execution.
 *
 * This record is used for solver-level comparisons:
 *
 * Q1 : solver success
 * Q2 : local success / fallback frequency
 * Q5 : BCT and low-level interaction
 * Q6 : total computational performance
 * Q7 : final solution quality
 */
struct InstanceLogRecord {

  // --------------------------------------------------------------------------
  // Identification
  // --------------------------------------------------------------------------

  std::string instance_id;

  // "CL-CBS" | "LCBR"
  std::string method;

  int delta_w_steps = 0;

  int delta_T_steps = 0;

  int num_agents = 0;

  // --------------------------------------------------------------------------
  // Solver outcome
  // --------------------------------------------------------------------------

  bool success = false;

  bool timeout = false;
  std::string status = "UNKNOWN";

  // --------------------------------------------------------------------------
  // Runtime
  // --------------------------------------------------------------------------

  double total_runtime_s = 0.0;

  double nominal_gen_runtime_s = 0.0;

  double lap_runtime_s = 0.0;

  double conflict_resolution_runtime_s = 0.0;

  // --------------------------------------------------------------------------
  // High-level BCT effort
  // --------------------------------------------------------------------------

  int bct_nodes_expanded = 0;

  // --------------------------------------------------------------------------
  // Low-level query counts
  // --------------------------------------------------------------------------
  //
  // Correct interpretation:
  //
  // CL-CBS:
  //
  //   N_LL = N_full_baseline
  //
  // LCBR:
  //
  //   N_LL =
  //       N_success
  //     + N_fail
  //     + N_full_fallback
  //
  // An inapplicable local repair does not launch local SHA*, therefore
  // N_inapplicable is NOT itself a SHA* call.
  // --------------------------------------------------------------------------

  long long N_success = 0;

  long long N_fail = 0;

  long long N_inapplicable = 0;

  long long N_full_baseline = 0;

  long long N_full_fallback = 0;

  // --------------------------------------------------------------------------
  // SHA* expansions
  // --------------------------------------------------------------------------

  // Total expansions of all REAL low-level calls made by the solver.
  //
  // IMPORTANT:
  // Counterfactual paired-probe expansions must NOT be included here.
  long long E_tot = 0;

  long long E_local_success = 0;

  // Expansions spent on failed local attempts
  long long E_local_fail = 0;

  // Full-horizon expansions:
  // baseline for CL-CBS or fallback for LCBR
  long long E_full = 0;

  // --------------------------------------------------------------------------
  // Solution quality
  // --------------------------------------------------------------------------

  double SoC_nominal = 0.0;

  double SoC_final = 0.0;

  double L_max_final = 0.0;

  double mission_completion_time = 0.0;

  // --------------------------------------------------------------------------
  // Secondary trajectory-quality metrics
  // --------------------------------------------------------------------------

  long long N_switch = 0;

  double D_reverse = 0.0;

  // ==========================================================================
  // PAIRED-PROBE DIAGNOSTICS
  // ==========================================================================
  //
  // These are kept separately so that the diagnostic experiment can be
  // audited.
  //
  // They MUST NOT be interpreted as normal LCBR computational cost.
  // ==========================================================================

  double paired_probe_runtime_s = 0.0;

  long long paired_probe_calls = 0;


  // ==========================================================================
  // CSV HEADER
  // ==========================================================================

  static std::string header() {

    return
        "instance_id,"
        "method,"
        "delta_w_steps,"
        "delta_T_steps,"
        "num_agents,"
        "success,"
        "timeout,"
        "status,"
        "total_runtime_s,"
        "nominal_gen_runtime_s,"
        "lap_runtime_s,"
        "conflict_resolution_runtime_s,"
        "bct_nodes_expanded,"
        "N_success,"
        "N_fail,"
        "N_inapplicable,"
        "N_full_baseline,"
        "N_full_fallback,"
        "E_tot,"
        "E_local_success,"
        "E_local_fail,"
        "E_full,"
        "SoC_nominal,"
        "SoC_final,"
        "L_max_final,"
        "mission_completion_time,"
        "N_switch,"
        "D_reverse,"
        "paired_probe_runtime_s,"
        "paired_probe_calls";
  }


  // ==========================================================================
  // CSV ROW
  // ==========================================================================

  std::string toCsvRow() const {

    std::ostringstream os;

    os
        << instance_id << ','
        << method << ','
        << delta_w_steps << ','
        << delta_T_steps << ','
        << num_agents << ','
        << (success ? 1 : 0) << ','
        << (timeout ? 1 : 0) << ','
        << status << ','
        << total_runtime_s << ','
        << nominal_gen_runtime_s << ','
        << lap_runtime_s << ','
        << conflict_resolution_runtime_s << ','
        << bct_nodes_expanded << ','
        << N_success << ','
        << N_fail << ','
        << N_inapplicable << ','
        << N_full_baseline << ','
        << N_full_fallback << ','
        << E_tot << ','
        << E_local_success << ','
        << E_local_fail << ','
        << E_full << ','
        << SoC_nominal << ','
        << SoC_final << ','
        << L_max_final << ','
        << mission_completion_time << ','
        << N_switch << ','
        << D_reverse << ','
        << paired_probe_runtime_s << ','
        << paired_probe_calls;

    return os.str();
  }
};


// ============================================================================
// CSV WRITER
// ============================================================================

/**
 * @brief Minimal append-only CSV writer.
 *
 * The header is written only when the destination file is empty.
 */
class CsvLogWriter {

 public:

  explicit CsvLogWriter(
      const std::string &path,
      const std::string &header)
      : m_path(path) {

    std::ifstream probe(path, std::ios::ate);

    bool hasContent =
        probe.good() &&
        probe.tellg() > 0;

    probe.close();

    m_out.open(path, std::ios::app);

    if (!hasContent) {

      m_out << header << "\n";

      m_out.flush();
    }
  }


  void writeRow(const std::string &row) {

    m_out << row << "\n";

    m_out.flush();
  }


  ~CsvLogWriter() {

    if (m_out.is_open()) {

      m_out.close();
    }
  }


 private:

  std::string m_path;

  std::ofstream m_out;
};


}  // namespace ugv