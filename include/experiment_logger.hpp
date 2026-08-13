/**
 * @file experiment_logger.hpp
 * @brief Query-level and instance-level logging, as specified in the
 * experimental plan (Sec. "logging scheme"): every low-level call produces
 * one QueryLogRecord row, every solved (or timed-out) instance produces one
 * InstanceLogRecord row. All tables in the paper's Experiments section
 * (Tables I-VI, Figures 1-5) are derivable from these two CSV files alone
 * -- no benchmark re-run should ever be needed to add a new plot.
 */
#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace ugv {

/// One row per low-level SHA* call (local query, full-horizon query, or
/// full-horizon fallback triggered by a failed/inapplicable local query).
struct QueryLogRecord {
  std::string instance_id;
  std::string method;        // "CL-CBS" | "LCBR"
  int bct_node_id = -1;
  size_t robot_id = 0;
  std::string query_type;    // "local" | "full"
  std::string local_outcome; // "success" | "fail" | "inapplicable" | "n/a"
  std::string failure_reason;  // "" | "no_local_path" | "constraint_violation_after_shift"
  int t_conflict = -1;
  int t_minus = -1;
  int t_plus = -1;
  double T_w = 0.0;
  double T_i_current = 0.0;
  double window_fraction = 0.0;  // r_w = T_w / T_i_current
  double T_star = 0.0;
  double delta_T_repair = 0.0;
  int sha_expansions = 0;
  double sha_runtime_s = 0.0;
  bool fallback_triggered = false;
  double g_before = 0.0;
  double g_after = 0.0;
  double path_length_before = 0.0;
  double path_length_after = 0.0;
  double reconnection_residual = 0.0;
  bool junction_correction_skipped = false;
  bool clipped_left = false;
  bool clipped_right = false;

  static std::string header() {
    return "instance_id,method,bct_node_id,robot_id,query_type,local_outcome,"
           "failure_reason,t_conflict,t_minus,t_plus,T_w,T_i_current,"
           "window_fraction,T_star,delta_T_repair,sha_expansions,"
           "sha_runtime_s,fallback_triggered,g_before,g_after,"
           "path_length_before,path_length_after,reconnection_residual,"
           "junction_correction_skipped,clipped_left,clipped_right";
  }

  std::string toCsvRow() const {
    std::ostringstream os;
    os << instance_id << ',' << method << ',' << bct_node_id << ',' << robot_id
       << ',' << query_type << ',' << local_outcome << ',' << failure_reason
       << ',' << t_conflict << ',' << t_minus << ',' << t_plus << ',' << T_w
       << ',' << T_i_current << ',' << window_fraction << ',' << T_star << ','
       << delta_T_repair << ',' << sha_expansions << ',' << sha_runtime_s
       << ',' << (fallback_triggered ? 1 : 0) << ',' << g_before << ','
       << g_after << ',' << path_length_before << ',' << path_length_after
       << ',' << reconnection_residual << ','
       << (junction_correction_skipped ? 1 : 0) << ','
       << (clipped_left ? 1 : 0) << ',' << (clipped_right ? 1 : 0);
    return os.str();
  }
};

/// One row per solved/attempted instance (Table I, II, III, V, VI).
struct InstanceLogRecord {
  std::string instance_id;
  std::string method;              // "CL-CBS" | "LCBR"
  int delta_w_steps = 0;
  int delta_T_steps = 0;
  int num_agents = 0;
  bool success = false;
  double total_runtime_s = 0.0;
  double nominal_gen_runtime_s = 0.0;
  double lap_runtime_s = 0.0;
  double conflict_resolution_runtime_s = 0.0;
  int bct_nodes_expanded = 0;

  // [FIX] Distinguishes "ran out of the --timeout wall-clock budget"
  // from "genuinely exhausted the BCT search space" -- both previously
  // collapsed into success=false with no way to tell them apart. Only
  // meaningful when success == false; left false on a successful run.
  bool timeout = false;

  // N_LL ("total number of actual SHA* calls") must be computed
  // DIFFERENTLY per method from the fields below -- there is no single
  // stored column for it because the correct formula depends on
  // `method`:
  //   N_LL(CL-CBS) = N_full_baseline
  //   N_LL(LCBR)   = N_success + N_fail + N_full_fallback   (NOT just
  //                  N_success + N_fail -- that undercounts by omitting
  //                  the full-horizon fallback calls, which are real
  //                  SHA* calls too)
  // N_branch = N_success + N_fail + N_inapplicable (every branch
  // constraint generated, whether or not a local query was attempted).
  long long N_success = 0;
  long long N_fail = 0;
  long long N_inapplicable = 0;
  long long N_full_baseline = 0;   // full-horizon calls when method == CL-CBS
  long long N_full_fallback = 0;   // full-horizon calls triggered by LCBR fallback

  long long E_tot = 0;             // total SHA* expansions, all calls
  long long E_local_success = 0;
  long long E_local_fail = 0;      // == E_wasted
  long long E_full = 0;            // expansions spent in full-horizon calls (baseline or fallback)

  double SoC_nominal = 0.0;
  double SoC_final = 0.0;
  double L_max_final = 0.0;
  double mission_completion_time = 0.0;

  // Secondary quality metrics (Sec. 5.2 "secondary metrics"): direction
  // changes and backward-travel distance summed over the FINAL solution,
  // i.e. exactly the two quantities CL-CBS's own g(.) penalizes
  // (penaltyCOD / penaltyReversing) but that SoC (physical length,
  // forward and backward weighted equally, Eq. 4) does not distinguish.
  long long N_switch = 0;    // number of forward<->backward transitions
  double D_reverse = 0.0;    // physical distance covered while reversing

  static std::string header() {
    return "instance_id,method,delta_w_steps,delta_T_steps,num_agents,success,"
           "timeout,"
           "total_runtime_s,nominal_gen_runtime_s,lap_runtime_s,"
           "conflict_resolution_runtime_s,bct_nodes_expanded,N_success,N_fail,"
           "N_inapplicable,N_full_baseline,N_full_fallback,E_tot,"
           "E_local_success,E_local_fail,E_full,SoC_nominal,SoC_final,"
           "L_max_final,mission_completion_time,N_switch,D_reverse";
  }

  std::string toCsvRow() const {
    std::ostringstream os;
    os << instance_id << ',' << method << ',' << delta_w_steps << ','
       << delta_T_steps << ',' << num_agents << ',' << (success ? 1 : 0) << ','
       << (timeout ? 1 : 0) << ','
       << total_runtime_s << ',' << nominal_gen_runtime_s << ','
       << lap_runtime_s << ',' << conflict_resolution_runtime_s << ','
       << bct_nodes_expanded << ',' << N_success << ',' << N_fail << ','
       << N_inapplicable << ',' << N_full_baseline << ',' << N_full_fallback
       << ',' << E_tot << ',' << E_local_success << ',' << E_local_fail << ','
       << E_full << ',' << SoC_nominal << ',' << SoC_final << ','
       << L_max_final << ',' << mission_completion_time << ',' << N_switch
       << ',' << D_reverse;
    return os.str();
  }
};

/// Minimal append-with-header-once CSV writer, used for both record types.
class CsvLogWriter {
 public:
  explicit CsvLogWriter(const std::string &path, const std::string &header)
      : m_path(path) {
    // Check file SIZE, not just existence: if a previous process was
    // killed (e.g. OOM -- see hybrid_astar.hpp's expansion cap and its
    // rationale) between opening the file and flushing the header line,
    // the file exists but is empty, and a naive existence check would
    // permanently skip writing the header on every subsequent run.
    std::ifstream probe(path, std::ios::ate);
    bool hasContent = probe.good() && probe.tellg() > 0;
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
    if (m_out.is_open()) m_out.close();
  }

 private:
  std::string m_path;
  std::ofstream m_out;
};

}  // namespace ugv
