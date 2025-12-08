/*
 * ===========================================================================
 * ENHANCED ZIEGLER-NICHOLS PID TUNING SYSTEM
 * ===========================================================================
 * 
 * This program supports multiple ways to test and optimize PID parameters:
 * 
 * 🔍 MODE 1: ZN AUTO-SEARCH (Automatic Ku/Pu Discovery)
 * -------------------------------------------------------
 * Set in simulation_params.yaml:
 *   ziegler_nichols_tuning:
 *     enable_auto_search: true
 *     auto_search_kp_start: 0.5    # Start testing from this Kp
 *     auto_search_kp_max: 12.0     # Test up to this Kp
 *     auto_search_kp_step: 0.25    # Increment step
 *     simulation_time: 15.0        # Duration of each test
 * 
 * This automatically finds Ultimate Gain (Ku) and Period (Pu), then suggests
 * 6 different ZN methods: Classic, Conservative, LessOvershoot, NoOvershoot, 
 * Aggressive, and ModernZN with calculated Kp, Ki, Kd values.
 * 
 * 🔧 MODE 2: ZN MANUAL TESTING (Single Kp Test)
 * -----------------------------------------------
 * Set in simulation_params.yaml:
 *   ziegler_nichols_tuning:
 *     enable: true
 *     kp_test_value: 3.5     # Test this specific Kp with Ki=Kd=0
 *     simulation_time: 15.0
 * 
 * Tests a single Kp value with Ki and Kd set to zero to observe oscillations.
 * 
 * 🎯 MODE 3: QUICK ZN METHOD TESTING (Code-Level Selection)
 * ----------------------------------------------------------
 * In the code below (around line 680), uncomment ONE of these lines:
 *   // quick_method = "Classic";        // Balanced response
 *   // quick_method = "Conservative";   // Less overshoot  
 *   // quick_method = "LessOvershoot";  // Minimal overshoot
 *   // quick_method = "NoOvershoot";    // No overshoot
 *   // quick_method = "Aggressive";     // Fast response
 *   // quick_method = "ModernZN";       // Modern interpretation
 * 
 * This applies pre-calculated ZN formulas using assumed Ku=6.0, Pu=2.0.
 * Adjust manual_ku and manual_pu values if you know your system's values.
 * 
 * 📊 RECOMMENDED TESTING WORKFLOW:
 * ================================
 * 1. Run AUTO-SEARCH first to find Ku and Pu
 * 2. Copy one of the suggested parameter sets to controller_settings
 * 3. Test different methods by updating YAML and running normal simulation
 * 4. Compare performance metrics (overshoot, settling time, etc.)
 * 5. Choose best method for your application requirements
 * 
 * 💡 METHOD SELECTION GUIDE:
 * ==========================
 * - Classic: Good starting point, moderate overshoot
 * - Conservative: When stability is more important than speed  
 * - LessOvershoot: When overshoot must be minimized
 * - NoOvershoot: Critical systems requiring no overshoot
 * - Aggressive: When fast response is priority
 * - ModernZN: Updated ZN approach for modern control systems
 * 
 * ===========================================================================
 */

#include "../include/agent_control_pkg/config_reader.hpp"
#include "../include/agent_control_pkg/it2_fuzzy_logic_system.hpp"
#include "../include/agent_control_pkg/pid_controller.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// --- START: New includes for file I/O and paths ---
#include <chrono>
#include <filesystem>
// --- END: New includes ---

// --- Start: Local FLS helper structs and functions (specific to this main
// file) ---
// ... (This section remains unchanged, so it's omitted for brevity) ...
struct FuzzySetFOU_Local {
  double l1, l2, l3, u1, u2, u3;
};

struct FuzzyParams_Local {
  std::map<std::string, std::map<std::string, FuzzySetFOU_Local>> sets;
  std::vector<std::array<std::string, 4>> rules;
};

static std::string trim_local(const std::string &s) {
  size_t start = s.find_first_not_of(" \t\r\n");
  size_t end = s.find_last_not_of(" \t\r\n");
  if (start == std::string::npos)
    return "";
  return s.substr(start, end - start + 1);
}

static std::vector<double> parseNumberList_local(const std::string &in) {
  std::vector<double> values;
  std::stringstream ss(in);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    tok = trim_local(tok);
    if (!tok.empty()) {
      try {
        values.push_back(std::stod(tok));
      } catch (const std::exception &e) {
        std::cerr << "Warning: Could not parse number '" << tok
                  << "' in list. Error: " << e.what() << std::endl;
      }
    }
  }
  return values;
}

static std::vector<std::string> parseStringList_local(const std::string &in) {
  std::vector<std::string> values;
  std::stringstream ss(in);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    values.push_back(trim_local(tok));
  }
  return values;
}

static bool loadFuzzyParamsLocal(const std::string &file,
                                 FuzzyParams_Local &fp) {
  std::string actual_path = agent_control_pkg::findConfigFilePath(file);
  std::ifstream in(actual_path);
  if (!in.is_open()) {
    std::cerr << "Could not open fuzzy params file: " << actual_path
              << std::endl;
    return false;
  }
  std::string line;
  std::string section;
  std::string current_var;
  fp.sets.clear();
  fp.rules.clear();

  while (std::getline(in, line)) {
    line = trim_local(line);
    if (line.empty() || line[0] == '#')
      continue;
    if (line == "membership_functions:") {
      section = "mf";
      current_var = "";
      continue;
    }
    if (line == "rules:") {
      section = "rules";
      current_var = "";
      continue;
    }
    if (section == "mf") {
      if (line.length() > 1 && line.back() == ':' &&
          line.find_first_of(" \t") == std::string::npos) {
        current_var = trim_local(line.substr(0, line.size() - 1));
        fp.sets[current_var];
        continue;
      }
      if (current_var.empty()) {
        continue;
      }
      auto pos = line.find(':');
      if (pos == std::string::npos)
        continue;
      std::string setname = trim_local(line.substr(0, pos));
      std::string rest = line.substr(pos + 1);
      auto lb = rest.find('[');
      auto rb = rest.find(']');
      if (lb == std::string::npos || rb == std::string::npos)
        continue;
      std::string nums = rest.substr(lb + 1, rb - lb - 1);
      auto values = parseNumberList_local(nums);
      if (values.size() == 6) {
        fp.sets[current_var][setname] = {values[0], values[1], values[2],
                                         values[3], values[4], values[5]};
      } else {
        std::cerr << "Warning: Fuzzy set '" << setname << "' for var '"
                  << current_var
                  << "' has incorrect number of params. Expected 6, got "
                  << values.size() << std::endl;
      }
    } else if (section == "rules") {
      if (line.rfind("- [", 0) == 0 && line.back() == ']') {
        auto tokens = parseStringList_local(line.substr(3, line.size() - 4));
        if (tokens.size() == 4) {
          fp.rules.push_back({tokens[0], tokens[1], tokens[2], tokens[3]});
        } else {
          std::cerr << "Warning: Fuzzy rule has incorrect number of tokens. "
                       "Expected 4, got "
                    << tokens.size() << " in: " << line << std::endl;
        }
      }
    }
  }
  return true;
}

static void applyFuzzyParamsLocal(agent_control_pkg::IT2FuzzyLogicSystem &fls,
                                  const FuzzyParams_Local &fp) {
  using FOU_Ctrl = agent_control_pkg::IT2FuzzyLogicSystem::IT2TriangularFS_FOU;
  for (const auto &varPair : fp.sets) {
    const std::string &var = varPair.first;
    if (var == "correction")
      fls.addOutputVariable(var);
    else
      fls.addInputVariable(var);
    for (const auto &setPair : varPair.second) {
      const auto &fou_data = setPair.second;
      fls.addFuzzySetToVariable(var, setPair.first,
                                FOU_Ctrl{fou_data.l1, fou_data.l2, fou_data.l3,
                                         fou_data.u1, fou_data.u2,
                                         fou_data.u3});
    }
  }
  for (const auto &r : fp.rules) {
    fls.addRule({{{"error", r[0]}, {"dError", r[1]}, {"wind", r[2]}},
                 {"correction", r[3]}});
  }
}
// --- End: Local FLS helper structs and functions ---

// --- FakeDrone Struct ---
// ... (This section remains unchanged) ...
struct FakeDrone {
  double position_x = 0.0;
  double position_y = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double prev_error_x_fls = 0.0;
  double prev_error_y_fls = 0.0;
  
  // Realistic drone physical parameters
  double mass = 1.5;              // kg - typical quadcopter mass
  double drag_coefficient = 0.1;  // Air resistance coefficient
  double max_acceleration = 15.0; // m/s² - realistic drone acceleration limit
  double response_time_constant = 0.1; // seconds - simulates motor/ESC response delay
  double commanded_accel_x = 0.0; // Commanded acceleration (for response delay)
  double commanded_accel_y = 0.0;

  void update(double accel_cmd_x, double accel_cmd_y, double dt,
              double external_force_x = 0.0, double external_force_y = 0.0) {
    // Clamp acceleration commands to realistic limits
    accel_cmd_x = std::clamp(accel_cmd_x, -max_acceleration, max_acceleration);
    accel_cmd_y = std::clamp(accel_cmd_y, -max_acceleration, max_acceleration);
    
    // Simulate motor/ESC response delay with first-order lag
    double alpha = dt / (response_time_constant + dt);
    commanded_accel_x += alpha * (accel_cmd_x - commanded_accel_x);
    commanded_accel_y += alpha * (accel_cmd_y - commanded_accel_y);
    
    // Calculate drag forces (quadratic air resistance)
    double speed = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
    double drag_force_x = 0.0;
    double drag_force_y = 0.0;
    
    if (speed > 1e-6) {
      double drag_magnitude = drag_coefficient * speed * speed;
      drag_force_x = -drag_magnitude * (velocity_x / speed) / mass;
      drag_force_y = -drag_magnitude * (velocity_y / speed) / mass;
    }
    
    // Apply forces: actual acceleration (after delay) + external forces + drag
    velocity_x += (commanded_accel_x + external_force_x / mass + drag_force_x) * dt;
    velocity_y += (commanded_accel_y + external_force_y / mass + drag_force_y) * dt;
    
    // Apply velocity limits (realistic drone max speed ~20 m/s)
    double max_velocity = 20.0;
    velocity_x = std::clamp(velocity_x, -max_velocity, max_velocity);
    velocity_y = std::clamp(velocity_y, -max_velocity, max_velocity);
    
    // Update positions
    position_x += velocity_x * dt;
    position_y += velocity_y * dt;
  }
};

// --- PerformanceMetrics Struct ---
// ... (This section remains unchanged) ...
struct PerformanceMetrics {
  double peak_value = 0.0;
  double peak_time = 0.0;
  double overshoot_percent = 0.0;
  double settling_time_2percent = -1.0;
  double settling_time_5percent = -1.0;  // More relaxed settling time
  double final_steady_state_error = 0.0; // How close we actually get
  double initial_value_for_metrics = 0.0;
  double target_value_for_metrics = 0.0;
  bool in_settling_band = false;
  bool in_settling_band_5percent = false;
  double time_entered_settling_band = -1.0;
  double time_entered_settling_band_5percent = -1.0;
  bool phase_active_for_metrics = false;

  void reset(double initial_val, double target_val) {
    peak_value = initial_val;
    peak_time = 0.0;
    overshoot_percent = 0.0;
    settling_time_2percent = -1.0;
    settling_time_5percent = -1.0;
    final_steady_state_error = 0.0;
    initial_value_for_metrics = initial_val;
    target_value_for_metrics = target_val;
    in_settling_band = false;
    in_settling_band_5percent = false;
    time_entered_settling_band = -1.0;
    time_entered_settling_band_5percent = -1.0;
    phase_active_for_metrics = true;
  }

  void update_metrics(double current_value, double time_now) {
    if (!phase_active_for_metrics)
      return;

    // Update peak tracking
    if (target_value_for_metrics > initial_value_for_metrics) {
      if (current_value > peak_value) {
        peak_value = current_value;
        peak_time = time_now;
      }
    } else if (target_value_for_metrics < initial_value_for_metrics) {
      if (current_value < peak_value) {
        peak_value = current_value;
        peak_time = time_now;
      }
    } else {
      peak_value = initial_value_for_metrics;
    }

    // Calculate settling tolerances
    double settling_range_abs = std::abs(target_value_for_metrics - initial_value_for_metrics);
    double settling_tolerance_2percent, settling_tolerance_5percent;

    if (settling_range_abs < 1e-3 && std::abs(target_value_for_metrics) > 1e-9) {
      settling_tolerance_2percent = std::abs(target_value_for_metrics * 0.02);
      settling_tolerance_5percent = std::abs(target_value_for_metrics * 0.05);
    } else if (settling_range_abs < 1e-3) {
      settling_tolerance_2percent = 0.02;
      settling_tolerance_5percent = 0.05;
    } else {
      settling_tolerance_2percent = settling_range_abs * 0.02;
      settling_tolerance_5percent = settling_range_abs * 0.05;
    }
    settling_tolerance_2percent = std::max(settling_tolerance_2percent, 1e-4);
    settling_tolerance_5percent = std::max(settling_tolerance_5percent, 1e-4);

    double current_error = std::abs(current_value - target_value_for_metrics);

    // Check 2% settling band with improved hysteresis and stability requirement
    if (current_error <= settling_tolerance_2percent) {
      if (!in_settling_band) {
        time_entered_settling_band = time_now;
        in_settling_band = true;
      }
      // Require stability for at least 0.5 seconds before declaring settled
      if (settling_time_2percent < 0.0 && (time_now - time_entered_settling_band) >= 0.5) {
        settling_time_2percent = time_entered_settling_band;
      }
    } else {
      // Use improved hysteresis: only reset if error exceeds 1.5x tolerance (not 2x)
      if (in_settling_band && current_error > settling_tolerance_2percent * 1.5) {
        settling_time_2percent = -1.0;
        in_settling_band = false;
        time_entered_settling_band = -1.0;
      }
    }

    // Check 5% settling band with improved consistency and stability requirement
    if (current_error <= settling_tolerance_5percent) {
      if (!in_settling_band_5percent) {
        time_entered_settling_band_5percent = time_now;
        in_settling_band_5percent = true;
      }
      // Require stability for at least 0.3 seconds (more lenient than 2% band)
      if (settling_time_5percent < 0.0 && (time_now - time_entered_settling_band_5percent) >= 0.3) {
        settling_time_5percent = time_entered_settling_band_5percent;
      }
    } else {
      // Consistent hysteresis: use 1.5x multiplier like 2% band
      if (in_settling_band_5percent && current_error > settling_tolerance_5percent * 1.5) {
        settling_time_5percent = -1.0;
        in_settling_band_5percent = false;
        time_entered_settling_band_5percent = -1.0;
      }
    }

    // Track final steady state error (continuously updated)
    final_steady_state_error = current_error;
  }

  void finalize_metrics_calculation(double phase_start_time_for_relative_metrics) {
    if (!phase_active_for_metrics)
      return;

    // Calculate overshoot
    if (std::abs(target_value_for_metrics - initial_value_for_metrics) > 1e-6) {
      if (target_value_for_metrics > initial_value_for_metrics) {
        overshoot_percent =
            ((peak_value - target_value_for_metrics) /
             (target_value_for_metrics - initial_value_for_metrics)) *
            100.0;
      } else {
        overshoot_percent =
            ((target_value_for_metrics - peak_value) /
             (initial_value_for_metrics - target_value_for_metrics)) *
            100.0;
      }

      if ((target_value_for_metrics > initial_value_for_metrics &&
           peak_value <= target_value_for_metrics) ||
          (target_value_for_metrics < initial_value_for_metrics &&
           peak_value >= target_value_for_metrics) ||
          (std::abs(peak_value - initial_value_for_metrics) < 1e-6 &&
           std::abs(target_value_for_metrics - initial_value_for_metrics) >
               1e-6)) {
        overshoot_percent = 0.0;
      }
      overshoot_percent = std::max(overshoot_percent, 0.0);
    } else {
      overshoot_percent = 0.0;
    }

    // Finalize settling times
    if (settling_time_2percent < 0.0 && in_settling_band) {
      settling_time_2percent = time_entered_settling_band;
    }
    if (settling_time_5percent < 0.0 && in_settling_band_5percent) {
      settling_time_5percent = time_entered_settling_band_5percent;
    }

    // Adjust times relative to phase start
    if (peak_time >= phase_start_time_for_relative_metrics)
      peak_time -= phase_start_time_for_relative_metrics;
    else
      peak_time = 0;
    
    if (settling_time_2percent >= phase_start_time_for_relative_metrics)
      settling_time_2percent -= phase_start_time_for_relative_metrics;
    else if (settling_time_2percent >= 0.0)
      settling_time_2percent = 0.0;
      
    if (settling_time_5percent >= phase_start_time_for_relative_metrics)
      settling_time_5percent -= phase_start_time_for_relative_metrics;
    else if (settling_time_5percent >= 0.0)
      settling_time_5percent = 0.0;

    // Convert final steady state error to percentage of range
    double range = std::abs(target_value_for_metrics - initial_value_for_metrics);
    if (range > 1e-6) {
      final_steady_state_error = (final_steady_state_error / range) * 100.0;
    } else {
      final_steady_state_error = (final_steady_state_error / std::max(std::abs(target_value_for_metrics), 1.0)) * 100.0;
    }
  }
};

// --- Custom Clamp Function ---
// ... (This section remains unchanged) ...
template <typename T> T clamp(T value, T min_val, T max_val) {
  return std::max(min_val, std::min(value, max_val));
}

// --- NEW: Timestamp for filenames ---
static std::string getCurrentTimestamp() {
  auto now = std::chrono::system_clock::now();
  std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm_struct;
#ifdef _WIN32
  localtime_s(&tm_struct, &tt);
#else
  localtime_r(&tt, &tm_struct);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm_struct, "%Y%m%d_%H%M%S");
  return oss.str();
}

// --- Function to setup default configuration if YAML fails ---
// ... (This section remains unchanged, but you would add the new CSV fields
// here too) ...
agent_control_pkg::SimulationConfig get_default_simulation_config() {
  std::cout << "WARNING: Using internal default simulation parameters."
            << std::endl;
  agent_control_pkg::SimulationConfig defaultConfig;
  defaultConfig.initial_positions.push_back({0.0, 0.0});
  defaultConfig.phases.push_back({{5.0, 0.0}, 0.0});
  defaultConfig.total_time = 15.0;
  // --- START: Add defaults for new params ---
  defaultConfig.csv_enabled = true;
  defaultConfig.output_directory = "output";
  defaultConfig.csv_prefix = "sim_data";
  defaultConfig.metrics_prefix = "metrics_report";
  // --- END: Add defaults for new params ---
  return defaultConfig;
}

// ... (ZN Tuning functions remain unchanged) ...
struct ZN_AnalysisResult {
  bool is_unstable = false;
  bool is_oscillating = false;
  double period = 0.0;
  double overshoot_percent = 0.0;
  double settling_time_2percent = -1.0;
};

ZN_AnalysisResult analyze_for_oscillations(
    const std::vector<std::pair<double, double>> &error_history) {
  std::vector<double> peaks;
  std::vector<double> peak_times;
  const double start_analysis_time = 2.0;

  for (size_t i = 1; i < error_history.size() - 1; ++i) {
    double current_time = error_history[i].first;
    double current_error = error_history[i].second;
    if (current_time < start_analysis_time)
      continue;

    if (std::abs(current_error) > std::abs(error_history[i - 1].second) &&
        std::abs(current_error) > std::abs(error_history[i + 1].second)) {
      if ((current_error > 0 && error_history[i - 1].second <= 0) ||
          (current_error < 0 && error_history[i - 1].second >= 0) ||
          peaks.empty()) {
        peaks.push_back(std::abs(current_error));
        peak_times.push_back(current_time);
      }
    }
  }

  if (peaks.size() < 3) {
    return {false, !peaks.empty(), 0.0};
  }

  double last_peak = peaks.back();
  double second_last_peak = peaks[peaks.size() - 2];

  if (last_peak > second_last_peak) {
    double estimated_period =
        peak_times.back() - peak_times[peak_times.size() - 2];
    return {true, true, estimated_period};
  }

  return {false, true, 0.0};
}

ZN_AnalysisResult
run_single_zn_simulation(double kp_test,
                         const agent_control_pkg::SimulationConfig &config) {
  FakeDrone drone;
  drone.position_x = 0.0;
  agent_control_pkg::PIDController pid_x(kp_test, 0.0, 0.0, -10.0, 10.0, 5.0);

  std::vector<std::pair<double, double>> error_history;
  PerformanceMetrics metrics;
  metrics.reset(0.0, pid_x.getSetpoint());
  double dt = config.dt;

  for (double t = 0; t <= config.zn_tuning_params.simulation_time; t += dt) {
    double error = pid_x.getSetpoint() - drone.position_x;
    error_history.push_back({t, error});
    double cmd = pid_x.calculate(drone.position_x, dt);
    drone.update(cmd, 0.0, dt);
    metrics.update_metrics(drone.position_x, t);
  }

  metrics.finalize_metrics_calculation(0.0);

  ZN_AnalysisResult res = analyze_for_oscillations(error_history);
  res.overshoot_percent = metrics.overshoot_percent;
  res.settling_time_2percent = metrics.settling_time_2percent;
  return res;
}

void run_zn_auto_search(const agent_control_pkg::SimulationConfig &config) {
  std::cout
      << "\n=========== STARTING ZIEGLER-NICHOLS AUTO-SEARCH ===========\n";
  const auto &params = config.zn_tuning_params;
  std::cout << "Searching for Ku with Kp from " << params.auto_search_kp_start
            << " to " << params.auto_search_kp_max << " with step "
            << params.auto_search_kp_step << "\n\n";

  // Prepare output directory and file for the tuning report
  std::filesystem::path zn_dir =
      std::filesystem::path(config.output_directory) / "zn_tuning";
  try {
    std::filesystem::create_directories(zn_dir);
  } catch (const std::exception &e) {
    std::cerr << "Filesystem error creating zn_tuning directory: " << e.what()
              << std::endl;
  }
  std::string timestamp = getCurrentTimestamp();
  std::filesystem::path report_path =
      zn_dir / (std::string("zn_auto_search_") + timestamp + ".txt");

  std::ofstream report_file(report_path);
  if (report_file.is_open()) {
    report_file << "Kp,OvershootPercent,SettlingTime2Percent,Stability\n";
  } else {
    std::cerr << "Could not open ZN report file: " << report_path << std::endl;
  }

  double Ku = -1.0;
  double Pu = -1.0;

  for (double kp = params.auto_search_kp_start; kp <= params.auto_search_kp_max;
       kp += params.auto_search_kp_step) {
    std::cout << "Testing Kp = " << std::fixed << std::setprecision(3) << kp
              << " ... ";
    ZN_AnalysisResult result = run_single_zn_simulation(kp, config);
    std::cout << "OS=" << std::fixed << std::setprecision(1)
              << result.overshoot_percent << "%";
    std::cout << ", ST(2%)=";
    if (result.settling_time_2percent >= 0.0)
      std::cout << result.settling_time_2percent;
    else
      std::cout << "N/A";
    std::cout << "s -> ";

    if (report_file.is_open()) {
      report_file << kp << ',' << std::fixed << std::setprecision(1)
                  << result.overshoot_percent << ',';
      if (result.settling_time_2percent >= 0.0)
        report_file << result.settling_time_2percent;
      else
        report_file << "N/A";
      report_file << ',' << (result.is_unstable ? "Unstable" : "Stable")
                  << '\n';
    }

    if (result.is_unstable) {
      std::cout << "UNSTABLE. Sustained oscillations found.\n";
      Ku = kp;
      Pu = result.period;
      break;
    }
    if (result.is_oscillating) {
      std::cout << "Stable oscillations.\n";
    } else {
      std::cout << "No oscillations.\n";
    }
  }

  std::cout
      << "\n==================== AUTO-SEARCH COMPLETE ====================\n";
  if (Ku > 0 && Pu > 0) {
    std::cout << "Found Ultimate Gain (Ku) = " << Ku << "\n";
    std::cout << "Found Ultimate Period (Pu) = " << Pu << " s\n\n";
    
    // Calculate multiple ZN tuning methods
    struct ZNMethod {
      std::string name;
      double kp_factor;
      double ti_factor; 
      double td_factor;
      std::string description;
    };
    
    std::vector<ZNMethod> methods = {
      {"Classic", 0.6, 0.5, 0.125, "Balanced response (original ZN)"},
      {"Conservative", 0.45, 0.54, 0.15, "Less overshoot, slower response"},
      {"LessOvershoot", 0.33, 0.5, 0.33, "Minimal overshoot method"},  
      {"NoOvershoot", 0.2, 0.5, 0.33, "No overshoot (Tyreus-Luyben)"},
      {"Aggressive", 0.8, 0.4, 0.1, "Fast response, more overshoot"},
      {"ModernZN", 0.5, 0.8, 0.15, "Modern interpretation"}
    };
    
    std::cout << "📊 RECOMMENDED PID GAINS (Multiple Methods):\n";
    std::cout << "==================================================\n";
    
    if (report_file.is_open()) {
      report_file << "\nKu," << Ku << "\nPu," << Pu << '\n';
      report_file << "\nMethod,Kp,Ki,Kd,Description\n";
    }
    
    for (const auto& method : methods) {
      double Ti = method.ti_factor * Pu;
      double Td = method.td_factor * Pu;
      
      double kp = method.kp_factor * Ku;
      double ki = (Ti > 1e-9) ? (kp / Ti) : 0.0;
      double kd = kp * Td;
      
      std::cout << std::setw(15) << method.name << ": ";
      std::cout << "Kp=" << std::fixed << std::setprecision(3) << kp;
      std::cout << ", Ki=" << std::fixed << std::setprecision(3) << ki;  
      std::cout << ", Kd=" << std::fixed << std::setprecision(3) << kd;
      std::cout << " (" << method.description << ")\n";
      
      if (report_file.is_open()) {
        report_file << method.name << "," << kp << "," << ki << "," << kd 
                    << "," << method.description << '\n';
      }
    }
    
    std::cout << "\n🎯 QUICK TEST RECOMMENDATIONS:\n";
    std::cout << "===============================\n";
    std::cout << "1. For STABLE systems: Use 'Conservative' method\n";
    std::cout << "2. For FAST response: Use 'Classic' method\n";
    std::cout << "3. For NO overshoot: Use 'NoOvershoot' method\n";
    std::cout << "4. For MODERN control: Use 'ModernZN' method\n\n";
    
    // Ask user if they want to test one of the methods automatically
    std::cout << "🚀 AUTO-TEST OPTION:\n";
    std::cout << "Do you want to automatically test one of these methods? (y/n): ";
    
    // For automated testing, we'll use the Classic method
    // In a real interactive version, you could add user input here
    char auto_test = 'n'; // Default to no auto-test for batch processing
    
    // Uncomment these lines for interactive mode:
    // std::cin >> auto_test;
    // if (auto_test == 'y' || auto_test == 'Y') {
    if (false) { // Change to true if you want auto-testing
      std::cout << "Which method to test? (1=Classic, 2=Conservative, 3=NoOvershoot): ";
      int method_choice = 1; // Default to Classic
      // std::cin >> method_choice;
      
      if (method_choice >= 1 && method_choice <= 3) {
        auto selected_method = methods[method_choice - 1];
        
        double Ti = selected_method.ti_factor * Pu;
        double Td = selected_method.td_factor * Pu;
        double test_kp = selected_method.kp_factor * Ku;
        double test_ki = (Ti > 1e-9) ? (test_kp / Ti) : 0.0;
        double test_kd = test_kp * Td;
        
        std::cout << "\n🧪 TESTING " << selected_method.name << " METHOD:\n";
        std::cout << "Kp=" << test_kp << ", Ki=" << test_ki << ", Kd=" << test_kd << "\n";
        
        // Create a modified config for testing
        auto test_config = config;
        test_config.pid_params.kp = test_kp;
        test_config.pid_params.ki = test_ki;
        test_config.pid_params.kd = test_kd;
        test_config.zn_tuning_params.enable = false;
        test_config.zn_tuning_params.enable_auto_search = false;
        
        // Run a test simulation with these parameters
        std::cout << "Running test simulation...\n";
        // This would require refactoring main() into a testable function
        // For now, just show the parameters
        
        std::cout << "✅ Test parameters calculated. Update your YAML file with:\n";
        std::cout << "controller_settings:\n";
        std::cout << "  pid:\n";
        std::cout << "    kp: " << test_kp << "\n";
        std::cout << "    ki: " << test_ki << "\n";
        std::cout << "    kd: " << test_kd << "\n";
      }
    }
    
    std::cout << "\n📝 To use these gains:\n";
    std::cout << "1. Copy one of the parameter sets above\n";
    std::cout << "2. Update 'controller_settings' in simulation_params.yaml\n";
    std::cout << "3. Set both ZN enable flags to false\n";
    std::cout << "4. Run simulation again to test performance\n";

  } else {
    std::cout << "❌ Could not find Ku within the specified Kp range.\n";
    std::cout << "💡 TROUBLESHOOTING:\n";
    std::cout << "- Try increasing 'auto_search_kp_max' (currently " << params.auto_search_kp_max << ")\n";
    std::cout << "- Try decreasing 'auto_search_kp_step' (currently " << params.auto_search_kp_step << ")\n";
    std::cout << "- Try increasing 'simulation_time' (currently " << params.simulation_time << "s)\n";
    std::cout << "- Check if your system is too stable or has too much damping\n";
    
    if (report_file.is_open()) {
      report_file << "\nKu,Not found\nPu,Not found\n";
      report_file << "Troubleshooting,Increase kp_max or decrease kp_step\n";
    }
  }
  std::cout << "============================================================\n";
  if (report_file.is_open()) {
    report_file.close();
  }
}

int main() {
  // --- Load Configuration ---
  agent_control_pkg::SimulationConfig config;
  bool config_loaded_successfully = false;
  try {
    config =
        agent_control_pkg::ConfigReader::loadConfig("simulation_params.yaml");
    config_loaded_successfully = true;
  } catch (const std::exception &e) {
    std::cerr << "Error during configuration loading: " << e.what()
              << std::endl;
  }
  if (!config_loaded_successfully) {
    config = get_default_simulation_config();
  }

  // --- MODE SELECTION ---
  if (config.zn_tuning_params.enable_auto_search) {
    run_zn_auto_search(config);
    return 0; // Exit after auto-search
  }

  const bool ZN_TUNING_ACTIVE = config.zn_tuning_params.enable;
  const double ZN_KP_TEST_VALUE = config.zn_tuning_params.kp_test_value;
  const double ZN_SIMULATION_TIME = config.zn_tuning_params.simulation_time;

  FuzzyParams_Local fls_yaml_params_local;
  bool fls_should_be_enabled = config.enable_fls && !ZN_TUNING_ACTIVE;
  if (fls_should_be_enabled) {
    if (!loadFuzzyParamsLocal(config.fuzzy_params_file,
                              fls_yaml_params_local)) {
      std::cerr << "Failed to load FLS params. Disabling FLS for this run."
                << std::endl;
      fls_should_be_enabled = false;
    }
  }

  // --- CHECK FOR QUICK ZN METHOD TESTING ---
  // Now reading from YAML configuration instead of hardcoded values
  std::string quick_method = config.zn_tuning_params.quick_method_test;
  double manual_ku = config.zn_tuning_params.manual_ku;
  double manual_pu = config.zn_tuning_params.manual_pu;
  
  bool using_quick_zn_method = false;
  double kp_actual, ki_actual, kd_actual;
  
  if (ZN_TUNING_ACTIVE) {
    kp_actual = ZN_KP_TEST_VALUE;
    ki_actual = 0.0;
    kd_actual = 0.0;
    std::cout << "🔧 ZN Manual Testing Mode: Kp=" << kp_actual << ", Ki=0, Kd=0\n";
  } else {
    // Check if user wants to use a quick ZN method from YAML config
    
    if (quick_method != "Off") {
      using_quick_zn_method = true;
      
      // ZN method definitions (same as in auto-search)
      struct ZNMethod {
        std::string name;
        double kp_factor;
        double ti_factor; 
        double td_factor;
        std::string description;
      };
      
      std::vector<ZNMethod> methods = {
        {"Classic", 0.6, 0.5, 0.125, "Balanced response (original ZN)"},
        {"Conservative", 0.45, 0.54, 0.15, "Less overshoot, slower response"},
        {"LessOvershoot", 0.33, 0.5, 0.33, "Minimal overshoot method"},  
        {"NoOvershoot", 0.2, 0.5, 0.33, "No overshoot (Tyreus-Luyben)"},
        {"Aggressive", 0.8, 0.4, 0.1, "Fast response, more overshoot"},
        {"ModernZN", 0.5, 0.8, 0.15, "Modern interpretation"}
      };
      
      // Find the selected method
      auto method_it = std::find_if(methods.begin(), methods.end(),
        [&quick_method](const ZNMethod& m) { return m.name == quick_method; });
      
      if (method_it != methods.end()) {
        double Ti = method_it->ti_factor * manual_pu;
        double Td = method_it->td_factor * manual_pu;
        
        kp_actual = method_it->kp_factor * manual_ku;
        ki_actual = (Ti > 1e-9) ? (kp_actual / Ti) : 0.0;
        kd_actual = kp_actual * Td;
        
        std::cout << "🎯 QUICK ZN METHOD ACTIVE: " << method_it->name << "\n";
        std::cout << "   Using Ku=" << manual_ku << ", Pu=" << manual_pu << "\n";
        std::cout << "   Calculated PID: Kp=" << std::fixed << std::setprecision(3) << kp_actual;
        std::cout << ", Ki=" << ki_actual << ", Kd=" << kd_actual << "\n";
        std::cout << "   Description: " << method_it->description << "\n\n";
      } else {
        std::cout << "⚠️  Unknown ZN method '" << quick_method << "', using regular PID settings\n";
        using_quick_zn_method = false;
      }
    }
    
    if (!using_quick_zn_method) {
      // Use regular PID parameters from config
      kp_actual = config.pid_params.kp;
      ki_actual = config.pid_params.ki;
      kd_actual = config.pid_params.kd;
    }
  }

  if (config.phases.empty() && !ZN_TUNING_ACTIVE) {
    std::cerr << "Warning: No phases defined for a normal run. Exiting."
              << std::endl;
    return 1;
  }

  const int NUM_DRONES = config.num_drones > 0 ? config.num_drones : 1;
  double dt = config.dt;
  double simulation_time_actual =
      ZN_TUNING_ACTIVE ? ZN_SIMULATION_TIME : config.total_time;
  bool ENABLE_WIND_ACTUAL = config.wind_enabled && !ZN_TUNING_ACTIVE;
  bool USE_FLS_ACTUAL = fls_should_be_enabled;

  std::vector<FakeDrone> drones(NUM_DRONES);
  if (config.initial_positions.size() >= (size_t)NUM_DRONES) {
    for (int i = 0; i < NUM_DRONES; ++i) {
      drones[i].position_x = config.initial_positions[i].first;
      drones[i].position_y = config.initial_positions[i].second;
    }
  } else {
    std::cerr << "Warning: Insufficient initial_positions. Using default "
                 "staggered positions."
              << std::endl;
    for (int i = 0; i < NUM_DRONES; ++i) {
      drones[i].position_x = static_cast<double>(i) * 1.0;
      drones[i].position_y = 0.0;
    }
  }

  std::vector<agent_control_pkg::PIDController> pid_x_controllers;
  std::vector<agent_control_pkg::PIDController> pid_y_controllers;
  std::vector<agent_control_pkg::IT2FuzzyLogicSystem> fls_x_controllers_vec(
      NUM_DRONES);
  std::vector<agent_control_pkg::IT2FuzzyLogicSystem> fls_y_controllers_vec(
      NUM_DRONES);

  for (int i = 0; i < NUM_DRONES; ++i) {
    // Create PID controllers with axis-specific limits and enhanced features
    pid_x_controllers.emplace_back(
        kp_actual, ki_actual, kd_actual, 
        config.pid_params.output_min_x, config.pid_params.output_max_x, 
        drones[i].position_x, 
        config.pid_params.enable_derivative_filter, config.pid_params.derivative_filter_alpha);
    pid_y_controllers.emplace_back(
        kp_actual, ki_actual, kd_actual, 
        config.pid_params.output_min_y, config.pid_params.output_max_y, 
        drones[i].position_y,
        config.pid_params.enable_derivative_filter, config.pid_params.derivative_filter_alpha);
    
    // Enable feed-forward control if configured
    if (config.pid_params.enable_feedforward) {
      pid_x_controllers[i].enableFeedForward(true, 
        config.pid_params.velocity_feedforward_gain, 
        config.pid_params.acceleration_feedforward_gain);
      pid_y_controllers[i].enableFeedForward(true, 
        config.pid_params.velocity_feedforward_gain, 
        config.pid_params.acceleration_feedforward_gain);
    }
    
    if (USE_FLS_ACTUAL) {
      applyFuzzyParamsLocal(fls_x_controllers_vec[i], fls_yaml_params_local);
      applyFuzzyParamsLocal(fls_y_controllers_vec[i], fls_yaml_params_local);
    }
  }

  std::vector<std::pair<double, double>> formation_offsets(NUM_DRONES);
  if (NUM_DRONES > 0)
    formation_offsets[0] = {0.0,
                            (sqrt(3.0) / 3.0) * config.formation_side_length};
  if (NUM_DRONES > 1)
    formation_offsets[1] = {-config.formation_side_length / 2.0,
                            -(sqrt(3.0) / 6.0) * config.formation_side_length};
  if (NUM_DRONES > 2)
    formation_offsets[2] = {config.formation_side_length / 2.0,
                            -(sqrt(3.0) / 6.0) * config.formation_side_length};
  for (int i = 3; i < NUM_DRONES; ++i)
    formation_offsets[i] = {0.0, 0.0};

  const int MAX_PHASES_FROM_CONFIG = static_cast<int>(config.phases.size());
  int current_phase_idx = -1;
  int num_metric_phases = std::max(1, MAX_PHASES_FROM_CONFIG);
  std::vector<std::vector<PerformanceMetrics>> drone_metrics_x(
      NUM_DRONES, std::vector<PerformanceMetrics>(num_metric_phases));
  std::vector<std::vector<PerformanceMetrics>> drone_metrics_y(
      NUM_DRONES, std::vector<PerformanceMetrics>(num_metric_phases));
  std::vector<bool> phase_has_been_active(num_metric_phases, false);
  std::vector<double> phase_actual_start_times(num_metric_phases, 0.0);

  // --- START: MODIFIED SECTION for File Naming & Output ---
  std::string timestamp = getCurrentTimestamp();
  std::string base_filename_prefix_val =
      ZN_TUNING_ACTIVE ? "zn_test" : config.csv_prefix;
  std::string directory_path_str = config.output_directory;
  if (ZN_TUNING_ACTIVE) {
    directory_path_str += "/zn_tuning";
  }
  std::filesystem::path output_dir_path(directory_path_str);
  try {
    std::filesystem::create_directories(output_dir_path);
  } catch (const std::exception &e) {
    std::cerr << "Filesystem error creating directory: " << e.what()
              << std::endl;
    output_dir_path = "."; // Fallback to current directory
  }

  std::ostringstream oss_filename_suffix;
  if (ZN_TUNING_ACTIVE) {
    oss_filename_suffix << "_Kp" << std::fixed << std::setprecision(3)
                        << ZN_KP_TEST_VALUE;
  } else {
    // Check if we're using a ZN method for quick testing
    if (!config.zn_tuning_params.quick_method_test.empty() && 
        config.zn_tuning_params.quick_method_test != "Off") {
      oss_filename_suffix << "_ZN_" << config.zn_tuning_params.quick_method_test;
    }
    oss_filename_suffix << "_Kp" << std::fixed << std::setprecision(3)
                        << kp_actual << "_Ki" << std::fixed
                        << std::setprecision(3) << ki_actual << "_Kd"
                        << std::fixed << std::setprecision(3) << kd_actual;
  }
  oss_filename_suffix << (USE_FLS_ACTUAL ? "_FLS_ON" : "_FLS_OFF")
                      << (ENABLE_WIND_ACTUAL ? "_WIND_ON" : "_WIND_OFF") << "_"
                      << timestamp;

  std::filesystem::path csv_filepath =
      output_dir_path /
      (base_filename_prefix_val + oss_filename_suffix.str() + ".csv");
  std::filesystem::path metrics_filepath =
      output_dir_path /
      ((ZN_TUNING_ACTIVE ? "zn_metrics_info" : config.metrics_prefix) +
       oss_filename_suffix.str() + ".txt");

  std::ofstream csv_file;
  if (config.csv_enabled) {
    csv_file.open(csv_filepath);
    if (csv_file.is_open()) {
      csv_file << "Time";
      for (int i = 0; i < NUM_DRONES; ++i) {
        csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i
                 << ",PIDOutX" << i << ",FLSCorrX" << i << ",FinalCmdX" << i
                 << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i
                 << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i
                 << ",PIDOutY" << i << ",FLSCorrY" << i << ",FinalCmdY" << i
                 << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i;
      }
      if (ENABLE_WIND_ACTUAL)
        csv_file << ",SimWindX,SimWindY";
      csv_file << "\n";
    } else {
      std::cerr << "Error opening CSV file: " << csv_filepath.string()
                << std::endl;
    }
  }

  std::ofstream metrics_file_stream;
  if (config.metrics_enabled && !ZN_TUNING_ACTIVE) {
    metrics_file_stream.open(metrics_filepath);
    if (metrics_file_stream.is_open()) {
      metrics_file_stream << "Simulation Metrics for: "
                          << (base_filename_prefix_val +
                              oss_filename_suffix.str())
                          << "\n";
      metrics_file_stream << "PID Gains: Kp=" << kp_actual
                          << ", Ki=" << ki_actual << ", Kd=" << kd_actual
                          << "\n";
      metrics_file_stream << "FLS: " << (USE_FLS_ACTUAL ? "ON" : "OFF")
                          << ", Wind: " << (ENABLE_WIND_ACTUAL ? "ON" : "OFF")
                          << "\n";
      metrics_file_stream << "================================================="
                             "======================\n";
    }
  } else if (ZN_TUNING_ACTIVE) {
    metrics_file_stream.open(metrics_filepath);
    if (metrics_file_stream.is_open()) {
      metrics_file_stream << "Ziegler-Nichols Tuning Run Information\n";
      metrics_file_stream << "Timestamp: " << timestamp << "\n";
      metrics_file_stream << "Kp_test_value: " << ZN_KP_TEST_VALUE << "\n";
      metrics_file_stream << "Simulation Duration: " << ZN_SIMULATION_TIME
                          << "s\n";
      metrics_file_stream << "CSV Data File: "
                          << csv_filepath.filename().string() << "\n";
      metrics_file_stream << "Objective: Observe Drone 0 X-axis for sustained "
                             "oscillations after first setpoint change.\n";
    }
  }
  // --- END: MODIFIED SECTION ---

  // --- Simulation Start & Main Loop ---
  std::cout << "Starting simulation..." << std::endl;
  if (ZN_TUNING_ACTIVE) {
    std::cout << "!!!!!!!! ZIEGLER-NICHOLS Ku/Pu FINDING MODE ACTIVE (from "
                 "config) !!!!!!!!"
              << std::endl;
    std::cout << "Testing with Kp = " << ZN_KP_TEST_VALUE << ", Ki = 0, Kd = 0"
              << std::endl;
  } else {
    std::cout << "Running with configured PID gains: Kp=" << kp_actual
              << ", Ki=" << ki_actual << ", Kd=" << kd_actual << std::endl;
  }
  std::cout << "FLS: " << (USE_FLS_ACTUAL ? "ON" : "OFF")
            << ", Wind: " << (ENABLE_WIND_ACTUAL ? "ON" : "OFF") << std::endl;
  if (config.csv_enabled && csv_file.is_open())
    std::cout << "CSV Output: " << csv_filepath.string() << std::endl;
  if (metrics_file_stream.is_open())
    std::cout << "Metrics/Info Output: " << metrics_filepath.string()
              << std::endl;

  double simulated_wind_x = 0.0;
  double simulated_wind_y = 0.0;
  double last_console_print_time = -config.console_update_interval;

  if (ZN_TUNING_ACTIVE && config.phases.empty()) {
    config.phases.push_back({{5.0, 0.0}, 0.0});
  }

  for (double time_now = 0.0; time_now <= simulation_time_actual + dt / 2.0;
       time_now += dt) {
    int new_phase_idx_candidate = current_phase_idx;
    if (!config.phases.empty()) {
      for (int p_cfg_idx = (int)config.phases.size() - 1; p_cfg_idx >= 0;
           --p_cfg_idx) {
        if (time_now >= config.phases[p_cfg_idx].start_time - dt / 2.0) {
          new_phase_idx_candidate = p_cfg_idx;
          break;
        }
      }
    }

    if (new_phase_idx_candidate != current_phase_idx) {
      current_phase_idx = new_phase_idx_candidate;
      phase_has_been_active[current_phase_idx] = true;
      phase_actual_start_times[current_phase_idx] = time_now;

      std::cout << "Time: " << std::fixed << std::setprecision(1) << time_now
                << "s - Activating PHASE " << current_phase_idx + 1;

      double formation_center_x = config.phases[current_phase_idx].center[0];
      double formation_center_y = config.phases[current_phase_idx].center[1];
      std::cout << " (Target Center: " << formation_center_x << ", "
                << formation_center_y << ")" << std::endl;

      for (int i = 0; i < NUM_DRONES; ++i) {
        double target_x = formation_center_x + formation_offsets[i].first;
        double target_y = formation_center_y + formation_offsets[i].second;

        pid_x_controllers[i].setSetpoint(target_x);
        pid_y_controllers[i].setSetpoint(target_y);
        pid_x_controllers[i].reset();
        pid_y_controllers[i].reset();

        drones[i].prev_error_x_fls = 0.0;
        drones[i].prev_error_y_fls = 0.0;

        drone_metrics_x[i][current_phase_idx].reset(drones[i].position_x,
                                                    target_x);
        drone_metrics_y[i][current_phase_idx].reset(drones[i].position_y,
                                                    target_y);
      }
    }

    simulated_wind_x = 0.0;
    simulated_wind_y = 0.0;
    if (ENABLE_WIND_ACTUAL && current_phase_idx >= 0) {
      for (const auto &wp_cfg : config.wind_phases) {
        if (wp_cfg.phase_number == (current_phase_idx + 1)) {
          for (const auto &tw : wp_cfg.time_windows) {
            double time_in_current_phase =
                time_now - phase_actual_start_times[current_phase_idx];
            if (time_in_current_phase >= tw.start_time &&
                time_in_current_phase < tw.end_time) {
              if (tw.is_sine_wave) {
                if (!tw.force.empty())
                  simulated_wind_x +=
                      tw.force[0] * sin(time_now * tw.sine_frequency_rad_s);
                if (tw.force.size() > 1)
                  simulated_wind_y +=
                      tw.force[1] * sin(time_now * tw.sine_frequency_rad_s);
              } else {
                if (!tw.force.empty())
                  simulated_wind_x += tw.force[0];
                if (tw.force.size() > 1)
                  simulated_wind_y += tw.force[1];
              }
            }
          }
        }
      }
    }

    // --- START: MODIFIED - Write to CSV file ---
    if (config.csv_enabled && csv_file.is_open())
      csv_file << time_now;
    // --- END: MODIFIED ---

    for (int i = 0; i < NUM_DRONES; ++i) {
      double error_x =
          pid_x_controllers[i].getSetpoint() - drones[i].position_x;
      double error_y =
          pid_y_controllers[i].getSetpoint() - drones[i].position_y;
      double d_error_x_fls =
          (dt > 1e-9) ? (error_x - drones[i].prev_error_x_fls) / dt : 0.0;
      double d_error_y_fls =
          (dt > 1e-9) ? (error_y - drones[i].prev_error_y_fls) / dt : 0.0;

      auto terms_x =
          pid_x_controllers[i].calculate_with_terms(drones[i].position_x, dt);
      auto terms_y =
          pid_y_controllers[i].calculate_with_terms(drones[i].position_y, dt);

      double fls_correction_x = 0.0;
      double fls_correction_y = 0.0;
      if (USE_FLS_ACTUAL) {
        fls_correction_x = fls_x_controllers_vec[i].calculateOutput(
            error_x, d_error_x_fls, simulated_wind_x);
        fls_correction_y = fls_y_controllers_vec[i].calculateOutput(
            error_y, d_error_y_fls, simulated_wind_y);
      }

      double final_cmd_x =
          clamp(terms_x.total_output + fls_correction_x,
                config.pid_params.output_min, config.pid_params.output_max);
      double final_cmd_y =
          clamp(terms_y.total_output + fls_correction_y,
                config.pid_params.output_min, config.pid_params.output_max);

      drones[i].update(final_cmd_x, final_cmd_y, dt, simulated_wind_x,
                       simulated_wind_y);
      drones[i].prev_error_x_fls = error_x;
      drones[i].prev_error_y_fls = error_y;

      if (current_phase_idx >= 0 && current_phase_idx < num_metric_phases) {
        drone_metrics_x[i][current_phase_idx].update_metrics(
            drones[i].position_x, time_now);
        drone_metrics_y[i][current_phase_idx].update_metrics(
            drones[i].position_y, time_now);
      }
      // --- START: MODIFIED - Write drone data to CSV ---
      if (config.csv_enabled && csv_file.is_open()) {
        csv_file << "," << pid_x_controllers[i].getSetpoint() << ","
                 << drones[i].position_x << "," << error_x << ","
                 << terms_x.total_output << "," << fls_correction_x << ","
                 << final_cmd_x << "," << terms_x.p << "," << terms_x.i << ","
                 << terms_x.d << "," << pid_y_controllers[i].getSetpoint()
                 << "," << drones[i].position_y << "," << error_y << ","
                 << terms_y.total_output << "," << fls_correction_y << ","
                 << final_cmd_y << "," << terms_y.p << "," << terms_y.i << ","
                 << terms_y.d;
      }
      // --- END: MODIFIED ---
    }
    // --- START: MODIFIED - Write wind data and newline to CSV ---
    if (config.csv_enabled && csv_file.is_open() && ENABLE_WIND_ACTUAL)
      csv_file << "," << simulated_wind_x << "," << simulated_wind_y;
    if (config.csv_enabled && csv_file.is_open())
      csv_file << "\n";
    // --- END: MODIFIED ---

    if (config.console_output_enabled &&
        (time_now - last_console_print_time >=
         config.console_update_interval - dt / 2.0)) {
      if (NUM_DRONES > 0) {
        std::cout << "T=" << std::fixed << std::setprecision(1) << time_now
                  << " Ph:"
                  << (current_phase_idx >= 0 ? current_phase_idx + 1 : 0)
                  << " D0_Pos:(" << std::fixed << std::setprecision(2)
                  << drones[0].position_x << "," << drones[0].position_y << ")";
        if (ENABLE_WIND_ACTUAL) {
          std::cout << " Wind:(" << std::fixed << std::setprecision(2)
                    << simulated_wind_x << "," << simulated_wind_y << ")";
        }
        if (ZN_TUNING_ACTIVE) {
          double zn_error_x =
              pid_x_controllers[0].getSetpoint() - drones[0].position_x;
          std::cout << " D0_ErrX: " << std::fixed << std::setprecision(3)
                    << zn_error_x;
        }
        std::cout << std::endl;
      }
      last_console_print_time = time_now;
    }
  }
  // --- START: MODIFIED - Close file handles ---
  if (csv_file.is_open())
    csv_file.close();
  // --- END: MODIFIED ---

  // --- Finalize and Print/Save Metrics ---
  std::cout << "\n--- FINAL PERFORMANCE METRICS ---" << std::endl;
  if (ZN_TUNING_ACTIVE) {
    std::cout << "Z-N Tuning Run (Kp_test = " << ZN_KP_TEST_VALUE
              << ") complete." << std::endl;
    std::cout << "Analyze CSV (" << csv_filepath.string()
              << ") for Drone 0 X-axis oscillations." << std::endl;
  }
  if (num_metric_phases > 0) {
    for (int p_idx = 0; p_idx < num_metric_phases; ++p_idx) {
      if (!phase_has_been_active[p_idx])
        continue;
      if (metrics_file_stream.is_open())
        metrics_file_stream << "\n-- METRICS FOR PHASE " << p_idx + 1
                            << " --\n";
      std::cout << "\n-- METRICS FOR PHASE " << p_idx + 1 << " --" << std::endl;

      if (p_idx < MAX_PHASES_FROM_CONFIG) {
        std::string phase_info =
            " Target Center: (" +
            std::to_string(config.phases[p_idx].center[0]) + ", " +
            std::to_string(config.phases[p_idx].center[1]) + ")\n";
        if (metrics_file_stream.is_open())
          metrics_file_stream << phase_info;
        std::cout << phase_info;
      }

      for (int i = 0; i < NUM_DRONES; ++i) {
        if (drone_metrics_x[i][p_idx].phase_active_for_metrics) {
          drone_metrics_x[i][p_idx].finalize_metrics_calculation(
              phase_actual_start_times[p_idx]);
          drone_metrics_y[i][p_idx].finalize_metrics_calculation(
              phase_actual_start_times[p_idx]);

          std::ostringstream oss;
          oss << " Drone " << i << ":\n";
          oss << "  X-axis: OS=" << std::fixed << std::setprecision(1)
              << drone_metrics_x[i][p_idx].overshoot_percent << "%"
              << ", ST(2%)="
              << (drone_metrics_x[i][p_idx].settling_time_2percent >= 0
                      ? std::to_string(
                            drone_metrics_x[i][p_idx].settling_time_2percent)
                      : "N/A")
              << "s, ST(5%)="
              << (drone_metrics_x[i][p_idx].settling_time_5percent >= 0
                      ? std::to_string(
                            drone_metrics_x[i][p_idx].settling_time_5percent)
                      : "N/A")
              << "s"
              << ", SSE=" << std::fixed << std::setprecision(1)
              << drone_metrics_x[i][p_idx].final_steady_state_error << "%"
              << ", Peak=" << std::fixed << std::setprecision(2)
              << drone_metrics_x[i][p_idx].peak_value << " @ "
              << drone_metrics_x[i][p_idx].peak_time << "s"
              << " (Tgt:" << drone_metrics_x[i][p_idx].target_value_for_metrics
              << " Init:" << drone_metrics_x[i][p_idx].initial_value_for_metrics
              << ")\n";
          oss << "  Y-axis: OS=" << std::fixed << std::setprecision(1)
              << drone_metrics_y[i][p_idx].overshoot_percent << "%"
              << ", ST(2%)="
              << (drone_metrics_y[i][p_idx].settling_time_2percent >= 0
                      ? std::to_string(
                            drone_metrics_y[i][p_idx].settling_time_2percent)
                      : "N/A")
              << "s, ST(5%)="
              << (drone_metrics_y[i][p_idx].settling_time_5percent >= 0
                      ? std::to_string(
                            drone_metrics_y[i][p_idx].settling_time_5percent)
                      : "N/A")
              << "s"
              << ", SSE=" << std::fixed << std::setprecision(1)
              << drone_metrics_y[i][p_idx].final_steady_state_error << "%"
              << ", Peak=" << std::fixed << std::setprecision(2)
              << drone_metrics_y[i][p_idx].peak_value << " @ "
              << drone_metrics_y[i][p_idx].peak_time << "s"
              << " (Tgt:" << drone_metrics_y[i][p_idx].target_value_for_metrics
              << " Init:" << drone_metrics_y[i][p_idx].initial_value_for_metrics
              << ")\n";

          std::cout << oss.str();
          if (metrics_file_stream.is_open())
            metrics_file_stream << oss.str();
        }
      }
    }
  }

  // --- START: MODIFIED - Close file handles ---
  if (metrics_file_stream.is_open())
    metrics_file_stream.close();
  // --- END: MODIFIED ---

  std::cout << "\nMulti-Drone Test complete." << std::endl;
  return 0;
}