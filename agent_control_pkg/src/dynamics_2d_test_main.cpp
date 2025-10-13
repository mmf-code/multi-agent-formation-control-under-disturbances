#include "agent_control_pkg/drone_dynamics_2d.hpp"
#include "agent_control_pkg/controllers/pid_adapter.hpp"
#include "agent_control_pkg/controllers/fuzzy_gt2_adapter.hpp"
#include "agent_control_pkg/controllers/combined_pid_fuzzy_adapter.hpp"
#include "agent_control_pkg/config_reader.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <sstream>
#include <cctype>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  using agent_control_pkg::DroneDynamics2D;
  using agent_control_pkg::controllers::PIDAdapter;
  using agent_control_pkg::controllers::FuzzyGT2Adapter;
  using agent_control_pkg::controllers::CombinedPidFuzzyAdapter;
  using agent_control_pkg::ConfigReader;
  using agent_control_pkg::SimulationConfig;

  // Resolve config path (default file or argv[1])
  std::string cfg_rel = (argc > 1) ? argv[1] : std::string("agent_control_pkg/config/dynamics_2d_test.yaml");
  std::string cfg_path = agent_control_pkg::findConfigFilePath(cfg_rel);
  std::cout << "Using config: " << cfg_path << std::endl;

  // Load SimulationConfig (dt, total_time, physics, controller pid)
  SimulationConfig cfg = ConfigReader::loadConfig(cfg_path);

  // Additional fields (wind, target) directly from YAML
  YAML::Node root = YAML::LoadFile(cfg_path);
  double target_x = 5.0, target_y = 5.0;
  if (root["target"]) {
    target_x = root["target"]["x"].as<double>(target_x);
    target_y = root["target"]["y"].as<double>(target_y);
  }
  double wind_vx = 0.0, wind_vy = 0.0;
  double wind_ax = 0.0, wind_ay = 0.0;
  if (root["wind"]) {
    wind_vx = root["wind"]["vx"].as<double>(wind_vx);
    wind_vy = root["wind"]["vy"].as<double>(wind_vy);
    // Optional constant acceleration bias (for integral action testing)
    if (root["wind"]["ax"]) wind_ax = root["wind"]["ax"].as<double>(wind_ax);
    if (root["wind"]["ay"]) wind_ay = root["wind"]["ay"].as<double>(wind_ay);
    if (root["wind"]["ax_bias"]) wind_ax = root["wind"]["ax_bias"].as<double>(wind_ax);
    if (root["wind"]["ay_bias"]) wind_ay = root["wind"]["ay_bias"].as<double>(wind_ay);
  }
  const double dt = cfg.dt;
  const double total_time = cfg.total_time;

  // Physics
  DroneDynamics2D drone;
  DroneDynamics2D::Params phys;
  phys.mass = cfg.physics.mass;
  phys.drag_coeff_lin = cfg.physics.drag_coeff_lin;
  phys.drag_coeff_quad = cfg.physics.drag_coeff_quad;
  phys.drag_speed_threshold = cfg.physics.drag_speed_threshold;
  phys.max_accel = cfg.physics.max_accel;
  phys.actuator_tau = cfg.physics.actuator_tau;
  phys.actuator_tau_up = cfg.physics.actuator_tau_up;
  phys.actuator_tau_down = cfg.physics.actuator_tau_down;
  drone.setParams(phys);
  // Ambient wind velocity (m/s)
  drone.setWindVelocity(wind_vx, wind_vy);
  // Optional constant acceleration bias (m/s^2)
  drone.setWindAccel(wind_ax, wind_ay);

  // Controllers (position -> acceleration) via interface
  std::unique_ptr<agent_control_pkg::controllers::IController1D> ctrl_x;
  std::unique_ptr<agent_control_pkg::controllers::IController1D> ctrl_y;

  auto make_pid = [&](double kp, double ki, double kd) {
    auto p = std::make_unique<PIDAdapter>(kp, ki, kd, -phys.max_accel, phys.max_accel);
    p->enableDerivativeFilter(cfg.pid_params.enable_derivative_filter, cfg.pid_params.derivative_filter_alpha);
    return p;
  };

  std::string ctype = cfg.controller_type;
  for (auto &ch : ctype) ch = static_cast<char>(::tolower(ch));
  if (ctype == "p") { cfg.pid_params.ki = 0.0; cfg.pid_params.kd = 0.0; }
  else if (ctype == "pi") { cfg.pid_params.kd = 0.0; }
  else if (ctype == "pd") { cfg.pid_params.ki = 0.0; }

  if (ctype == "fuzzy" || (cfg.enable_fls && ctype != "pid" && ctype != "pid_fuzzy")) {
    // Attempt to configure GT2 FLS from file; fall back to PID on failure
    agent_control_pkg::FuzzyParams fp;
    bool ok = agent_control_pkg::loadFuzzyParamsYAML(cfg.fuzzy_params_file, fp);
    if (ok) {
      FuzzyGT2Adapter::Options opt;
      opt.umin = -phys.max_accel; opt.umax = phys.max_accel;
      opt.wind_scalar = std::hypot(wind_vx, wind_vy);
      auto f1 = std::make_unique<FuzzyGT2Adapter>(opt);
      auto f2 = std::make_unique<FuzzyGT2Adapter>(opt);
      // Build from params (variables: error, dError, optional wind, output)
      bool include_wind = fp.sets.find("wind") != fp.sets.end();
      f1->configureFromFuzzyParams(fp, include_wind);
      f2->configureFromFuzzyParams(fp, include_wind);
      ctrl_x = std::move(f1);
      ctrl_y = std::move(f2);
      std::cout << "Using controller: fuzzy (GT2) with params file '" << cfg.fuzzy_params_file << "'" << std::endl;
    } else {
      std::cerr << "Fuzzy params failed to load; falling back to PID." << std::endl;
      ctrl_x = make_pid(cfg.pid_params.kp, cfg.pid_params.ki, cfg.pid_params.kd);
      ctrl_y = make_pid(cfg.pid_params.kp, cfg.pid_params.ki, cfg.pid_params.kd);
    }
  } else {
    // Handle pid_fuzzy (hybrid) or pure PID
    if (ctype == "pid_fuzzy" || ctype == "fuzzy_pid" || ctype == "hybrid") {
      // Read mix gains from YAML if present
      double k_pid_mix = 1.0, k_fuzzy_mix = 1.0;
      if (root["controller_settings"] && root["controller_settings"]["mix"]) {
        const auto mix = root["controller_settings"]["mix"];
        k_pid_mix = mix["k_pid"].as<double>(k_pid_mix);
        k_fuzzy_mix = mix["k_fuzzy"].as<double>(k_fuzzy_mix);
      }
      // Build PID
      auto pidx = make_pid(cfg.pid_params.kp, cfg.pid_params.ki, cfg.pid_params.kd);
      auto pidy = make_pid(cfg.pid_params.kp, cfg.pid_params.ki, cfg.pid_params.kd);
      // Build Fuzzy (if params load OK); fall back to PID-only if not
      agent_control_pkg::FuzzyParams fp;
      bool ok = agent_control_pkg::loadFuzzyParamsYAML(cfg.fuzzy_params_file, fp);
      if (ok) {
        FuzzyGT2Adapter::Options opt;
        opt.umin = -phys.max_accel; opt.umax = phys.max_accel;
        opt.wind_scalar = std::hypot(wind_vx, wind_vy);
        auto fx = std::make_unique<FuzzyGT2Adapter>(opt);
        auto fy = std::make_unique<FuzzyGT2Adapter>(opt);
        bool include_wind = fp.sets.find("wind") != fp.sets.end();
        fx->configureFromFuzzyParams(fp, include_wind);
        fy->configureFromFuzzyParams(fp, include_wind);
        ctrl_x = std::make_unique<CombinedPidFuzzyAdapter>(std::move(pidx), std::move(fx), k_pid_mix, k_fuzzy_mix, -phys.max_accel, phys.max_accel);
        ctrl_y = std::make_unique<CombinedPidFuzzyAdapter>(std::move(pidy), std::move(fy), k_pid_mix, k_fuzzy_mix, -phys.max_accel, phys.max_accel);
        std::cout << "Using controller: pid+fuzzy (k_pid=" << k_pid_mix << ", k_fuzzy=" << k_fuzzy_mix << ")\n";
      } else {
        std::cerr << "Fuzzy params failed to load; using pure PID." << std::endl;
        ctrl_x = std::move(pidx);
        ctrl_y = std::move(pidy);
      }
    } else {
      ctrl_x = make_pid(cfg.pid_params.kp, cfg.pid_params.ki, cfg.pid_params.kd);
      ctrl_y = make_pid(cfg.pid_params.kp, cfg.pid_params.ki, cfg.pid_params.kd);
    }
  }

  // Output directory and CSV file with improved naming and options
  fs::path base_out = fs::path("outputs/simulations/dynamics2d");
  // Day folder (YYYYMMDD) and incremental run_### folder
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream day;
  day << std::put_time(&tm, "%Y%m%d");
  fs::path day_dir = base_out / day.str();
  fs::create_directories(day_dir);
  // Custom label from YAML (optional)
  auto sanitize = [](std::string s) {
    for (char &c : s) {
      if (!(std::isalnum(static_cast<unsigned char>(c)) || c=='_' || c=='-' )) c = '_';
    }
    // trim consecutive underscores
    std::string out; out.reserve(s.size());
    bool prev_us = false; for(char c: s){ bool us = (c=='_'); if(!(us && prev_us)) out.push_back(c); prev_us = us; }
    return out;
  };
  std::string user_label;
  int max_runs_per_day = -1;
  bool auto_plot = false;
  std::string plot_script = "analysis/plot_dynamics_2d.py";
  try {
    if (root["output_settings"]) {
      auto os = root["output_settings"];
      if (os["run_label"]) user_label = os["run_label"].as<std::string>(user_label);
      if (os["auto_plot"]) auto_plot = os["auto_plot"].as<bool>(auto_plot);
      if (os["plot_script"]) plot_script = os["plot_script"].as<std::string>(plot_script);
      if (os["max_runs_per_day"]) max_runs_per_day = os["max_runs_per_day"].as<int>(max_runs_per_day);
    }
  } catch (...) {}
  // If no user label, build a short default label
  if (user_label.empty()) {
    std::ostringstream lab;
    lab << (ctype.empty() ? cfg.controller_type : ctype);
    if (std::abs(wind_vx) > 1e-6 || std::abs(wind_vy) > 1e-6) {
      lab << "__wind";
    } else {
      lab << "__nowind";
    }
    user_label = lab.str();
  }
  const std::string label_sanitized = sanitize(user_label);

  // find next run index (accepts run_### or run_###__label)
  int next_idx = 1;
  std::vector<std::pair<int, fs::path>> existing_runs;
  for (const auto &entry : fs::directory_iterator(day_dir)) {
    if (!entry.is_directory()) continue;
    auto name = entry.path().filename().string();
    if (name.rfind("run_", 0) == 0) {
      try {
        // parse until non-digit after run_
        size_t p = 4; while (p < name.size() && std::isdigit(static_cast<unsigned char>(name[p]))) ++p;
        int idx = std::stoi(name.substr(4, p-4));
        if (idx >= next_idx) next_idx = idx + 1;
        existing_runs.emplace_back(idx, entry.path());
      } catch (...) {}
    }
  }
  std::ostringstream runname;
  runname << "run_" << std::setw(3) << std::setfill('0') << next_idx;
  if (!label_sanitized.empty()) runname << "__" << label_sanitized;
  fs::path run_dir = day_dir / runname.str();
  fs::create_directories(run_dir);
  fs::path csv_path = run_dir / fs::path(runname.str() + ".csv");

  std::ofstream csv(csv_path.string());
  if (!csv.is_open()) {
    std::cerr << "Failed to open output CSV: " << csv_path << std::endl;
    return 1;
  }
  // Header must match row contents exactly
  csv << "time,x,y,vx,vy,ax_cmd,ay_cmd,ax_pid,ay_pid,ax_fuzzy,ay_fuzzy,ax_cmd_f,ay_cmd_f,ax_drag,ay_drag,vrel_norm,ax_est,ay_est,";
  csv << "target_x,target_y,e_x,e_x_abs,e_y,e_y_abs,";
  csv << "kp,ki,kd,vx_wind,vy_wind,ax_wind,ay_wind,cd_lin,cd_quad,v_thr,tau_up,tau_down,a_max,dt\n";

  double time = 0.0;
  double prev_vx = 0.0, prev_vy = 0.0;

  for (; time <= total_time + 1e-12; time += dt) {
    const auto &st = drone.getState();
    // Compute acceleration commands from position error
    const double ax_cmd = ctrl_x->compute(st.x, target_x, dt);
    const double ay_cmd = ctrl_y->compute(st.y, target_y, dt);

    // Diagnostics: if hybrid controller, capture contributions
    double ax_pid = 0.0, ay_pid = 0.0, ax_fuzzy = 0.0, ay_fuzzy = 0.0;
    if (auto cx = dynamic_cast<CombinedPidFuzzyAdapter*>(ctrl_x.get())) {
      ax_pid = cx->lastPidContribution();
      ax_fuzzy = cx->lastFuzzyContribution();
    }
    if (auto cy = dynamic_cast<CombinedPidFuzzyAdapter*>(ctrl_y.get())) {
      ay_pid = cy->lastPidContribution();
      ay_fuzzy = cy->lastFuzzyContribution();
    }

    // Step dynamics
    drone.step(ax_cmd, ay_cmd, dt);

    // Estimate acceleration from velocity delta
    const auto &st2 = drone.getState();
    const double e_x = target_x - st2.x;
    const double e_y = target_y - st2.y;
    const double ax_est = (st2.vx - prev_vx) / dt;
    const double ay_est = (st2.vy - prev_vy) / dt;
    prev_vx = st2.vx;
    prev_vy = st2.vy;

    double ax_cmd_f = 0.0, ay_cmd_f = 0.0;
    double ax_drag = 0.0, ay_drag = 0.0;
    drone.getFilteredCommand(ax_cmd_f, ay_cmd_f);
    drone.getDragAccel(ax_drag, ay_drag);
    const double vrel_norm = drone.getRelativeAirSpeedNorm();

    csv << std::fixed << std::setprecision(4)
        << time << ','
        << st2.x << ',' << st2.y << ','
        << st2.vx << ',' << st2.vy << ','
        << ax_cmd << ',' << ay_cmd << ','
        << ax_pid << ',' << ay_pid << ','
        << ax_fuzzy << ',' << ay_fuzzy << ','
        << ax_cmd_f << ',' << ay_cmd_f << ','
        << ax_drag << ',' << ay_drag << ','
        << vrel_norm << ','
        << ax_est << ',' << ay_est << ','
        << target_x << ',' << target_y << ','
        << e_x << ',' << std::abs(e_x) << ','
        << e_y << ',' << std::abs(e_y) << ','
        << cfg.pid_params.kp << ',' << cfg.pid_params.ki << ',' << cfg.pid_params.kd << ','
        << wind_vx << ',' << wind_vy << ',' << wind_ax << ',' << wind_ay << ','
        << phys.drag_coeff_lin << ',' << phys.drag_coeff_quad << ',' << phys.drag_speed_threshold << ','
        << phys.actuator_tau_up << ',' << phys.actuator_tau_down << ',' << phys.max_accel << ',' << dt << '\n';
  }

  csv.close();

  std::cout << "Wrote: " << csv_path.string() << std::endl;

  // Optional auto-plot: call python plot script
  if (auto_plot) {
    try {
      std::ostringstream cmd;
#ifdef _WIN32
      cmd << "python \"" << plot_script << "\" --csv \"" << csv_path.string() << "\"";
#else
      cmd << "python3 \"" << plot_script << "\" --csv \"" << csv_path.string() << "\"";
#endif
      int rc = std::system(cmd.str().c_str());
      (void)rc;
    } catch (...) {}
  }

  // Optional pruning of old runs per day
  if (max_runs_per_day > 0) {
    try {
      existing_runs.emplace_back(next_idx, run_dir);
      std::sort(existing_runs.begin(), existing_runs.end(), [](auto &a, auto &b){ return a.first < b.first; });
      if (static_cast<int>(existing_runs.size()) > max_runs_per_day) {
        int to_remove = static_cast<int>(existing_runs.size()) - max_runs_per_day;
        for (int i = 0; i < to_remove; ++i) {
          std::error_code ec; fs::remove_all(existing_runs[i].second, ec);
        }
      }
    } catch (...) {}
  }
  return 0;
}
