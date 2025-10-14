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
#include <random>

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
  // Simple model-based feedforward (optional)
  bool ff_enable = false;
  bool ff_cancel_drag = true;
  bool ff_cancel_wind = true;
  double ff_k_drag = 1.0, ff_k_wind = 1.0;
  // Reference prefilter (optional; cascaded first-order approx. of 2nd-order LPF)
  bool pf_enable = false;
  double pf_tau = 0.8; // seconds (time constant). Smaller -> more aggressive filtering
  double pf_alpha = 0.0; // computed from dt and tau
  double pf_x1 = 0.0, pf_x2 = 0.0; // X axis filter states
  double pf_y1 = 0.0, pf_y2 = 0.0; // Y axis filter states
  // Time-varying wind model (optional)
  std::string wind_model_type{"steady"};
  double wm_base_vx = 0.0, wm_base_vy = 0.0;
  double wm_amp_vx = 0.0, wm_amp_vy = 0.0;
  double wm_freq_hz = 0.5;
  double wm_step_time = 5.0, wm_step_vx = 0.0, wm_step_vy = 0.0;
  double wm_gust_std = 0.0, wm_gust_alpha = 0.9; // LPF noise
  double wm_gust_x = 0.0, wm_gust_y = 0.0;
  std::mt19937 rng{static_cast<unsigned int>(std::random_device{}())};
  std::normal_distribution<double> gauss{0.0, 1.0};
  if (root["wind"]) {
    wind_vx = root["wind"]["vx"].as<double>(wind_vx);
    wind_vy = root["wind"]["vy"].as<double>(wind_vy);
    // Optional constant acceleration bias (for integral action testing)
    if (root["wind"]["ax"]) wind_ax = root["wind"]["ax"].as<double>(wind_ax);
    if (root["wind"]["ay"]) wind_ay = root["wind"]["ay"].as<double>(wind_ay);
    if (root["wind"]["ax_bias"]) wind_ax = root["wind"]["ax_bias"].as<double>(wind_ax);
    if (root["wind"]["ay_bias"]) wind_ay = root["wind"]["ay_bias"].as<double>(wind_ay);
  }
  // Feedforward options (YAML: feedforward: {enable, cancel_drag, cancel_wind, k_drag, k_wind})
  if (root["feedforward"]) {
    const auto ff = root["feedforward"]; 
    ff_enable = ff["enable"].as<bool>(ff_enable);
    ff_cancel_drag = ff["cancel_drag"].as<bool>(ff_cancel_drag);
    ff_cancel_wind = ff["cancel_wind"].as<bool>(ff_cancel_wind);
    ff_k_drag = ff["k_drag"].as<double>(ff_k_drag);
    ff_k_wind = ff["k_wind"].as<double>(ff_k_wind);
  }
  // Prefilter options (YAML: target_prefilter: {enabled, tau})
  if (root["target_prefilter"]) {
    const auto pf = root["target_prefilter"];
    pf_enable = pf["enabled"].as<bool>(pf_enable);
    pf_tau = pf["tau"].as<double>(pf_tau);
  }
  // Wind model options: wind_model: {type: steady|sinusoid|step|gust, ...}
  if (root["wind_model"]) {
    const auto wm = root["wind_model"];
    wind_model_type = wm["type"].as<std::string>(wind_model_type);
    wm_base_vx = wm["base_vx"].as<double>(wind_vx);
    wm_base_vy = wm["base_vy"].as<double>(wind_vy);
    wm_amp_vx = wm["amp_vx"].as<double>(wm_amp_vx);
    wm_amp_vy = wm["amp_vy"].as<double>(wm_amp_vy);
    wm_freq_hz = wm["frequency_hz"].as<double>(wm_freq_hz);
    wm_step_time = wm["step_time"].as<double>(wm_step_time);
    wm_step_vx = wm["step_vx"].as<double>(wm_base_vx);
    wm_step_vy = wm["step_vy"].as<double>(wm_base_vy);
    wm_gust_std = wm["gust_std"].as<double>(wm_gust_std);
    wm_gust_alpha = wm["gust_alpha"].as<double>(wm_gust_alpha);
  } else {
    wm_base_vx = wind_vx;
    wm_base_vy = wind_vy;
  }
  const double dt = cfg.dt;
  const double total_time = cfg.total_time;
  if (pf_enable) {
    pf_alpha = std::clamp(dt / std::max(pf_tau, 1e-6), 0.0, 1.0);
    // initialize states at initial target to avoid initial blip
    pf_x1 = target_x; pf_x2 = target_x;
    pf_y1 = target_y; pf_y2 = target_y;
  }

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
  csv << "time,x,y,vx,vy,ax_cmd,ay_cmd,ax_pid,ay_pid,ax_fuzzy,ay_fuzzy,ax_ff,ay_ff,ax_cmd_f,ay_cmd_f,ax_drag,ay_drag,vrel_norm,ax_est,ay_est,";
  csv << "target_x,target_y,e_x,e_x_abs,e_y,e_y_abs,";
  csv << "kp,ki,kd,vx_wind,vy_wind,ax_wind,ay_wind,cd_lin,cd_quad,v_thr,tau_up,tau_down,a_max,dt\n";

  double time = 0.0;
  double prev_vx = 0.0, prev_vy = 0.0;

  for (; time <= total_time + 1e-12; time += dt) {
    const auto &st = drone.getState();
    // Compute acceleration commands from position error
    double ref_x = target_x;
    double ref_y = target_y;
    if (pf_enable) {
      // two cascaded first-order filters towards target (critical-damped approximation)
      pf_x1 += pf_alpha * (target_x - pf_x1);
      pf_x2 += pf_alpha * (pf_x1 - pf_x2);
      pf_y1 += pf_alpha * (target_y - pf_y1);
      pf_y2 += pf_alpha * (pf_y1 - pf_y2);
      ref_x = pf_x2;
      ref_y = pf_y2;
    }
    const double ax_cmd = ctrl_x->compute(st.x, ref_x, dt);
    const double ay_cmd = ctrl_y->compute(st.y, ref_y, dt);

    // Time-varying wind velocity per model
    double wind_vx_t = wm_base_vx;
    double wind_vy_t = wm_base_vy;
    if (wind_model_type == "sinusoid") {
      constexpr double kPi = 3.14159265358979323846;
      const double w = 2.0 * kPi * wm_freq_hz;
      wind_vx_t = wm_base_vx + wm_amp_vx * std::sin(w * time);
      wind_vy_t = wm_base_vy + wm_amp_vy * std::sin(w * time);
    } else if (wind_model_type == "step") {
      wind_vx_t = (time >= wm_step_time) ? wm_step_vx : wm_base_vx;
      wind_vy_t = (time >= wm_step_time) ? wm_step_vy : wm_base_vy;
    } else if (wind_model_type == "gust") {
      // Low-pass filtered white noise
      wm_gust_x = wm_gust_alpha * wm_gust_x + (1.0 - wm_gust_alpha) * gauss(rng) * wm_gust_std;
      wm_gust_y = wm_gust_alpha * wm_gust_y + (1.0 - wm_gust_alpha) * gauss(rng) * wm_gust_std;
      wind_vx_t = wm_base_vx + wm_gust_x;
      wind_vy_t = wm_base_vy + wm_gust_y;
    }
    // Apply ambient wind for this step
    drone.setWindVelocity(wind_vx_t, wind_vy_t);

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

    // Model-based feedforward (drag cancellation + wind accel bias)
    double ax_ff = 0.0, ay_ff = 0.0;
    if (ff_enable) {
      // Predict drag using current state (same logic as dynamics)
      const double vx_rel = drone.getState().vx - wind_vx_t;
      const double vy_rel = drone.getState().vy - wind_vy_t;
      const double vrel_norm = std::hypot(vx_rel, vy_rel);
      const double inv_m = (phys.mass > 1e-9) ? (1.0 / phys.mass) : 0.0;
      const double v_thr = std::max(0.0, phys.drag_speed_threshold);
      auto drag_axis_pred = [&](double v_component){
        const double a_lin = -(phys.drag_coeff_lin * inv_m) * v_component;
        if (phys.drag_coeff_quad <= 0.0) return a_lin;
        const double a_quad = -(phys.drag_coeff_quad * inv_m) * vrel_norm * v_component;
        if (v_thr <= 1e-9 || vrel_norm <= v_thr) return a_lin;
        const double w = std::clamp((vrel_norm - v_thr) / std::max(v_thr, 1e-6), 0.0, 1.0);
        return (1.0 - w) * a_lin + w * a_quad;
      };
      if (ff_cancel_drag) {
        const double ax_drag_pred = drag_axis_pred(vx_rel);
        const double ay_drag_pred = drag_axis_pred(vy_rel);
        ax_ff += -ff_k_drag * ax_drag_pred;
        ay_ff += -ff_k_drag * ay_drag_pred;
      }
      if (ff_cancel_wind) {
        ax_ff += -ff_k_wind * wind_ax;
        ay_ff += -ff_k_wind * wind_ay;
      }
    }

    // Final command with feedforward
    const double ax_cmd_total = ax_cmd + ax_ff;
    const double ay_cmd_total = ay_cmd + ay_ff;

    // Step dynamics
    drone.step(ax_cmd_total, ay_cmd_total, dt);

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
        << ax_ff << ',' << ay_ff << ','
        << ax_cmd_f << ',' << ay_cmd_f << ','
        << ax_drag << ',' << ay_drag << ','
        << vrel_norm << ','
        << ax_est << ',' << ay_est << ','
        << target_x << ',' << target_y << ','
        << e_x << ',' << std::abs(e_x) << ','
        << e_y << ',' << std::abs(e_y) << ','
        << cfg.pid_params.kp << ',' << cfg.pid_params.ki << ',' << cfg.pid_params.kd << ','
        << wind_vx_t << ',' << wind_vy_t << ',' << wind_ax << ',' << wind_ay << ','
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
