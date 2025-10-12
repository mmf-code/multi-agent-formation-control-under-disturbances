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
  if (root["wind"]) {
    wind_vx = root["wind"]["vx"].as<double>(wind_vx);
    wind_vy = root["wind"]["vy"].as<double>(wind_vy);
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
  // Optionally keep small constant acceleration bias (legacy)
  // drone.setWindAccel(0.0, 0.0);

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

  // Output directory and CSV file
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
  // find next run index
  int next_idx = 1;
  for (const auto &entry : fs::directory_iterator(day_dir)) {
    if (!entry.is_directory()) continue;
    auto name = entry.path().filename().string();
    if (name.rfind("run_", 0) == 0) {
      try {
        int idx = std::stoi(name.substr(4));
        if (idx >= next_idx) next_idx = idx + 1;
      } catch (...) {}
    }
  }
  std::ostringstream runname;
  runname << "run_" << std::setw(3) << std::setfill('0') << next_idx;
  fs::path run_dir = day_dir / runname.str();
  fs::create_directories(run_dir);
  fs::path csv_path = run_dir / fs::path(runname.str() + ".csv");

  std::ofstream csv(csv_path.string());
  if (!csv.is_open()) {
    std::cerr << "Failed to open output CSV: " << csv_path << std::endl;
    return 1;
  }
  // Header must match row contents exactly
  csv << "time,x,y,vx,vy,ax_cmd,ay_cmd,ax_cmd_f,ay_cmd_f,ax_drag,ay_drag,vrel_norm,ax_est,ay_est,";
  csv << "target_x,target_y,e_x,e_x_abs,e_y,e_y_abs,";
  csv << "kp,ki,kd,vx_wind,vy_wind,cd_lin,cd_quad,v_thr,tau_up,tau_down,a_max,dt\n";

  double time = 0.0;
  double prev_vx = 0.0, prev_vy = 0.0;

  for (; time <= total_time + 1e-12; time += dt) {
    const auto &st = drone.getState();
    // Compute acceleration commands from position error
    const double ax_cmd = ctrl_x->compute(st.x, target_x, dt);
    const double ay_cmd = ctrl_y->compute(st.y, target_y, dt);

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
        << ax_cmd_f << ',' << ay_cmd_f << ','
        << ax_drag << ',' << ay_drag << ','
        << vrel_norm << ','
        << ax_est << ',' << ay_est << ','
        << target_x << ',' << target_y << ','
        << e_x << ',' << std::abs(e_x) << ','
        << e_y << ',' << std::abs(e_y) << ','
        << cfg.pid_params.kp << ',' << cfg.pid_params.ki << ',' << cfg.pid_params.kd << ','
        << wind_vx << ',' << wind_vy << ','
        << phys.drag_coeff_lin << ',' << phys.drag_coeff_quad << ',' << phys.drag_speed_threshold << ','
        << phys.actuator_tau_up << ',' << phys.actuator_tau_down << ',' << phys.max_accel << ',' << dt << '\n';
  }

  csv.close();

  std::cout << "Wrote: " << csv_path.string() << std::endl;
  return 0;
}
