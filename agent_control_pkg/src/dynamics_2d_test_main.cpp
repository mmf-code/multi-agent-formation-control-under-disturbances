#include "agent_control_pkg/drone_dynamics_2d.hpp"
#include "agent_control_pkg/controllers/pid_adapter.hpp"
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
  PIDAdapter pid_x(cfg.pid_params.kp, cfg.pid_params.ki, cfg.pid_params.kd, -phys.max_accel, phys.max_accel);
  PIDAdapter pid_y(cfg.pid_params.kp, cfg.pid_params.ki, cfg.pid_params.kd, -phys.max_accel, phys.max_accel);
  pid_x.enableDerivativeFilter(cfg.pid_params.enable_derivative_filter, cfg.pid_params.derivative_filter_alpha);
  pid_y.enableDerivativeFilter(cfg.pid_params.enable_derivative_filter, cfg.pid_params.derivative_filter_alpha);

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
    const double ax_cmd = pid_x.compute(st.x, target_x, dt);
    const double ay_cmd = pid_y.compute(st.y, target_y, dt);

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
