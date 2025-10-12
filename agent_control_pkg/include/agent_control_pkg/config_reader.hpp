#ifndef AGENT_CONTROL_PKG__CONFIG_READER_HPP_
#define AGENT_CONTROL_PKG__CONFIG_READER_HPP_

#include <string>
#include <vector>
#include <utility> // For std::pair
#include <stdexcept> // For std::runtime_error
#include <map>    // For FuzzyParams
#include <array>  // For FuzzyParams
#include <yaml-cpp/yaml.h> // YAML library

namespace agent_control_pkg {

// Structure for PID parameters
struct SimPIDParams {
    double kp{3.1}; 
    double ki{0.4};
    double kd{2.2};
    
    // Configurable output limits per axis  
    double output_min_x{-10.0};   // X-axis acceleration limits (m/s²)
    double output_max_x{10.0};
    double output_min_y{-8.0};    // Y-axis often has different limits due to gravity compensation
    double output_max_y{12.0};
    
    // Backward compatibility
    double output_min{-10.0};     // Fallback for single-axis systems
    double output_max{10.0};
    
    // Derivative filter settings
    bool enable_derivative_filter{true};
    double derivative_filter_alpha{0.1};  // Low-pass filter coefficient (0.05-0.2 typical)
    
    // Feed-forward control parameters
    bool enable_feedforward{false};
    double velocity_feedforward_gain{0.8};      // Velocity feed-forward coefficient
    double acceleration_feedforward_gain{0.1};  // Acceleration feed-forward coefficient
};

// Structure for individual phase configuration
struct PhaseConfig {
    std::vector<double> center; // {x, y}
    double start_time{0.0};
};

// Structure for wind time windows
struct TimeWindow {
    double start_time{0.0};
    double end_time{0.0};
    std::vector<double> force; // {wind_x, wind_y}
    bool is_sine_wave{false};
    double sine_frequency_rad_s{1.0};
};

// Structure for wind configuration per phase
struct WindPhaseConfig {
    int phase_number{0}; // 1-indexed
    std::vector<TimeWindow> time_windows;
};

// Structure for Ziegler-Nichols tuning parameters
struct ZNTuningParams {
    bool enable{false}; // For single manual run
    double kp_test_value{1.0};
    double simulation_time{30.0};
    
    // Parameters for the automated search
    bool enable_auto_search{false};
    double auto_search_kp_start{0.5};
    double auto_search_kp_step{0.25};
    double auto_search_kp_max{10.0};
    
    // Parameters for quick method testing
    std::string quick_method_test{"Off"}; // "Classic", "Conservative", "LessOvershoot", "NoOvershoot", "Aggressive", "ModernZN", "Off"
    double manual_ku{6.0};               // Ultimate gain for quick method calculations
    double manual_pu{2.0};               // Ultimate period for quick method calculations
};

// Physics parameters (2D model)
struct PhysicsParams {
    double mass{1.5};                 // kg
    double drag_coeff_lin{0.1};       // linear drag coefficient (kg/s)
    double drag_coeff_quad{0.0};      // quadratic drag coefficient (kg/m)
    double drag_speed_threshold{2.0}; // [m/s] linear→quadratic blend threshold
    double max_accel{15.0};           // m/s^2 per-axis limit
    double actuator_tau{0.1};         // s (first-order actuator time constant)
    double actuator_tau_up{0.0};      // s (optional asymmetric up time constant)
    double actuator_tau_down{0.0};    // s (optional asymmetric down time constant)
};

// --- CORRECTED & MERGED: Main simulation configuration structure ---
struct SimulationConfig {
    // Simulation Settings
    double dt{0.05};
    double total_time{120.0};
    int num_drones{3};
    ZNTuningParams zn_tuning_params;

    // Controller Settings
    SimPIDParams pid_params;
    // Controller selection: "pid" (default), "p", "pi", "pd", "fuzzy"
    std::string controller_type{"pid"};
    bool enable_fls{false};
    std::string fuzzy_params_file{"fuzzy_params.yaml"}; 

    // Physics Settings (2D dynamics)
    PhysicsParams physics;

    // Scenario Settings
    bool wind_enabled{false};
    double formation_side_length{4.0};
    std::vector<std::pair<double, double>> initial_positions; 
    std::vector<PhaseConfig> phases;
    std::vector<WindPhaseConfig> wind_phases;

    // Output Settings
    std::string output_directory{"simulation_outputs"};
    bool csv_enabled{true}; 
    std::string csv_prefix{"multi_agent_sim"}; 
    bool metrics_enabled{true};
    std::string metrics_prefix{"metrics_sim"};
    bool console_output_enabled{true};
    double console_update_interval{5.0};
};

// Class responsible for loading configurations
class ConfigReader {
public:
    static SimulationConfig loadConfig(const std::string& config_filepath);

private:
    static void loadSimulationSettings(const YAML::Node& node, SimulationConfig& config);
    static void loadControllerSettings(const YAML::Node& node, SimulationConfig& config);
    static void loadPhysicsSettings(const YAML::Node& node, SimulationConfig& config);
    static void loadScenarioSettings(const YAML::Node& node, SimulationConfig& config);
    static void loadOutputSettings(const YAML::Node& node, SimulationConfig& config);
};

// Structs for fuzzy parameter loading
struct FuzzySetFOU { double l1, l2, l3, u1, u2, u3; };
struct FuzzyParams {
    std::map<std::string, std::map<std::string, FuzzySetFOU>> sets;
    std::vector<std::array<std::string,4>> rules;
};

// Utility function to find config file paths
std::string findConfigFilePath(const std::string& filename);

// Declaration for loading fuzzy parameters from a file
bool loadFuzzyParamsYAML(const std::string& file_path, FuzzyParams& fp);

} // namespace agent_control_pkg
#endif // AGENT_CONTROL_PKG__CONFIG_READER_HPP_
