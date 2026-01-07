// ----------------- FIXES -----------------
// 1. Added required headers for yaml-cpp and std::string functionality.
// 2. Corrected the preprocessor logic to be more robust and removed
//    unused Windows-specific headers that were causing macro conflicts.
// 3. Updated loadControllerSettings to parse all PID parameters from the YAML.
// 4. Fixed getline issues by including proper headers and using correct function calls.
// 5. Fixed ifstream boolean conversion issues.
// 6. Simplified YAML dependency - using actual yaml-cpp as confirmed by CMake build.
// -----------------------------------------

#include "../include/agent_control_pkg/config_reader.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cctype>

// Include filesystem with robust cross-platform and compiler-version checks
#ifdef _WIN32
    #if defined(_MSC_VER) && _MSC_VER >= 1914 && defined(_MSVC_LANG) && _MSVC_LANG >= 201703L
        #include <filesystem>
        namespace fs = std::filesystem;
        #define HAS_FILESYSTEM 1
    #else
        #define HAS_FILESYSTEM 0
    #endif
#else
    #if __cplusplus >= 201703L
        #include <filesystem>
        namespace fs = std::filesystem;
        #define HAS_FILESYSTEM 1
    #else
        #define HAS_FILESYSTEM 0
    #endif
#endif

namespace agent_control_pkg {

// --- Utility functions for fuzzy_params.yaml parsing ---
static std::string trim_fuzzy_cfg(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    if(start == std::string::npos) return "";
    return s.substr(start, end - start + 1);
}

static std::vector<double> parseNumberList_fuzzy_cfg(const std::string& in) {
    std::vector<double> values;
    std::stringstream ss(in);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        tok = trim_fuzzy_cfg(tok);
        if(!tok.empty()) {
            try {
                values.push_back(std::stod(tok));
            } catch (const std::invalid_argument&) {
                std::cerr << "Warning: Invalid number '" << tok << "' in list. Skipping." << std::endl;
            } catch (const std::out_of_range&) {
                std::cerr << "Warning: Number '" << tok << "' out of range. Skipping." << std::endl;
            }
        }
    }
    return values;
}

static std::vector<std::string> parseStringList_fuzzy_cfg(const std::string& in) {
    std::vector<std::string> values;
    std::stringstream ss(in);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        values.push_back(trim_fuzzy_cfg(tok));
    }
    return values;
}

std::string findConfigFilePath(const std::string& filename) {
#if HAS_FILESYSTEM
    std::vector<fs::path> candidate_dirs = {
        ".", "config", "../config", "../../../agent_control_pkg/config", "../../config", "../../agent_control_pkg/config" 
    };

    fs::path direct_path(filename);
    if (direct_path.is_absolute() && fs::exists(direct_path) && fs::is_regular_file(direct_path)) {
        return direct_path.string();
    }

    for (const auto& dir : candidate_dirs) {
        fs::path full_path = fs::absolute(dir) / filename;
        if (fs::exists(full_path) && fs::is_regular_file(full_path)) {
            return fs::weakly_canonical(full_path).string(); 
        }
    }
#else
    std::vector<std::string> candidate_dirs = {
        ".", "config", "../config", "../../../agent_control_pkg/config", "../../config", "../../agent_control_pkg/config"
    };
    
    #ifdef _WIN32
        bool is_absolute = filename.length() > 2 && filename[1] == ':' && (std::isalpha(filename[0]));
    #else
        bool is_absolute = !filename.empty() && filename[0] == '/';
    #endif
    
    if (is_absolute) {
        std::ifstream test_file(filename);
        if (test_file.is_open()) {
            test_file.close();
            return filename;
        }
    }

    for (const auto& dir : candidate_dirs) {
        std::string full_path = dir + "/" + filename;
        std::ifstream test_file(full_path);
        if (test_file.is_open()) {
            test_file.close();
            return full_path;
        }
    }
#endif
    
    std::cerr << "WARNING: Config file '" << filename << "' not found. Trying original path as last resort." << std::endl;
    return filename;
}

SimulationConfig ConfigReader::loadConfig(const std::string& config_filepath) {
    SimulationConfig config;
    std::string actual_filepath = findConfigFilePath(config_filepath);

    YAML::Node root_node;
    try {
        root_node = YAML::LoadFile(actual_filepath);
        std::cout << "Successfully loaded configuration from: " << actual_filepath << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "FATAL: Error loading YAML file '" << actual_filepath << "': " << e.what() << std::endl;
        std::cerr << "Using default configuration values." << std::endl;
        return config;
    }

    if (root_node["simulation_settings"]) { loadSimulationSettings(root_node["simulation_settings"], config); }
    if (root_node["controller_settings"]) { loadControllerSettings(root_node["controller_settings"], config); }
    // Physics settings can be provided under either 'physics' or 'physics_settings'
    if (root_node["physics"]) { loadPhysicsSettings(root_node["physics"], config); }
    if (root_node["physics_settings"]) { loadPhysicsSettings(root_node["physics_settings"], config); }
    if (root_node["scenario_settings"]) { loadScenarioSettings(root_node["scenario_settings"], config); }
    if (root_node["output_settings"]) { loadOutputSettings(root_node["output_settings"], config); }
    
    return config;
}

void ConfigReader::loadSimulationSettings(const YAML::Node& node, SimulationConfig& config) {
    if (!node.IsMap()) {
        std::cerr << "Warning: 'simulation_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    config.dt = node["dt"].as<double>(config.dt);
    config.total_time = node["total_time"].as<double>(config.total_time);
    config.num_drones = node["num_drones"].as<int>(config.num_drones);

    YAML::Node zn_node = node["ziegler_nichols_tuning"];
    if (zn_node) {
        config.zn_tuning_params.enable = zn_node["enable"].as<bool>(config.zn_tuning_params.enable);
        config.zn_tuning_params.kp_test_value = zn_node["kp_test_value"].as<double>(config.zn_tuning_params.kp_test_value);
        config.zn_tuning_params.simulation_time = zn_node["simulation_time"].as<double>(config.zn_tuning_params.simulation_time);
        config.zn_tuning_params.enable_auto_search = zn_node["enable_auto_search"].as<bool>(config.zn_tuning_params.enable_auto_search);
        config.zn_tuning_params.auto_search_kp_start = zn_node["auto_search_kp_start"].as<double>(config.zn_tuning_params.auto_search_kp_start);
        config.zn_tuning_params.auto_search_kp_step = zn_node["auto_search_kp_step"].as<double>(config.zn_tuning_params.auto_search_kp_step);
        config.zn_tuning_params.auto_search_kp_max = zn_node["auto_search_kp_max"].as<double>(config.zn_tuning_params.auto_search_kp_max);
        config.zn_tuning_params.quick_method_test = zn_node["quick_method_test"].as<std::string>(config.zn_tuning_params.quick_method_test);
        config.zn_tuning_params.manual_ku = zn_node["manual_ku"].as<double>(config.zn_tuning_params.manual_ku);
        config.zn_tuning_params.manual_pu = zn_node["manual_pu"].as<double>(config.zn_tuning_params.manual_pu);
    }
}

void ConfigReader::loadControllerSettings(const YAML::Node& node, SimulationConfig& config) {
    if (!node.IsMap()) {
        std::cerr << "Warning: 'controller_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    // Optional controller type at this level
    if (node["type"]) {
        try {
            config.controller_type = node["type"].as<std::string>(config.controller_type);
        } catch (...) {}
    }
    YAML::Node pid_node = node["pid"];
    if (pid_node) {
        config.pid_params.kp = pid_node["kp"].as<double>(config.pid_params.kp);
        config.pid_params.ki = pid_node["ki"].as<double>(config.pid_params.ki);
        config.pid_params.kd = pid_node["kd"].as<double>(config.pid_params.kd);
        config.pid_params.output_min = pid_node["output_min"].as<double>(config.pid_params.output_min);
        config.pid_params.output_max = pid_node["output_max"].as<double>(config.pid_params.output_max);
        config.pid_params.enable_derivative_filter = pid_node["enable_derivative_filter"].as<bool>(config.pid_params.enable_derivative_filter);
        config.pid_params.enable_feedforward = pid_node["enable_feedforward"].as<bool>(config.pid_params.enable_feedforward);
    }
    YAML::Node fls_node = node["fls"];
    if (fls_node) {
        config.enable_fls = fls_node["enable"].as<bool>(config.enable_fls);
        config.fuzzy_params_file = fls_node["params_file"].as<std::string>(config.fuzzy_params_file);
    }
}

void ConfigReader::loadPhysicsSettings(const YAML::Node& node, SimulationConfig& config) {
    if (!node.IsMap()) {
        std::cerr << "Warning: 'physics' node is not a map. Skipping." << std::endl;
        return;
    }
    // Allow both explicit names and aliases for compatibility
    config.physics.mass = node["mass"].as<double>(config.physics.mass);
    // drag_coeff_lin may be provided as 'cd_lin' or 'drag_coeff_lin'
    if (node["drag_coeff_lin"]) {
        config.physics.drag_coeff_lin = node["drag_coeff_lin"].as<double>(config.physics.drag_coeff_lin);
    } else if (node["cd_lin"]) {
        config.physics.drag_coeff_lin = node["cd_lin"].as<double>(config.physics.drag_coeff_lin);
    }
    // optional quadratic drag
    if (node["drag_coeff_quad"]) {
        config.physics.drag_coeff_quad = node["drag_coeff_quad"].as<double>(config.physics.drag_coeff_quad);
    } else if (node["cd_quad"]) {
        config.physics.drag_coeff_quad = node["cd_quad"].as<double>(config.physics.drag_coeff_quad);
    }
    if (node["drag_speed_threshold"]) {
        config.physics.drag_speed_threshold = node["drag_speed_threshold"].as<double>(config.physics.drag_speed_threshold);
    } else if (node["drag_v_threshold"]) {
        config.physics.drag_speed_threshold = node["drag_v_threshold"].as<double>(config.physics.drag_speed_threshold);
    } else if (node["v_thr"]) {
        config.physics.drag_speed_threshold = node["v_thr"].as<double>(config.physics.drag_speed_threshold);
    }
    // max_accel may be provided as 'a_max'
    if (node["max_accel"]) {
        config.physics.max_accel = node["max_accel"].as<double>(config.physics.max_accel);
    } else if (node["a_max"]) {
        config.physics.max_accel = node["a_max"].as<double>(config.physics.max_accel);
    }
    // actuator time constant may be provided as 'tau_act' or 'actuator_tau'
    if (node["actuator_tau"]) {
        config.physics.actuator_tau = node["actuator_tau"].as<double>(config.physics.actuator_tau);
    } else if (node["tau_act"]) {
        config.physics.actuator_tau = node["tau_act"].as<double>(config.physics.actuator_tau);
    }
    // asymmetric actuator taus (optional)
    if (node["actuator_tau_up"]) {
        config.physics.actuator_tau_up = node["actuator_tau_up"].as<double>(config.physics.actuator_tau_up);
    } else if (node["tau_up"]) {
        config.physics.actuator_tau_up = node["tau_up"].as<double>(config.physics.actuator_tau_up);
    }
    if (node["actuator_tau_down"]) {
        config.physics.actuator_tau_down = node["actuator_tau_down"].as<double>(config.physics.actuator_tau_down);
    } else if (node["tau_down"]) {
        config.physics.actuator_tau_down = node["tau_down"].as<double>(config.physics.actuator_tau_down);
    }
}

void ConfigReader::loadScenarioSettings(const YAML::Node& node, SimulationConfig& config) {
    if (!node.IsMap()) {
        std::cerr << "Warning: 'scenario_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    config.wind_enabled = node["enable_wind"].as<bool>(config.wind_enabled);
    config.formation_side_length = node["formation_side_length"].as<double>(config.formation_side_length);

    YAML::Node ip_node = node["formation"]["initial_positions"];
    if (ip_node) {
        if (ip_node.IsMap()) {
            config.initial_positions.clear(); 
            for (int i = 0; i < config.num_drones; ++i) {
                std::string drone_key = "drone_" + std::to_string(i);
                if (ip_node[drone_key] && ip_node[drone_key].IsSequence() && ip_node[drone_key].size() == 2) {
                    config.initial_positions.push_back({
                        ip_node[drone_key][0].as<double>(),
                        ip_node[drone_key][1].as<double>()
                    });
                } else {
                    config.initial_positions.push_back({0.0, 0.0});
                }
            }
        }
    }
    
    if (config.initial_positions.size() != static_cast<size_t>(config.num_drones)) {
        config.initial_positions.clear();
        for(int i = 0; i < config.num_drones; ++i) {
            double x_pos = (static_cast<double>(i) * 2.0) - (static_cast<double>(config.num_drones - 1) * 1.0);
            config.initial_positions.push_back({x_pos, 0.0});
        }
    }

    YAML::Node phases_node = node["phases"];
    if (phases_node) {
        if(phases_node.IsSequence()) {
            config.phases.clear();
            for (const auto& phase_node : phases_node) {
                PhaseConfig pc;
                pc.center = phase_node["center"].as<std::vector<double>>(std::vector<double>{0.0, 0.0});
                pc.start_time = phase_node["start_time"].as<double>(0.0);
                config.phases.push_back(pc);
            }
        }
    }

    if (config.wind_enabled && node["wind"] && node["wind"]["phases"]) {
        YAML::Node wind_phases_node = node["wind"]["phases"];
        if(wind_phases_node.IsSequence()){
            config.wind_phases.clear();
            for (const auto& wp_node : wind_phases_node) {
                WindPhaseConfig wpc;
                wpc.phase_number = wp_node["phase"].as<int>(0);
                YAML::Node tw_nodes = wp_node["time_windows"];
                if (tw_nodes.IsSequence()) {
                    for (const auto& tw_node : tw_nodes) {
                        TimeWindow tw;
                        tw.start_time = tw_node["start_time"].as<double>(0.0);
                        tw.end_time = tw_node["end_time"].as<double>(0.0);
                        tw.force = tw_node["force"].as<std::vector<double>>(std::vector<double>{0.0, 0.0});
                        tw.is_sine_wave = tw_node["is_sine_wave"].as<bool>(false);
                        tw.sine_frequency_rad_s = tw_node["sine_frequency_rad_s"].as<double>(1.0);
                        wpc.time_windows.push_back(tw);
                    }
                }
                config.wind_phases.push_back(wpc);
            }
        }
    }
}

void ConfigReader::loadOutputSettings(const YAML::Node& node, SimulationConfig& config) {
    if (!node.IsMap()) {
        std::cerr << "Warning: 'output_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    config.output_directory = node["output_directory"].as<std::string>(config.output_directory);
    config.csv_enabled = node["csv_enabled"].as<bool>(config.csv_enabled);
    config.csv_prefix = node["csv_prefix"].as<std::string>(config.csv_prefix);
    config.metrics_enabled = node["metrics_enabled"].as<bool>(config.metrics_enabled);
    config.metrics_prefix = node["metrics_prefix"].as<std::string>(config.metrics_prefix);

    YAML::Node co_node = node["console_output"];
    if (co_node) {
        config.console_output_enabled = co_node["enabled"].as<bool>(config.console_output_enabled);
        config.console_update_interval = co_node["update_interval"].as<double>(config.console_update_interval);
    }
}

bool loadFuzzyParamsYAML(const std::string& file_path, FuzzyParams& fp) {
    std::string actual_filepath = findConfigFilePath(file_path);
    std::ifstream in(actual_filepath);
    if (!in.is_open()) {
        std::cerr << "Error: Could not open fuzzy params file: " << actual_filepath << std::endl;
        return false;
    }
    std::string line, section, current_var;
    fp.sets.clear();
    fp.rules.clear();

    while (std::getline(in, line)) {
        line = trim_fuzzy_cfg(line);
        if (line.empty() || line[0] == '#') continue;
        if (line == "membership_functions:") { section = "mf"; current_var = ""; continue; }
        if (line == "rules:") { section = "rules"; current_var = ""; continue; }

        if (section == "mf") {
            if (line.length() > 1 && line.back() == ':' && line.find_first_of(" \t") == std::string::npos) {
                current_var = trim_fuzzy_cfg(line.substr(0, line.size() - 1));
                fp.sets[current_var];
            } else if (!current_var.empty()) {
                auto pos = line.find(':');
                if (pos == std::string::npos) continue;
                std::string setname = trim_fuzzy_cfg(line.substr(0, pos));
                std::string rest = line.substr(pos + 1);
                auto lb = rest.find('[');
                auto rb = rest.find(']');
                if (lb == std::string::npos || rb == std::string::npos) continue;
                std::string nums_str = rest.substr(lb + 1, rb - lb - 1);
                auto values = parseNumberList_fuzzy_cfg(nums_str);
                if (values.size() == 6) {
                    fp.sets[current_var][setname] = {values[0], values[1], values[2], values[3], values[4], values[5]};
                }
            }
        } else if (section == "rules") {
            // Handle rules with optional comments: "- [A, B, C, D]  # comment"
            if (line.rfind("- [", 0) == 0) {
                // Find the closing bracket (ignore anything after it, like comments)
                auto rb = line.find(']');
                if (rb != std::string::npos && rb > 3) {
                    // Extract content between [ and ]
                    std::string rule_content = line.substr(3, rb - 3);
                    auto tokens = parseStringList_fuzzy_cfg(rule_content);
                    if (tokens.size() == 4) {
                        fp.rules.push_back({tokens[0], tokens[1], tokens[2], tokens[3]});
                    }
                }
            }
        }
    }
    in.close();
    return true;
}

} // namespace agent_control_pkg
