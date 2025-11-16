# ControllerParams message for real-time controller parameter monitoring
# Published by each agent to report current controller configuration

# Controller type identifier
string controller_type  # "pid", "fuzzy", "hybrid"

# PID parameters
float64 pid_kp
float64 pid_ki
float64 pid_kd

# Fuzzy logic parameters
bool fuzzy_enable
float64 fuzzy_wind_scalar

# Hybrid mixing parameters
float64 mix_k_pid
float64 mix_k_fuzzy

# Feed-forward parameters
bool feedforward_enable_drag
bool feedforward_enable_wind
float64 feedforward_k_drag
float64 feedforward_k_wind

# Output limits
float64 output_limit_x_min
float64 output_limit_x_max
float64 output_limit_y_min
float64 output_limit_y_max

# Control frequency
float64 control_frequency_hz
float64 dt
