# MetricsData message for controller performance monitoring

# Current and target positions (for step response analysis)
float64 current_x
float64 current_y
float64 current_z
float64 target_x
float64 target_y
float64 target_z

# Position errors
float64 error_x
float64 error_y
float64 error_z
float64 error_magnitude

# RMSE (Root Mean Square Error) - per-waypoint
float64 rmse_x
float64 rmse_y
float64 rmse_z
float64 rmse_total

# Integral metrics - per-waypoint
float64 iae_x
float64 iae_y
float64 itae_x
float64 itae_y

# Response characteristics - per-waypoint
float64 settling_time
bool is_settled
float64 max_overshoot_x
float64 max_overshoot_y

# Mission-wide cumulative metrics - never reset, accumulated across all waypoints
float64 mission_rmse            
float64 mission_itae_x          
float64 mission_itae_y          
float64 mission_elapsed_time    
int32 waypoint_count            

# Wind disturbance metrics 
float64 wind_velocity_x         
float64 wind_velocity_y         
float64 wind_velocity_z         
float64 wind_magnitude          
float64 disturbance_rejection_ratio  # |error| / |wind|
float64 wind_normalized_rmse    # RMSE / |wind|
float64 wind_exposure_time      # Cumulative time under wind > 1.0 m/s [s]
