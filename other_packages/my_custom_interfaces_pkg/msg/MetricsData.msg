# MetricsData message for controller performance monitoring

# Position errors
float64 error_x
float64 error_y
float64 error_z
float64 error_magnitude

# RMSE (Root Mean Square Error)
float64 rmse_x
float64 rmse_y
float64 rmse_z
float64 rmse_total

# Integral metrics
float64 iae_x
float64 iae_y
float64 itae_x
float64 itae_y

# Response characteristics
float64 settling_time
bool is_settled
float64 max_overshoot_x
float64 max_overshoot_y
