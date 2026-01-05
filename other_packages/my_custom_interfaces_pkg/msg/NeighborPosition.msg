# NeighborPosition message for Plan S: Safety Layer as Perception Hub
# Represents a single neighbor's position relative to the receiving agent

# Neighbor identification
string agent_id              # e.g., "agent_0", "agent_1"

# Neighbor's absolute position in world frame
float64 position_x           # [m]
float64 position_y           # [m]
float64 position_z           # [m]

# Relative information (from receiver's perspective)
float64 distance             # Euclidean distance to this neighbor [m]
float64 bearing_rad          # Bearing angle from receiver to neighbor [rad]

# Neighbor's velocity (for prediction/collision avoidance)
float64 velocity_x           # [m/s]
float64 velocity_y           # [m/s]
