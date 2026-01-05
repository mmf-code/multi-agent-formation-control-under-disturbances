# NeighborInfo message for Plan S: Safety Layer as Perception Hub
# Contains the list of neighbors within perception range for a single agent
#
# This message is published by the Collision Safety Layer (acting as Perception Hub)
# to provide each agent with local situational awareness without requiring
# point-to-point communication between all agents (O(N) vs O(N^2) complexity)

std_msgs/Header header

# Receiver agent identification
string agent_id              # Agent receiving this neighbor info (e.g., "agent_0")

# Perception parameters
float64 perception_radius    # Maximum distance for neighbor inclusion [m]
int32 neighbor_count         # Number of neighbors in list

# List of neighbors within perception radius
my_custom_interfaces_pkg/NeighborPosition[] neighbors

# Safety metrics (computed by Perception Hub)
float64 min_neighbor_distance    # Minimum distance to any neighbor [m]
string closest_neighbor_id       # ID of the closest neighbor
bool is_near_collision           # True if min_distance < safety_threshold
