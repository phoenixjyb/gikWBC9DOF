# EndEffectorTrajectory.msg
# Rolling window of end-effector trajectory waypoints
# Published by trajectory manager, consumed by IK solver

std_msgs/Header header

# Waypoint data (rolling window: 5-10 waypoints)
geometry_msgs/PoseStamped[] waypoints

# Timing information
float64[] timestamps  # Relative time from header.stamp (seconds)

# Trajectory metadata
uint32 sequence_id           # Incremental ID for tracking
bool is_final_segment        # True if this is the last segment
float64 lookahead_time       # Total lookahead duration (seconds)

# Execution parameters
float64 max_velocity         # Maximum end-effector velocity (m/s)
float64 max_acceleration     # Maximum acceleration (m/s²)
