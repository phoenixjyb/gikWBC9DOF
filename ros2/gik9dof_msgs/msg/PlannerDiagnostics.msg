# PlannerDiagnostics.msg
# Hybrid A* planner diagnostic information

std_msgs/Header header

# Planning result
bool success                    # Path found successfully
float64 planning_time_sec       # Planning time (seconds)
float64 iterations              # Number of iterations
float64 nodes_expanded          # Number of nodes expanded
float64 path_cost               # Total path cost
float64 path_length             # Total path length (meters)

# Path information
int32 num_waypoints             # Number of waypoints in path
float64 estimated_time          # Estimated execution time (seconds)

# Current tracking status
int32 current_waypoint_idx      # Current waypoint being tracked
float64 distance_to_goal        # Euclidean distance to goal (meters)
float64 heading_error           # Heading error to goal (radians)

# Stage B specific
string stage_b_mode             # "B1" or "B2"
bool replan_triggered           # Replanning triggered this cycle
