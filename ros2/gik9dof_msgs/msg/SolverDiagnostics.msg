# SolverDiagnostics.msg
# Diagnostic information from IK solver
# Published by gik9dof_solver node for monitoring and debugging

std_msgs/Header header

# Solver performance
string status                # Solver status: "success", "failed", "max_iterations", etc.
uint32 iterations            # Number of iterations taken
int32 exit_flag              # Solver exit flag (MATLAB solver specific)
float64 solve_time_ms        # Actual solve time (milliseconds)
float64 pose_error_norm      # Final pose error magnitude

# Constraint satisfaction
float64 position_error       # End-effector position error (meters)
float64 orientation_error    # End-effector orientation error (radians)
bool joint_limits_violated   # True if any joint hit limits
bool distance_constraint_met # True if ALL enabled distance constraints satisfied

# Distance constraints (20 total) - NEW detailed reporting
int32[] dist_body_indices       # Body indices for each constraint [20]
int32[] dist_ref_body_indices   # Reference body indices [20]
float64[] dist_lower_bounds     # Lower bounds configured [20]
float64[] dist_upper_bounds     # Upper bounds configured [20]
float64[] dist_weights          # Weights (0.0=disabled, >0.0=enabled) [20]
float64[] dist_violations       # Actual violation values from solver [20]
bool[] dist_constraint_met      # Per-constraint status (true=satisfied) [20]
uint8 num_active_constraints    # Number of enabled constraints (weight>0)
uint8 num_violated_constraints  # Number of violated constraints

# Current state
float64[] current_config     # Current joint configuration (9 DOF)
float64[] target_config      # Solved target configuration (9 DOF)
geometry_msgs/Pose current_ee_pose   # Current end-effector pose
geometry_msgs/Pose target_ee_pose    # Target end-effector pose

# Timestamps for latency analysis
builtin_interfaces/Time trajectory_stamp  # Original trajectory timestamp
builtin_interfaces/Time solve_start       # When solve started
builtin_interfaces/Time solve_end         # When solve finished
