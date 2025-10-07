function state = HybridState()
%HYBRIDSTATE Create empty SE(2) state for Hybrid A* planning
%   state = HybridState() returns struct for front-diff + passive-rear robot
%
%   Platform: WHEELTEC front differential + passive rear omniwheels
%   - Wheelbase: 0.36 m
%   - Track: 0.573 m  
%   - Min turning radius: ~0.34 m (NOT zero - passive rear constraint)
%   - Control: (Vx, Wz) via front axle differential
%
%   State struct fields:
%       .x          - X position in world frame [m] (continuous)
%       .y          - Y position in world frame [m] (continuous)
%       .theta      - Heading angle [rad], 0=+X axis, CCW+ (continuous)
%       
%       .grid_x     - Discrete grid X index (1-based)
%       .grid_y     - Discrete grid Y index (1-based)
%       .theta_bin  - Discrete heading bin index (1-based, 16 bins = 22.5° resolution)
%       
%       .Vx         - Commanded forward velocity [m/s] (to reach this state)
%       .Wz         - Commanded yaw rate [rad/s] (to reach this state)
%       .dt         - Duration of command [s]
%       
%       .g          - Cost from start [m or s]
%       .h          - Heuristic cost to goal [m or s]
%       .f          - Total cost g + h [m or s]
%       
%       .parent_idx - Index of parent state in state list (0 = no parent)
%       .is_valid   - Boolean flag (1 = valid state, 0 = invalid/obstacle)
%
%   Grid discretization:
%       grid_resolution = 0.1 m    (10 cm spatial)
%       theta_resolution = pi/8    (16 bins, 22.5° heading)
%       grid_size = 200 x 200 x 16 (20m x 20m x 360°)
%
%   Coordinate conventions:
%       - World frame: X-right, Y-forward, Z-up
%       - Grid frame: origin at bottom-left corner
%       - Theta: 0 = +X axis (right), pi/2 = +Y axis (forward)
%
%   See also planHybridAStar, computeMotionPrimitive, worldToGrid

% Continuous state (SE(2) manifold)
state.x = 0.0;         % [m]
state.y = 0.0;         % [m]
state.theta = 0.0;     % [rad]

% Discrete state (lattice indices)
state.grid_x = int32(0);      % Grid X index
state.grid_y = int32(0);      % Grid Y index  
state.theta_bin = int32(0);   % Heading bin (1-16)

% Command to reach this state (from parent)
state.Vx = 0.0;        % [m/s] Forward velocity
state.Wz = 0.0;        % [rad/s] Yaw rate
state.dt = 0.0;        % [s] Duration

% A* search costs
state.g = 0.0;         % Cost from start
state.h = 0.0;         % Heuristic to goal
state.f = 0.0;         % Total cost (g + h)

% Path tracking
state.parent_idx = int32(0);  % Index in state array (0 = root/start)

% Validity flag
state.is_valid = true;        % False if in collision or out of bounds

end
