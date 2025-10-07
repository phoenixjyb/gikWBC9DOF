function start_state = odomToStartState(odom_x, odom_y, odom_quat, grid, params)
%ODOMTOSTARTSTATE Convert odometry to Hybrid A* start state
%   Transforms ROS2 odometry (position + quaternion) to planner state
%
%   Inputs:
%       odom_x, odom_y: Position [m] in map frame
%       odom_quat: [w, x, y, z] quaternion orientation
%       grid: OccupancyGrid2D object
%       params: Planner parameters struct
%
%   Output:
%       start_state: GridState2D struct (for now, will be HybridState later)

    % Extract yaw from quaternion (assume planar motion, pitch=roll=0)
    qw = odom_quat(1);
    qx = odom_quat(2);
    qy = odom_quat(3);
    qz = odom_quat(4);
    
    % Quaternion to yaw (heading angle)
    % yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
    theta = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
    
    % Create start state
    start_state.x = odom_x;
    start_state.y = odom_y;
    start_state.theta = theta;  % Will be used in Hybrid A*
    
    % Discretize to grid
    start_state.grid_x = grid.worldToGridX(odom_x);
    start_state.grid_y = grid.worldToGridY(odom_y);
    
    % Initialize costs
    start_state.g_cost = 0.0;
    start_state.h_cost = 0.0;
    start_state.f_cost = 0.0;
    
    % Parent tracking
    start_state.parent_x = int32(-1);
    start_state.parent_y = int32(-1);
end
