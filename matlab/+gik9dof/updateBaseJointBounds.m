function updateBaseJointBounds(gikBundle, baseIndices, q_base_pred, yawTolerance, positionTolerance, lateralTolerance)
%UPDATEBASEJOINTBOUNDS Update GIK joint bounds to constrain base motion.
%   UPDATEBASEJOINTBOUNDS(gikBundle, baseIndices, q_base_pred, yawTolerance, positionTolerance)
%   modifies the constraintJointBounds in the GIK bundle to create a "yaw
%   corridor" and position box around the Pure Pursuit prediction.
%
%   UPDATEBASEJOINTBOUNDS(..., positionTolerance, lateralTolerance) supports
%   separate longitudinal and lateral corridor sizing (NEW: Phase 1 improvement)
%
%   Inputs:
%       gikBundle          - Solver bundle from createGikSolver
%       baseIndices        - [1x3] indices for [x, y, theta]
%       q_base_pred        - [3x1] predicted base pose [x, y, theta]
%       yawTolerance       - Yaw corridor half-width (rad, e.g., deg2rad(15))
%       positionTolerance  - Position box half-width (m, e.g., 0.15)
%                            OR longitudinal corridor if lateralTolerance provided
%       lateralTolerance   - (Optional) Lateral corridor half-width (m, e.g., 0.01)
%                            If omitted, uses positionTolerance for both axes
%
%   NEW: When lateralTolerance is provided, corridors are yaw-aligned:
%       - Longitudinal (forward/back): ±positionTolerance along body x-axis
%       - Lateral (left/right): ±lateralTolerance along body y-axis
%       This respects differential drive kinematics (tight lateral control).
%
%   Updates gikBundle.constraints.joint.Bounds in place for base joints.
%
%   Example (standard isotropic):
%       gikBundle = gik9dof.createGikSolver(robot);
%       q_base_pred = [1.5; 0.8; pi/4];
%       gik9dof.updateBaseJointBounds(gikBundle, [1 2 3], q_base_pred, deg2rad(15), 0.15);
%
%   Example (yaw-aligned anisotropic - NEW):
%       % Tight lateral (10mm), dynamic longitudinal (velocity-based)
%       eps_long = max(0.01, abs(v_cmd)*dt + 0.01);  % velocity-limited
%       eps_lat = 0.010;  % tight for diff-drive
%       gik9dof.updateBaseJointBounds(gikBundle, [1 2 3], q_base_pred, ...
%           deg2rad(15), eps_long, eps_lat);
%
%   See also createGikSolver, runStageCPPFirst, runStageCPPFirst_enhanced.

arguments
    gikBundle (1,1) struct
    baseIndices (1,3) double
    q_base_pred (3,1) double
    yawTolerance (1,1) double
    positionTolerance (1,1) double
    lateralTolerance (1,1) double = positionTolerance  % Default: isotropic
end

jointConst = gikBundle.constraints.joint;

% NEW: Yaw-aligned corridor if lateralTolerance differs from positionTolerance
if lateralTolerance ~= positionTolerance
    % Anisotropic corridor aligned with predicted yaw
    % Transform world-frame bounds to body-frame corridor
    theta = q_base_pred(3);
    cos_th = cos(theta);
    sin_th = sin(theta);
    
    % Body-frame corridor: [±eps_long along x_body, ±eps_lat along y_body]
    % Map to world frame bounds (conservative rectangular hull)
    % For tight control, we approximate by rotating the corridor
    dx_max = abs(positionTolerance * cos_th) + abs(lateralTolerance * sin_th);
    dy_max = abs(positionTolerance * sin_th) + abs(lateralTolerance * cos_th);
    
    % X position bounds (world frame)
    jointConst.Bounds(baseIndices(1), :) = [q_base_pred(1) - dx_max, ...
                                             q_base_pred(1) + dx_max];
    
    % Y position bounds (world frame)
    jointConst.Bounds(baseIndices(2), :) = [q_base_pred(2) - dy_max, ...
                                             q_base_pred(2) + dy_max];
else
    % Standard isotropic corridor (backward compatible)
    % X position bounds
    jointConst.Bounds(baseIndices(1), :) = [q_base_pred(1) - positionTolerance, ...
                                             q_base_pred(1) + positionTolerance];
    
    % Y position bounds
    jointConst.Bounds(baseIndices(2), :) = [q_base_pred(2) - positionTolerance, ...
                                             q_base_pred(2) + positionTolerance];
end

% Theta (yaw) corridor bounds (always tight)
theta_min = q_base_pred(3) - yawTolerance;
theta_max = q_base_pred(3) + yawTolerance;
jointConst.Bounds(baseIndices(3), :) = [theta_min, theta_max];

end
