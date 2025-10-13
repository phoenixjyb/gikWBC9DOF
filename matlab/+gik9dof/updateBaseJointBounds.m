function updateBaseJointBounds(gikBundle, baseIndices, q_base_pred, yawTolerance, positionTolerance)
%UPDATEBASEJOINTBOUNDS Update GIK joint bounds to constrain base motion.
%   UPDATEBASEJOINTBOUNDS(gikBundle, baseIndices, q_base_pred, yawTolerance, positionTolerance)
%   modifies the constraintJointBounds in the GIK bundle to create a "yaw
%   corridor" and position box around the Pure Pursuit prediction.
%
%   Inputs:
%       gikBundle          - Solver bundle from createGikSolver
%       baseIndices        - [1x3] indices for [x, y, theta]
%       q_base_pred        - [3x1] predicted base pose [x, y, theta]
%       yawTolerance       - Yaw corridor half-width (rad, e.g., deg2rad(15))
%       positionTolerance  - Position box half-width (m, e.g., 0.15)
%
%   Updates gikBundle.constraints.joint.Bounds in place for base joints.
%
%   Example:
%       gikBundle = gik9dof.createGikSolver(robot);
%       q_base_pred = [1.5; 0.8; pi/4];
%       gik9dof.updateBaseJointBounds(gikBundle, [1 2 3], q_base_pred, deg2rad(15), 0.15);
%
%   See also createGikSolver, runStageCPPFirst.

arguments
    gikBundle (1,1) struct
    baseIndices (1,3) double
    q_base_pred (3,1) double
    yawTolerance (1,1) double
    positionTolerance (1,1) double
end

jointConst = gikBundle.constraints.joint;

% X position bounds
jointConst.Bounds(baseIndices(1), :) = [q_base_pred(1) - positionTolerance, ...
                                         q_base_pred(1) + positionTolerance];

% Y position bounds
jointConst.Bounds(baseIndices(2), :) = [q_base_pred(2) - positionTolerance, ...
                                         q_base_pred(2) + positionTolerance];

% Theta (yaw) corridor bounds
theta_min = q_base_pred(3) - yawTolerance;
theta_max = q_base_pred(3) + yawTolerance;
jointConst.Bounds(baseIndices(3), :) = [theta_min, theta_max];

end
