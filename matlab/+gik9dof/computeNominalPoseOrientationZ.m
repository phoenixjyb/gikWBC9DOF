function [nominalConfig, basePose, diagInfo] = computeNominalPoseOrientationZ(robot, T_ee_target, q_current, options)
%COMPUTENOMINALPOSEORIENTATIONZ Generate nominal pose matching EE orientation+height.
%   [nominalConfig, basePose] = computeNominalPoseOrientationZ(robot, T_ee_target, q_current)
%   computes a nominal configuration that prioritizes matching the target end-effector
%   orientation and Z-height, while allowing the base X-Y position to vary.
%
%   This is a key Phase 2A improvement: instead of treating all EE position dimensions
%   equally, we recognize that orientation and height are more critical for manipulation
%   tasks, and the mobile base can adjust X-Y position to achieve better arm configurations.
%
%   Inputs:
%       robot        - rigidBodyTree (9-DOF mobile manipulator)
%       T_ee_target  - 4x4 SE(3) target pose for end-effector
%       q_current    - 9x1 current configuration (optional, default: home)
%       options      - struct with optional fields:
%           .EndEffector      - string, EE name (default: "left_gripper_link")
%           .BaseIndices      - [1,2,3] base joint indices
%           .ArmIndices       - [4:9] arm joint indices
%           .OrientationWeight - scalar, weight for orientation (default: 1.0)
%           .PositionWeightXY  - scalar, weight for x-y position (default: 0.1)
%           .PositionWeightZ   - scalar, weight for z position (default: 1.0)
%           .MaxIterations    - scalar, IK iterations (default: 500)
%
%   Outputs:
%       nominalConfig - 9x1 nominal configuration (base + arm)
%       basePose      - [x, y, theta] extracted base pose
%       diagInfo      - struct with diagnostic information:
%           .xy_error      - XY position error (m)
%           .z_error       - Z position error (m)
%           .orient_error  - Orientation error (Frobenius norm)
%           .ik_iterations - Number of IK iterations used
%           .success       - Boolean, whether IK converged
%
%   Example:
%       T_target = trajStruct.Poses(:,:,k);
%       [q_nom, base_nom, info] = computeNominalPoseOrientationZ(robot, T_target, q_current);
%       fprintf('Nominal XY error: %.3f m, Z error: %.3f m\n', info.xy_error, info.z_error);
%
%   Algorithm:
%       1. Extract target position p_target = [x, y, z] and rotation R_target
%       2. Create weighted IK solver with weights [w_x, w_y, w_z, w_rx, w_ry, w_rz]
%       3. Set w_x = w_y = 0.1 (relax), w_z = 1.0 (tight), w_orient = 1.0 (tight)
%       4. Solve IK starting from q_current
%       5. Extract base pose [x, y, theta] from solution
%       6. Compute diagnostics (position/orientation errors)
%
%   Reference:
%       method4_Guide.md lines 197-213 (orientation+Z nominal approach)
%
%   See also: baseSeedFromEE, inverseKinematics, generalizedInverseKinematics

arguments
    robot (1,1) rigidBodyTree
    T_ee_target (4,4) double
    q_current (:,1) double = homeConfiguration(robot)
    options.EndEffector (1,1) string = "left_gripper_link"
    options.BaseIndices (1,3) double = [1 2 3]
    options.ArmIndices (1,6) double = [4 5 6 7 8 9]
    options.OrientationWeight (1,1) double = 1.0
    options.PositionWeightXY (1,1) double = 0.1
    options.PositionWeightZ (1,1) double = 1.0
    options.MaxIterations (1,1) double = 500
end

% Extract target EE position and orientation
p_target = T_ee_target(1:3, 4);
R_target = T_ee_target(1:3, 1:3);

% Create IK solver with weighted constraints
% Weights: [x, y, z, rot_x, rot_y, rot_z]
ikWeights = [options.PositionWeightXY, ...  % X position (relaxed)
             options.PositionWeightXY, ...  % Y position (relaxed)
             options.PositionWeightZ, ...   % Z position (tight)
             options.OrientationWeight, ... % Rotation about X (tight)
             options.OrientationWeight, ... % Rotation about Y (tight)
             options.OrientationWeight];    % Rotation about Z (tight)

% Create inverse kinematics solver
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = options.MaxIterations;

% Solve IK with orientation+Z priority
[nominalConfig, solInfo] = ik(char(options.EndEffector), T_ee_target, ...
    ikWeights, q_current);

% Extract base pose
basePose = nominalConfig(options.BaseIndices);

% Validate solution quality
T_ee_achieved = getTransform(robot, nominalConfig, char(options.EndEffector));
p_achieved = T_ee_achieved(1:3, 4);
R_achieved = T_ee_achieved(1:3, 1:3);

% Position error breakdown
pos_error = p_achieved - p_target;
xy_error = norm(pos_error(1:2));
z_error = abs(pos_error(3));

% Orientation error (Frobenius norm of rotation difference)
% ||R_target' * R_achieved - I||_F measures rotation difference
R_error = R_target' * R_achieved - eye(3);
orient_error = norm(R_error, 'fro');

% Store diagnostics
diagInfo = struct();
diagInfo.xy_error = xy_error;
diagInfo.z_error = z_error;
diagInfo.orient_error = orient_error;
diagInfo.ik_iterations = solInfo.Iterations;
diagInfo.success = (solInfo.Iterations < options.MaxIterations);
diagInfo.position_error_total = norm(pos_error);

% Ensure column vector output
nominalConfig = nominalConfig(:);

end
