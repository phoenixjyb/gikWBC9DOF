function [qNext, solverInfo] = solveGIKStepRealtime(robot, solver, qCurrent, targetPose, ...
    distBodyIndices, distRefBodyIndices, distBoundsLower, distBoundsUpper, distWeights)
%SOLVEGIKSTEPREALTIME GIK step with up to 20 distance constraints for code generation
%   This version supports multiple distance constraints with fixed-size arrays
%   to avoid dynamic memory allocation in generated C++ code.
%
%   Inputs:
%       robot              - rigidBodyTree object (9 DOF)
%       solver             - generalizedInverseKinematics object
%       qCurrent           - Current joint configuration (9x1 double)
%       targetPose         - Target end-effector pose (4x4 homogeneous transform)
%       distBodyIndices    - Body indices for distance constraints (20x1 int32)
%                            Index into robot body list: 0 = disabled, 1-11 = valid body
%       distRefBodyIndices - Reference body indices (20x1 int32)
%                            Index into robot body list: 0 = disabled, 1-11 = valid body
%       distBoundsLower    - Lower bounds for each constraint (20x1 double)
%       distBoundsUpper    - Upper bounds for each constraint (20x1 double)
%       distWeights        - Weights for each constraint (20x1 double)
%                            Weight = 0 disables the constraint
%
%   Outputs:
%       qNext      - Next joint configuration (9x1 double)
%       solverInfo - Solver diagnostics structure
%
%   Body Index Mapping (for distBodyIndices and distRefBodyIndices):
%       0  = DISABLED (constraint not used)
%       1  = 'base' (world frame)
%       2  = 'base_link_x'
%       3  = 'base_link_y'
%       4  = 'abstract_chassis_link'
%       5  = 'left_arm_base_link'
%       6  = 'left_arm_link1'
%       7  = 'left_arm_link2'
%       8  = 'left_arm_link3'
%       9  = 'left_arm_link4'
%       10 = 'left_arm_link5'
%       11 = 'left_arm_link6'
%       12 = 'left_gripper_link'
%
%   Example: To constrain gripper (12) to stay > 0.3m from chassis (4):
%       distBodyIndices(1) = 12
%       distRefBodyIndices(1) = 4
%       distBoundsLower(1) = 0.3
%       distBoundsUpper(1) = 100.0
%       distWeights(1) = 1.0
%
%#codegen

% Fixed body name list (matches buildRobotForCodegen.m)
% Index 0 is reserved for "disabled"
persistent bodyNames
if isempty(bodyNames)
    bodyNames = {'base', 'base_link_x', 'base_link_y', 'abstract_chassis_link', ...
                 'left_arm_base_link', 'left_arm_link1', 'left_arm_link2', ...
                 'left_arm_link3', 'left_arm_link4', 'left_arm_link5', ...
                 'left_arm_link6', 'left_gripper_link'};
end

coder.inline('never'); % Force function call for better debugging

%% Create pose and joint constraints (always active)
poseConstraint = constraintPoseTarget('left_gripper_link');
poseConstraint.TargetTransform = targetPose;

jointConstraint = constraintJointBounds(robot);

%% Create 20 fixed distance constraint objects
% All constraints are created, but only active ones (weight > 0) affect the solver
distConstraints = cell(1, 20);

for i = 1:20
    % Get body indices
    bodyIdx = int32(distBodyIndices(i));
    refBodyIdx = int32(distRefBodyIndices(i));
    
    % Default to disabled (gripper to base with zero weight)
    bodyName = 'left_gripper_link';
    refBodyName = 'base';
    lowerBound = 0.0;
    upperBound = 100.0;
    weight = 0.0;
    
    % Enable constraint if indices are valid and weight > 0
    if bodyIdx >= 1 && bodyIdx <= 12 && refBodyIdx >= 1 && refBodyIdx <= 12
        if distWeights(i) > 0
            bodyName = bodyNames{bodyIdx};
            refBodyName = bodyNames{refBodyIdx};
            
            % Process bounds
            if distBoundsLower(i) <= 0
                lowerBound = 0.0;
            else
                lowerBound = distBoundsLower(i);
            end
            
            if distBoundsUpper(i) <= 0 || distBoundsUpper(i) > 100.0
                upperBound = 100.0;  % Cap at 100m (effectively unlimited)
            else
                upperBound = distBoundsUpper(i);
            end
            
            weight = distWeights(i);
        end
    end
    
    % Create constraint object
    distConstraints{i} = constraintDistanceBounds(bodyName);
    distConstraints{i}.ReferenceBody = refBodyName;
    distConstraints{i}.Bounds = [lowerBound, upperBound];
    distConstraints{i}.Weights = weight;
end

%% Solve IK with all constraints
% The solver automatically ignores constraints with zero weight
[qNext, solverInfo] = solver(qCurrent, poseConstraint, jointConstraint, ...
    distConstraints{1}, distConstraints{2}, distConstraints{3}, distConstraints{4}, ...
    distConstraints{5}, distConstraints{6}, distConstraints{7}, distConstraints{8}, ...
    distConstraints{9}, distConstraints{10}, distConstraints{11}, distConstraints{12}, ...
    distConstraints{13}, distConstraints{14}, distConstraints{15}, distConstraints{16}, ...
    distConstraints{17}, distConstraints{18}, distConstraints{19}, distConstraints{20});

end
