function [basePose_predicted, score, diagInfo] = computeArmAwareLookahead(ppFollower, robot, q_current, T_ee_target, options)
% computeArmAwareLookahead - Arm-aware Pure Pursuit prediction
%
% This function extends standard Pure Pursuit by sampling multiple candidate
% base poses around the PP prediction and scoring them based on arm reachability,
% joint configuration quality, and manipulability. This makes PP aware of arm
% constraints, leading to better predictions and reduced tracking error.
%
% Syntax:
%   [basePose, score, diagInfo] = computeArmAwareLookahead(ppFollower, robot, q_current, T_ee_target)
%   [basePose, score, diagInfo] = computeArmAwareLookahead(..., Name, Value)
%
% Inputs:
%   ppFollower    - Pure Pursuit controller object
%   robot         - rigidBodyTree robot model
%   q_current     - Current configuration [9x1]: [base(3); arm(6)]
%   T_ee_target   - Target EE pose [4x4 homogeneous transform]
%
% Name-Value Arguments:
%   NumCandidates        - Number of candidate poses to sample (default: 5)
%   PositionRadius       - Radial sampling radius around PP pose [m] (default: 0.10)
%   YawRange             - Angular sampling range ±[rad] (default: deg2rad(15))
%   SampleTime           - Time step for PP prediction [s] (default: 0.1)
%   ReachabilityWeight   - Weight for reachability score (default: 1.0)
%   JointDeviationWeight - Weight for joint deviation penalty (default: 0.5)
%   ManipulabilityWeight - Weight for manipulability score (default: 0.3)
%   EndEffector          - End effector link name (default: 'left_gripper_link')
%   Verbose              - Verbosity level (default: false)
%
% Outputs:
%   basePose_predicted - Best base pose [x, y, theta] (1x3)
%   score              - Quality score of best pose (higher = better)
%   diagInfo           - Diagnostic information struct:
%       .candidates       - All candidate poses [Nx3]
%       .scores           - Scores for all candidates [Nx1]
%       .ppPose           - Standard PP predicted pose [1x3]
%       .numEvaluated     - Number of candidates evaluated
%       .reachabilityRate - Fraction of candidates that were reachable

arguments
    ppFollower
    robot
    q_current (9,1) double
    T_ee_target (4,4) double
    options.NumCandidates (1,1) double = 5
    options.PositionRadius (1,1) double = 0.10
    options.YawRange (1,1) double = deg2rad(15)
    options.SampleTime (1,1) double = 0.1
    options.ReachabilityWeight (1,1) double = 1.0
    options.JointDeviationWeight (1,1) double = 0.5
    options.ManipulabilityWeight (1,1) double = 0.3
    options.EndEffector string = "left_gripper_link"
    options.Verbose (1,1) logical = false
end

%% Step 1: Get standard Pure Pursuit prediction
basePose_current = q_current(1:3)';  % [x, y, theta]
[v_cmd, w_cmd, ~] = ppFollower.step(basePose_current, options.SampleTime);

% Predict base pose after one time step
dt = options.SampleTime;
theta_new = basePose_current(3) + w_cmd * dt;
x_new = basePose_current(1) + v_cmd * cos(basePose_current(3)) * dt;
y_new = basePose_current(2) + v_cmd * sin(basePose_current(3)) * dt;

basePose_pp = [x_new, y_new, theta_new];

if options.Verbose
    fprintf('    PP prediction: [%.3f, %.3f, %.1f°]\n', ...
        basePose_pp(1), basePose_pp(2), rad2deg(basePose_pp(3)));
end

%% Step 2: Sample candidate poses around PP prediction
candidates = zeros(options.NumCandidates, 3);

% First candidate is always the standard PP prediction
candidates(1, :) = basePose_pp;

% Sample additional candidates around PP pose
if options.NumCandidates > 1
    for i = 2:options.NumCandidates
        % Random sampling in polar coordinates
        radius = options.PositionRadius * rand();
        angle = 2 * pi * rand();
        
        % Position offset
        x_offset = radius * cos(angle);
        y_offset = radius * sin(angle);
        
        % Yaw offset
        yaw_offset = options.YawRange * (2 * rand() - 1);
        
        candidates(i, :) = basePose_pp + [x_offset, y_offset, yaw_offset];
    end
end

if options.Verbose
    fprintf('    Generated %d candidates around PP pose\n', options.NumCandidates);
end

%% Step 3: Score each candidate
scores = zeros(options.NumCandidates, 1);
reachable_count = 0;

% Create IK solver for quick reachability checks
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
ik.SolverParameters.MaxIterations = 50;  % Fast IK

% Nominal arm configuration (for deviation penalty)
q_arm_nominal = [0, 0, 0, 0, 0, 0]';  % Home position

for i = 1:options.NumCandidates
    % Build full configuration with candidate base
    q_test = [candidates(i, :)'; q_current(4:9)];
    
    % Quick IK reachability check
    ikWeights = [0.1, 0.1, 1.0, 1.0, 1.0, 1.0];  % Same as Phase 2A
    [q_ik, solInfo] = ik(char(options.EndEffector), T_ee_target, ikWeights, q_test);
    
    % Compute score components
    if solInfo.ExitFlag > 0  % Reachable
        reachable_count = reachable_count + 1;
        
        % 1. Reachability score (1.0 if converged, scaled by pose error)
        pose_error = solInfo.PoseErrorNorm;
        reachability_score = exp(-pose_error * 10);  % Exponential decay
        
        % 2. Joint deviation penalty (prefer configurations near nominal)
        q_arm_ik = q_ik(4:9);
        joint_deviation = norm(q_arm_ik - q_arm_nominal) / sqrt(6);  % Normalized
        deviation_score = exp(-joint_deviation);
        
        % 3. Manipulability score (avoid singularities)
        % Simple approximation: check if any joints near limits
        joint_limits = [-pi, pi; -pi/2, pi/2; -pi, pi; -pi, pi; -pi, pi; -pi, pi];
        near_limit = false;
        for j = 1:6
            if abs(q_arm_ik(j) - joint_limits(j, 1)) < 0.1 || ...
               abs(q_arm_ik(j) - joint_limits(j, 2)) < 0.1
                near_limit = true;
                break;
            end
        end
        manipulability_score = tern(near_limit, 0.5, 1.0);
        
        % Combined score
        scores(i) = options.ReachabilityWeight * reachability_score + ...
                    options.JointDeviationWeight * deviation_score + ...
                    options.ManipulabilityWeight * manipulability_score;
    else
        % Unreachable - very low score
        scores(i) = 0.0;
    end
end

%% Step 4: Select best candidate
[score, best_idx] = max(scores);
basePose_predicted = candidates(best_idx, :);

if options.Verbose
    fprintf('    Best candidate: [%.3f, %.3f, %.1f°], score: %.3f\n', ...
        basePose_predicted(1), basePose_predicted(2), rad2deg(basePose_predicted(3)), score);
    fprintf('    Reachability: %d/%d (%.1f%%)\n', ...
        reachable_count, options.NumCandidates, reachable_count / options.NumCandidates * 100);
end

%% Step 5: Package diagnostic info
diagInfo = struct();
diagInfo.candidates = candidates;
diagInfo.scores = scores;
diagInfo.ppPose = basePose_pp;
diagInfo.numEvaluated = options.NumCandidates;
diagInfo.reachabilityRate = reachable_count / options.NumCandidates;
diagInfo.bestIndex = best_idx;

end

%% Helper function
function result = tern(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
