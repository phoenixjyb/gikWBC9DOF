%% Test Phase 2A with MaxIterations=1500 (Verification Test)
% This script tests if limiting MaxIterations to 1500 reproduces the
% second-half failure observed in the staged run.
%
% Purpose: Confirm that MaxIterations=1500 is the root cause of failures

clear; clc;

fprintf('====================================================\n');
fprintf('  Phase 2A Test: MaxIterations=1500 (Verification)\n');
fprintf('====================================================\n\n');

%% Setup
fprintf('Setting up robot and trajectory...\n');

% Create robot model
robot = gik9dof.createRobotModel();

% Load test trajectory
trajectory_file = '1_pull_world_scaled.json';
if ~isfile(trajectory_file)
    error('Trajectory file not found: %s', trajectory_file);
end

% Load trajectory
raw = jsondecode(fileread(trajectory_file));
if ~isfield(raw, "poses")
    error("JSON file %s does not contain a 'poses' field.", trajectory_file);
end

numWaypoints = numel(raw.poses);
poses = repmat(eye(4), 1, 1, numWaypoints);
posXYZ = zeros(3, numWaypoints);

for k = 1:numWaypoints
    entry = raw.poses(k);
    position = reshape(entry.position, [], 1);
    posXYZ(:,k) = position;
    
    quatXYZW = reshape(entry.orientation, 1, []);
    quatWXYZ = [quatXYZW(4), quatXYZW(1:3)];
    quatWXYZ = quatWXYZ ./ norm(quatWXYZ);
    
    T = quat2tform(quatWXYZ);
    T(1:3,4) = position;
    poses(:,:,k) = T;
end

trajStruct = struct();
trajStruct.Poses = poses;
trajStruct.EndEffectorPositions = posXYZ;
trajStruct.EndEffectorName = "left_gripper_link";
fprintf('  Loaded %d waypoints from %s\n', numWaypoints, trajectory_file);

% Load chassis parameters
chassisParams = gik9dof.control.loadChassisProfile('wide_track');
fprintf('  Loaded chassis profile: wide_track\n');

% Home configuration as starting point
q0 = homeConfiguration(robot);

%% Run Phase 2A with MaxIterations=1500
fprintf('\n--- Running Phase 2A with MaxIterations=1500 ---\n');
fprintf('(Testing if iteration limit causes second-half failures)\n\n');

tic;
log_phase2a = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, q0, ...
    'ChassisParams', chassisParams, ...
    'YawTolerance', deg2rad(15), ...
    'PositionTolerance', 0.15, ...
    'EEErrorTolerance', 0.015, ...
    'MaxIterations', 1500, ...  % ← REDUCED from 2000 to match staged run
    'LookaheadDistance', 0.8, ...
    'LookaheadMin', 0.15, ...
    'UseAdaptiveLookahead', true, ...
    'UseMicroSegment', true, ...
    'UseWarmStarting', true, ...
    'UseVelocityCorridor', true, ...
    'LogLateralVelocity', true, ...
    'RelaxedTolerances', true, ...
    'EpsLatMax', 0.015, ...
    'EpsLongMin', 0.05, ...
    'UseOrientationZNominal', true, ...    % Phase 2A enabled
    'OrientationWeight', 1.0, ...
    'PositionWeightXY', 0.1, ...
    'PositionWeightZ', 1.0, ...
    'VerboseLevel', 1);
phase2a_time = toc;

fprintf('\nResults:\n');
fprintf('  Mean EE Error: %.1f mm\n', log_phase2a.avgEEError * 1000);
fprintf('  Max EE Error: %.1f mm\n', log_phase2a.maxEEError * 1000);
fprintf('  Fallback Rate: %.1f%%\n', log_phase2a.fallbackRate * 100);
fprintf('  Convergence Rate: %.1f%%\n', sum(log_phase2a.successMask) / numel(log_phase2a.successMask) * 100);
fprintf('  Execution Time: %.2f s\n', phase2a_time);

%% Analyze First Half vs Second Half
fprintf('\n====================================================\n');
fprintf('  FIRST HALF vs SECOND HALF ANALYSIS\n');
fprintf('====================================================\n\n');

numWaypoints = length(log_phase2a.positionErrorNorm);
firstHalf = 1:floor(numWaypoints/2);
secondHalf = (floor(numWaypoints/2)+1):numWaypoints;

err_mm = log_phase2a.positionErrorNorm * 1000;
err_first = err_mm(firstHalf);
err_second = err_mm(secondHalf);

conv_first = log_phase2a.successMask(firstHalf);
conv_second = log_phase2a.successMask(secondHalf);

iter_first = log_phase2a.gikIterations(firstHalf);
iter_second = log_phase2a.gikIterations(secondHalf);

fprintf('First Half (waypoints 1-%d):\n', length(firstHalf));
fprintf('  Mean Error: %.1f mm\n', mean(err_first));
fprintf('  Max Error: %.1f mm\n', max(err_first));
fprintf('  Convergence: %.1f%% (%d/%d)\n', 100*sum(conv_first)/length(conv_first), sum(conv_first), length(conv_first));
fprintf('  Mean Iterations: %.0f\n', mean(iter_first));
fprintf('  Waypoints hitting limit (1500): %d (%.1f%%)\n', sum(iter_first >= 1500), 100*sum(iter_first >= 1500)/length(iter_first));

fprintf('\nSecond Half (waypoints %d-%d):\n', length(firstHalf)+1, numWaypoints);
fprintf('  Mean Error: %.1f mm\n', mean(err_second));
fprintf('  Max Error: %.1f mm\n', max(err_second));
fprintf('  Convergence: %.1f%% (%d/%d)\n', 100*sum(conv_second)/length(conv_second), sum(conv_second), length(conv_second));
fprintf('  Mean Iterations: %.0f\n', mean(iter_second));
fprintf('  Waypoints hitting limit (1500): %d (%.1f%%)\n', sum(iter_second >= 1500), 100*sum(iter_second >= 1500)/length(iter_second));

%% Check Around Waypoint 105-106 Transition
fprintf('\n--- Critical Transition (Waypoints 100-110) ---\n');
fprintf('WP#   Conv?  Error(mm)  Iterations\n');
fprintf('---   -----  ---------  ----------\n');
for k = 100:110
    convStr = ternary(log_phase2a.successMask(k), 'YES', 'NO ');
    fprintf('%3d   %s    %8.2f   %8d\n', k, convStr, err_mm(k), log_phase2a.gikIterations(k));
end

%% Save Results
resultsDir = sprintf('results/maxiter1500_%s', datestr(now, 'yyyymmdd_HHMMSS'));
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

logFile = fullfile(resultsDir, 'log_phase2a_maxiter1500.mat');
save(logFile, 'log_phase2a');
fprintf('\n✓ Results saved to: %s\n', logFile);

%% Conclusion
fprintf('\n====================================================\n');
fprintf('  CONCLUSION\n');
fprintf('====================================================\n\n');

if mean(err_second) > 100
    fprintf('❌ HYPOTHESIS CONFIRMED!\n');
    fprintf('   Reducing MaxIterations from 2000 to 1500 causes second-half failures.\n');
    fprintf('   Second half mean error: %.1f mm (>100mm indicates failure)\n', mean(err_second));
    fprintf('   Waypoints hitting iteration limit: %d/%d in second half\n', sum(iter_second >= 1500), length(iter_second));
elseif mean(err_second) < 10
    fprintf('✓ HYPOTHESIS REJECTED\n');
    fprintf('   MaxIterations=1500 is sufficient. Second half works fine (%.1f mm).\n', mean(err_second));
    fprintf('   Need to investigate other factors.\n');
else
    fprintf('⚠️  INCONCLUSIVE\n');
    fprintf('   Second half error: %.1f mm (borderline case)\n', mean(err_second));
    fprintf('   Need more investigation.\n');
end
function result = ternary(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
