% Test Stage C with MaxIterations=1500 (matching staged run default)
% This will confirm if the iteration limit is the root cause

clearvars; clc;

fprintf('========================================\n');
fprintf('  Stage C Test: MaxIterations=1500\n');
fprintf('========================================\n');
fprintf('Testing Phase 2A with REDUCED iterations to match staged run\n\n');

%% Setup
fprintf('Setting up robot and trajectory...\n');
robot = gik9dof.createRobotModel();

% Load trajectory
trajData = jsondecode(fileread('1_pull_world_scaled.json'));
poses_raw = trajData.poses;
nPoses = length(poses_raw);

% Build trajectory struct
trajStruct = struct();
trajStruct.Poses = zeros(4, 4, nPoses);
trajStruct.EndEffectorName = "left_gripper_link";

for i = 1:nPoses
    entry = poses_raw(i);
    position = reshape(entry.position, [], 1);
    quatXYZW = reshape(entry.orientation, 1, []);
    quatWXYZ = [quatXYZW(4), quatXYZW(1:3)];
    quatWXYZ = quatWXYZ ./ norm(quatWXYZ);
    T = quat2tform(quatWXYZ);
    T(1:3, 4) = position;
    
    trajStruct.Poses(:, :, i) = T;
end

fprintf('Robot ready, trajectory loaded (%d waypoints)\n\n', nPoses);

%% Configure chassis parameters
chassisParams = struct();
chassisParams.track = 0.574;
chassisParams.wheel_base = 0.360;
chassisParams.wheel_speed_max = 3.3;
chassisParams.vx_max = 1.5;
chassisParams.vx_min = -0.4;
chassisParams.wz_max = 2.5;
chassisParams.accel_limit = 0.8;
chassisParams.decel_limit = 1.8;
chassisParams.jerk_limit = 5.0;
chassisParams.lookahead_base = 0.8;
chassisParams.lookahead_vel_gain = 0.3;
chassisParams.lookahead_time_gain = 0.05;
chassisParams.heading_kp = 1.2;
chassisParams.heading_ki = 0.0;
chassisParams.heading_kd = 0.1;
chassisParams.feedforward_gain = 0.9;
chassisParams.goal_tolerance = 0.1;
chassisParams.reverse_enabled = false;
chassisParams.interp_spacing_min = 0.05;
chassisParams.interp_spacing_max = 0.2;

%% Run test with MaxIterations=1500
fprintf('--- Running Stage C with MaxIterations=1500 ---\n');
fprintf('(This matches the staged run default)\n\n');

q0 = homeConfiguration(robot);

tic;
log = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, q0, ...
    'ChassisParams', chassisParams, ...
    'YawTolerance', deg2rad(15), ...
    'PositionTolerance', 0.15, ...
    'EEErrorTolerance', 0.015, ...
    'MaxIterations', 1500, ...        % ← REDUCED from 2000
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
totalTime = toc;

%% Analyze results
fprintf('\n========================================\n');
fprintf('  RESULTS (MaxIterations=1500)\n');
fprintf('========================================\n\n');

posErr = log.positionErrorNorm * 1000;  % mm
N = length(posErr);

fprintf('Overall Performance:\n');
fprintf('  Mean EE Error: %.1f mm\n', mean(posErr));
fprintf('  Max EE Error: %.1f mm\n', max(posErr));
fprintf('  Fallback Rate: %.1f%% (%d/%d waypoints)\n', ...
    log.fallbackRate * 100, sum(log.fallbackUsed), N);
fprintf('  Convergence Rate: %.1f%%\n', 100 * sum(log.successMask) / N);
fprintf('  Execution Time: %.1f s\n', totalTime);

fprintf('\nError Distribution:\n');
fprintf('  < 5mm:    %.1f%% (%d waypoints)\n', 100*sum(posErr < 5)/N, sum(posErr < 5));
fprintf('  5-50mm:   %.1f%% (%d waypoints)\n', 100*sum(posErr >= 5 & posErr < 50)/N, sum(posErr >= 5 & posErr < 50));
fprintf('  50-500mm: %.1f%% (%d waypoints)\n', 100*sum(posErr >= 50 & posErr < 500)/N, sum(posErr >= 50 & posErr < 500));
fprintf('  > 500mm:  %.1f%% (%d waypoints)\n', 100*sum(posErr >= 500)/N, sum(posErr >= 500));

% First half vs second half
firstHalf = posErr(1:floor(N/2));
secondHalf = posErr(floor(N/2)+1:end);

fprintf('\nFirst Half vs Second Half:\n');
fprintf('  First half (1-%d):  Mean=%.1fmm, Failures=%d\n', ...
    floor(N/2), mean(firstHalf), sum(firstHalf > 500));
fprintf('  Second half (%d-%d): Mean=%.1fmm, Failures=%d\n', ...
    floor(N/2)+1, N, mean(secondHalf), sum(secondHalf > 500));

% Check convergence around waypoint 100-110
fprintf('\nConvergence Around Waypoint 100-110:\n');
fprintf('WP#   Conv?  Error(mm)  Iterations\n');
fprintf('---   -----  ---------  ----------\n');
for k = 100:110
    convStr = ternary(log.successMask(k), 'YES', 'NO ');
    fprintf('%3d   %s    %8.2f   %8d\n', k, convStr, posErr(k), log.gikIterations(k));
end

%% Comparison
fprintf('\n========================================\n');
fprintf('  COMPARISON\n');
fprintf('========================================\n\n');

fprintf('Expected (with MaxIterations=2000):\n');
fprintf('  Mean EE Error: 1.2 mm\n');
fprintf('  Second half: Mean=2.4mm, Failures=0\n\n');

fprintf('Actual (with MaxIterations=1500):\n');
fprintf('  Mean EE Error: %.1f mm\n', mean(posErr));
fprintf('  Second half: Mean=%.1fmm, Failures=%d\n\n', mean(secondHalf), sum(secondHalf > 500));

if mean(secondHalf) > 100
    fprintf('✅ HYPOTHESIS CONFIRMED!\n');
    fprintf('   MaxIterations=1500 causes second-half failures\n');
    fprintf('   Increasing to 2000 should fix the staged run\n');
else
    fprintf('❌ HYPOTHESIS REJECTED!\n');
    fprintf('   MaxIterations is NOT the issue\n');
    fprintf('   Need to investigate further\n');
end

%% Save results
resultsDir = 'results/stagec_maxiter1500';
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
logFile = fullfile(resultsDir, sprintf('log_%s.mat', timestamp));
save(logFile, 'log', '-v7.3');

fprintf('\nResults saved to: %s\n', logFile);

fprintf('\n========================================\n');
fprintf('  Test Complete\n');
fprintf('========================================\n');

function result = ternary(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
