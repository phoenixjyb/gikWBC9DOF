%% Test Phase 2A Improvements for Method 4
% This script tests the Orientation+Z nominal pose generation feature:
%   - Phase 1: All Phase 1 improvements (baseline for Phase 2A)
%   - Phase 2A: Orientation+Z priority weighted IK for nominal pose
%
% Expected outcomes (Phase 2A vs Phase 1):
%   - Mean EE Error: ~400mm (vs 507mm Phase 1, -20%)
%   - Convergence: ~75% (vs 68.6% Phase 1, +9%)
%   - Fallback: ~12% (vs 14.8% Phase 1, -19%)

clear; clc;

fprintf('====================================================\n');
fprintf('  Method 4 Phase 2A Test: Orientation+Z Nominal\n');
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

%% Run Phase 1 (Baseline for Phase 2A comparison)
fprintf('\n--- Running Phase 1 (All Phase 1 Improvements) ---\n');

tic;
log_phase1 = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, q0, ...
    'ChassisParams', chassisParams, ...
    'YawTolerance', deg2rad(15), ...
    'PositionTolerance', 0.15, ...
    'EEErrorTolerance', 0.015, ...
    'MaxIterations', 2000, ...
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
    'UseOrientationZNominal', false, ...  % Phase 1 only
    'VerboseLevel', 1);
phase1_time = toc;

fprintf('\nPhase 1 Results:\n');
fprintf('  Mean EE Error: %.1f mm\n', log_phase1.avgEEError * 1000);
fprintf('  Max EE Error: %.1f mm\n', log_phase1.maxEEError * 1000);
fprintf('  Fallback Rate: %.1f%%\n', log_phase1.fallbackRate * 100);
fprintf('  Convergence Rate: %.1f%%\n', sum(log_phase1.successMask) / numel(log_phase1.successMask) * 100);
fprintf('  Execution Time: %.2f s\n', phase1_time);

%% Run Phase 2A (Orientation+Z Nominal Pose)
fprintf('\n--- Running Phase 2A (Orientation+Z Priority) ---\n');

tic;
log_phase2a = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, q0, ...
    'ChassisParams', chassisParams, ...
    'YawTolerance', deg2rad(15), ...
    'PositionTolerance', 0.15, ...
    'EEErrorTolerance', 0.015, ...
    'MaxIterations', 2000, ...
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
    'UseOrientationZNominal', true, ...    % NEW: Enable Phase 2A!
    'OrientationWeight', 1.0, ...           % Tight orientation constraint
    'PositionWeightXY', 0.1, ...            % Relaxed XY position
    'PositionWeightZ', 1.0, ...             % Tight Z constraint
    'VerboseLevel', 1);
phase2a_time = toc;

fprintf('\nPhase 2A Results:\n');
fprintf('  Mean EE Error: %.1f mm\n', log_phase2a.avgEEError * 1000);
fprintf('  Max EE Error: %.1f mm\n', log_phase2a.maxEEError * 1000);
fprintf('  Fallback Rate: %.1f%%\n', log_phase2a.fallbackRate * 100);
fprintf('  Convergence Rate: %.1f%%\n', sum(log_phase2a.successMask) / numel(log_phase2a.successMask) * 100);
fprintf('  Execution Time: %.2f s\n', phase2a_time);

%% Comparison
fprintf('\n====================================================\n');
fprintf('  COMPARISON: Phase 1 vs Phase 2A\n');
fprintf('====================================================\n\n');

% Error reduction
error_reduction = (1 - log_phase2a.avgEEError / log_phase1.avgEEError) * 100;
fallback_reduction = (1 - log_phase2a.fallbackRate / log_phase1.fallbackRate) * 100;
convergence_improvement = (sum(log_phase2a.successMask) / numel(log_phase2a.successMask)) / ...
                          (sum(log_phase1.successMask) / numel(log_phase1.successMask)) - 1;

fprintf('Mean EE Error:\n');
fprintf('  Phase 1:   %.1f mm\n', log_phase1.avgEEError * 1000);
fprintf('  Phase 2A:  %.1f mm\n', log_phase2a.avgEEError * 1000);
fprintf('  Change:    %.1f%% %s\n', abs(error_reduction), tern(error_reduction > 0, '✓ BETTER', '✗ WORSE'));

fprintf('\nFallback Rate:\n');
fprintf('  Phase 1:   %.1f%%\n', log_phase1.fallbackRate * 100);
fprintf('  Phase 2A:  %.1f%%\n', log_phase2a.fallbackRate * 100);
fprintf('  Change:    %.1f%% %s\n', abs(fallback_reduction), tern(fallback_reduction > 0, '✓ BETTER', '✗ WORSE'));

fprintf('\nConvergence Rate:\n');
fprintf('  Phase 1:   %.1f%%\n', sum(log_phase1.successMask) / numel(log_phase1.successMask) * 100);
fprintf('  Phase 2A:  %.1f%%\n', sum(log_phase2a.successMask) / numel(log_phase2a.successMask) * 100);
fprintf('  Change:    %.1f%% %s\n', abs(convergence_improvement * 100), tern(convergence_improvement > 0, '✓ BETTER', '✗ WORSE'));

%% Phase 2A Diagnostics
fprintf('\n--- Phase 2A Diagnostics ---\n');
fprintf('Lateral Velocity (nonholonomy metric):\n');
fprintf('  Phase 1 mean |v_lat|:  %.4f m/s\n', log_phase1.meanLateralVelocity);
fprintf('  Phase 2A mean |v_lat|: %.4f m/s\n', log_phase2a.meanLateralVelocity);
if log_phase2a.meanLateralVelocity < log_phase1.meanLateralVelocity
    fprintf('  Status: ✓ IMPROVED (lower lateral velocity)\n');
else
    fprintf('  Status: → SIMILAR (no change in lateral velocity)\n');
end

%% Success Criteria Check
fprintf('\n====================================================\n');
fprintf('  SUCCESS CRITERIA CHECK\n');
fprintf('====================================================\n\n');

% Phase 2A targets (relative to Phase 1: 507mm, 14.8% fallback, 68.6% convergence)
targets = struct();
targets.meanError = 420;  % mm (target: 507 * 0.8 = ~406mm, allow 420mm with margin)
targets.fallback = 12;    % % (target: <12% vs 14.8% Phase 1)
targets.convergence = 72; % % (target: >72% vs 68.6% Phase 1)
targets.vlat = 0.02;      % m/s (maintain Phase 1 level)

results = struct();
results.meanError = log_phase2a.avgEEError * 1000;
results.fallback = log_phase2a.fallbackRate * 100;
results.convergence = sum(log_phase2a.successMask) / numel(log_phase2a.successMask) * 100;
results.vlat = log_phase2a.meanLateralVelocity;

checks = struct();
checks.meanError = results.meanError <= targets.meanError;
checks.fallback = results.fallback <= targets.fallback;
checks.convergence = results.convergence >= targets.convergence;
checks.vlat = results.vlat <= targets.vlat;

fprintf('1. Mean EE Error: %.1f mm (target: ≤%.1f mm) ... %s\n', ...
    results.meanError, targets.meanError, tern(checks.meanError, '✓ PASS', '✗ FAIL'));
fprintf('2. Fallback Rate: %.1f%% (target: <%.1f%%) ... %s\n', ...
    results.fallback, targets.fallback, tern(checks.fallback, '✓ PASS', '✗ FAIL'));
fprintf('3. Convergence:   %.1f%% (target: >%.1f%%) ... %s\n', ...
    results.convergence, targets.convergence, tern(checks.convergence, '✓ PASS', '✗ FAIL'));
fprintf('4. Lateral Vel:   %.4f m/s (target: <%.2f) ... %s\n', ...
    results.vlat, targets.vlat, tern(checks.vlat, '✓ PASS', '✗ FAIL'));

% Success: 2/4 minimum (must include error + one other)
passCount = sum([checks.meanError, checks.fallback, checks.convergence, checks.vlat]);
minSuccess = passCount >= 2 && checks.meanError;

fprintf('\nPassed: %d/4 criteria\n', passCount);
if passCount == 4
    fprintf('🎉 EXCELLENT! All criteria passed!\n');
    fprintf('   → Proceed to Phase 2B (Arm-aware Pure Pursuit)\n');
elseif passCount >= 3
    fprintf('✓ GOOD! 3/4 criteria passed.\n');
    fprintf('   → Consider Phase 2B or minor tuning\n');
elseif minSuccess
    fprintf('→ ACCEPTABLE. Error improved + 1 other metric.\n');
    fprintf('   → Consider tuning weights or Phase 2B\n');
else
    fprintf('⚠️  NEEDS WORK. Review implementation or weights.\n');
    fprintf('   → Debug Phase 2A before Phase 2B\n');
end

%% Save results
fprintf('\nSaving results...\n');
results_dir = 'results/phase2a_orientation_z';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
save(fullfile(results_dir, sprintf('log_phase1_%s.mat', timestamp)), 'log_phase1');
save(fullfile(results_dir, sprintf('log_phase2a_%s.mat', timestamp)), 'log_phase2a');

fprintf('  Saved to: %s\n', results_dir);

fprintf('\n====================================================\n');
fprintf('  Test Complete\n');
fprintf('====================================================\n');

%% Helper function
function result = tern(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
