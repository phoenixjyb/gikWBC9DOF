%% Test Phase 2B Improvements for Method 4
% This script tests the Arm-aware Pure Pursuit feature:
%   - Phase 2A: Orientation+Z nominal pose (baseline for Phase 2B)
%   - Phase 2B: Phase 2A + Arm-aware Pure Pursuit
%
% Expected outcomes (Phase 2B vs Phase 2A):
%   - Mean EE Error: <0.5mm (vs 1.2mm Phase 2A, -58%)
%   - Max EE Error: <50mm (vs 104mm Phase 2A, -52%)
%   - Convergence: >80% (vs 74.3% Phase 2A, +8%)
%   - Fallback: <0.2% (vs 0.5% Phase 2A, -60%)

clear; clc;

fprintf('====================================================\n');
fprintf('  Method 4 Phase 2B Test: Arm-Aware Pure Pursuit\n');
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

%% Run Phase 2A (Baseline for Phase 2B comparison)
fprintf('\n--- Running Phase 2A (Orientation+Z Nominal) ---\n');

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
    'UseOrientationZNominal', true, ...  % Phase 2A feature
    'UseArmAwarePP', false, ...           % Phase 2B OFF
    'VerboseLevel', 1);
phase2a_time = toc;

fprintf('\nPhase 2A Results:\n');
fprintf('  Mean EE Error: %.2f mm\n', log_phase2a.avgEEError * 1000);
fprintf('  Max EE Error: %.1f mm\n', log_phase2a.maxEEError * 1000);
fprintf('  Fallback Rate: %.2f%%\n', log_phase2a.fallbackRate * 100);
fprintf('  Convergence Rate: %.1f%%\n', sum(log_phase2a.successMask) / numel(log_phase2a.successMask) * 100);
fprintf('  Execution Time: %.2f s\n', phase2a_time);

%% Run Phase 2B (Arm-Aware Pure Pursuit)
fprintf('\n--- Running Phase 2B (+ Arm-Aware Pure Pursuit) ---\n');

tic;
log_phase2b = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, q0, ...
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
    'UseOrientationZNominal', true, ...  % Phase 2A feature
    'UseArmAwarePP', true, ...            % NEW: Enable Phase 2B!
    'PPNumCandidates', 5, ...
    'PPPositionRadius', 0.10, ...
    'PPYawRange', deg2rad(15), ...
    'VerboseLevel', 1);
phase2b_time = toc;

fprintf('\nPhase 2B Results:\n');
fprintf('  Mean EE Error: %.2f mm\n', log_phase2b.avgEEError * 1000);
fprintf('  Max EE Error: %.1f mm\n', log_phase2b.maxEEError * 1000);
fprintf('  Fallback Rate: %.2f%%\n', log_phase2b.fallbackRate * 100);
fprintf('  Convergence Rate: %.1f%%\n', sum(log_phase2b.successMask) / numel(log_phase2b.successMask) * 100);
fprintf('  Execution Time: %.2f s\n', phase2b_time);

%% Comparison
fprintf('\n====================================================\n');
fprintf('  COMPARISON: Phase 2A vs Phase 2B\n');
fprintf('====================================================\n\n');

% Error reduction
error_reduction = (1 - log_phase2b.avgEEError / log_phase2a.avgEEError) * 100;
max_error_reduction = (1 - log_phase2b.maxEEError / log_phase2a.maxEEError) * 100;
fallback_reduction = (1 - log_phase2b.fallbackRate / log_phase2a.fallbackRate) * 100;
convergence_improvement = (sum(log_phase2b.successMask) / numel(log_phase2b.successMask)) / ...
                          (sum(log_phase2a.successMask) / numel(log_phase2a.successMask)) - 1;

fprintf('Mean EE Error:\n');
fprintf('  Phase 2A:  %.2f mm\n', log_phase2a.avgEEError * 1000);
fprintf('  Phase 2B:  %.2f mm\n', log_phase2b.avgEEError * 1000);
fprintf('  Change:    %.1f%% %s\n', abs(error_reduction), tern(error_reduction > 0, '✓ BETTER', '✗ WORSE'));

fprintf('\nMax EE Error:\n');
fprintf('  Phase 2A:  %.1f mm\n', log_phase2a.maxEEError * 1000);
fprintf('  Phase 2B:  %.1f mm\n', log_phase2b.maxEEError * 1000);
fprintf('  Change:    %.1f%% %s\n', abs(max_error_reduction), tern(max_error_reduction > 0, '✓ BETTER', '✗ WORSE'));

fprintf('\nFallback Rate:\n');
fprintf('  Phase 2A:  %.2f%%\n', log_phase2a.fallbackRate * 100);
fprintf('  Phase 2B:  %.2f%%\n', log_phase2b.fallbackRate * 100);
fprintf('  Change:    %.1f%% %s\n', abs(fallback_reduction), tern(fallback_reduction > 0, '✓ BETTER', '→ SIMILAR'));

fprintf('\nConvergence Rate:\n');
fprintf('  Phase 2A:  %.1f%%\n', sum(log_phase2a.successMask) / numel(log_phase2a.successMask) * 100);
fprintf('  Phase 2B:  %.1f%%\n', sum(log_phase2b.successMask) / numel(log_phase2b.successMask) * 100);
fprintf('  Change:    %.1f%% %s\n', abs(convergence_improvement * 100), tern(convergence_improvement > 0, '✓ BETTER', '✗ WORSE'));

%% Phase 2B Diagnostics
fprintf('\n--- Phase 2B Diagnostics ---\n');
fprintf('Arm-aware Pure Pursuit:\n');
fprintf('  Mean score: %.3f\n', mean(log_phase2b.armAwareScores));
fprintf('  Mean reachability: %.1f%%\n', mean(log_phase2b.armAwareReachability) * 100);

%% Success Criteria Check
fprintf('\n====================================================\n');
fprintf('  SUCCESS CRITERIA CHECK\n');
fprintf('====================================================\n\n');

% Phase 2B targets (relative to Phase 2A: 1.2mm, 104mm max, 0.5% fallback, 74.3% convergence)
targets = struct();
targets.meanError = 0.5;     % mm (target: 1.2 → 0.5mm, -58%)
targets.maxError = 50;       % mm (target: 104 → 50mm, -52%)
targets.fallback = 0.2;      % % (target: 0.5 → 0.2%, -60%)
targets.convergence = 80;    % % (target: 74.3 → 80%, +8%)

results = struct();
results.meanError = log_phase2b.avgEEError * 1000;
results.maxError = log_phase2b.maxEEError * 1000;
results.fallback = log_phase2b.fallbackRate * 100;
results.convergence = sum(log_phase2b.successMask) / numel(log_phase2b.successMask) * 100;

checks = struct();
checks.meanError = results.meanError <= targets.meanError;
checks.maxError = results.maxError <= targets.maxError;
checks.fallback = results.fallback <= targets.fallback;
checks.convergence = results.convergence >= targets.convergence;

fprintf('1. Mean EE Error: %.2f mm (target: ≤%.2f mm) ... %s\n', ...
    results.meanError, targets.meanError, tern(checks.meanError, '✓ PASS', '✗ FAIL'));
fprintf('2. Max EE Error:  %.1f mm (target: ≤%.1f mm) ... %s\n', ...
    results.maxError, targets.maxError, tern(checks.maxError, '✓ PASS', '✗ FAIL'));
fprintf('3. Fallback Rate: %.2f%% (target: <%.2f%%) ... %s\n', ...
    results.fallback, targets.fallback, tern(checks.fallback, '✓ PASS', '✗ FAIL'));
fprintf('4. Convergence:   %.1f%% (target: >%.1f%%) ... %s\n', ...
    results.convergence, targets.convergence, tern(checks.convergence, '✓ PASS', '✗ FAIL'));

% Success: 2/4 minimum (must include mean error)
passCount = sum([checks.meanError, checks.maxError, checks.fallback, checks.convergence]);
minSuccess = passCount >= 2 && checks.meanError;

fprintf('\nPassed: %d/4 criteria\n', passCount);
if passCount == 4
    fprintf('🎉 EXCELLENT! All criteria passed!\n');
    fprintf('   → Phase 2B successful, system highly optimized\n');
elseif passCount >= 3
    fprintf('✓ GOOD! 3/4 criteria passed.\n');
    fprintf('   → Phase 2B effective, consider minor tuning\n');
elseif minSuccess
    fprintf('→ ACCEPTABLE. Error improved + 1 other metric.\n');
    fprintf('   → Consider tuning weights or accept Phase 2A\n');
else
    fprintf('⚠️  Phase 2B did not improve significantly.\n');
    fprintf('   → Phase 2A (1.2mm) is already excellent. Accept as final.\n');
end

%% Save results
fprintf('\nSaving results...\n');
results_dir = 'results/phase2b_arm_aware_pp';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
save(fullfile(results_dir, sprintf('log_phase2a_%s.mat', timestamp)), 'log_phase2a');
save(fullfile(results_dir, sprintf('log_phase2b_%s.mat', timestamp)), 'log_phase2b');

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
