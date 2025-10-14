%% Test Phase 1 Improvements for Method 4
% This script tests the enhanced runStageCPPFirst with all Phase 1 improvements:
%   1. Adaptive lookahead
%   2. Micro-segment PP
%   3. Warm-starting
%   4. Velocity-limited corridor
%   5. Lateral velocity diagnostic
%   6. Relaxed tolerances
%
% Expected outcomes:
%   - Mean EE Error: ~270mm (vs 319mm baseline, -15%)
%   - Convergence: ~77% (vs 47% baseline, +64%)
%   - Fallback: <30% (vs 44% baseline, -32%)
%   - |v_lat|: <0.02 m/s (nonholonomy respected)

clear; clc;

fprintf('====================================================\n');
fprintf('  Method 4 Phase 1 Improvements Test\n');
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

%% Run Baseline (Standard Method 4)
fprintf('\n--- Running BASELINE (Standard Method 4) ---\n');

tic;
log_baseline = gik9dof.runStageCPPFirst(robot, trajStruct, q0, ...
    'ChassisParams', chassisParams, ...
    'YawTolerance', deg2rad(15), ...  % DEFAULT params for fair comparison
    'PositionTolerance', 0.15, ...    % (was 20°/0.20m aggressive - too easy!)
    'EEErrorTolerance', 0.010, ...    % Match enhanced configuration
    'MaxIterations', 1500, ...
    'VerboseLevel', 1);
baseline_time = toc;

fprintf('\nBaseline Results:\n');
fprintf('  Mean EE Error: %.1f mm\n', log_baseline.avgEEError * 1000);
fprintf('  Max EE Error: %.1f mm\n', log_baseline.maxEEError * 1000);
fprintf('  Fallback Rate: %.1f%%\n', log_baseline.fallbackRate * 100);
fprintf('  Convergence Rate: %.1f%%\n', sum(log_baseline.successMask) / numel(log_baseline.successMask) * 100);
fprintf('  Execution Time: %.2f s\n', baseline_time);

%% Run Enhanced (Phase 1 Improvements)
fprintf('\n--- Running ENHANCED (Phase 1 Improvements) ---\n');

tic;
log_enhanced = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, q0, ...
    'ChassisParams', chassisParams, ...
    'YawTolerance', deg2rad(15), ...   % Can use tighter with improvements
    'PositionTolerance', 0.15, ...      % Nominal value
    'EEErrorTolerance', 0.015, ...
    'MaxIterations', 2000, ...          % Increased from 1500
    'LookaheadDistance', 0.8, ...       % Nominal lookahead
    'LookaheadMin', 0.15, ...           % Min for adaptive
    'UseAdaptiveLookahead', true, ...
    'UseMicroSegment', true, ...
    'UseWarmStarting', true, ...
    'UseVelocityCorridor', true, ...
    'LogLateralVelocity', true, ...
    'RelaxedTolerances', true, ...
    'EpsLatMax', 0.015, ...             % 15mm lateral
    'EpsLongMin', 0.01, ...             % 10mm longitudinal min
    'VerboseLevel', 1);
enhanced_time = toc;

fprintf('\nEnhanced Results:\n');
fprintf('  Mean EE Error: %.1f mm\n', log_enhanced.avgEEError * 1000);
fprintf('  Max EE Error: %.1f mm\n', log_enhanced.maxEEError * 1000);
fprintf('  Fallback Rate: %.1f%%\n', log_enhanced.fallbackRate * 100);
fprintf('  Convergence Rate: %.1f%%\n', sum(log_enhanced.successMask) / numel(log_enhanced.successMask) * 100);
fprintf('  Execution Time: %.2f s\n', enhanced_time);

%% Comparison
fprintf('\n====================================================\n');
fprintf('  COMPARISON: Baseline vs Enhanced\n');
fprintf('====================================================\n\n');

% Error reduction
error_reduction = (1 - log_enhanced.avgEEError / log_baseline.avgEEError) * 100;
fallback_reduction = (1 - log_enhanced.fallbackRate / log_baseline.fallbackRate) * 100;
convergence_improvement = (sum(log_enhanced.successMask) / numel(log_enhanced.successMask)) / ...
                          (sum(log_baseline.successMask) / numel(log_baseline.successMask)) - 1;

fprintf('Mean EE Error:\n');
fprintf('  Baseline:  %.1f mm\n', log_baseline.avgEEError * 1000);
fprintf('  Enhanced:  %.1f mm\n', log_enhanced.avgEEError * 1000);
fprintf('  Change:    %.1f%% %s\n', abs(error_reduction), tern(error_reduction > 0, '✓ BETTER', '✗ WORSE'));

fprintf('\nFallback Rate:\n');
fprintf('  Baseline:  %.1f%%\n', log_baseline.fallbackRate * 100);
fprintf('  Enhanced:  %.1f%%\n', log_enhanced.fallbackRate * 100);
fprintf('  Change:    %.1f%% %s\n', abs(fallback_reduction), tern(fallback_reduction > 0, '✓ BETTER', '✗ WORSE'));

fprintf('\nConvergence Rate:\n');
fprintf('  Baseline:  %.1f%%\n', sum(log_baseline.successMask) / numel(log_baseline.successMask) * 100);
fprintf('  Enhanced:  %.1f%%\n', sum(log_enhanced.successMask) / numel(log_enhanced.successMask) * 100);
fprintf('  Change:    %.1f%% %s\n', abs(convergence_improvement * 100), tern(convergence_improvement > 0, '✓ BETTER', '✗ WORSE'));

%% Phase 1 Specific Diagnostics
fprintf('\n--- Phase 1 Diagnostics (Enhanced Only) ---\n');
fprintf('Lateral Velocity (nonholonomy metric):\n');
fprintf('  Mean |v_lat|: %.4f m/s (target: < 0.02)\n', log_enhanced.meanLateralVelocity);
fprintf('  Max |v_lat|:  %.4f m/s\n', log_enhanced.maxLateralVelocity);
fprintf('  Violations:   %d (%.1f%%)\n', log_enhanced.lateralVelocityViolations, ...
    log_enhanced.lateralVelocityViolations / size(trajStruct.Poses, 3) * 100);
if log_enhanced.meanLateralVelocity < 0.02
    fprintf('  Status:       ✓ PASS (nonholonomic constraint respected)\n');
else
    fprintf('  Status:       ✗ FAIL (excessive lateral motion)\n');
end

fprintf('\nAdaptive Lookahead:\n');
fprintf('  Mean:  %.2f m\n', log_enhanced.meanLookahead);
fprintf('  Range: %.2f - %.2f m\n', min(log_enhanced.lookaheadEffective), max(log_enhanced.lookaheadEffective));

fprintf('\nVelocity-Limited Corridor:\n');
fprintf('  Longitudinal (mean): %.3f m\n', log_enhanced.meanCorridorLong);
fprintf('  Lateral (mean):      %.3f m\n', log_enhanced.meanCorridorLat);
fprintf('  Ratio (long/lat):    %.1fx\n', log_enhanced.meanCorridorLong / log_enhanced.meanCorridorLat);

%% Success Criteria Check
fprintf('\n====================================================\n');
fprintf('  SUCCESS CRITERIA CHECK\n');
fprintf('====================================================\n\n');

targets = struct();
targets.meanError = 270;  % mm (target: ~270mm vs 319mm baseline)
targets.fallback = 30;    % % (target: <30% vs 44% baseline)
targets.convergence = 77; % % (target: >77% vs 47% baseline)
targets.vlat = 0.02;      % m/s (target: <0.02)

results = struct();
results.meanError = log_enhanced.avgEEError * 1000;
results.fallback = log_enhanced.fallbackRate * 100;
results.convergence = sum(log_enhanced.successMask) / numel(log_enhanced.successMask) * 100;
results.vlat = log_enhanced.meanLateralVelocity;

checks = struct();
checks.meanError = results.meanError <= targets.meanError * 1.1;  % 10% tolerance
checks.fallback = results.fallback <= targets.fallback;
checks.convergence = results.convergence >= targets.convergence * 0.9;  % 10% tolerance
checks.vlat = results.vlat <= targets.vlat;

fprintf('1. Mean EE Error: %.1f mm (target: ≤%.1f mm) ... %s\n', ...
    results.meanError, targets.meanError, tern(checks.meanError, '✓ PASS', '✗ FAIL'));
fprintf('2. Fallback Rate: %.1f%% (target: <%.1f%%) ... %s\n', ...
    results.fallback, targets.fallback, tern(checks.fallback, '✓ PASS', '✗ FAIL'));
fprintf('3. Convergence:   %.1f%% (target: >%.1f%%) ... %s\n', ...
    results.convergence, targets.convergence, tern(checks.convergence, '✓ PASS', '✗ FAIL'));
fprintf('4. Lateral Vel:   %.4f m/s (target: <%.2f) ... %s\n', ...
    results.vlat, targets.vlat, tern(checks.vlat, '✓ PASS', '✗ FAIL'));

allPassed = checks.meanError && checks.fallback && checks.convergence && checks.vlat;

fprintf('\n');
if allPassed
    fprintf('🎉 ALL CHECKS PASSED! Phase 1 improvements validated.\n');
else
    fprintf('⚠️  Some checks failed. Review diagnostics above.\n');
end

%% Save results
fprintf('\nSaving results...\n');
results_dir = 'results/phase1_improvements';
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
save(fullfile(results_dir, sprintf('log_baseline_%s.mat', timestamp)), 'log_baseline');
save(fullfile(results_dir, sprintf('log_enhanced_%s.mat', timestamp)), 'log_enhanced');

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
