%% Phase 2B Smoke Test - Quick Validation
% Tests Phase 2B on first 10 waypoints only to verify:
%   1. computeArmAwareLookahead works correctly
%   2. Candidate sampling is reasonable
%   3. Scoring function produces valid results
%   4. No runtime errors or crashes

clear; clc;

fprintf('====================================================\n');
fprintf('  Phase 2B Smoke Test (10 waypoints)\n');
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

% Load trajectory (only first 10 waypoints)
raw = jsondecode(fileread(trajectory_file));
numWaypoints = min(10, numel(raw.poses));

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
fprintf('  Loaded %d waypoints (first 10 only)\n', numWaypoints);

% Load chassis parameters
chassisParams = gik9dof.control.loadChassisProfile('wide_track');

% Home configuration
q0 = homeConfiguration(robot);

%% Test Phase 2B
fprintf('\n--- Testing Phase 2B (Arm-Aware Pure Pursuit) ---\n');

try
    tic;
    log = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, q0, ...
        'ChassisParams', chassisParams, ...
        'YawTolerance', deg2rad(15), ...
        'PositionTolerance', 0.15, ...
        'MaxIterations', 2000, ...
        'UseAdaptiveLookahead', true, ...
        'UseMicroSegment', true, ...
        'UseWarmStarting', true, ...
        'UseVelocityCorridor', true, ...
        'LogLateralVelocity', true, ...
        'RelaxedTolerances', true, ...
        'UseOrientationZNominal', true, ...  % Phase 2A
        'UseArmAwarePP', true, ...            % Phase 2B
        'PPNumCandidates', 5, ...
        'PPPositionRadius', 0.10, ...
        'PPYawRange', deg2rad(15), ...
        'VerboseLevel', 2);  % Verbose for debugging
    elapsed = toc;
    
    fprintf('\n✓ Phase 2B completed without errors!\n');
    fprintf('  Execution time: %.2f s\n', elapsed);
    
    % Basic validation
    fprintf('\nSmoke Test Results:\n');
    fprintf('  Mean EE Error: %.2f mm\n', log.avgEEError * 1000);
    fprintf('  Max EE Error: %.1f mm\n', log.maxEEError * 1000);
    fprintf('  Fallback Rate: %.2f%%\n', log.fallbackRate * 100);
    fprintf('  Convergence: %.1f%%\n', sum(log.successMask) / numel(log.successMask) * 100);
    
    % Phase 2B diagnostics
    fprintf('\nArm-Aware PP Diagnostics:\n');
    fprintf('  Mean score: %.3f\n', mean(log.armAwareScores));
    fprintf('  Mean reachability: %.1f%%\n', mean(log.armAwareReachability) * 100);
    
    % Plausibility checks
    fprintf('\nPlausibility Checks:\n');
    checks = struct();
    checks.errorReasonable = log.avgEEError < 2.0;  % < 2m
    checks.hasConverged = sum(log.successMask) > 0;
    checks.notAllFallback = log.fallbackRate < 1.0;
    checks.armAwareWorks = mean(log.armAwareScores) > 0;
    
    fprintf('  Error < 2m: %s\n', tern(checks.errorReasonable, '✓', '✗'));
    fprintf('  Some converged: %s\n', tern(checks.hasConverged, '✓', '✗'));
    fprintf('  Not all fallback: %s\n', tern(checks.notAllFallback, '✓', '✗'));
    fprintf('  Arm-aware active: %s\n', tern(checks.armAwareWorks, '✓', '✗'));
    
    allPlausible = checks.errorReasonable && checks.hasConverged && ...
                   checks.notAllFallback && checks.armAwareWorks;
    
    fprintf('\n');
    if allPlausible
        fprintf('🎉 SMOKE TEST PASSED!\n');
        fprintf('   → Ready to run full Phase 2B test\n');
        fprintf('   → Run: test_method4_phase2b\n');
    else
        fprintf('⚠️  SMOKE TEST FAILED!\n');
        fprintf('   → Review implementation before full test\n');
    end
    
catch ME
    fprintf('\n✗ ERROR during Phase 2B execution:\n');
    fprintf('  %s\n', ME.message);
    fprintf('  File: %s\n', ME.stack(1).file);
    fprintf('  Line: %d\n', ME.stack(1).line);
    fprintf('\n⚠️  SMOKE TEST FAILED - Fix errors before proceeding\n');
    rethrow(ME);
end

fprintf('\n====================================================\n');
fprintf('  Smoke Test Complete\n');
fprintf('====================================================\n');

%% Helper function
function result = tern(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
