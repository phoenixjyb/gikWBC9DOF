%% Phase 2A Smoke Test - Quick Validation
% Tests Phase 2A on first 10 waypoints only to verify:
%   1. computeNominalPoseOrientationZ works correctly
%   2. Integration into baseSeedFromEE is successful
%   3. No runtime errors or crashes
%   4. Basic plausibility of results

clear; clc;

fprintf('====================================================\n');
fprintf('  Phase 2A Smoke Test (10 waypoints)\n');
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

%% Test Phase 2A
fprintf('\n--- Testing Phase 2A (Orientation+Z Nominal) ---\n');

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
        'UseOrientationZNominal', true, ...    % Enable Phase 2A
        'OrientationWeight', 1.0, ...
        'PositionWeightXY', 0.1, ...
        'PositionWeightZ', 1.0, ...
        'VerboseLevel', 2);  % Verbose for debugging
    elapsed = toc;
    
    fprintf('\n✓ Phase 2A completed without errors!\n');
    fprintf('  Execution time: %.2f s\n', elapsed);
    
    % Basic validation
    fprintf('\nSmoke Test Results:\n');
    fprintf('  Mean EE Error: %.1f mm\n', log.avgEEError * 1000);
    fprintf('  Max EE Error: %.1f mm\n', log.maxEEError * 1000);
    fprintf('  Fallback Rate: %.1f%%\n', log.fallbackRate * 100);
    fprintf('  Convergence: %.1f%%\n', sum(log.successMask) / numel(log.successMask) * 100);
    
    % Plausibility checks
    fprintf('\nPlausibility Checks:\n');
    checks = struct();
    checks.errorReasonable = log.avgEEError < 2.0;  % < 2m
    checks.hasConverged = sum(log.successMask) > 0;
    checks.notAllFallback = log.fallbackRate < 1.0;
    
    fprintf('  Error < 2m: %s\n', tern(checks.errorReasonable, '✓', '✗'));
    fprintf('  Some converged: %s\n', tern(checks.hasConverged, '✓', '✗'));
    fprintf('  Not all fallback: %s\n', tern(checks.notAllFallback, '✓', '✗'));
    
    allPlausible = checks.errorReasonable && checks.hasConverged && checks.notAllFallback;
    
    fprintf('\n');
    if allPlausible
        fprintf('🎉 SMOKE TEST PASSED!\n');
        fprintf('   → Ready to run full Phase 2A test\n');
        fprintf('   → Run: test_method4_phase2a\n');
    else
        fprintf('⚠️  SMOKE TEST FAILED!\n');
        fprintf('   → Review implementation before full test\n');
    end
    
catch ME
    fprintf('\n✗ ERROR during Phase 2A execution:\n');
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
