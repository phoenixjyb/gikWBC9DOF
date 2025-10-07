%% Generate MATLAB Reference IK Solutions for Validation
% This script solves the first 20 waypoints of 1_pull_world_scaled.json
% using native MATLAB GIK and exports results for comparison with C++ solver.
%
% Output: matlab_reference_results.json
%
% Author: Validation script for ARM64 deployment
% Date: October 7, 2025

clear; clc; close all;

%% Configuration
PROJECT_ROOT = fileparts(fileparts(fileparts(mfilename('fullpath'))));
URDF_FILE = fullfile(PROJECT_ROOT, 'mobile_manipulator_PPR_base_corrected_sltRdcd.urdf');
TRAJECTORY_FILE = fullfile(PROJECT_ROOT, '1_pull_world_scaled.json');
OUTPUT_FILE = fullfile(PROJECT_ROOT, 'validation', 'matlab_reference_results.json');
NUM_TEST_WAYPOINTS = 20; % Test subset size

fprintf('========================================\n');
fprintf('MATLAB Reference IK Generation\n');
fprintf('========================================\n\n');

%% Step 1: Load trajectory data
fprintf('Step 1: Loading trajectory...\n');
trajData = jsondecode(fileread(TRAJECTORY_FILE));
totalWaypoints = length(trajData.poses);
fprintf('  Total waypoints in file: %d\n', totalWaypoints);
fprintf('  Using first %d waypoints for validation\n\n', NUM_TEST_WAYPOINTS);

% Extract test subset
testWaypoints = trajData.poses(1:NUM_TEST_WAYPOINTS);

%% Step 2: Build robot model
fprintf('Step 2: Building robot model...\n');
robot = importrobot(URDF_FILE, 'DataFormat', 'column');
robot.Gravity = [0 0 -9.81];

% Get DOF info
numBodies = robot.NumBodies;
% Count non-fixed joints
numDOF = 0;
for i = 1:numBodies
    if ~strcmp(robot.Bodies{i}.Joint.Type, 'fixed')
        numDOF = numDOF + 1;
    end
end
fprintf('  Robot: %d bodies, %d DOF\n', numBodies, numDOF);
fprintf('  End-effector: %s\n\n', robot.Bodies{end}.Name);

%% Step 3: Initialize GIK solver
fprintf('Step 3: Initializing GIK solver...\n');
gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'pose', 'joint'});

% Solver parameters (match C++ codegen settings)
gik.SolverParameters.AllowRandomRestart = false;
gik.SolverParameters.MaxIterations = 100;
gik.SolverParameters.MaxTime = 0.05; % 50ms
gik.SolverParameters.SolutionTolerance = 1e-6;
gik.SolverParameters.GradientTolerance = 1e-7;

% Create constraints
poseConstraint = constraintPoseTarget('left_gripper_link');
poseConstraint.TargetTransform = eye(4);

% Joint bounds constraint (uses robot's built-in joint limits from URDF)
jointConstraint = constraintJointBounds(robot);

fprintf('  Solver configured:\n');
fprintf('    Max iterations: %d\n', gik.SolverParameters.MaxIterations);
fprintf('    Max time: %.0f ms\n', gik.SolverParameters.MaxTime * 1000);
fprintf('    Solution tolerance: %.1e\n\n', gik.SolverParameters.SolutionTolerance);

%% Step 4: Get initial configuration
fprintf('Step 4: Solving initial configuration...\n');
pose1 = testWaypoints(1);
initialTarget = trvec2tform(pose1.position') * quat2tform(pose1.orientation');

% Use home configuration as initial guess
initialGuess = homeConfiguration(robot);
initialGuess(1) = pose1.position(1); % Base x close to target
initialGuess(2) = pose1.position(2); % Base y close to target
initialGuess(3) = 0; % Base theta

% Temporarily relax solver parameters for initial solve
gik.SolverParameters.MaxTime = 0.5; % 500ms for first solve
gik.SolverParameters.AllowRandomRestart = true;

% Solve for initial config
poseConstraint.TargetTransform = initialTarget;
tic;
[initialConfig, solInfo0] = gik(initialGuess, poseConstraint, jointConstraint);
initialTime = toc * 1000;

% Restore solver parameters
gik.SolverParameters.MaxTime = 0.05; % Back to 50ms
gik.SolverParameters.AllowRandomRestart = false;

if strcmpi(solInfo0.Status, 'success')
    fprintf('  Initial solve: ✅ SUCCESS\n');
    fprintf('    Time: %.2f ms\n', initialTime);
    fprintf('    Iterations: %d\n', solInfo0.Iterations);
    fprintf('    Config: [');
    fprintf('%.3f ', initialConfig');
    fprintf(']\n\n');
else
    fprintf('  ⚠️  Initial solve: %s (using best available configuration)\n', solInfo0.Status);
    fprintf('    Time: %.2f ms\n', initialTime);
    fprintf('    Iterations: %d\n', solInfo0.Iterations);
    fprintf('    Note: Proceeding with best available solution\n\n');
end

%% Step 5: Solve all test waypoints
fprintf('Step 5: Solving %d waypoints...\n', NUM_TEST_WAYPOINTS);
fprintf('  Progress: ');

results = struct();
results.metadata = struct();
results.metadata.platform = 'MATLAB R2024b';
results.metadata.architecture = 'x86_64';
results.metadata.date = datestr(now, 'yyyy-mm-dd HH:MM:SS');
results.metadata.num_waypoints = NUM_TEST_WAYPOINTS;
results.metadata.urdf_file = URDF_FILE;
results.metadata.trajectory_file = TRAJECTORY_FILE;

results.waypoints = repmat(struct(...
    'index', 0, ...
    'target_position', [], ...
    'target_orientation', [], ...
    'target_transform', [], ...
    'joint_config', [], ...
    'solve_time_ms', 0, ...
    'status', '', ...
    'iterations', 0, ...
    'pose_error', []), NUM_TEST_WAYPOINTS, 1);

% Use initial config as starting point
currentConfig = initialConfig;

for i = 1:NUM_TEST_WAYPOINTS
    % Progress indicator
    if mod(i, 5) == 0
        fprintf('%d ', i);
    else
        fprintf('.');
    end
    
    % Extract target pose
    pose = testWaypoints(i);
    targetTransform = trvec2tform(pose.position') * quat2tform(pose.orientation');
    
    % Update constraint
    poseConstraint.TargetTransform = targetTransform;
    
    % Solve IK (use previous solution as warm start)
    tic;
    [qSol, solInfo] = gik(currentConfig, poseConstraint, jointConstraint);
    solveTime = toc * 1000; % milliseconds
    
    % Compute forward kinematics to get actual pose
    T_achieved = getTransform(robot, qSol, 'left_gripper_link');
    
    % Compute pose error
    posError = norm(T_achieved(1:3,4) - targetTransform(1:3,4)) * 1000; % mm
    R_error = T_achieved(1:3,1:3)' * targetTransform(1:3,1:3);
    axang = rotm2axang(R_error);
    rotError = rad2deg(abs(axang(4))); % degrees
    
    % Store results
    results.waypoints(i).index = i;
    results.waypoints(i).target_position = pose.position;
    results.waypoints(i).target_orientation = pose.orientation;
    results.waypoints(i).target_transform = targetTransform;
    results.waypoints(i).joint_config = qSol';
    results.waypoints(i).solve_time_ms = solveTime;
    results.waypoints(i).status = solInfo.Status;
    results.waypoints(i).iterations = solInfo.Iterations;
    results.waypoints(i).pose_error = struct(...
        'position_mm', posError, ...
        'orientation_deg', rotError);
    
    % Use this solution as next initial guess
    currentConfig = qSol;
end

fprintf(' DONE\n\n');

%% Step 6: Compute summary statistics
fprintf('Step 6: Computing summary statistics...\n');

successCount = sum(strcmp({results.waypoints.status}, 'success'));
failCount = NUM_TEST_WAYPOINTS - successCount;
solveTimes = [results.waypoints.solve_time_ms];
iterations = [results.waypoints.iterations];
posErrors = [results.waypoints.pose_error];
posErrorsVal = [posErrors.position_mm];
rotErrorsVal = [posErrors.orientation_deg];

results.summary = struct();
results.summary.success_count = successCount;
results.summary.fail_count = failCount;
results.summary.success_rate = (successCount / NUM_TEST_WAYPOINTS) * 100;
results.summary.solve_time = struct(...
    'min_ms', min(solveTimes), ...
    'max_ms', max(solveTimes), ...
    'mean_ms', mean(solveTimes), ...
    'std_ms', std(solveTimes));
results.summary.iterations = struct(...
    'min', min(iterations), ...
    'max', max(iterations), ...
    'mean', mean(iterations));
results.summary.pose_error = struct(...
    'max_position_mm', max(posErrorsVal), ...
    'mean_position_mm', mean(posErrorsVal), ...
    'max_orientation_deg', max(rotErrorsVal), ...
    'mean_orientation_deg', mean(rotErrorsVal));

fprintf('  Success rate: %d/%d (%.1f%%)\n', successCount, NUM_TEST_WAYPOINTS, results.summary.success_rate);
fprintf('  Solve time: %.2f ± %.2f ms (range: %.2f - %.2f ms)\n', ...
    results.summary.solve_time.mean_ms, ...
    results.summary.solve_time.std_ms, ...
    results.summary.solve_time.min_ms, ...
    results.summary.solve_time.max_ms);
fprintf('  Pose errors:\n');
fprintf('    Position: %.3f mm (max), %.3f mm (mean)\n', ...
    results.summary.pose_error.max_position_mm, ...
    results.summary.pose_error.mean_position_mm);
fprintf('    Orientation: %.3f deg (max), %.3f deg (mean)\n\n', ...
    results.summary.pose_error.max_orientation_deg, ...
    results.summary.pose_error.mean_orientation_deg);

%% Step 7: Export results to JSON
fprintf('Step 7: Exporting results...\n');

% Create validation directory if needed
[outputDir, ~, ~] = fileparts(OUTPUT_FILE);
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% Convert to JSON
jsonText = jsonencode(results, 'PrettyPrint', true);

% Write to file
fid = fopen(OUTPUT_FILE, 'w');
if fid == -1
    error('Failed to open output file: %s', OUTPUT_FILE);
end
fprintf(fid, '%s', jsonText);
fclose(fid);

fprintf('  ✅ Results saved to:\n');
fprintf('     %s\n', OUTPUT_FILE);
fprintf('  File size: %.1f KB\n\n', dir(OUTPUT_FILE).bytes / 1024);

%% Done
fprintf('========================================\n');
fprintf('✅ MATLAB Reference Generation Complete\n');
fprintf('========================================\n');
fprintf('\nNext steps:\n');
fprintf('  1. Transfer this file to AGX Orin\n');
fprintf('  2. Run C++ solver test on Orin\n');
fprintf('  3. Run comparison analysis\n\n');
