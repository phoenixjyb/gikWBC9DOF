%% Validate C++ Solver Against MATLAB Implementation
% This script runs the same trajectory through both MATLAB and C++ solvers
% and compares the results to validate code generation.
%
% Author: Generated for gikWBC9DOF validation
% Date: 2025-10-06

clear; clc; close all;

%% Configuration
URDF_FILE = '../mobile_manipulator_PPR_base_corrected_sltRdcd.urdf';
TRAJECTORY_FILE = '../1_pull_world_scaled.json';
CPP_RESULTS_FILE = '../validation_results_cpp.json';
MATLAB_RESULTS_FILE = '../validation_results_matlab.json';
COMPARISON_FILE = '../validation_comparison.json';

fprintf('=== MATLAB vs C++ Solver Validation ===\n\n');

%% Step 1: Load trajectory data
fprintf('Step 1: Loading trajectory from %s...\n', TRAJECTORY_FILE);
trajData = jsondecode(fileread(TRAJECTORY_FILE));
numWaypoints = length(trajData.poses);
fprintf('  Loaded %d waypoints\n\n', numWaypoints);

%% Step 2: Build robot model
fprintf('Step 2: Building robot model from URDF...\n');
robot = importrobot(URDF_FILE, 'DataFormat', 'column');
robot.Gravity = [0 0 -9.81];

% Set joint limits to match code generation
for i = 1:length(robot.Bodies)
    body = robot.Bodies{i};
    if body.Joint.Type ~= "fixed"
        % Match the bounds used in solveGIKStepRealtime.m
        if contains(body.Name, 'base')
            % Base joints (x, y, theta)
            if contains(body.Name, 'joint_base_x')
                body.Joint.PositionLimits = [-5, 5];
            elseif contains(body.Name, 'joint_base_y')
                body.Joint.PositionLimits = [-5, 5];
            elseif contains(body.Name, 'joint_base_theta')
                body.Joint.PositionLimits = [-pi, pi];
            end
        else
            % Arm joints
            body.Joint.PositionLimits = [-pi, pi];
        end
    end
end

fprintf('  Robot model ready: %d bodies, %d DOF\n\n', robot.NumBodies, sum(~strcmp({robot.Bodies.Joint.Type}', 'fixed')));

%% Step 3: Initialize MATLAB solver
fprintf('Step 3: Initializing MATLAB GIK solver...\n');
gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'pose', 'joint'});
gik.SolverParameters.AllowRandomRestart = false;
gik.SolverParameters.MaxIterations = 100;
gik.SolverParameters.MaxTime = 0.05; % 50ms
gik.SolverParameters.SolutionTolerance = 1e-6;
gik.SolverParameters.GradientTolerance = 1e-7;

% Create constraints
poseConstraint = constraintPoseTarget('end_effector');
poseConstraint.TargetTransform = eye(4);

jointConstraint = constraintJointBounds(robot);
jointConstraint.Bounds = [-pi pi; -pi pi; -pi pi; -pi pi; -pi pi; -pi pi]; % 6 arm joints

fprintf('  Solver configured\n\n');

%% Step 4: Get initial configuration from first waypoint
fprintf('Step 4: Solving for initial configuration (waypoint 1)...\n');
pose1 = trajData.poses(1);
initialTarget = trvec2tform(pose1.position') * quat2tform(pose1.orientation');

% Use home config as initial guess
initialGuess = homeConfiguration(robot);
initialGuess(1) = pose1.position(1); % Base x
initialGuess(2) = pose1.position(2); % Base y
initialGuess(3) = 0; % Base theta

% Solve for initial config
poseConstraint.TargetTransform = initialTarget;
[initialConfig, solInfo] = gik(initialGuess, poseConstraint, jointConstraint);

if strcmpi(solInfo.Status, 'success')
    fprintf('  Initial solve: SUCCESS (%.3f ms, %d iterations)\n', ...
        solInfo.SolveTime*1000, solInfo.Iterations);
    fprintf('  Initial config: [');
    fprintf('%.4f ', initialConfig');
    fprintf(']\n\n');
else
    error('Failed to solve for initial configuration: %s', solInfo.Status);
end

%% Step 5: Solve trajectory with MATLAB
fprintf('Step 5: Solving trajectory with MATLAB solver...\n');
matlabResults = struct();
matlabResults.waypoints = [];
matlabResults.configurations = zeros(length(initialConfig), numWaypoints);
matlabResults.solve_times_ms = zeros(1, numWaypoints);
matlabResults.iterations = zeros(1, numWaypoints);
matlabResults.status = cell(1, numWaypoints);

currentConfig = initialConfig;

% Start from waypoint 2 (waypoint 1 is initial config)
fprintf('  Solving waypoints 2 to %d...\n', numWaypoints);
progressThreshold = 0;

for i = 2:numWaypoints
    % Show progress every 10%
    progress = (i-1) / (numWaypoints-1) * 100;
    if progress >= progressThreshold
        fprintf('    Progress: %.0f%% (waypoint %d/%d)\n', progress, i, numWaypoints);
        progressThreshold = progressThreshold + 10;
    end
    
    pose = trajData.poses(i);
    targetTform = trvec2tform(pose.position') * quat2tform(pose.orientation');
    
    % Solve IK
    poseConstraint.TargetTransform = targetTform;
    tic;
    [newConfig, solInfo] = gik(currentConfig, poseConstraint, jointConstraint);
    solveTime = toc;
    
    % Store results
    matlabResults.configurations(:, i) = newConfig;
    matlabResults.solve_times_ms(i) = solveTime * 1000;
    matlabResults.iterations(i) = solInfo.Iterations;
    matlabResults.status{i} = solInfo.Status;
    
    % Update current config
    currentConfig = newConfig;
end

% Store first waypoint (initial config)
matlabResults.configurations(:, 1) = initialConfig;
matlabResults.solve_times_ms(1) = 0;
matlabResults.iterations(1) = 0;
matlabResults.status{1} = 'initial';

fprintf('  MATLAB solving complete!\n\n');

%% Step 6: Analyze MATLAB results
fprintf('Step 6: Analyzing MATLAB results...\n');
successCount = sum(strcmpi(matlabResults.status(2:end), 'success'));
successRate = successCount / (numWaypoints-1) * 100;
avgSolveTime = mean(matlabResults.solve_times_ms(2:end));
maxSolveTime = max(matlabResults.solve_times_ms(2:end));
avgIterations = mean(matlabResults.iterations(2:end));

fprintf('  Success rate: %.1f%% (%d/%d)\n', successRate, successCount, numWaypoints-1);
fprintf('  Avg solve time: %.2f ms\n', avgSolveTime);
fprintf('  Max solve time: %.2f ms\n', maxSolveTime);
fprintf('  Avg iterations: %.1f\n\n', avgIterations);

%% Step 7: Save MATLAB results
fprintf('Step 7: Saving MATLAB results to %s...\n', MATLAB_RESULTS_FILE);

% Prepare JSON structure
matlabOutput = struct();
matlabOutput.metadata = struct('solver', 'MATLAB', 'num_waypoints', numWaypoints, ...
    'success_rate', successRate, 'avg_solve_time_ms', avgSolveTime);
matlabOutput.waypoints = cell(1, numWaypoints);

for i = 1:numWaypoints
    wp = struct();
    wp.index = i;
    wp.configuration = matlabResults.configurations(:, i)';
    wp.solve_time_ms = matlabResults.solve_times_ms(i);
    wp.iterations = matlabResults.iterations(i);
    wp.status = matlabResults.status{i};
    
    % Store target pose
    pose = trajData.poses(i);
    wp.target_position = pose.position';
    wp.target_orientation = pose.orientation';
    
    matlabOutput.waypoints{i} = wp;
end

% Write JSON
jsonStr = jsonencode(matlabOutput);
fid = fopen(MATLAB_RESULTS_FILE, 'w');
fprintf(fid, '%s', jsonStr);
fclose(fid);

fprintf('  MATLAB results saved\n\n');

%% Step 8: Wait for C++ results
fprintf('Step 8: Waiting for C++ results...\n');
fprintf('  Please run the C++ solver on WSL and generate: %s\n', CPP_RESULTS_FILE);
fprintf('  Press any key when ready...\n');
pause;

%% Step 9: Load and compare results
if ~isfile(CPP_RESULTS_FILE)
    warning('C++ results file not found: %s', CPP_RESULTS_FILE);
    fprintf('\nValidation incomplete. MATLAB results saved for later comparison.\n');
    return;
end

fprintf('Step 9: Loading C++ results and comparing...\n');
cppData = jsondecode(fileread(CPP_RESULTS_FILE));

% Compare configurations
comparison = struct();
comparison.max_error = 0;
comparison.avg_error = 0;
comparison.waypoint_errors = zeros(1, numWaypoints);

totalError = 0;
for i = 1:numWaypoints
    matlabConfig = matlabResults.configurations(:, i);
    cppConfig = cppData.waypoints{i}.configuration';
    
    % Compute joint-wise error
    error = abs(matlabConfig - cppConfig);
    maxError = max(error);
    
    comparison.waypoint_errors(i) = maxError;
    totalError = totalError + mean(error);
    
    if maxError > comparison.max_error
        comparison.max_error = maxError;
    end
end

comparison.avg_error = totalError / numWaypoints;

% Save comparison
comparison.matlab_success_rate = successRate;
comparison.cpp_success_rate = cppData.metadata.success_rate;
comparison.matlab_avg_time = avgSolveTime;
comparison.cpp_avg_time = cppData.metadata.avg_solve_time_ms;

jsonStr = jsonencode(comparison);
fid = fopen(COMPARISON_FILE, 'w');
fprintf(fid, '%s', jsonStr);
fclose(fid);

fprintf('  Max joint error: %.6f rad (%.3f deg)\n', comparison.max_error, rad2deg(comparison.max_error));
fprintf('  Avg joint error: %.6f rad (%.3f deg)\n', comparison.avg_error, rad2deg(comparison.avg_error));
fprintf('  MATLAB success rate: %.1f%%\n', comparison.matlab_success_rate);
fprintf('  C++ success rate: %.1f%%\n', comparison.cpp_success_rate);
fprintf('  Comparison saved to: %s\n\n', COMPARISON_FILE);

%% Step 10: Validation verdict
fprintf('=== VALIDATION VERDICT ===\n');
TOLERANCE_RAD = deg2rad(1.0); % 1 degree tolerance

if comparison.max_error < TOLERANCE_RAD
    fprintf('✅ VALIDATION PASSED\n');
    fprintf('   C++ solver matches MATLAB solver within %.3f degree tolerance\n', rad2deg(TOLERANCE_RAD));
else
    fprintf('❌ VALIDATION FAILED\n');
    fprintf('   Max error %.3f deg exceeds tolerance %.3f deg\n', ...
        rad2deg(comparison.max_error), rad2deg(TOLERANCE_RAD));
end

fprintf('\nDone!\n');
