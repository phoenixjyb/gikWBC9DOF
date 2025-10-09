%% extract_test_cases_from_mat.m
% Extract test cases from MAT log files for MATLAB vs C++ validation
% 
% This script:
% 1. Loads trajectory data from MAT files
% 2. Extracts key data (qCurrent, targetPose, constraints)
% 3. Runs MATLAB GIK solver to get reference solutions
% 4. Saves test cases and reference results as JSON for C++ testing
%
% Usage:
%   extract_test_cases_from_mat('log_holistic_iter0150.mat', 'test_cases.json', 10)

function extract_test_cases_from_mat(matFileName, outputJsonFile, numTestCases)
    
    arguments
        matFileName char {mustBeFile} = 'validation/crossCheckMatVsCpp/log_matfile/log_holistic_iter0150.mat'
        outputJsonFile char = 'validation/gik_test_cases.json'
        numTestCases (1,1) double {mustBePositive, mustBeInteger} = 10
    end
    
    fprintf('===================================================\n');
    fprintf('GIK Test Case Extraction from MAT File\n');
    fprintf('===================================================\n\n');
    
    %% Load MAT file
    fprintf('Loading MAT file: %s\n', matFileName);
    data = load(matFileName);
    log = data.log;
    
    fprintf('  Completed waypoints: %d\n', log.completedWaypoints);
    fprintf('  qTraj size: %s\n', mat2str(size(log.qTraj)));
    fprintf('  targetPoses size: %s\n\n', mat2str(size(log.targetPoses)));
    
    %% Extract test cases
    numAvailable = min(size(log.qTraj, 2) - 1, log.completedWaypoints);
    numTests = min(numTestCases, numAvailable);
    
    fprintf('Extracting %d test cases...\n', numTests);
    
    % Sample indices evenly across the trajectory
    if numTests == 1
        testIndices = 1;
    else
        testIndices = round(linspace(1, numAvailable, numTests));
    end
    
    % Prepare test case structure
    testCases = struct();
    testCases.metadata = struct();
    testCases.metadata.sourceFile = matFileName;
    testCases.metadata.totalWaypoints = log.completedWaypoints;
    testCases.metadata.numTestCases = numTests;
    testCases.metadata.extractionDate = datestr(now);
    testCases.metadata.matlabVersion = version;
    
    if isfield(log, 'distanceSpecs')
        testCases.metadata.hasDistanceConstraints = true;
    else
        testCases.metadata.hasDistanceConstraints = false;
    end
    
    testCases.cases = cell(numTests, 1);
    
    %% Process each test case
    addpath(genpath('matlab'));
    
    for i = 1:numTests
        idx = testIndices(i);
        
        fprintf('  Test %d/%d (waypoint %d)...\n', i, numTests, idx);
        
        % Input data
        qCurrent = log.qTraj(:, idx);
        targetPose = log.targetPoses(:, :, idx);
        
        % Distance constraints (if available)
        if isfield(log, 'distanceSpecs') && ~isempty(log.distanceSpecs)
            distBodyIndices = int32(zeros(20, 1));
            distRefBodyIndices = int32(zeros(20, 1));
            distBoundsLower = zeros(20, 1);
            distBoundsUpper = zeros(20, 1);
            distWeights = zeros(20, 1);
            
            % Extract active constraints from log
            % (Adjust based on actual structure)
            if isfield(log.distanceSpecs, 'bodyIndices')
                numConstraints = min(20, length(log.distanceSpecs.bodyIndices));
                distBodyIndices(1:numConstraints) = int32(log.distanceSpecs.bodyIndices(1:numConstraints));
                distRefBodyIndices(1:numConstraints) = int32(log.distanceSpecs.refBodyIndices(1:numConstraints));
                distBoundsLower(1:numConstraints) = log.distanceSpecs.boundsLower(1:numConstraints);
                distBoundsUpper(1:numConstraints) = log.distanceSpecs.boundsUpper(1:numConstraints);
                distWeights(1:numConstraints) = log.distanceSpecs.weights(1:numConstraints);
            end
        else
            % Use default constraints from test_gik_20constraints.m
            distBodyIndices = int32(zeros(20, 1));
            distRefBodyIndices = int32(zeros(20, 1));
            distBoundsLower = zeros(20, 1);
            distBoundsUpper = zeros(20, 1);
            distWeights = zeros(20, 1);
            
            % Constraint 1: Gripper > 0.3m from chassis
            distBodyIndices(1) = int32(12);
            distRefBodyIndices(1) = int32(4);
            distBoundsLower(1) = 0.3;
            distBoundsUpper(1) = 100.0;
            distWeights(1) = 1.0;
            
            % Constraint 2: Gripper < 2.0m from base
            distBodyIndices(2) = int32(12);
            distRefBodyIndices(2) = int32(1);
            distBoundsLower(2) = 0.0;
            distBoundsUpper(2) = 2.0;
            distWeights(2) = 0.5;
        end
        
        % Run MATLAB solver to get reference solution
        tic;
        try
            [qNext_matlab, solverInfo_matlab] = gik9dof.codegen_inuse.solveGIKStepWrapper(...
                qCurrent, targetPose, ...
                distBodyIndices, distRefBodyIndices, ...
                distBoundsLower, distBoundsUpper, distWeights);
            
            solveTime_matlab = toc * 1000; % Convert to ms
            success = strcmp(solverInfo_matlab.Status, 'success');
            
        catch ME
            fprintf('    ⚠ MATLAB solver failed: %s\n', ME.message);
            qNext_matlab = qCurrent;
            solverInfo_matlab = struct();
            solverInfo_matlab.Status = 'error';
            solverInfo_matlab.Iterations = 0;
            solveTime_matlab = 0;
            success = false;
        end
        
        % Store test case
        testCase = struct();
        testCase.id = i;
        testCase.waypointIndex = idx;
        
        % Input data
        testCase.input = struct();
        testCase.input.qCurrent = qCurrent;
        testCase.input.targetPose = targetPose;
        testCase.input.distBodyIndices = distBodyIndices;
        testCase.input.distRefBodyIndices = distRefBodyIndices;
        testCase.input.distBoundsLower = distBoundsLower;
        testCase.input.distBoundsUpper = distBoundsUpper;
        testCase.input.distWeights = distWeights;
        
        % MATLAB reference solution
        testCase.matlab_reference = struct();
        testCase.matlab_reference.qNext = qNext_matlab;
        testCase.matlab_reference.success = success;
        testCase.matlab_reference.solveTime_ms = solveTime_matlab;
        if isfield(solverInfo_matlab, 'Iterations')
            testCase.matlab_reference.iterations = solverInfo_matlab.Iterations;
        end
        if isfield(solverInfo_matlab, 'Status')
            testCase.matlab_reference.status = solverInfo_matlab.Status;
        end
        if isfield(solverInfo_matlab, 'PoseErrorNorm')
            testCase.matlab_reference.poseErrorNorm = solverInfo_matlab.PoseErrorNorm;
        end
        
        % Log data (if available)
        if isfield(log, 'iterations') && length(log.iterations) >= idx
            testCase.log_reference = struct();
            testCase.log_reference.iterations = log.iterations(idx);
            testCase.log_reference.solveTime_ms = log.solveTime(idx) * 1000;
            testCase.log_reference.success = log.successMask(idx);
        end
        
        testCases.cases{i} = testCase;
        
        fprintf('    ✓ MATLAB solve: %.2f ms, %d iters, status=%s\n', ...
            solveTime_matlab, ...
            testCase.matlab_reference.iterations, ...
            testCase.matlab_reference.status);
    end
    
    %% Save to JSON
    fprintf('\nSaving test cases to: %s\n', outputJsonFile);
    
    % Create output directory if needed
    [outDir, ~, ~] = fileparts(outputJsonFile);
    if ~isempty(outDir) && ~exist(outDir, 'dir')
        mkdir(outDir);
    end
    
    % Convert to JSON-friendly format
    jsonData = struct();
    jsonData.metadata = testCases.metadata;
    jsonData.testCases = testCases.cases;
    
    % Write JSON
    jsonStr = jsonencode(jsonData, 'PrettyPrint', true);
    fid = fopen(outputJsonFile, 'w');
    if fid == -1
        error('Could not open file for writing: %s', outputJsonFile);
    end
    fprintf(fid, '%s', jsonStr);
    fclose(fid);
    
    fprintf('  ✓ Saved %d test cases\n', numTests);
    fprintf('  File size: %.1f KB\n\n', dir(outputJsonFile).bytes / 1024);
    
    %% Summary
    fprintf('===================================================\n');
    fprintf('Extraction Complete\n');
    fprintf('===================================================\n');
    fprintf('Test cases file: %s\n', outputJsonFile);
    fprintf('Number of cases: %d\n', numTests);
    fprintf('\nNext steps:\n');
    fprintf('  1. Copy JSON file to WSL/Linux environment\n');
    fprintf('  2. Build C++ validation program\n');
    fprintf('  3. Run: ./validate_gik_cpp %s\n', outputJsonFile);
    fprintf('  4. Compare results\n\n');
end
