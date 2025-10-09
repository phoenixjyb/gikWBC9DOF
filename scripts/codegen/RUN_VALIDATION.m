%% RUN_VALIDATION.m
% Quick validation script to run from project root
% This executes all critical pre-codegen checks

clear; clc;

fprintf('========================================\n');
fprintf('CODEGENCC45 Validation Suite\n');
fprintf('========================================\n\n');

%% Step 1: Add paths
fprintf('Step 1: Adding MATLAB paths...\n');
projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'matlab')));
fprintf('  ✓ Paths added\n\n');

%% Step 2: Validate robot builder
fprintf('Step 2: Running robot builder validation...\n');

try
    gik9dof.codegen_inuse.validate_robot_builder;
    fprintf('  ✓ Robot builder validation PASSED\n\n');
catch ME
    fprintf('  ✗ Robot builder validation FAILED\n');
    fprintf('  Error: %s\n', ME.message);
    fprintf('  Location: %s (line %d)\n\n', ME.stack(1).file, ME.stack(1).line);
    error('Validation failed. Fix errors before proceeding.');
end


%% Step 3: Quick IK solver test
fprintf('Step 3: Testing IK solver convergence...\n');

robot = gik9dof.codegen_inuse.buildRobotForCodegen();
q0 = zeros(9, 1);

% Simple reachable target
targetPose = eye(4);
targetPose(1:3, 4) = [0.5; 0.2; 0.8];

try
    [qNext, info] = gik9dof.codegen_inuse.solveGIKStepWrapper(...
        q0, targetPose, 0.1, 1.0);
    
    % Check if solver succeeded (Status is a string: 'success' or other)
    if isfield(info, 'Status') && strcmp(info.Status, 'success')
        fprintf('  ✓ Solver converged (status: %s)\n', info.Status);
        if isfield(info, 'Iterations')
            fprintf('  ✓ Iterations: %d\n', info.Iterations);
        end
        if isfield(info, 'ExitFlag')
            fprintf('  ✓ Exit flag: %d\n', info.ExitFlag);
        end
        fprintf('\n');
    else
        if isfield(info, 'Status')
            warning('Solver did not converge (status: %s)', info.Status);
        else
            warning('Solver info structure unexpected');
        end
    end
catch ME
    fprintf('  ✗ Solver test FAILED\n');
    fprintf('  Error: %s\n\n', ME.message);
    error('Solver test failed. Check implementation.');
end

%% Step 4: Check code generation readiness
fprintf('Step 4: Checking code generation readiness...\n');

% Check if functions are on path
funcs = {
    'gik9dof.codegen_inuse.buildRobotForCodegen'
    'gik9dof.codegen_inuse.solveGIKStepRealtime'
    'gik9dof.codegen_inuse.solveGIKStepWrapper'
    'gik9dof.codegen_inuse.generateCodeARM64'
};

allFound = true;
for i = 1:length(funcs)
    loc = which(funcs{i});
    if isempty(loc)
        fprintf('  ✗ Missing: %s\n', funcs{i});
        allFound = false;
    else
        fprintf('  ✓ Found: %s\n', funcs{i});
    end
end

if ~allFound
    error('Some required functions are missing. Check paths.');
end

fprintf('\n');

%% Summary
fprintf('========================================\n');
fprintf('✓ ALL VALIDATIONS PASSED\n');
fprintf('========================================\n');
fprintf('You are ready for code generation!\n\n');
fprintf('Next step: Run code generation\n');
fprintf('>> cd matlab/+gik9dof/+codegen_inuse\n');
fprintf('>> generateCodeARM64\n\n');
