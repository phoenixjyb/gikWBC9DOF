%% screen_gik_codegen.m
% Run coder.screener to check MATLAB Coder compatibility
% This identifies potential code generation issues before running full codegen

clear; clc;

fprintf('===================================================\n');
fprintf('Screening GIK Solver for MATLAB Coder Compatibility\n');
fprintf('===================================================\n\n');

%% Setup
addpath(genpath(pwd));

%% Prepare example inputs
fprintf('Preparing example inputs...\n');

q0 = zeros(9, 1);
targetPose = eye(4);
targetPose(1:3, 4) = [0.5; 0.2; 0.8];

distBodyIndices = int32(zeros(20, 1));
distRefBodyIndices = int32(zeros(20, 1));
distBoundsLower = zeros(20, 1);
distBoundsUpper = zeros(20, 1);
distWeights = zeros(20, 1);

% Enable one constraint
distBodyIndices(1) = int32(12);
distRefBodyIndices(1) = int32(4);
distBoundsLower(1) = 0.3;
distBoundsUpper(1) = 100.0;
distWeights(1) = 1.0;

fprintf('  ✓ Example inputs ready\n\n');

%% Run coder.screener
fprintf('Running coder.screener on solveGIKStepWrapper...\n');
fprintf('This checks for code generation readiness issues.\n\n');

try
    % Screen the wrapper function
    coder.screener('gik9dof.codegen_inuse.solveGIKStepWrapper', ...
        q0, targetPose, ...
        distBodyIndices, distRefBodyIndices, ...
        distBoundsLower, distBoundsUpper, distWeights);
    
    fprintf('\n===================================================\n');
    fprintf('✓ Screening completed!\n');
    fprintf('===================================================\n');
    fprintf('Check the generated report for details.\n');
    fprintf('Green checkmarks = ready for code generation\n');
    fprintf('Yellow/Red warnings = potential issues to review\n');
    
catch ME
    fprintf('\n===================================================\n');
    fprintf('✗ Screening failed!\n');
    fprintf('===================================================\n');
    fprintf('Error: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('Location: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
    fprintf('\nThis might indicate code generation compatibility issues.\n');
    rethrow(ME);
end

fprintf('\nNext step: Review the screening report, then run generateCodeARM64.m\n');
fprintf('===================================================\n');
