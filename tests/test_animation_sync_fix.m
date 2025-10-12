%% Test Animation Sync Fix
% Regenerate one animation to verify the EE marker sync is fixed

clear; clc;
addpath(genpath('matlab'));

fprintf('=== Testing Animation EE Marker Sync Fix ===\n\n');

% Load one of the parametric study results
dataFile = 'results/parametric_study_20251011_085252/parametric_study_results.mat';
load(dataFile, 'results');

% Use the baseline config for testing
testConfig = results(1);
fprintf('Testing with: %s\n', testConfig.config.name);
fprintf('  Parameters: SM=%.2f LC=%.1f Iters=%d\n', ...
        testConfig.config.SafetyMargin, testConfig.config.LambdaCusp, ...
        testConfig.config.MaxRSIterations);

% Generate test animation
testFile = 'results/parametric_study_20251011_085252/animations/TEST_sync_fix.mp4';
fprintf('\nGenerating test animation: %s\n', testFile);

tic;
gik9dof.animateStagedWithHelper( ...
    testConfig.log, ...
    'ExportVideo', testFile, ...
    'FrameRate', 20, ...
    'SampleStep', 2);
elapsed = toc;

fprintf('✓ Test animation generated in %.1f seconds\n', elapsed);
fprintf('\nPlease review the animation to verify:\n');
fprintf('  1. Red dot (Stage C ref EE) is synchronized with robot motion\n');
fprintf('  2. Red dot starts at the same time as robot movement\n');
fprintf('  3. Green square (actual EE) tracks smoothly\n');
fprintf('\nFile: %s\n', testFile);
