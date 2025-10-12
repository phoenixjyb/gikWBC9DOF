%% Test Animation Sync Fix - Single Sample
% Generate ONE animation to verify the fix works

clear; clc;
addpath(genpath('matlab'));

fprintf('=== Testing Animation Sync Fix (Single Sample) ===\n\n');

% Load test result
dataFile = 'results/parametric_study_20251011_085252/parametric_study_results.mat';
load(dataFile, 'results');

testConfig = results(1); % Baseline
fprintf('Test config: %s\n', testConfig.config.name);
fprintf('Parameters: SM=%.2f LC=%.1f Iters=%d\n\n', ...
        testConfig.config.SafetyMargin, testConfig.config.LambdaCusp, ...
        testConfig.config.MaxRSIterations);

% Generate single test animation
testFile = 'results/parametric_study_20251011_085252/animations/Baseline_SYNC_FIXED.mp4';
fprintf('Generating test animation: %s\n', testFile);
fprintf('This will take ~2-3 minutes...\n\n');

tic;
gik9dof.animateStagedWithHelper( ...
    testConfig.log, ...
    'ExportVideo', testFile, ...
    'FrameRate', 20, ...
    'SampleStep', 2);
elapsed = toc;

fprintf('\n✓ Test animation generated in %.1f seconds\n\n', elapsed);

fprintf('=== VERIFICATION CHECKLIST ===\n');
fprintf('Please review the animation and confirm:\n\n');
fprintf('1. [  ] Red dot (Stage C ref EE) moves SYNCHRONOUSLY with robot\n');
fprintf('2. [  ] Red dot starts immediately when robot starts (no lag)\n');
fprintf('3. [  ] Red dot stays aligned with green square throughout\n');
fprintf('4. [  ] No visible delay between robot motion and red dot\n');
fprintf('5. [  ] EE position error display is accurate\n\n');

fprintf('File: %s\n', testFile);
fprintf('Size: ');
system(sprintf('ls -lh "%s" | awk ''{print $5}''', testFile));

fprintf('\nIf all checks pass, run: generate_parametric_animations\n');
fprintf('This will regenerate all 5 animations with the fix (~11 min).\n');
