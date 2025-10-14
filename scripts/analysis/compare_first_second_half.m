%% Compare isolated vs staged - waypoint-by-waypoint
clear; clc;

% Load isolated test
data1 = load('results/phase2a_orientation_z/log_phase2a_20251014_154209.mat');
isolatedLog = data1.log_phase2a;

% Load staged test
load('results/20251014_145719_fresh_no_discs/log_staged_ppForIk.mat');
stagedLog = log.stageLogs.stageC;

fprintf('========================================\n');
fprintf('  Isolated vs Staged: First/Second Half\n');
fprintf('========================================\n\n');

isolatedErrors = isolatedLog.positionErrorNorm * 1000;
stagedErrors = stagedLog.positionErrorNorm * 1000;

fprintf('ISOLATED TEST:\n');
fprintf('  First half (1-105):  Mean=%.1fmm, Fail(>500mm)=%d\n', ...
    mean(isolatedErrors(1:105)), sum(isolatedErrors(1:105)>500));
fprintf('  Second half (106-210): Mean=%.1fmm, Fail(>500mm)=%d\n\n', ...
    mean(isolatedErrors(106:210)), sum(isolatedErrors(106:210)>500));

fprintf('STAGED TEST:\n');
fprintf('  First half (1-105):  Mean=%.1fmm, Fail(>500mm)=%d\n', ...
    mean(stagedErrors(1:105)), sum(stagedErrors(1:105)>500));
fprintf('  Second half (106-210): Mean=%.1fmm, Fail(>500mm)=%d\n\n', ...
    mean(stagedErrors(106:210)), sum(stagedErrors(106:210)>500));

fprintf('TARGET Z-HEIGHTS (Isolated):\n');
fprintf('  wp1:   %.3fm\n', isolatedLog.targetPositions(3,1));
fprintf('  wp105: %.3fm\n', isolatedLog.targetPositions(3,105));
fprintf('  wp106: %.3fm\n', isolatedLog.targetPositions(3,106));
fprintf('  wp210: %.3fm\n\n', isolatedLog.targetPositions(3,210));

fprintf('TARGET Z-HEIGHTS (Staged):\n');
fprintf('  wp1:   %.3fm\n', stagedLog.targetPositions(3,1));
fprintf('  wp105: %.3fm\n', stagedLog.targetPositions(3,105));
fprintf('  wp106: %.3fm\n', stagedLog.targetPositions(3,106));
fprintf('  wp210: %.3fm\n\n', stagedLog.targetPositions(3,210));

fprintf('========================================\n');
fprintf('  CONCLUSION\n');
fprintf('========================================\n\n');

if sum(isolatedErrors(106:210) > 500) == 0 && sum(stagedErrors(106:210) > 500) > 50
    fprintf('❌ STAGED RUN FAILS in second half, but ISOLATED TEST SUCCEEDS!\n\n');
    fprintf('This means:\n');
    fprintf('  • The reference trajectory is NOT the issue\n');
    fprintf('  • Phase 2A CAN handle the second half (proven by isolated test)\n');
    fprintf('  • Something in the STAGED PIPELINE causes failure\n\n');
    fprintf('Possible causes:\n');
    fprintf('  1. Different base starting position affects PP seed path\n');
    fprintf('  2. Accumulated drift from first half\n');
    fprintf('  3. Parameters not being passed in staged version\n');
    fprintf('  4. Warm-starting from different initial conditions\n');
else
    fprintf('Both tests show similar pattern - trajectory-dependent issue\n');
end
