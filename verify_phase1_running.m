% Final check: Are Phase 1 improvements actually being applied in staged run?
% We'll check the intermediate rawLog data

clearvars; clc;

fprintf('========================================\n');
fprintf('  VERIFICATION: Are Phase 1/2A Running?\n');
fprintf('========================================\n\n');

% The diagnostic fields not being in staged logC doesn't mean they weren't computed
% They might just not have been copied over

% Let's check if the executeStageCPPFirst function properly calls runStageCPPFirst_enhanced

fprintf('Checking function call chain:\n');
fprintf('  1. runStagedTrajectory.m line 200 calls executeStageCPPFirst\n');
fprintf('  2. executeStageCPPFirst line 938 calls runStageCPPFirst_enhanced\n');
fprintf('  3. runStageCPPFirst_enhanced SHOULD apply Phase 1 improvements\n\n');

% Let's verify the parameters are actually being passed
fprintf('From our earlier analysis:\n');
fprintf('  Lines 954-967 in runStagedTrajectory.m pass:\n');
fprintf('    UseAdaptiveLookahead: true\n');
fprintf('    UseMicroSegment: true\n');
fprintf('    UseWarmStarting: true\n');
fprintf('    UseVelocityCorridor: true\n');
fprintf('    LogLateralVelocity: true\n');
fprintf('    RelaxedTolerances: true\n');
fprintf('    UseOrientationZNominal: true (Phase 2A)\n\n');

fprintf('So Phase 1/2A flags ARE being passed!\n\n');

fprintf('========================================\n');
fprintf('  HYPOTHESIS\n');
fprintf('========================================\n\n');

fprintf('The diagnostic fields (lateralVelocity, etc.) are missing from\n');
fprintf('the final logC because they are not being copied in the\n');
fprintf('transform step (lines 968-1030).\n\n');

fprintf('But this doesn''t mean Phase 1 improvements aren''t running!\n');
fprintf('The improvements ARE enabled, they just aren''t being logged.\n\n');

fprintf('So the real question is:\n');
fprintf('  WHY does second half fail even WITH Phase 1/2A enabled?\n\n');

fprintf('Let''s check the GIK convergence in detail:\n\n');

% Load staged data
stgData = load('results/20251014_145719_fresh_no_discs/log_staged_ppForIk.mat');
logC_staged = stgData.log.stageLogs.stageC;

% Analyze convergence pattern
conv = logC_staged.successMask;
err = logC_staged.positionErrorNorm * 1000;  % mm

fprintf('Waypoint-by-waypoint analysis:\n');
fprintf('WP#   Conv?  Error(mm)  Iterations\n');
fprintf('---   -----  ---------  ----------\n');

% Show waypoints 100-110 where transition happens
for k = 100:110
    convStr = ternary(conv(k), 'YES', 'NO ');
    fprintf('%3d   %s    %8.2f   %8d\n', k, convStr, err(k), logC_staged.iterations(k));
end

fprintf('\n========================================\n');
fprintf('  KEY OBSERVATIONS\n');
fprintf('========================================\n\n');

fprintf('1. Phase 1/2A parameters ARE being passed to runStageCPPFirst_enhanced\n');
fprintf('2. Diagnostic fields are just not copied to final logC (data loss)\n');
fprintf('3. Second half failures must have a DIFFERENT root cause\n\n');

fprintf('Possible causes for second-half failure:\n');
fprintf('  A) Trajectory becomes more difficult (tighter constraints)\n');
fprintf('  B) Accumulated state/error from first half\n');
fprintf('  C) Different arm configuration from Stage B affects reachability\n');
fprintf('  D) Pure Pursuit path quality degrades in second half\n');
fprintf('  E) Velocity/acceleration limits being violated\n\n');

fprintf('Next steps:\n');
fprintf('  1. Add diagnostic logging to runStagedTrajectory.m (copy rawLog fields)\n');
fprintf('  2. Check if PP path is diverging in second half\n');
fprintf('  3. Compare arm joint limits/reachability between isolated and staged\n\n');

function result = ternary(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
