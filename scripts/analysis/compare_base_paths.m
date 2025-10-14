%% Compare base seed paths: isolated vs staged
clear; clc;

% Load isolated test
data1 = load('results/phase2a_orientation_z/log_phase2a_20251014_154209.mat');
isolatedLog = data1.log_phase2a;

% Load staged test
load('results/20251014_145719_fresh_no_discs/log_staged_ppForIk.mat');
stagedLog = log.stageLogs.stageC;

fprintf('========================================\n');
fprintf('  Base Path Comparison\n');
fprintf('========================================\n\n');

% Extract base positions (x, y, theta)
isolatedBase = isolatedLog.qTraj(1:3, :);
stagedBase = stagedLog.qTraj(1:3, :);

fprintf('STARTING POSITIONS:\n');
fprintf('  Isolated: [%.3f, %.3f, %.3f°]\n', ...
    isolatedBase(1,1), isolatedBase(2,1), rad2deg(isolatedBase(3,1)));
fprintf('  Staged:   [%.3f, %.3f, %.3f°]\n\n', ...
    stagedBase(1,1), stagedBase(2,1), rad2deg(stagedBase(3,1)));

fprintf('ENDING POSITIONS:\n');
fprintf('  Isolated: [%.3f, %.3f, %.3f°]\n', ...
    isolatedBase(1,end), isolatedBase(2,end), rad2deg(isolatedBase(3,end)));
fprintf('  Staged:   [%.3f, %.3f, %.3f°]\n\n', ...
    stagedBase(1,end), stagedBase(2,end), rad2deg(stagedBase(3,end)));

fprintf('BASE PATH AT TRANSITION (wp 105-106):\n');
fprintf('  Isolated wp105: [%.3f, %.3f, %.3f°]\n', ...
    isolatedBase(1,105), isolatedBase(2,105), rad2deg(isolatedBase(3,105)));
fprintf('  Isolated wp106: [%.3f, %.3f, %.3f°]\n', ...
    isolatedBase(1,106), isolatedBase(2,106), rad2deg(isolatedBase(3,106)));
fprintf('  Staged wp105:   [%.3f, %.3f, %.3f°]\n', ...
    stagedBase(1,105), stagedBase(2,105), rad2deg(stagedBase(3,105)));
fprintf('  Staged wp106:   [%.3f, %.3f, %.3f°]\n\n', ...
    stagedBase(1,106), stagedBase(2,106), rad2deg(stagedBase(3,106)));

% Compute base path differences
baseDiff = vecnorm(isolatedBase(1:2,:) - stagedBase(1:2,:), 2, 1);

fprintf('BASE POSITION DIVERGENCE (XY distance):\n');
fprintf('  First half mean:  %.3fm\n', mean(baseDiff(1:105)));
fprintf('  First half max:   %.3fm (wp %d)\n', ...
    max(baseDiff(1:105)), find(baseDiff(1:105) == max(baseDiff(1:105)), 1));
fprintf('  Second half mean: %.3fm\n', mean(baseDiff(106:end)));
fprintf('  Second half max:  %.3fm (wp %d)\n\n', ...
    max(baseDiff(106:end)), 105 + find(baseDiff(106:end) == max(baseDiff(106:end)), 1));

% Check if divergence correlates with errors
stagedErrors = stagedLog.positionErrorNorm * 1000;

fprintf('CORRELATION ANALYSIS:\n');
[r, p] = corrcoef(baseDiff', stagedErrors');
fprintf('  Base divergence vs EE error: r=%.3f, p=%.4f\n\n', r(1,2), p(1,2));

if abs(r(1,2)) > 0.7
    fprintf('✅ STRONG CORRELATION! Base path divergence explains the errors.\n\n');
    fprintf('This means:\n');
    fprintf('  • Phase 2A nominal pose generation produces DIFFERENT base\n');
    fprintf('    seed paths depending on initial configuration\n');
    fprintf('  • Starting from Stage B position leads to suboptimal path\n');
    fprintf('  • The path works in first half but fails in second half\n');
else
    fprintf('⚠️  WEAK CORRELATION. Base divergence may not fully explain errors.\n');
end

fprintf('\n========================================\n');
fprintf('  BASE PATH CHARACTERISTICS\n');
fprintf('========================================\n\n');

% Compute path statistics
isolatedTravelDist = sum(vecnorm(diff(isolatedBase(1:2,:), 1, 2), 2, 1));
stagedTravelDist = sum(vecnorm(diff(stagedBase(1:2,:), 1, 2), 2, 1));

fprintf('TOTAL TRAVEL DISTANCE:\n');
fprintf('  Isolated: %.2fm\n', isolatedTravelDist);
fprintf('  Staged:   %.2fm\n', stagedTravelDist);
fprintf('  Difference: %.2fm (%.1f%%)\n\n', ...
    stagedTravelDist - isolatedTravelDist, ...
    (stagedTravelDist / isolatedTravelDist - 1) * 100);

% Check for sudden changes at wp 106
isolatedStep105_106 = norm(isolatedBase(1:2,106) - isolatedBase(1:2,105));
stagedStep105_106 = norm(stagedBase(1:2,106) - stagedBase(1:2,105));

fprintf('STEP SIZE at wp 105→106:\n');
fprintf('  Isolated: %.3fm\n', isolatedStep105_106);
fprintf('  Staged:   %.3fm\n', stagedStep105_106);

if stagedStep105_106 > 2 * isolatedStep105_106
    fprintf('  ⚠️  STAGED has MUCH LARGER step!\n');
end
