% Check if the fix created a mismatch between starting position and seed path
data = load('results/20251014_164148_fresh_no_discs/log_staged_ppForIk.mat');
logC = data.log.stageLogs.stageC;
logB = data.log.stageLogs.stageB;

% Stage B ending position
qB_end = logB.qTraj(:, end);
posB_end = qB_end(1:3);

% Stage C starting position (should be same as B ending)
qC_start = logC.qTraj(:, 1);
posC_start = qC_start(1:3);

% Home configuration for comparison
robot = loadrobot("kinovaGen3", DataFormat="row", Gravity=[0 0 -9.81]);
qHome = homeConfiguration(robot);
posHome = qHome(1:3);

fprintf('=== Configuration Analysis ===\n\n');

fprintf('Stage B ending position:\n');
fprintf('  [%.3f, %.3f, %.1f°]\n', posB_end(1), posB_end(2), rad2deg(posB_end(3)));

fprintf('\nStage C starting position (should match B ending):\n');
fprintf('  [%.3f, %.3f, %.1f°]\n', posC_start(1), posC_start(2), rad2deg(posC_start(3)));

fprintf('\nHome configuration:\n');
fprintf('  [%.3f, %.3f, %.1f°]\n', posHome(1), posHome(2), rad2deg(posHome(3)));

fprintf('\nMismatch (B end to Home):\n');
fprintf('  Position: %.3f m\n', norm(posB_end(1:2) - posHome(1:2)));
fprintf('  Orientation: %.1f°\n', rad2deg(abs(posB_end(3) - posHome(3))));

% Check first waypoint error
posErr = logC.positionErrorNorm * 1000;
fprintf('\n=== First 10 Waypoint Errors (mm) ===\n');
for k = 1:min(10, length(posErr))
    fprintf('  Waypoint %2d: %.1f mm\n', k, posErr(k));
end

fprintf('\n=== Error Pattern ===\n');
fprintf('Waypoints 1-10:  Mean=%.1fmm\n', mean(posErr(1:10)));
fprintf('Waypoints 1-50:  Mean=%.1fmm\n', mean(posErr(1:50)));
fprintf('Waypoints 1-105: Mean=%.1fmm\n', mean(posErr(1:105)));
fprintf('Waypoints 106-210: Mean=%.1fmm\n', mean(posErr(106:end)));
