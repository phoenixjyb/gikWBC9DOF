% Deep dive: What's special about waypoint 106?
% Load both isolated and staged results

% Isolated test (works perfectly)
isoFile = 'results/phase2a_orientation_z/log_phase2a_20251014_154209.mat';
isoData = load(isoFile);

% Staged test (fails at wp106)
stgFile = 'results/20251014_145719_fresh_no_discs/log_staged_ppForIk.mat';
stgData = load(stgFile);
logC_staged = stgData.log.stageLogs.stageC;

% Load trajectory to check EE waypoints
trajData = jsondecode(fileread('1_pull_world_scaled.json'));

fprintf('=== Waypoint 106 Analysis ===\n\n');

% Extract EE positions around waypoint 106
fprintf('EE Target Positions (world frame):\n');
for k = [104, 105, 106, 107, 108]
    if k <= length(trajData.waypoints)
        wp = trajData.waypoints(k);
        fprintf('  WP %3d: [%.3f, %.3f, %.3f]\n', k, wp.position(1), wp.position(2), wp.position(3));
    end
end

% Check if there's a discontinuity or special characteristic
if length(trajData.waypoints) >= 106
    wp105 = trajData.waypoints(105);
    wp106 = trajData.waypoints(106);
    
    % Position jump
    dPos = norm([wp106.position(1) - wp105.position(1), ...
                 wp106.position(2) - wp105.position(2), ...
                 wp106.position(3) - wp105.position(3)]);
    
    fprintf('\nPosition jump 105->106: %.3f m\n', dPos);
    
    % Orientation change
    q105 = [wp105.orientation.w, wp105.orientation.x, wp105.orientation.y, wp105.orientation.z];
    q106 = [wp106.orientation.w, wp106.orientation.x, wp106.orientation.y, wp106.orientation.z];
    
    % Angular distance between quaternions
    angDist = 2 * acos(min(abs(dot(q105, q106)), 1));
    fprintf('Orientation change 105->106: %.1f°\n', rad2deg(angDist));
end

% Check staged run convergence around waypoint 106
if isfield(logC_staged, 'successMask')
    fprintf('\n=== GIK Convergence Around WP 106 ===\n');
    for k = [104, 105, 106, 107, 108]
        if k <= length(logC_staged.successMask)
            status = logC_staged.successMask(k);
            err = logC_staged.positionErrorNorm(k) * 1000;
            fprintf('  WP %3d: %s, Error=%.1fmm\n', k, ...
                ternary(status, 'CONVERGED', 'FAILED   '), err);
        end
    end
end

% Check isolated run for comparison
fprintf('\n=== Isolated Test (working) Around WP 106 ===\n');
for k = [104, 105, 106, 107, 108]
    if k <= length(isoData.successMask)
        status = isoData.successMask(k);
        err = isoData.positionErrorNorm(k) * 1000;
        fprintf('  WP %3d: %s, Error=%.1fmm\n', k, ...
            ternary(status, 'CONVERGED', 'FAILED   '), err);
    end
end

function result = ternary(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
