%TEST_METHOD1_PERFORMANCE Quick test to isolate Method 1 performance bottleneck.
%   Tests with reduced waypoints to determine if issue is linear scaling
%   or a specific bottleneck in the pipeline.

clear; close all; clc;

fprintf('═══════════════════════════════════════════════════════\n');
fprintf('  Method 1 Performance Investigation\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

%% Test Configuration
testCases = [10, 20, 30];  % Test with 10, 20, 30 waypoints
results = struct();

%% Load robot and environment
fprintf('[1/3] Loading robot model and environment...\n');
robot = gik9dof.createRobotModel();

% Create simple environment without loading JSON
env = struct();
env.BaseHome = [-2, -2, 0];
env.FloorDiscs = [];  % Simplified - no obstacles for performance test
env.DistanceMargin = 0.30;
env.DistanceWeight = 5.0;
env.StageBMode = "pureHyb";
env.StageBDockingPositionTolerance = 0.02;
env.StageBDockingYawTolerance = 2*pi/180;

% Load full trajectory directly from JSON
fprintf('  Loading trajectory...\n');
try
    % Read JSON file directly
    jsonFilese  = fullfile(pwd, 'refEETrajs', '1_pull_world_scaled.json');
    if ~exist(jsonFile, 'file')
        jsonFile = fullfile(pwd, '1_pull_world_scaled.json');
    end
    
    % Parse JSON
    fid = fopen(jsonFile, 'r');
    rawJson = fread(fid, inf, 'uint8=>char')';
    fclose(fid);
    
    % Fix corrupted first line if needed
    if startsWith(rawJson, 'ca{')
        rawJson = rawJson(3:end);  % Skip 'ca'
    end
    
    data = jsondecode(rawJson);
    
    % Convert to trajectory structure expected by runStagedTrajectory
    nTotal = length(data.poses);
    trajFull = struct();
    trajFull.EndEffectorName = 'left_gripper_link';
    trajFull.Poses = zeros(4, 4, nTotal);
    
    for i = 1:nTotal
        pos = data.poses(i).position;
        quat = data.poses(i).orientation;  % [w x y z]
        
        % Convert quaternion to rotation matrix
        R = quat2rotm(quat);
        
        % Build homogeneous transform
        T = eye(4);
        T(1:3, 1:3) = R;
        T(1:3, 4) = pos;
        
        trajFull.Poses(:, :, i) = T;
    end
    
    fprintf('  ✓ Loaded %d waypoints from JSON\n', nTotal);
    trajectoryLoaded = true;
    
catch ME
    fprintf('  ⚠ Warning: Could not load trajectory: %s\n', ME.message);
    fprintf('  Will use full trajectory for all tests\n');
    nTotal = 148;  % Known trajectory size
    trajectoryLoaded = false;
end
fprintf('  Will test with subsets: %s\n\n', mat2str(testCases));

%% Run tests with increasing waypoint counts
fprintf('[2/3] Running performance tests...\n');
fprintf('────────────────────────────────────────────────────\n\n');

for i = 1:length(testCases)
    nWaypoints = testCases(i);
    fprintf('Test %d/%d: %d waypoints\n', i, length(testCases), nWaypoints);
    fprintf('─────────────────────────────────\n');
    
    % Time the full pipeline
    fprintf('  Running full staged pipeline (Method 1)...\n');
    tic;
    try
        % If we have trajectory loaded, create subset
        if trajectoryLoaded
            traj = trajFull;
            traj.Poses = trajFull.Poses(:, :, 1:nWaypoints);
            
            % Run with explicit trajectory
            log_full = gik9dof.runStagedTrajectory(robot, traj, ...
                'RateHz', 10, ...
                'Verbose', false, ...
                'EnvironmentConfig', env, ...
                'MaxIterations', 1500, ...
                'UseStageBHybridAStar', true, ...
                'StageBMode', 'pureHyb', ...
                'ExecutionMode', 'ppForIk');
        else
            % Fall back to using full trajectory via trackReferenceTrajectory
            fprintf('    ⚠ Running full trajectory (cannot create subset)\n');
            log_full = gik9dof.trackReferenceTrajectory( ...
                'Mode', 'staged', ...
                'RateHz', 10, ...
                'Verbose', false, ...
                'EnvironmentConfig', env, ...
                'UseStageBHybridAStar', true, ...
                'StageBMode', 'pureHyb', ...
                'ExecutionMode', 'ppForIk');
            
            % Adjust waypoint count to actual
            if isfield(log_full, 'stageLogs') && isfield(log_full.stageLogs, 'stageC')
                nWaypoints = size(log_full.stageLogs.stageC.qTraj, 2);
            end
        end
        
        total_time = toc;
        
        % Try to extract stage times if available
        if isfield(log_full, 'timings')
            stageB_time = log_full.timings.stageB;
            stageC_time = log_full.timings.stageC;
        else
            stageB_time = NaN;
            stageC_time = NaN;
        end
        
        fprintf('    ✓ Total time: %.2f s\n', total_time);
        fprintf('    • Stage B time: %.2f s\n', stageB_time);
        fprintf('    • Stage C time: %.2f s\n', stageC_time);
        fprintf('    • Time per waypoint: %.3f s\n', total_time / nWaypoints);
        
        % Store results
        results(i).nWaypoints = nWaypoints;
        results(i).totalTime = total_time;
        results(i).stageBTime = stageB_time;
        results(i).stageCTime = stageC_time;
        results(i).timePerWaypoint = total_time / nWaypoints;
        
        % Estimate time for full trajectory
        estimated_full = (total_time / nWaypoints) * nTotal;
        fprintf('    📊 Extrapolated time for %d waypoints: %.1f min\n', ...
            nTotal, estimated_full / 60);
        
    catch ME
        fprintf('    ✗ Failed: %s\n', ME.message);
        fprintf('    Location: %s:%d\n', ME.stack(1).name, ME.stack(1).line);
        results(i).totalTime = NaN;
        results(i).error = ME.message;
    end
    
    fprintf('\n');
end

%% Analyze results
fprintf('[3/3] Performance Analysis\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

if all(~isnan([results.totalTime]))
    % Plot scaling
    figure('Name', 'Method 1 Performance Scaling');
    waypoints = [results.nWaypoints];
    times = [results.totalTime];
    
    subplot(1,2,1);
    plot(waypoints, times, 'bo-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Number of Waypoints');
    ylabel('Total Time (s)');
    title('Execution Time vs Waypoints');
    
    % Fit linear model
    p = polyfit(waypoints, times, 1);
    hold on;
    plot(waypoints, polyval(p, waypoints), 'r--', 'LineWidth', 1.5);
    legend('Measured', sprintf('Linear fit: %.3f s/wp', p(1)), 'Location', 'best');
    
    subplot(1,2,2);
    timePerWP = [results.timePerWaypoint];
    plot(waypoints, timePerWP, 'go-', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('Number of Waypoints');
    ylabel('Time per Waypoint (s)');
    title('Scaling Efficiency');
    yline(mean(timePerWP), 'r--', sprintf('Mean: %.3f s/wp', mean(timePerWP)));
    
    % Print summary table
    fprintf('RESULTS TABLE:\n');
    fprintf('──────────────────────────────────────────────────\n');
    fprintf('Waypoints │  Total (s) │  Per WP (s) │  Est. Full\n');
    fprintf('──────────┼────────────┼─────────────┼────────────\n');
    for i = 1:length(results)
        est_full_min = (results(i).timePerWaypoint * nTotal) / 60;
        fprintf('   %3d    │   %6.2f   │    %6.3f   │  %5.1f min\n', ...
            results(i).nWaypoints, results(i).totalTime, ...
            results(i).timePerWaypoint, est_full_min);
    end
    fprintf('──────────────────────────────────────────────────\n\n');
    
    % Diagnosis
    fprintf('DIAGNOSIS:\n');
    fprintf('──────────\n');
    avg_per_wp = mean(timePerWP);
    estimated_full_time = avg_per_wp * nTotal / 60;
    
    fprintf('• Average time per waypoint: %.3f seconds\n', avg_per_wp);
    fprintf('• Estimated time for %d waypoints: %.1f minutes\n', nTotal, estimated_full_time);
    
    if estimated_full_time > 15
        fprintf('\n⚠️  WARNING: Estimated time (%.1f min) is TOO LONG!\n', estimated_full_time);
        fprintf('Expected: <10 minutes for %d waypoints\n\n', nTotal);
        
        fprintf('POSSIBLE CAUSES:\n');
        fprintf('1. Stage B Hybrid A* is too slow\n');
        fprintf('   → Try disabling: UseStageBHybridAStar=false\n');
        fprintf('2. GIK solver hitting max iterations\n');
        fprintf('   → Check convergence rate in logs\n');
        fprintf('3. Too many distance constraints (obstacles)\n');
        fprintf('   → Reduce obstacle count or DistanceMargin\n');
        fprintf('4. Path interpolation creating too many waypoints\n');
        fprintf('   → Check actual waypoint count in Stage C\n');
    else
        fprintf('\n✓ Performance is acceptable (%.1f min estimated)\n', estimated_full_time);
    end
    
else
    fprintf('⚠️  Some tests failed. Check error messages above.\n');
end

fprintf('\n═══════════════════════════════════════════════════════\n');
fprintf('Performance investigation complete.\n');
fprintf('═══════════════════════════════════════════════════════\n');
