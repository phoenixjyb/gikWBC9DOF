%% Test Trajectory Smoothing with Real Waypoint Data (1_pull_world_scaled.json)
% This script loads actual waypoints from the pull world scenario
% and tests the trajectory smoothing module with real 100ms-spaced data
%
% Expected: 300 waypoints at ~100ms spacing (30 seconds of motion)
% Output: Smoothed velocities at 50Hz with acceleration/jerk limits

clear; close all; clc;

%% Load Real Waypoint Data
fprintf('Loading waypoint data from 1_pull_world_scaled.json...\n');

% Read JSON file
json_file = fullfile(fileparts(mfilename('fullpath')), '..', 'data', '1_pull_world_scaled.json');
if ~isfile(json_file)
    error('Could not find: %s', json_file);
end

% Parse JSON
json_text = fileread(json_file);
data = jsondecode(json_text);

% Extract poses
poses = data.poses;
N_waypoints = length(poses);
fprintf('Loaded %d waypoints\n', N_waypoints);

%% Convert to Waypoint Arrays
% Preallocate
waypoints_x = zeros(N_waypoints, 1);
waypoints_y = zeros(N_waypoints, 1);
waypoints_z = zeros(N_waypoints, 1);
waypoints_theta = zeros(N_waypoints, 1);

% Extract position and orientation
for i = 1:N_waypoints
    % Position (assuming [x, y, z])
    waypoints_x(i) = poses(i).position(1);
    waypoints_y(i) = poses(i).position(2);
    waypoints_z(i) = poses(i).position(3);
    
    % Orientation (quaternion [w, x, y, z] or [x, y, z, w])
    % Based on the data: [0.5, 0.5, 0.5, -0.5]
    % This looks like [x, y, z, w] format
    qx = poses(i).orientation(1);
    qy = poses(i).orientation(2);
    qz = poses(i).orientation(3);
    qw = poses(i).orientation(4);
    
    % Convert quaternion to yaw (assuming planar motion)
    % Yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
    waypoints_theta(i) = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
end

% Assume waypoints are spaced at 100ms (10Hz)
dt_waypoint = 0.1;  % seconds
t_waypoints = (0:N_waypoints-1)' * dt_waypoint;
t_total = t_waypoints(end);

fprintf('Trajectory duration: %.1f seconds\n', t_total);
fprintf('Position range: X=[%.3f, %.3f], Y=[%.3f, %.3f], Z=[%.3f, %.3f]\n', ...
    min(waypoints_x), max(waypoints_x), ...
    min(waypoints_y), max(waypoints_y), ...
    min(waypoints_z), max(waypoints_z));
fprintf('Theta range: [%.2f, %.2f] rad ([%.1f, %.1f] deg)\n', ...
    min(waypoints_theta), max(waypoints_theta), ...
    rad2deg(min(waypoints_theta)), rad2deg(max(waypoints_theta)));

%% Smoothing Parameters
params = struct();
params.vx_max = 1.5;        % m/s - max forward velocity
params.ax_max = 1.0;        % m/s² - max forward acceleration
params.jx_max = 5.0;        % m/s³ - max forward jerk
params.wz_max = 2.0;        % rad/s - max angular velocity
params.alpha_max = 3.0;     % rad/s² - max angular acceleration
params.jerk_wz_max = 10.0;  % rad/s³ - max angular jerk
params.smoothing_method = 'scurve';  % 'scurve' or 'exponential'

fprintf('\nSmoothing Parameters:\n');
fprintf('  vx_max  = %.2f m/s\n', params.vx_max);
fprintf('  ax_max  = %.2f m/s²\n', params.ax_max);
fprintf('  jx_max  = %.2f m/s³\n', params.jx_max);
fprintf('  wz_max  = %.2f rad/s\n', params.wz_max);
fprintf('  alpha_max = %.2f rad/s²\n', params.alpha_max);
fprintf('  jerk_wz_max = %.2f rad/s³\n', params.jerk_wz_max);
fprintf('  method  = %s\n', params.smoothing_method);

%% Simulation Setup
% Control loop at 50 Hz (20ms timestep)
dt_control = 0.02;  % seconds (50 Hz)
t_sim = 0:dt_control:(t_total + 1.0);  % Add 1 second to see deceleration
N_sim = length(t_sim);

fprintf('\nSimulation:\n');
fprintf('  Control frequency: %.0f Hz (dt=%.3f s)\n', 1/dt_control, dt_control);
fprintf('  Simulation points: %d\n', N_sim);
fprintf('  Total sim time: %.1f seconds\n', t_sim(end));

%% Run Smoothing Algorithm
fprintf('\nRunning trajectory smoothing...\n');

% Preallocate output arrays
vx_smooth = zeros(N_sim, 1);
wz_smooth = zeros(N_sim, 1);
ax_smooth = zeros(N_sim, 1);
alpha_smooth = zeros(N_sim, 1);
jerk_vx = zeros(N_sim, 1);  % TRUE jerk from algorithm
jerk_wz = zeros(N_sim, 1);  % TRUE jerk from algorithm

% Clear persistent variables in the function
clear gik9dof.control.smoothTrajectoryVelocity;

% Run simulation loop
tic;
for i = 1:N_sim
    t_current = t_sim(i);
    
    % Call smoothing function - now returns TRUE jerk values
    [vx_smooth(i), wz_smooth(i), ax_smooth(i), alpha_smooth(i), jerk_vx(i), jerk_wz(i)] = ...
        gik9dof.control.smoothTrajectoryVelocity(...
            waypoints_x, waypoints_y, waypoints_theta, t_waypoints, ...
            t_current, params);
end
elapsed = toc;

fprintf('Smoothing completed in %.3f seconds (%.1f µs per call)\n', ...
    elapsed, elapsed/N_sim * 1e6);

%% Compute Raw (Unsmoothed) Velocities for Comparison
fprintf('\nComputing raw velocities from waypoints...\n');

vx_raw = zeros(N_sim, 1);
wz_raw = zeros(N_sim, 1);

for i = 1:N_sim
    t_current = t_sim(i);
    
    % Find current waypoint segment
    idx = find(t_waypoints >= t_current, 1, 'first');
    
    if isempty(idx) || idx == 1
        % Before first or past last waypoint
        vx_raw(i) = 0;
        wz_raw(i) = 0;
    else
        % Between waypoints idx-1 and idx
        p0 = [waypoints_x(idx-1), waypoints_y(idx-1)];
        p1 = [waypoints_x(idx), waypoints_y(idx)];
        theta0 = waypoints_theta(idx-1);
        theta1 = waypoints_theta(idx);
        t0 = t_waypoints(idx-1);
        t1 = t_waypoints(idx);
        
        % Compute segment velocity
        dp = p1 - p0;
        dt = t1 - t0;
        
        if dt > 1e-6
            vx_raw(i) = norm(dp) / dt;  % Forward velocity
            dtheta = theta1 - theta0;
            % Wrap angle difference to [-pi, pi]
            dtheta = atan2(sin(dtheta), cos(dtheta));
            wz_raw(i) = dtheta / dt;
        end
    end
end

%% Compute Derivatives (Acceleration and Jerk)
% For smooth trajectories:
% - ax_smooth and alpha_smooth are returned directly from algorithm
% - jerk_vx and jerk_wz are the TRUE jerk values enforced by the algorithm
%   (NOT computed from finite differences, which would add numerical error)

% For comparison with raw trajectories
ax_raw = [0; diff(vx_raw)] / dt_control;
alpha_raw = [0; diff(wz_raw)] / dt_control;

%% Analysis
fprintf('\n========== ANALYSIS ==========\n');
fprintf('Forward Velocity:\n');
fprintf('  Raw:    max=%.3f m/s,  mean=%.3f m/s\n', max(abs(vx_raw)), mean(abs(vx_raw)));
fprintf('  Smooth: max=%.3f m/s,  mean=%.3f m/s (limit: %.2f m/s)\n', ...
    max(abs(vx_smooth)), mean(abs(vx_smooth)), params.vx_max);

fprintf('\nAngular Velocity:\n');
fprintf('  Raw:    max=%.3f rad/s, mean=%.3f rad/s\n', max(abs(wz_raw)), mean(abs(wz_raw)));
fprintf('  Smooth: max=%.3f rad/s, mean=%.3f rad/s (limit: %.2f rad/s)\n', ...
    max(abs(wz_smooth)), mean(abs(wz_smooth)), params.wz_max);

fprintf('\nForward Acceleration:\n');
fprintf('  Raw:    max=%.3f m/s²\n', max(abs(ax_raw)));
fprintf('  Smooth: max=%.3f m/s² (limit: %.2f m/s²)\n', ...
    max(abs(ax_smooth)), params.ax_max);

fprintf('\nAngular Acceleration:\n');
fprintf('  Raw:    max=%.3f rad/s²\n', max(abs(alpha_raw)));
fprintf('  Smooth: max=%.3f rad/s² (limit: %.2f rad/s²)\n', ...
    max(abs(alpha_smooth)), params.alpha_max);

fprintf('\nForward Jerk:\n');
fprintf('  Smooth: max=%.3f m/s³ (limit: %.2f m/s³)\n', ...
    max(abs(jerk_vx)), params.jx_max);

fprintf('\nAngular Jerk:\n');
fprintf('  Smooth: max=%.3f rad/s³ (limit: %.2f rad/s³)\n', ...
    max(abs(jerk_wz)), params.jerk_wz_max);

% Check for violations
fprintf('\n========== LIMIT CHECKS ==========\n');
violations = false;

if max(abs(vx_smooth)) > params.vx_max * 1.01
    fprintf('⚠️  WARNING: Forward velocity limit VIOLATED!\n');
    violations = true;
end

if max(abs(wz_smooth)) > params.wz_max * 1.01
    fprintf('⚠️  WARNING: Angular velocity limit VIOLATED!\n');
    violations = true;
end

if max(abs(ax_smooth)) > params.ax_max * 1.01
    fprintf('⚠️  WARNING: Forward acceleration limit VIOLATED!\n');
    violations = true;
end

if max(abs(alpha_smooth)) > params.alpha_max * 1.01
    fprintf('⚠️  WARNING: Angular acceleration limit VIOLATED!\n');
    violations = true;
end

if max(abs(jerk_vx)) > params.jx_max * 1.01
    fprintf('⚠️  WARNING: Forward jerk limit VIOLATED!\n');
    violations = true;
end

if max(abs(jerk_wz)) > params.jerk_wz_max * 1.01
    fprintf('⚠️  WARNING: Angular jerk limit VIOLATED!\n');
    violations = true;
end

if ~violations
    fprintf('✅ All limits respected!\n');
end

%% Visualization
fprintf('\nGenerating plots...\n');

figure('Position', [100, 100, 1600, 1000], 'Name', 'Real Trajectory Smoothing Analysis');

% Plot 1: 3D Trajectory with waypoints
subplot(3, 3, 1);
plot3(waypoints_x, waypoints_y, waypoints_z, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 8);
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('3D Trajectory (%d waypoints)', N_waypoints));
axis equal;
view(45, 30);

% Plot 2: XY trajectory (top view)
subplot(3, 3, 2);
plot(waypoints_x, waypoints_y, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 4);
grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('XY Trajectory (Top View)');
axis equal;

% Plot 3: Forward velocity
subplot(3, 3, 3);
hold on;
stairs(t_sim, vx_raw, 'r-', 'LineWidth', 1, 'DisplayName', 'Raw (discrete)');
plot(t_sim, vx_smooth, 'b-', 'LineWidth', 2, 'DisplayName', 'Smoothed');
yline(params.vx_max, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Limit');
yline(-params.vx_max, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
hold off;
grid on;
xlabel('Time (s)'); ylabel('v_x (m/s)');
title('Forward Velocity');
legend('Location', 'best');

% Plot 4: Angular velocity
subplot(3, 3, 4);
hold on;
stairs(t_sim, wz_raw, 'r-', 'LineWidth', 1, 'DisplayName', 'Raw');
plot(t_sim, wz_smooth, 'b-', 'LineWidth', 2, 'DisplayName', 'Smoothed');
yline(params.wz_max, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Limit');
yline(-params.wz_max, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
hold off;
grid on;
xlabel('Time (s)'); ylabel('\omega_z (rad/s)');
title('Angular Velocity');
legend('Location', 'best');

% Plot 5: Forward acceleration
subplot(3, 3, 5);
hold on;
plot(t_sim, ax_smooth, 'b-', 'LineWidth', 2, 'DisplayName', 'Smooth');
plot(t_sim, ax_raw, 'r:', 'LineWidth', 1, 'DisplayName', 'Raw');
yline(params.ax_max, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Limit');
yline(-params.ax_max, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
hold off;
grid on;
xlabel('Time (s)'); ylabel('a_x (m/s²)');
title('Forward Acceleration');
legend('Location', 'best');

% Plot 6: Angular acceleration
subplot(3, 3, 6);
hold on;
plot(t_sim, alpha_smooth, 'b-', 'LineWidth', 2, 'DisplayName', 'Smooth');
plot(t_sim, alpha_raw, 'r:', 'LineWidth', 1, 'DisplayName', 'Raw');
yline(params.alpha_max, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Limit');
yline(-params.alpha_max, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
hold off;
grid on;
xlabel('Time (s)'); ylabel('\alpha (rad/s²)');
title('Angular Acceleration');
legend('Location', 'best');

% Plot 7: Forward jerk
subplot(3, 3, 7);
hold on;
plot(t_sim, jerk_vx, 'b-', 'LineWidth', 2);
yline(params.jx_max, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Limit');
yline(-params.jx_max, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
hold off;
grid on;
xlabel('Time (s)'); ylabel('Jerk (m/s³)');
title('Forward Jerk');
legend('Location', 'best');

% Plot 8: Angular jerk
subplot(3, 3, 8);
hold on;
plot(t_sim, jerk_wz, 'b-', 'LineWidth', 2);
yline(params.jerk_wz_max, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Limit');
yline(-params.jerk_wz_max, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
hold off;
grid on;
xlabel('Time (s)'); ylabel('Jerk (rad/s³)');
title('Angular Jerk');
legend('Location', 'best');

% Plot 9: Summary statistics
subplot(3, 3, 9);
axis off;
text(0.1, 0.9, 'Summary Statistics:', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.8, sprintf('Waypoints: %d', N_waypoints), 'FontSize', 10);
text(0.1, 0.7, sprintf('Duration: %.1f s', t_total), 'FontSize', 10);
text(0.1, 0.6, sprintf('Control freq: %.0f Hz', 1/dt_control), 'FontSize', 10);
text(0.1, 0.5, sprintf('Waypoint freq: %.0f Hz', 1/dt_waypoint), 'FontSize', 10);

y_pos = 0.35;
if ~violations
    text(0.1, y_pos, '✅ All limits OK', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0 0.6 0]);
else
    text(0.1, y_pos, '⚠️  Limit violations!', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.8 0 0]);
end

y_pos = 0.2;
text(0.1, y_pos, sprintf('Max |a_x|: %.2f/%.2f m/s²', max(abs(ax_smooth)), params.ax_max), 'FontSize', 9);
y_pos = y_pos - 0.08;
text(0.1, y_pos, sprintf('Max |jerk|: %.2f/%.2f m/s³', max(abs(jerk_vx)), params.jx_max), 'FontSize', 9);

sgtitle('Real Trajectory Smoothing Analysis (1\_pull\_world\_scaled.json)', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n✅ Test complete!\n');
fprintf('Figure generated with 9 analysis plots.\n');
