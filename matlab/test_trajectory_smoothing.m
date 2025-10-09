%% Test Trajectory Smoothing Module
% This script tests the smoothTrajectoryVelocity function with simulated
% waypoints to verify acceleration and jerk limiting works correctly.
%
% Expected Behavior:
%   - Smooth velocity transitions between waypoints
%   - No sudden jumps in velocity (respects acceleration limits)
%   - No sudden jumps in acceleration (respects jerk limits)
%   - Robot stays stable (no tipping)

clear; clc; close all;

%% Add Path
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));

%% Parameters
params = struct();
params.vx_max = 1.5;        % m/s - max forward velocity
params.ax_max = 1.0;        % m/s² - max forward acceleration
params.jx_max = 5.0;        % m/s³ - max forward jerk
params.wz_max = 2.0;        % rad/s - max angular velocity
params.alpha_max = 3.0;     % rad/s² - max angular acceleration
params.jerk_wz_max = 10.0;  % rad/s³ - max angular jerk
params.smoothing_method = 'scurve';  % 'scurve' or 'exponential'

%% Generate Test Waypoints (Simulating GIK Output at 10Hz)
% Scenario: Robot moves forward, then turns, then forward again

% Waypoint timing
dt_waypoint = 0.1;  % 10 Hz
t_waypoints = (0:dt_waypoint:2.0)';  % 2 seconds of motion, 21 waypoints

N_waypoints = length(t_waypoints);

% Waypoint positions (straight → turn → straight)
waypoints_x = zeros(N_waypoints, 1);
waypoints_y = zeros(N_waypoints, 1);
waypoints_theta = zeros(N_waypoints, 1);

for i = 1:N_waypoints
    t = t_waypoints(i);
    
    if t < 0.6
        % Phase 1: Move forward (0-0.6s)
        waypoints_x(i) = t * 0.5;  % 0.5 m/s forward
        waypoints_y(i) = 0;
        waypoints_theta(i) = 0;
        
    elseif t < 1.2
        % Phase 2: Turn in place (0.6-1.2s)
        waypoints_x(i) = 0.3;  % Stop at 0.3m
        waypoints_y(i) = 0;
        % Turn 90 degrees over 0.6 seconds
        waypoints_theta(i) = (t - 0.6) / 0.6 * pi/2;
        
    else
        % Phase 3: Move forward in new direction (1.2-2.0s)
        waypoints_x(i) = 0.3;
        waypoints_y(i) = (t - 1.2) * 0.5;  % 0.5 m/s in Y direction
        waypoints_theta(i) = pi/2;
    end
end

%% Simulate Real-Time Execution at 50Hz
dt_control = 0.02;  % 50 Hz control loop
t_sim = (0:dt_control:2.5)';  % Simulate 2.5 seconds
N_sim = length(t_sim);

% Preallocate outputs
vx_smooth = zeros(N_sim, 1);
wz_smooth = zeros(N_sim, 1);
ax_smooth = zeros(N_sim, 1);
alpha_smooth = zeros(N_sim, 1);

% Simulate control loop
fprintf('Simulating trajectory smoothing at 50Hz...\n');
for i = 1:N_sim
    t_current = t_sim(i);
    
    % Call smoothing function (mimics real-time call)
    [vx_smooth(i), wz_smooth(i), ax_smooth(i), alpha_smooth(i)] = ...
        gik9dof.control.smoothTrajectoryVelocity(...
            waypoints_x, waypoints_y, waypoints_theta, t_waypoints, ...
            t_current, params);
    
    if mod(i, 50) == 0
        fprintf('  t=%.2fs: vx=%.3f m/s, wz=%.3f rad/s, ax=%.3f m/s², alpha=%.3f rad/s²\n', ...
            t_current, vx_smooth(i), wz_smooth(i), ax_smooth(i), alpha_smooth(i));
    end
end

fprintf('Simulation complete.\n\n');

%% Compute Raw (Unsmoothed) Velocities for Comparison
% What velocities would be WITHOUT smoothing (direct waypoint tracking)
vx_raw = zeros(N_sim, 1);
wz_raw = zeros(N_sim, 1);

for i = 1:N_sim
    t_current = t_sim(i);
    
    % Find current segment
    idx = find(t_waypoints >= t_current, 1, 'first');
    if isempty(idx) || idx == 1
        vx_raw(i) = 0;
        wz_raw(i) = 0;
        continue;
    end
    
    % Compute raw velocity from segment
    p0 = [waypoints_x(idx-1); waypoints_y(idx-1)];
    p1 = [waypoints_x(idx); waypoints_y(idx)];
    theta0 = waypoints_theta(idx-1);
    theta1 = waypoints_theta(idx);
    t0 = t_waypoints(idx-1);
    t1 = t_waypoints(idx);
    
    dp = p1 - p0;
    segment_length = norm(dp);
    segment_duration = t1 - t0;
    
    if segment_length > 1e-6 && segment_duration > 1e-6
        vx_raw(i) = segment_length / segment_duration;
    end
    
    dtheta = wrapToPi(theta1 - theta0);
    if segment_duration > 1e-6
        wz_raw(i) = dtheta / segment_duration;
    end
end

%% Analyze Results
fprintf('=== Analysis ===\n');
fprintf('Max forward velocity (smooth): %.3f m/s (limit: %.3f m/s)\n', max(abs(vx_smooth)), params.vx_max);
fprintf('Max angular velocity (smooth): %.3f rad/s (limit: %.3f rad/s)\n', max(abs(wz_smooth)), params.wz_max);
fprintf('Max forward acceleration: %.3f m/s² (limit: %.3f m/s²)\n', max(abs(ax_smooth)), params.ax_max);
fprintf('Max angular acceleration: %.3f rad/s² (limit: %.3f rad/s²)\n', max(abs(alpha_smooth)), params.alpha_max);

% Compute jerk (derivative of acceleration)
jerk_vx = diff(ax_smooth) / dt_control;
jerk_wz = diff(alpha_smooth) / dt_control;
fprintf('Max forward jerk: %.3f m/s³ (limit: %.3f m/s³)\n', max(abs(jerk_vx)), params.jx_max);
fprintf('Max angular jerk: %.3f rad/s³ (limit: %.3f rad/s³)\n', max(abs(jerk_wz)), params.jerk_wz_max);

% Check for violations
if max(abs(ax_smooth)) > params.ax_max * 1.01
    warning('Acceleration limit violated!');
end
if max(abs(alpha_smooth)) > params.alpha_max * 1.01
    warning('Angular acceleration limit violated!');
end
if max(abs(jerk_vx)) > params.jx_max * 1.01
    warning('Jerk limit violated!');
end
if max(abs(jerk_wz)) > params.jerk_wz_max * 1.01
    warning('Angular jerk limit violated!');
end

fprintf('\n');

%% Plot Results
figure('Name', 'Trajectory Smoothing Test', 'Position', [100, 100, 1400, 900]);

% Subplot 1: Waypoints and Path
subplot(3, 3, 1);
plot(waypoints_x, waypoints_y, 'ro-', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
quiver(waypoints_x, waypoints_y, cos(waypoints_theta)*0.05, sin(waypoints_theta)*0.05, 0, 'b', 'LineWidth', 1.5);
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Waypoints (10Hz from GIK)');
axis equal;
legend('Waypoints', 'Orientation', 'Location', 'best');

% Subplot 2: Forward Velocity
subplot(3, 3, 2);
plot(t_sim, vx_raw, 'r--', 'LineWidth', 1.5); hold on;
plot(t_sim, vx_smooth, 'b-', 'LineWidth', 2);
plot([0, t_sim(end)], [params.vx_max, params.vx_max], 'k--', 'LineWidth', 1);
plot([0, t_sim(end)], [-params.vx_max, -params.vx_max], 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Vx (m/s)');
title('Forward Velocity');
legend('Raw (unsmoothed)', 'Smoothed (S-curve)', 'Limit', 'Location', 'best');

% Subplot 3: Angular Velocity
subplot(3, 3, 3);
plot(t_sim, wz_raw, 'r--', 'LineWidth', 1.5); hold on;
plot(t_sim, wz_smooth, 'b-', 'LineWidth', 2);
plot([0, t_sim(end)], [params.wz_max, params.wz_max], 'k--', 'LineWidth', 1);
plot([0, t_sim(end)], [-params.wz_max, -params.wz_max], 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Wz (rad/s)');
title('Angular Velocity');
legend('Raw (unsmoothed)', 'Smoothed (S-curve)', 'Limit', 'Location', 'best');

% Subplot 4: Forward Acceleration
subplot(3, 3, 5);
plot(t_sim, ax_smooth, 'b-', 'LineWidth', 2); hold on;
plot([0, t_sim(end)], [params.ax_max, params.ax_max], 'r--', 'LineWidth', 1);
plot([0, t_sim(end)], [-params.ax_max, -params.ax_max], 'r--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Ax (m/s²)');
title('Forward Acceleration');
legend('Smoothed', 'Limit', 'Location', 'best');

% Subplot 5: Angular Acceleration
subplot(3, 3, 6);
plot(t_sim, alpha_smooth, 'b-', 'LineWidth', 2); hold on;
plot([0, t_sim(end)], [params.alpha_max, params.alpha_max], 'r--', 'LineWidth', 1);
plot([0, t_sim(end)], [-params.alpha_max, -params.alpha_max], 'r--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Alpha (rad/s²)');
title('Angular Acceleration');
legend('Smoothed', 'Limit', 'Location', 'best');

% Subplot 6: Forward Jerk
subplot(3, 3, 8);
plot(t_sim(1:end-1), jerk_vx, 'b-', 'LineWidth', 2); hold on;
plot([0, t_sim(end)], [params.jx_max, params.jx_max], 'r--', 'LineWidth', 1);
plot([0, t_sim(end)], [-params.jx_max, -params.jx_max], 'r--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Jerk (m/s³)');
title('Forward Jerk (derivative of acceleration)');
legend('Smoothed', 'Limit', 'Location', 'best');

% Subplot 7: Angular Jerk
subplot(3, 3, 9);
plot(t_sim(1:end-1), jerk_wz, 'b-', 'LineWidth', 2); hold on;
plot([0, t_sim(end)], [params.jerk_wz_max, params.jerk_wz_max], 'r--', 'LineWidth', 1);
plot([0, t_sim(end)], [-params.jerk_wz_max, -params.jerk_wz_max], 'r--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Jerk (rad/s³)');
title('Angular Jerk (derivative of angular acceleration)');
legend('Smoothed', 'Limit', 'Location', 'best');

% Subplot 8: Velocity Comparison
subplot(3, 3, 4);
stairs(t_waypoints, [0; diff(waypoints_x)./diff(t_waypoints)], 'r--', 'LineWidth', 1.5); hold on;
plot(t_sim, vx_smooth, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Vx (m/s)');
title('Smoothing Effect on Forward Velocity');
legend('Discrete Waypoints', 'Smoothed Output', 'Location', 'best');

sgtitle('Trajectory Smoothing: Acceleration and Jerk Limiting', 'FontSize', 14, 'FontWeight', 'bold');

%% Save Results
fprintf('Test complete. Check plots for smoothing quality.\n');
fprintf('Key Points:\n');
fprintf('  1. Smooth transitions between waypoints (no jumps)\n');
fprintf('  2. Acceleration stays within limits (%.2f m/s²)\n', params.ax_max);
fprintf('  3. Jerk stays within limits (%.2f m/s³)\n', params.jx_max);
fprintf('  4. Robot will not tip over with these smooth commands\n\n');

% Clear persistent variables for next test
clear gik9dof.control.smoothTrajectoryVelocity
