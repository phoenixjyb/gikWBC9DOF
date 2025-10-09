% Visualize the smoothing strategy: Input waypoints vs Output commands
clear; close all; clc;

fprintf('========================================\n');
fprintf('Trajectory Smoothing Strategy Visualization\n');
fprintf('========================================\n\n');

%% Generate Sample Data
% Input: 10 waypoints at 10 Hz (1 second)
t_waypoints = (0:0.1:0.9)';  % 10 waypoints
N_waypoints = length(t_waypoints);

% Simple trajectory: acceleration from 0 to 0.5 m/s
x_waypoints = 0.05 * t_waypoints.^2;  % x = 0.5 * a * t^2, a=0.2 m/s²
y_waypoints = zeros(size(x_waypoints));
theta_waypoints = zeros(size(x_waypoints));

% Output: 50 commands at 50 Hz (1 second)
t_commands = (0:0.02:1.0)';  % 51 commands
N_commands = length(t_commands);

fprintf('INPUT:\n');
fprintf('  Waypoints: %d points\n', N_waypoints);
fprintf('  Frequency: %.0f Hz (dt=%.3f s)\n', 1/0.1, 0.1);
fprintf('  Type: Position [x, y, θ]\n');
fprintf('  Duration: %.1f seconds\n\n', t_waypoints(end));

fprintf('OUTPUT:\n');
fprintf('  Commands: %d points\n', N_commands);
fprintf('  Frequency: %.0f Hz (dt=%.3f s)\n', 1/0.02, 0.02);
fprintf('  Type: Velocity [vx, wz]\n');
fprintf('  Duration: %.1f seconds\n\n', t_commands(end));

fprintf('RATIO: %.1f× more output points\n\n', N_commands / N_waypoints);

%% Compute Raw Velocities (Without Smoothing)
vx_raw = zeros(N_commands, 1);
for i = 1:N_commands
    t_current = t_commands(i);
    idx = find(t_waypoints >= t_current, 1, 'first');
    
    if isempty(idx) || idx == 1
        vx_raw(i) = 0;
    else
        % Velocity from segment
        dx = x_waypoints(idx) - x_waypoints(idx-1);
        dt = t_waypoints(idx) - t_waypoints(idx-1);
        vx_raw(i) = dx / dt;
    end
end

%% Compute Smooth Velocities (With Smoothing)
% Use our smoothing function
params = struct();
params.vx_max = 1.5;
params.ax_max = 1.0;
params.jx_max = 5.0;
params.wz_max = 2.0;
params.alpha_max = 3.0;
params.jerk_wz_max = 10.0;
params.smoothing_method = 'scurve';

vx_smooth = zeros(N_commands, 1);
ax_smooth = zeros(N_commands, 1);

clear gik9dof.control.smoothTrajectoryVelocity;
for i = 1:N_commands
    [vx_smooth(i), ~, ax_smooth(i), ~, ~, ~] = ...
        gik9dof.control.smoothTrajectoryVelocity(...
            x_waypoints, y_waypoints, theta_waypoints, t_waypoints, ...
            t_commands(i), params);
end

%% Visualization
figure('Position', [100, 100, 1400, 800], 'Name', 'Smoothing Strategy');

% Plot 1: Position waypoints (INPUT)
subplot(2, 3, 1);
plot(t_waypoints, x_waypoints, 'bo-', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
title(sprintf('INPUT: Position Waypoints\n%d points @ 10 Hz', N_waypoints));
set(gca, 'FontSize', 11);

% Plot 2: Waypoint count diagram
subplot(2, 3, 2);
axis off;
text(0.5, 0.9, 'INPUT', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(0.5, 0.75, sprintf('%d waypoints', N_waypoints), 'FontSize', 14, 'HorizontalAlignment', 'center');
text(0.5, 0.65, '@ 10 Hz (100ms)', 'FontSize', 12, 'HorizontalAlignment', 'center');
text(0.5, 0.55, 'Type: Position [x,y,θ]', 'FontSize', 11, 'HorizontalAlignment', 'center');

text(0.5, 0.4, '↓ SMOOTHING ↓', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', [0 0.6 0]);
text(0.5, 0.3, '5× Temporal Upsampling', 'FontSize', 11, 'HorizontalAlignment', 'center', 'Color', [0 0.6 0]);

text(0.5, 0.15, 'OUTPUT', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(0.5, 0, sprintf('%d commands', N_commands), 'FontSize', 14, 'HorizontalAlignment', 'center');
text(0.5, -0.1, '@ 50 Hz (20ms)', 'FontSize', 12, 'HorizontalAlignment', 'center');
text(0.5, -0.2, 'Type: Velocity [vx,wz]', 'FontSize', 11, 'HorizontalAlignment', 'center');

% Plot 3: Velocity commands (OUTPUT - Raw)
subplot(2, 3, 3);
stairs(t_commands, vx_raw, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title(sprintf('OUTPUT: Raw Velocity\n%d commands @ 50 Hz\n(No Smoothing)', N_commands));
set(gca, 'FontSize', 11);
ylim([0 max(vx_raw)*1.2]);

% Plot 4: Velocity comparison
subplot(2, 3, 4);
hold on;
stairs(t_commands, vx_raw, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Raw (stairs)');
plot(t_commands, vx_smooth, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Smooth (curve)');
hold off;
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity: Raw vs Smoothed');
legend('Location', 'northwest');
set(gca, 'FontSize', 11);

% Plot 5: Acceleration comparison
subplot(2, 3, 5);
ax_raw = [0; diff(vx_raw)] / 0.02;
hold on;
plot(t_commands, ax_raw, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Raw');
plot(t_commands, ax_smooth, 'b-', 'LineWidth', 2, 'DisplayName', 'Smooth');
yline(params.ax_max, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Limit');
yline(-params.ax_max, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
hold off;
grid on;
xlabel('Time (s)');
ylabel('Acceleration (m/s²)');
title('Acceleration: Raw vs Smoothed');
legend('Location', 'best');
set(gca, 'FontSize', 11);

% Plot 6: Data flow diagram
subplot(2, 3, 6);
axis off;
y = 0.9;
dy = 0.15;

% GIK Solver
text(0.5, y, '⬛ GIK Solver (10 Hz)', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
y = y - dy;
text(0.5, y, '↓', 'FontSize', 14, 'HorizontalAlignment', 'center');
text(0.7, y, '300 waypoints', 'FontSize', 9, 'Color', [0.5 0.5 0.5]);
y = y - dy;

% Smoothing Module
text(0.5, y, '⬛ Smoothing Module (50 Hz)', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', [0 0.6 0]);
y = y - dy;
text(0.5, y, '↓', 'FontSize', 14, 'HorizontalAlignment', 'center');
text(0.7, y, '1546 commands', 'FontSize', 9, 'Color', [0.5 0.5 0.5]);
y = y - dy;

% Motor Controller
text(0.5, y, '⬛ Motor Controller (50 Hz)', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

sgtitle('Trajectory Smoothing Strategy: Temporal Upsampling (10Hz → 50Hz)', 'FontSize', 16, 'FontWeight', 'bold');

%% Print Summary
fprintf('========================================\n');
fprintf('SUMMARY\n');
fprintf('========================================\n\n');

fprintf('Strategy: TEMPORAL UPSAMPLING + JERK-LIMITED TRACKING\n\n');

fprintf('NOT:\n');
fprintf('  ❌ 1-to-1 waypoint modification\n');
fprintf('  ❌ Spatial resampling\n');
fprintf('  ❌ Filtering existing points\n\n');

fprintf('IS:\n');
fprintf('  ✅ Temporal interpolation (10 Hz → 50 Hz)\n');
fprintf('  ✅ Position → Velocity conversion\n');
fprintf('  ✅ Real-time tracking with jerk limits\n');
fprintf('  ✅ Online algorithm (processes one timestep at a time)\n\n');

fprintf('Results:\n');
fprintf('  Max raw acceleration:    %.3f m/s²\n', max(abs(ax_raw)));
fprintf('  Max smooth acceleration: %.3f m/s²\n', max(abs(ax_smooth)));
fprintf('  Acceleration reduction:  %.1f%%\n', (1 - max(abs(ax_smooth))/max(abs(ax_raw)))*100);

fprintf('\n✅ Demonstration complete!\n');
