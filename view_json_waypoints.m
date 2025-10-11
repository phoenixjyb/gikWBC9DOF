%% View JSON Waypoints
% Simple script to read and visualize the end-effector trajectory waypoints
% from the JSON file in 3D (X, Y, Z)

clear; clc; close all;

%% Read JSON file
jsonFile = '1_pull_world_scaled.json';
fprintf('Reading: %s\n', jsonFile);

if ~exist(jsonFile, 'file')
    error('JSON file not found: %s', jsonFile);
end

data = jsondecode(fileread(jsonFile));
numWaypoints = length(data.poses);

fprintf('Found %d waypoints\n\n', numWaypoints);

%% Extract XYZ positions
X = zeros(numWaypoints, 1);
Y = zeros(numWaypoints, 1);
Z = zeros(numWaypoints, 1);

for i = 1:numWaypoints
    pos = data.poses(i).position;
    X(i) = pos(1);
    Y(i) = pos(2);
    Z(i) = pos(3);
end

%% Display statistics
fprintf('--- Position Statistics ---\n');
fprintf('X: min=%.4f, max=%.4f, range=%.4f\n', min(X), max(X), max(X)-min(X));
fprintf('Y: min=%.4f, max=%.4f, range=%.4f\n', min(Y), max(Y), max(Y)-min(Y));
fprintf('Z: min=%.4f, max=%.4f, range=%.4f\n', min(Z), max(Z), max(Z)-min(Z));
fprintf('\n');

fprintf('First waypoint: [%.4f, %.4f, %.4f]\n', X(1), Y(1), Z(1));
fprintf('Last waypoint:  [%.4f, %.4f, %.4f]\n', X(end), Y(end), Z(end));
fprintf('\n');

%% Create 3D visualization
figure('Name', 'End-Effector Trajectory Waypoints', 'Position', [100, 100, 1200, 800]);

% 3D view
subplot(2, 2, 1);
plot3(X, Y, Z, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 8);
hold on;
plot3(X(1), Y(1), Z(1), 'go', 'MarkerSize', 12, 'LineWidth', 2); % Start
plot3(X(end), Y(end), Z(end), 'ro', 'MarkerSize', 12, 'LineWidth', 2); % End
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(sprintf('3D Trajectory (%d waypoints)', numWaypoints));
legend('Trajectory', 'Start', 'End', 'Location', 'best');
axis equal;
view(45, 30);

% XY top view
subplot(2, 2, 2);
plot(X, Y, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 8);
hold on;
plot(X(1), Y(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(X(end), Y(end), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('Top View (XY)');
axis equal;
legend('Trajectory', 'Start', 'End', 'Location', 'best');

% XZ side view
subplot(2, 2, 3);
plot(X, Z, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 8);
hold on;
plot(X(1), Z(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(X(end), Z(end), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
grid on;
xlabel('X (m)');
ylabel('Z (m)');
title('Side View (XZ)');
axis equal;
legend('Trajectory', 'Start', 'End', 'Location', 'best');

% YZ front view
subplot(2, 2, 4);
plot(Y, Z, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 8);
hold on;
plot(Y(1), Z(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(Y(end), Z(end), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
grid on;
xlabel('Y (m)');
ylabel('Z (m)');
title('Front View (YZ)');
axis equal;
legend('Trajectory', 'Start', 'End', 'Location', 'best');

%% Display data table (first and last 10 waypoints)
fprintf('=== First 10 Waypoints ===\n');
fprintf('  #     X         Y         Z\n');
fprintf('---  --------  --------  --------\n');
for i = 1:min(10, numWaypoints)
    fprintf('%3d  %8.4f  %8.4f  %8.4f\n', i, X(i), Y(i), Z(i));
end

if numWaypoints > 20
    fprintf('...\n');
    fprintf('=== Last 10 Waypoints ===\n');
    fprintf('  #     X         Y         Z\n');
    fprintf('---  --------  --------  --------\n');
    for i = max(1, numWaypoints-9):numWaypoints
        fprintf('%3d  %8.4f  %8.4f  %8.4f\n', i, X(i), Y(i), Z(i));
    end
end

fprintf('\nVisualization complete!\n');
