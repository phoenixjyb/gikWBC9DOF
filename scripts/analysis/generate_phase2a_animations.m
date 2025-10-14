%% Generate Phase 2A Animations
% Create 3D animations comparing Phase 1 vs Phase 2A robot motion
% Shows full robot visualization with EE tracking

clear; clc;

fprintf('====================================================\n');
fprintf('  Phase 2A Animation Generation\n');
fprintf('====================================================\n\n');

%% Load results and robot
results_dir = 'results/phase2a_orientation_z';
files = dir(fullfile(results_dir, 'log_phase*.mat'));

latest_phase1 = fullfile(results_dir, files(find(contains({files.name}, 'phase1'), 1, 'first')).name);
latest_phase2a = fullfile(results_dir, files(find(contains({files.name}, 'phase2a'), 1, 'first')).name);

fprintf('Loading results...\n');
load(latest_phase1, 'log_phase1');
load(latest_phase2a, 'log_phase2a');

% Create robot model
robot = gik9dof.createRobotModel();

% Animation settings
anim_dir = fullfile(results_dir, 'animations');
if ~exist(anim_dir, 'dir')
    mkdir(anim_dir);
end

fps = 10;  % Frames per second
skip = 5;  % Show every 5th waypoint for speed

%% Animation 1: Phase 1
fprintf('\nGenerating Phase 1 animation...\n');

fig1 = figure('Position', [100, 100, 1200, 800]);

% Determine axis limits from both logs
all_base_x = [log_phase1.baseActual(1,:), log_phase2a.baseActual(1,:)];
all_base_y = [log_phase1.baseActual(2,:), log_phase2a.baseActual(2,:)];
all_ee_x = [log_phase1.eePositions(1,:), log_phase2a.eePositions(1,:)];
all_ee_y = [log_phase1.eePositions(2,:), log_phase2a.eePositions(2,:)];
all_ee_z = [log_phase1.eePositions(3,:), log_phase2a.eePositions(3,:)];

xlim_range = [min([all_base_x, all_ee_x]) - 0.5, max([all_base_x, all_ee_x]) + 0.5];
ylim_range = [min([all_base_y, all_ee_y]) - 0.5, max([all_base_y, all_ee_y]) + 0.5];
zlim_range = [0, max(all_ee_z) + 0.5];

frames_phase1 = {};
for k = 1:skip:size(log_phase1.qTraj, 2)
    clf;
    
    % Show robot
    ax = subplot(1, 1, 1);
    show(robot, log_phase1.qTraj(:, k), 'Parent', ax, 'Frames', 'off', 'PreservePlot', false);
    hold on;
    
    % Plot EE trajectory (past and future)
    plot3(log_phase1.targetPositions(1, 1:k), log_phase1.targetPositions(2, 1:k), ...
        log_phase1.targetPositions(3, 1:k), 'g-', 'LineWidth', 2, 'DisplayName', 'Target Traj');
    plot3(log_phase1.eePositions(1, 1:k), log_phase1.eePositions(2, 1:k), ...
        log_phase1.eePositions(3, 1:k), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Traj');
    
    % Current position markers
    plot3(log_phase1.targetPositions(1, k), log_phase1.targetPositions(2, k), ...
        log_phase1.targetPositions(3, k), 'go', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Target');
    plot3(log_phase1.eePositions(1, k), log_phase1.eePositions(2, k), ...
        log_phase1.eePositions(3, k), 'bo', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Actual');
    
    % Error line
    plot3([log_phase1.targetPositions(1, k), log_phase1.eePositions(1, k)], ...
          [log_phase1.targetPositions(2, k), log_phase1.eePositions(2, k)], ...
          [log_phase1.targetPositions(3, k), log_phase1.eePositions(3, k)], ...
          'r--', 'LineWidth', 2, 'DisplayName', 'Error');
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('Phase 1 - Waypoint %d/%d | Error: %.1f mm | Fallback: %s', ...
        k, size(log_phase1.qTraj, 2), log_phase1.positionErrorNorm(k) * 1000, ...
        tern(log_phase1.fallbackUsed(k), 'YES', 'NO')));
    legend('Location', 'northeast');
    grid on;
    axis equal;
    xlim(xlim_range); ylim(ylim_range); zlim(zlim_range);
    view(45, 30);
    
    drawnow;
    frames_phase1{end+1} = getframe(fig1);
end

% Save Phase 1 video
fprintf('  Saving Phase 1 video...\n');
v1 = VideoWriter(fullfile(anim_dir, 'phase1_animation.mp4'), 'MPEG-4');
v1.FrameRate = fps;
open(v1);
for i = 1:length(frames_phase1)
    writeVideo(v1, frames_phase1{i});
end
close(v1);
fprintf('  ✓ Saved: phase1_animation.mp4 (%d frames)\n', length(frames_phase1));

%% Animation 2: Phase 2A
fprintf('\nGenerating Phase 2A animation...\n');

fig2 = figure('Position', [100, 100, 1200, 800]);

frames_phase2a = {};
for k = 1:skip:size(log_phase2a.qTraj, 2)
    clf;
    
    % Show robot
    ax = subplot(1, 1, 1);
    show(robot, log_phase2a.qTraj(:, k), 'Parent', ax, 'Frames', 'off', 'PreservePlot', false);
    hold on;
    
    % Plot EE trajectory (past and future)
    plot3(log_phase2a.targetPositions(1, 1:k), log_phase2a.targetPositions(2, 1:k), ...
        log_phase2a.targetPositions(3, 1:k), 'g-', 'LineWidth', 2, 'DisplayName', 'Target Traj');
    plot3(log_phase2a.eePositions(1, 1:k), log_phase2a.eePositions(2, 1:k), ...
        log_phase2a.eePositions(3, 1:k), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Traj');
    
    % Current position markers
    plot3(log_phase2a.targetPositions(1, k), log_phase2a.targetPositions(2, k), ...
        log_phase2a.targetPositions(3, k), 'go', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Target');
    plot3(log_phase2a.eePositions(1, k), log_phase2a.eePositions(2, k), ...
        log_phase2a.eePositions(3, k), 'bo', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Actual');
    
    % Error line
    plot3([log_phase2a.targetPositions(1, k), log_phase2a.eePositions(1, k)], ...
          [log_phase2a.targetPositions(2, k), log_phase2a.eePositions(2, k)], ...
          [log_phase2a.targetPositions(3, k), log_phase2a.eePositions(3, k)], ...
          'r--', 'LineWidth', 2, 'DisplayName', 'Error');
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('Phase 2A - Waypoint %d/%d | Error: %.1f mm | Fallback: %s', ...
        k, size(log_phase2a.qTraj, 2), log_phase2a.positionErrorNorm(k) * 1000, ...
        tern(log_phase2a.fallbackUsed(k), 'YES', 'NO')));
    legend('Location', 'northeast');
    grid on;
    axis equal;
    xlim(xlim_range); ylim(ylim_range); zlim(zlim_range);
    view(45, 30);
    
    drawnow;
    frames_phase2a{end+1} = getframe(fig2);
end

% Save Phase 2A video
fprintf('  Saving Phase 2A video...\n');
v2 = VideoWriter(fullfile(anim_dir, 'phase2a_animation.mp4'), 'MPEG-4');
v2.FrameRate = fps;
open(v2);
for i = 1:length(frames_phase2a)
    writeVideo(v2, frames_phase2a{i});
end
close(v2);
fprintf('  ✓ Saved: phase2a_animation.mp4 (%d frames)\n', length(frames_phase2a));

%% Side-by-side comparison animation
fprintf('\nGenerating side-by-side comparison animation...\n');

fig3 = figure('Position', [100, 100, 1600, 600]);

frames_comparison = {};
num_frames = min(length(frames_phase1), length(frames_phase2a));

for i = 1:num_frames
    clf;
    
    % Phase 1 on left
    subplot(1, 2, 1);
    imshow(frames_phase1{i}.cdata);
    title('Phase 1: Standard Nominal', 'FontSize', 12, 'FontWeight', 'bold');
    
    % Phase 2A on right
    subplot(1, 2, 2);
    imshow(frames_phase2a{i}.cdata);
    title('Phase 2A: Orientation+Z Priority', 'FontSize', 12, 'FontWeight', 'bold');
    
    % Overall title
    idx = (i-1) * skip + 1;
    sgtitle(sprintf('Phase 1 vs Phase 2A Comparison - Waypoint %d', idx), ...
        'FontSize', 14, 'FontWeight', 'bold');
    
    drawnow;
    frames_comparison{end+1} = getframe(fig3);
end

% Save comparison video
fprintf('  Saving comparison video...\n');
v3 = VideoWriter(fullfile(anim_dir, 'phase1_vs_phase2a_comparison.mp4'), 'MPEG-4');
v3.FrameRate = fps;
open(v3);
for i = 1:length(frames_comparison)
    writeVideo(v3, frames_comparison{i});
end
close(v3);
fprintf('  ✓ Saved: phase1_vs_phase2a_comparison.mp4 (%d frames)\n', length(frames_comparison));

close all;

%% Summary
fprintf('\n====================================================\n');
fprintf('  ANIMATION GENERATION COMPLETE\n');
fprintf('====================================================\n\n');

fprintf('Generated 3 animations in: %s\n\n', anim_dir);
fprintf('  1. phase1_animation.mp4\n');
fprintf('  2. phase2a_animation.mp4\n');
fprintf('  3. phase1_vs_phase2a_comparison.mp4\n\n');

fprintf('Key Observations:\n');
fprintf('  • Phase 1: Mean error %.1f mm, fallback rate %.1f%%\n', ...
    log_phase1.avgEEError * 1000, log_phase1.fallbackRate * 100);
fprintf('  • Phase 2A: Mean error %.1f mm, fallback rate %.1f%%\n', ...
    log_phase2a.avgEEError * 1000, log_phase2a.fallbackRate * 100);
fprintf('  • Improvement: %.1f%% error reduction\n', ...
    (1 - log_phase2a.avgEEError / log_phase1.avgEEError) * 100);

fprintf('\n✅ Ready to proceed to Phase 2B!\n');

%% Helper function
function result = tern(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
