% Quick debug script to find where jerk violation occurs
cd matlab
clear gik9dof.control.smoothTrajectoryVelocity;

% Run simulation 
test_smoothing_real_data;

% Find where jerk is maximum
figure;
plot(t_sim(2:end), abs(jerk_vx(2:end)));
grid on;
xlabel('Time (s)');
ylabel('|Jerk| (m/s³)');
title('Jerk vs Time');
yline(params.jx_max, 'r--', 'Limit');

% Find peak
[max_jerk, idx_max] = max(abs(jerk_vx));
fprintf('\nMax jerk occurs at:\n');
fprintf('  Time: %.3f s (index %d)\n', t_sim(idx_max), idx_max);
fprintf('  Jerk: %.3f m/s³\n', jerk_vx(idx_max));
fprintf('  ax_smooth: %.3f m/s²\n', ax_smooth(idx_max));
if idx_max > 1
    fprintf('  ax_smooth(prev): %.3f m/s²\n', ax_smooth(idx_max-1));
    fprintf('  da = %.3f m/s²\n', ax_smooth(idx_max) - ax_smooth(idx_max-1));
end
