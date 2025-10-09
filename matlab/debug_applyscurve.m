% Debug applySCurve to understand jerk violation
clear; close all; clc;

% Simulate one call to applySCurve that creates high jerk
v_current = 0.15;
a_current = 0.0;
v_target = 0.20;
dt = 0.02;
a_max = 1.0;
jerk_max = 5.0;

fprintf('Input state:\n');
fprintf('  v_current = %.3f m/s\n', v_current);
fprintf('  a_current = %.3f m/s²\n', a_current);
fprintf('  v_target = %.3f m/s\n', v_target);
fprintf('  dt = %.3f s\n', dt);
fprintf('  jerk_max = %.3f m/s³\n', jerk_max);

% Reproduce applySCurve logic
dv = v_target - v_current;
fprintf('\nVelocity error: dv = %.3f m/s\n', dv);

k_p = 1.5;
a_desired = k_p * dv;
fprintf('Desired acceleration: a_desired = %.3f m/s²\n', a_desired);

da_max = jerk_max * dt;
fprintf('Max accel change (jerk limit): da_max = %.3f m/s²\n', da_max);

da = a_desired - a_current;
fprintf('Requested accel change: da = %.3f m/s²\n', da);

if abs(da) > da_max
    da = sign(da) * da_max;
    fprintf('→ Clamped to: da = %.3f m/s²\n', da);
end

a_out = a_current + da;
fprintf('Output acceleration: a_out = %.3f m/s²\n', a_out);

% Check jerk
jerk_actual = da / dt;
fprintf('Actual jerk: %.3f m/s³ (limit: %.3f)\n', jerk_actual, jerk_max);

if abs(jerk_actual) > jerk_max * 1.01
    fprintf('❌ JERK VIOLATION!\n');
else
    fprintf('✅ Jerk OK\n');
end

% Now simulate what happens if a_out was clamped to a_max
fprintf('\n========== SCENARIO: If we clamped a_out to a_max ==========\n');
a_out_clamped = sign(a_out) * min(abs(a_out), a_max);
fprintf('Clamped acceleration: %.3f m/s²\n', a_out_clamped);
da_actual = a_out_clamped - a_current;
jerk_from_clamp = da_actual / dt;
fprintf('Jerk from clamping: %.3f m/s³\n', jerk_from_clamp);
if abs(jerk_from_clamp) > jerk_max * 1.01
    fprintf('❌ CLAMPING CREATES JERK VIOLATION!\n');
end
