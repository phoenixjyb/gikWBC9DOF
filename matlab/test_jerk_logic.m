% Test case where k_p * dv creates large da
clear;

v_current = 0.0;
a_current = 0.2;  % Already have some acceleration
v_target = 0.5;   % Big velocity jump
dt = 0.02;
jerk_max = 5.0;

dv = v_target - v_current;
k_p = 1.5;
a_desired = k_p * dv;

fprintf('dv = %.3f m/s\n', dv);
fprintf('a_desired = k_p * dv = %.3f m/s²\n', a_desired);

da = a_desired - a_current;
fprintf('da (before jerk limit) = %.3f m/s²\n', da);

da_max = jerk_max * dt;
fprintf('da_max (jerk limit) = %.3f m/s²\n', da_max);

if abs(da) > da_max
    da = sign(da) * da_max;
end

fprintf('da (after jerk limit) = %.3f m/s²\n', da);

a_out = a_current + da;
fprintf('a_out = %.3f m/s²\n', a_out);

jerk = da / dt;
fprintf('jerk = %.3f m/s³ (limit: %.3f)\n', jerk, jerk_max);

if abs(jerk) > jerk_max * 1.01
    error('JERK VIOLATION!');
else
    fprintf('✅ OK\n');
end
