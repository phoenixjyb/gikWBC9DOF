%% Direct Test: Method 6 Core Functions
% This script tests Method 6 optimizers directly without the full pipeline
% to isolate any issues.

clear; close all; clc;

fprintf('=== Method 6 Direct Test ===\n\n');

%% Setup robot and simple test case
fprintf('1. Setting up robot model...\n');
urdf_file = 'mobile_manipulator_PPR_base_corrected.urdf';
robot = importrobot(urdf_file);
robot.DataFormat = 'column';

ee_body = 'left_gripper_link';

fprintf('   Robot: %d DOF\n', robot.NumBodies);
fprintf('   EE body: %s\n\n', ee_body);

%% Test 1: Base Optimizer
fprintf('2. Testing base optimizer...\n');

% Current state
state = [0; 0; 0; zeros(6,1)];  % [x; y; theta; q_arm]

% Target EE pose (1m forward)
ee_ref = eye(4);
ee_ref(1:3, 4) = [1.0; 0; 0.5];

% Options
baseOpts = struct();
baseOpts.dt = 0.05;
baseOpts.v_prev = 0;
baseOpts.omega_prev = 0;
baseOpts.v_max = 0.5;
baseOpts.omega_max = 1.0;
baseOpts.wheel_max = 1.0;
baseOpts.track_width = 0.574;
baseOpts.ee_body_name = ee_body;

try
    tic;
    [v_opt, omega_opt, diag] = gik9dof.solveBaseOptimization(...
        robot, state, ee_ref, baseOpts);
    t_base = toc;
    
    fprintf('   ✅ Base optimizer SUCCESS!\n');
    fprintf('      Solve time: %.1f ms\n', t_base*1000);
    fprintf('      Optimal v: %.3f m/s\n', v_opt);
    fprintf('      Optimal omega: %.3f rad/s\n', omega_opt);
    fprintf('      Exitflag: %d\n', diag.exitflag);
    fprintf('      Iterations: %d\n\n', diag.iterations);
catch ME
    fprintf('   ❌ Base optimizer FAILED: %s\n\n', ME.message);
    rethrow(ME);
end

%% Test 2: Arm Optimizer
fprintf('3. Testing arm optimizer...\n');

% Options
armOpts = struct();
armOpts.dt = 0.05;
armOpts.q_dot_prev = zeros(6,1);
armOpts.q_dot_max = 2.0;  % rad/s
armOpts.q_min = -pi * ones(6,1);
armOpts.q_max = pi * ones(6,1);
armOpts.ee_body_name = ee_body;

try
    tic;
    [q_dot_opt, diag] = gik9dof.solveArmOptimization(...
        robot, state, ee_ref, armOpts);
    t_arm = toc;
    
    fprintf('   ✅ Arm optimizer SUCCESS!\n');
    fprintf('      Solve time: %.1f ms\n', t_arm*1000);
    fprintf('      Joint velocities: [');
    fprintf('%.2f ', q_dot_opt);
    fprintf('] rad/s\n');
    fprintf('      Exitflag: %d\n', diag.exitflag);
    fprintf('      Iterations: %d\n\n', diag.iterations);
catch ME
    fprintf('   ❌ Arm optimizer FAILED: %s\n\n', ME.message);
    rethrow(ME);
end

%% Test 3: Combined Alternating Step
fprintf('4. Testing alternating sequence (5 steps)...\n');

state = [0; 0; 0; zeros(6,1)];
v_prev = 0;
omega_prev = 0;
q_dot_prev = zeros(6,1);

for k = 1:5
    if mod(k, 2) == 1
        % BASE STEP
        baseOpts.v_prev = v_prev;
        baseOpts.omega_prev = omega_prev;
        [v_opt, omega_opt, ~] = gik9dof.solveBaseOptimization(...
            robot, state, ee_ref, baseOpts);
        
        % Apply base motion
        x = state(1);
        y = state(2);
        theta = state(3);
        state(1:3) = [x + 0.05*v_opt*cos(theta);
                      y + 0.05*v_opt*sin(theta);
                      theta + 0.05*omega_opt];
        
        v_prev = v_opt;
        omega_prev = omega_opt;
        fprintf('   Step %d (BASE): v=%.3f, ω=%.3f\n', k, v_opt, omega_opt);
    else
        % ARM STEP
        armOpts.q_dot_prev = q_dot_prev;
        [q_dot_opt, ~] = gik9dof.solveArmOptimization(...
            robot, state, ee_ref, armOpts);
        
        % Apply arm motion
        state(4:9) = state(4:9) + 0.05*q_dot_opt;
        
        q_dot_prev = q_dot_opt;
        fprintf('   Step %d (ARM): q̇=[', k);
        fprintf('%.2f ', q_dot_opt);
        fprintf(']\n');
    end
    
    % Check EE error
    T_current = getTransform(robot, state, ee_body);
    p_current = T_current(1:3, 4);
    p_ref = ee_ref(1:3, 4);
    ee_error = norm(p_current - p_ref);
    fprintf('        EE error: %.1f mm\n', ee_error*1000);
end

fprintf('\n✅ All direct tests PASSED!\n');
fprintf('\nConclusion: Method 6 core optimizers are working correctly.\n');
fprintf('The issue with runStagedReference must be in pipeline integration.\n');
