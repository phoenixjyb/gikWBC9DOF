%% Code Generation: Holistic Velocity Controller
% Generates C++ code for heading-based velocity controller
% Used to replace simple 5-point differentiation with true trajectory tracking
%
% Date: October 7, 2025
% Target: ARM64 (Cortex-A) and x86_64

clear; clc;

%% Add paths
addpath(fullfile(pwd, 'matlab'));
addpath(fullfile(pwd, 'matlab', '+gik9dof', '+control'));

%% Define example inputs for type inference

% Reference: target position from IK solver
refX = 0.0;
refY = 0.0;
refTheta = 0.0;
refTime = 0.0;

% Estimate: current robot pose
estX = 0.0;
estY = 0.0;
estYaw = 0.0;

% State: controller memory (stores previous reference for differentiation)
% Initialize with empty prev field - controller will handle first call
stateIn = struct('prev', struct('x', 0.0, 'y', 0.0, 'theta', 0.0, 't', 0.0));

% Parameters: robot configuration and controller gains
params = struct(...
    'track', 0.5, ...          % Wheel track width [m]
    'Vwheel_max', 1.0, ...     % Max wheel speed [m/s]
    'Vx_max', 0.8, ...         % Max forward velocity [m/s]
    'W_max', 1.0, ...          % Max yaw rate [rad/s]
    'yawKp', 1.5, ...          % Heading proportional gain
    'yawKff', 0.8 ...          % Heading feedforward gain
);

%% Configure code generation

% Create ARM64 configuration
cfg_arm64 = coder.config('lib');
cfg_arm64.TargetLang = 'C++';
cfg_arm64.CppNamespace = 'gik9dof_velocity';
cfg_arm64.GenerateReport = true;
cfg_arm64.ReportPotentialDifferences = false;
cfg_arm64.GenCodeOnly = true;
cfg_arm64.PreserveVariableNames = 'UserNames';
cfg_arm64.SupportNonFinite = false;

% ARM Cortex-A optimization
cfg_arm64.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
cfg_arm64.HardwareImplementation.ProdWordSize = 64;

% Compiler optimizations
cfg_arm64.BuildConfiguration = 'Faster Runs';
cfg_arm64.EnableOpenMP = false;  % Use ARM NEON instead

fprintf('═══════════════════════════════════════════════════════\n');
fprintf('  Code Generation: Holistic Velocity Controller\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

%% Generate ARM64 code

fprintf('[1/2] Generating ARM64 code (Cortex-A)...\n');
try
    codegen -config cfg_arm64 ...
        holisticVelocityController ...
        -args {refX, refY, refTheta, refTime, ...
               estX, estY, estYaw, ...
               stateIn, params} ...
        -d codegen/velocity_controller_arm64 ...
        -report;
    
    fprintf('✅ ARM64 code generation successful!\n');
    fprintf('   Location: codegen/velocity_controller_arm64/\n\n');
catch ME
    fprintf('❌ ARM64 code generation failed:\n');
    fprintf('   %s\n', ME.message);
    rethrow(ME);
end

%% Generate x86_64 code (for WSL testing)

fprintf('[2/2] Generating x86_64 code (local testing)...\n');

cfg_x64 = cfg_arm64;
cfg_x64.HardwareImplementation.ProdHWDeviceType = 'Intel->x86-64 (Linux 64)';

try
    codegen -config cfg_x64 ...
        holisticVelocityController ...
        -args {refX, refY, refTheta, refTime, ...
               estX, estY, estYaw, ...
               stateIn, params} ...
        -d codegen/velocity_controller_x64 ...
        -report;
    
    fprintf('✅ x86_64 code generation successful!\n');
    fprintf('   Location: codegen/velocity_controller_x64/\n\n');
catch ME
    fprintf('❌ x86_64 code generation failed:\n');
    fprintf('   %s\n', ME.message);
    rethrow(ME);
end

%% Summary

fprintf('═══════════════════════════════════════════════════════\n');
fprintf('  Code Generation Complete!\n');
fprintf('═══════════════════════════════════════════════════════\n\n');

fprintf('Generated Functions:\n');
fprintf('  • holisticVelocityController() - Main entry point\n');
fprintf('  • unifiedChassisCtrl()         - Core controller\n');
fprintf('  • clampYawByWheelLimit()       - Wheel limit enforcement\n\n');

fprintf('Key Files:\n');
fprintf('  ARM64:\n');
fprintf('    codegen/velocity_controller_arm64/holisticVelocityController.h\n');
fprintf('    codegen/velocity_controller_arm64/holisticVelocityController.cpp\n');
fprintf('  x86_64:\n');
fprintf('    codegen/velocity_controller_x64/holisticVelocityController.h\n');
fprintf('    codegen/velocity_controller_x64/holisticVelocityController.cpp\n\n');

fprintf('Next Steps:\n');
fprintf('  1. Verify generated code compiles (check reports)\n');
fprintf('  2. Integrate into ROS2 gik9dof_solver_node.cpp\n');
fprintf('  3. Replace 5-point differentiation in publishBaseCommand()\n');
fprintf('  4. Test trajectory tracking on Orin\n\n');

fprintf('Algorithm Upgrade:\n');
fprintf('  FROM: 5-point finite difference (open-loop)\n');
fprintf('  TO:   Heading controller with P+FF gains (closed-loop)\n');
fprintf('  Benefits: True trajectory tracking, wheel limit enforcement\n\n');

disp('✅ Ready for ROS2 integration!');
