%% generateCodeARM64.m
% MATLAB Coder script for generating C++ code for ARM64 (NVIDIA AGX Orin)
% Target: Ubuntu 22.04, ROS2 Humble, ARM64 architecture
%
% Prerequisites:
%   - MATLAB R2024b with MATLAB Coder
%   - Robotics System Toolbox
%   - Embedded Coder (for better optimization)
%
% This script generates a standalone C++ library that can be wrapped
% by ROS2 nodes for real-time inverse kinematics on AGX Orin.

clear; clc;

%% Configuration
PROJECT_ROOT = fileparts(mfilename('fullpath'));
CODEGEN_OUTPUT = fullfile(PROJECT_ROOT, '..', '..', 'codegen', 'arm64_realtime');

% Clean previous build
if exist(CODEGEN_OUTPUT, 'dir')
    fprintf('Cleaning previous build...\n');
    rmdir(CODEGEN_OUTPUT, 's');
end
mkdir(CODEGEN_OUTPUT);

%% Define input types for code generation (20-constraint version)
% All inputs must be strongly typed for code generation

% qCurrent: 9x1 joint configuration
qCurrent = coder.typeof(double(0), [9 1]);

% targetPose: 4x4 homogeneous transformation matrix
targetPose = coder.typeof(double(0), [4 4]);

% distBodyIndices: 20x1 int32 array of body indices
distBodyIndices = coder.typeof(int32(0), [20 1]);

% distRefBodyIndices: 20x1 int32 array of reference body indices
distRefBodyIndices = coder.typeof(int32(0), [20 1]);

% distBoundsLower: 20x1 double array of lower bounds
distBoundsLower = coder.typeof(double(0), [20 1]);

% distBoundsUpper: 20x1 double array of upper bounds
distBoundsUpper = coder.typeof(double(0), [20 1]);

% distWeights: 20x1 double array of constraint weights
distWeights = coder.typeof(double(0), [20 1]);

%% Create coder configuration for C++ library (ARM64)
cfg = coder.config('lib');

% Target settings
cfg.TargetLang = 'C++';

% Code generation settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;

%% Code generation execution
fprintf('===================================================\n');
fprintf('Generating C++ code for 20-Constraint GIK Solver...\n');
fprintf('===================================================\n');
fprintf('Target function: gik9dof.codegen_inuse.solveGIKStepWrapper\n');
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);
fprintf('Configuration:\n');
fprintf('  - Language: C++\n');
fprintf('  - Distance Constraints: 20 (fixed array)\n');
fprintf('===================================================\n\n');

try
    % Generate code with new 20-constraint interface
    codegen('-config', cfg, ...
        'gik9dof.codegen_inuse.solveGIKStepWrapper', ...
        '-args', {qCurrent, targetPose, distBodyIndices, distRefBodyIndices, ...
                  distBoundsLower, distBoundsUpper, distWeights}, ...
        '-d', CODEGEN_OUTPUT, ...
        '-report');
    
    fprintf('\n===================================================\n');
    fprintf('✓ Code generation successful!\n');
    fprintf('===================================================\n');
    fprintf('Generated files location: %s\n', CODEGEN_OUTPUT);
    fprintf('\nNext steps:\n');
    fprintf('1. Copy generated code to ROS2 workspace\n');
    fprintf('2. Create ROS2 wrapper node (gik9dof_solver_node)\n');
    fprintf('3. Build on AGX Orin with colcon\n');
    fprintf('4. Test with real robot\n');
    fprintf('===================================================\n');
    
    % List generated files
    fprintf('\nGenerated files:\n');
    generatedFiles = dir(fullfile(CODEGEN_OUTPUT, '**', '*.*'));
    for i = 1:length(generatedFiles)
        if ~generatedFiles(i).isdir
            fprintf('  - %s\n', generatedFiles(i).name);
        end
    end
    
catch ME
    fprintf('\n===================================================\n');
    fprintf('✗ Code generation failed!\n');
    fprintf('===================================================\n');
    fprintf('Error: %s\n', ME.message);
    fprintf('Location: %s (line %d)\n', ME.stack(1).file, ME.stack(1).line);
    fprintf('\nTroubleshooting:\n');
    fprintf('1. Check MATLAB version (requires R2024b)\n');
    fprintf('2. Verify Robotics System Toolbox installation\n');
    fprintf('3. Ensure all dependencies are on path\n');
    fprintf('4. Review code generation report for details\n');
    fprintf('===================================================\n');
    rethrow(ME);
end

%% Generate test harness (optional)
fprintf('\nGenerating test harness...\n');
testFile = fullfile(CODEGEN_OUTPUT, 'test_gik_solver.m');
fid = fopen(testFile, 'w');
fprintf(fid, '%% Test harness for generated C++ code (20-constraint version)\n');
fprintf(fid, 'function test_gik_solver()\n');
fprintf(fid, '    %% Test configuration\n');
fprintf(fid, '    q0 = zeros(9, 1);\n');
fprintf(fid, '    targetPose = eye(4);\n');
fprintf(fid, '    targetPose(1:3, 4) = [0.5; 0.2; 0.8];\n');
fprintf(fid, '    \n');
fprintf(fid, '    %% Distance constraint arrays (20 elements each)\n');
fprintf(fid, '    distBodyIndices = int32(zeros(20, 1));\n');
fprintf(fid, '    distRefBodyIndices = int32(zeros(20, 1));\n');
fprintf(fid, '    distBoundsLower = zeros(20, 1);\n');
fprintf(fid, '    distBoundsUpper = zeros(20, 1);\n');
fprintf(fid, '    distWeights = zeros(20, 1);\n');
fprintf(fid, '    \n');
fprintf(fid, '    %% Example: Enable constraint 1 (gripper to chassis, keep > 0.3m)\n');
fprintf(fid, '    distBodyIndices(1) = int32(12);     %% left_gripper_link\n');
fprintf(fid, '    distRefBodyIndices(1) = int32(4);   %% abstract_chassis_link\n');
fprintf(fid, '    distBoundsLower(1) = 0.3;\n');
fprintf(fid, '    distBoundsUpper(1) = 100.0;\n');
fprintf(fid, '    distWeights(1) = 1.0;\n');
fprintf(fid, '    \n');
fprintf(fid, '    %% Call solver\n');
fprintf(fid, '    [qNext, info] = gik9dof.codegen_inuse.solveGIKStepWrapper(q0, targetPose, ...\n');
fprintf(fid, '        distBodyIndices, distRefBodyIndices, distBoundsLower, distBoundsUpper, distWeights);\n');
fprintf(fid, '    \n');
fprintf(fid, '    %% Display results\n');
fprintf(fid, '    fprintf(''Solution found: %%d\\n'', info.Status);\n');
fprintf(fid, '    fprintf(''Iterations: %%d\\n'', info.Iterations);\n');
fprintf(fid, '    fprintf(''Pose error: %%.6f\\n'', info.PoseErrorNorm);\n');
fprintf(fid, '    disp(''Joint configuration:'');\n');
fprintf(fid, '    disp(qNext);\n');
fprintf(fid, 'end\n');
fclose(fid);
fprintf('✓ Test harness saved: %s\n', testFile);

fprintf('\n===================================================\n');
fprintf('Code generation complete!\n');
fprintf('===================================================\n');
