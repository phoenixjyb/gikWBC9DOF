#!/bin/bash
# WSL Linux Code Generation - Fixed Path Version
# This script runs MATLAB codegen entirely within WSL using Linux paths

set -e

MATLAB_PATH="/home/yanbo/MATLAB/R2024a/bin/matlab"
# Use Linux path (mounted Windows drive)
PROJECT_DIR="/mnt/c/Users/yanbo/wSpace/codegenGIKsample/Trial/gikWBC9DOF"

echo "=========================================="
echo "WSL Linux Code Generation (Fixed Paths)"
echo "=========================================="
echo "MATLAB: $MATLAB_PATH"
echo "Project: $PROJECT_DIR"
echo ""

# Navigate to project
cd "$PROJECT_DIR"

echo "Starting code generation..."
echo ""

# Run MATLAB with inline script to avoid path issues
$MATLAB_PATH -batch "
fprintf('Starting Linux x86_64 code generation with MaxIterations=1000...\n');

% Add paths
addpath('matlab');
addpath(genpath('matlab/+gik9dof'));

% Configuration
outputDir = 'codegen/x86_64_validation_noCollision';
urdfPath = 'mobile_manipulator_PPR_base_corrected_sltRdcd.urdf';

fprintf('===================================================================\n');
fprintf('Generating x86_64 Linux Code (No Collision) - MaxIterations=1000\n');
fprintf('===================================================================\n');

% Import robot without collision
fprintf('Building robot model...\n');
robot = importrobot(urdfPath, 'DataFormat', 'column');
robot.Gravity = [0; 0; -9.81];
fprintf('✓ Robot loaded (9 DOF, collision checking will NOT be used)\n');
fprintf('  Bodies: %d\n', robot.NumBodies);

% Create IK solver
fprintf('Creating IK solver...\n');
solver = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position', 'position'});
solver.SolverParameters.MaxIterations = 1000;
fprintf('✓ Solver created with MaxIterations=1000\n');

% Code generation config
fprintf('Configuring code generation for x86_64 Linux...\n');
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.CppInterfaceStyle = 'Classes';
cfg.CppInterfaceClassName = 'GIKSolver';
cfg.GenCodeOnly = false;
cfg.GenerateMakefile = true;
cfg.GenerateReport = true;
cfg.EnableOpenMP = true;

% Hardware config - x86-64 Linux
hwcfg = coder.hardware('Intel x86-64 (Linux 64)');
cfg.Hardware = hwcfg;

fprintf('===================================================\n');
fprintf('Generating C++ code for x86_64 Linux (WSL)...\n');
fprintf('===================================================\n');
fprintf('Target function: gik9dof.codegen_inuse.solveGIKStepWrapper\n');
fprintf('Output directory: %s\n', outputDir);
fprintf('Configuration:\n');
fprintf('  - Language: C++17 class-based interface\n');
fprintf('  - Architecture: Intel x86-64 (Linux 64-bit)\n');
fprintf('  - Collision: DISABLED\n');
fprintf('  - OpenMP: Enabled\n');
fprintf('  - MaxIterations: 1000 ← KEY CHANGE!\n');
fprintf('===================================================\n\n');
fprintf('This may take 2-3 minutes...\n\n');

% Example inputs for code generation
q_current = zeros(9, 1);
base_pose = eye(4);
ee_targets = zeros(4, 4, 20);
for i = 1:20
    ee_targets(:,:,i) = eye(4);
end

% Run codegen using function handle (not string path)
codegen('-config', cfg, ...
    'gik9dof.codegen_inuse.solveGIKStepWrapper', ...
    '-args', {q_current, base_pose, ee_targets}, ...
    '-d', outputDir, ...
    '-report');

fprintf('\n===================================================\n');
fprintf('✓ Code generation COMPLETE!\n');
fprintf('===================================================\n');
fprintf('Output: %s\n', outputDir);
fprintf('Next: Run build_validation_wsl.sh\n');
"

echo ""
echo "=========================================="
echo "Code Generation Complete!"
echo "=========================================="
echo "Output: codegen/x86_64_validation_noCollision/"
echo ""
echo "Next: Run build_validation_wsl.sh to build the validator"
