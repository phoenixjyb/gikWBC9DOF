% WSL Linux Code Generation Script
% Generates x86_64 Linux binaries with MaxIterations=1000
% Run from WSL: matlab -batch "run_wsl_codegen_matlab"

fprintf('Starting Linux x86_64 code generation with MaxIterations=1000...\n');

% Add paths (don't add namespace directories directly)
addpath('matlab');

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

% Create IK solver with 20 constraints
fprintf('Creating IK solver...\n');
solver = generalizedInverseKinematics('RigidBodyTree', robot, ...
    'ConstraintInputs', {'position', 'position', 'position', 'position', 'position', ...
                         'position', 'position', 'position', 'position', 'position', ...
                         'position', 'position', 'position', 'position', 'position', ...
                         'position', 'position', 'position', 'position', 'position'});
solver.SolverParameters.MaxIterations = 1000;
fprintf('✓ Solver created with MaxIterations=1000\n');

% Code generation config
fprintf('Configuring code generation for x86_64 Linux...\n');
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
% R2024a uses 'Methods' instead of 'Classes' (Classes added in R2024b)
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'GIKSolver';
cfg.GenCodeOnly = false;
cfg.GenerateMakefile = true;
cfg.GenerateReport = true;
cfg.EnableOpenMP = true;

% R2024a doesn't use coder.hardware() for x86-64
% Just set the target architecture directly
cfg.TargetLang = 'C++';

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
fprintf('This may take 5-10 minutes...\n\n');

% Example inputs for code generation (matching solveGIKStepWrapper signature)
qCurrent = zeros(9, 1);
targetPose = eye(4);
distBodyIndices = int32(ones(20, 1));
distRefBodyIndices = int32(ones(20, 1));
distBoundsLower = zeros(20, 1);
distBoundsUpper = ones(20, 1);
distWeights = ones(20, 1);

% Get the full path to the wrapper function
wrapperPath = fullfile(pwd, 'matlab', '+gik9dof', '+codegen_inuse', 'solveGIKStepWrapper.m');
fprintf('Entry point: %s\n', wrapperPath);

% Run codegen
try
    codegen('-config', cfg, ...
        wrapperPath, ...
        '-args', {qCurrent, targetPose, distBodyIndices, distRefBodyIndices, ...
                  distBoundsLower, distBoundsUpper, distWeights}, ...
        '-d', outputDir, ...
        '-report');
    
    fprintf('\n===================================================\n');
    fprintf('✓ CODE GENERATION COMPLETE!\n');
    fprintf('===================================================\n');
    fprintf('Output: %s\n', outputDir);
    fprintf('\nNext steps:\n');
    fprintf('1. Verify Linux binaries: file %s/*.o\n', outputDir);
    fprintf('2. Build validator: cd validation && bash build_validation_wsl.sh\n');
    fprintf('3. Run tests: ./validate_gik_standalone\n');
    fprintf('===================================================\n');
catch ME
    fprintf('\n❌ CODE GENERATION FAILED!\n');
    fprintf('Error: %s\n', ME.message);
    fprintf('Identifier: %s\n', ME.identifier);
    rethrow(ME);
end
