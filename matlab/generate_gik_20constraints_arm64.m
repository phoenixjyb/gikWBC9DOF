%% generate_gik_20constraints_arm64.m
% Generate C++ code for 20-constraint GIK solver (ARM64 target)
% Simplified version with absolute paths

clear; clc;

fprintf('===================================================\n');
fprintf('Generating C++ Code for 20-Constraint GIK Solver\n');
fprintf('===================================================\n\n');

%% Setup paths
MATLAB_ROOT = pwd;
CODEGEN_OUTPUT = fullfile(MATLAB_ROOT, '..', 'codegen', 'gik9dof_arm64_20constraints');

% Add paths
addpath(genpath(MATLAB_ROOT));

% Clean previous build
if exist(CODEGEN_OUTPUT, 'dir')
    fprintf('Cleaning previous build: %s\n', CODEGEN_OUTPUT);
    rmdir(CODEGEN_OUTPUT, 's');
end
mkdir(CODEGEN_OUTPUT);
fprintf('Output directory: %s\n\n', CODEGEN_OUTPUT);

%% Define input types for code generation (20-constraint version)
fprintf('Defining input types...\n');

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

fprintf('  ✓ Input types defined\n\n');

%% Create coder configuration
fprintf('Creating coder configuration...\n');

cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.GenerateReport = true;
cfg.LaunchReport = false;

fprintf('  ✓ Configuration created\n');
fprintf('  - Target: C++ library\n');
fprintf('  - Report: Enabled\n\n');

%% Generate code
fprintf('===================================================\n');
fprintf('Starting code generation...\n');
fprintf('This may take several minutes...\n');
fprintf('===================================================\n\n');

try
    tic;
    codegen('-config', cfg, ...
        'gik9dof.codegen_inuse.solveGIKStepWrapper', ...
        '-args', {qCurrent, targetPose, distBodyIndices, distRefBodyIndices, ...
                  distBoundsLower, distBoundsUpper, distWeights}, ...
        '-d', CODEGEN_OUTPUT, ...
        '-report');
    elapsed = toc;
    
    fprintf('\n===================================================\n');
    fprintf('✓ Code generation successful!\n');
    fprintf('===================================================\n');
    fprintf('Time elapsed: %.1f seconds\n', elapsed);
    fprintf('Output directory: %s\n\n', CODEGEN_OUTPUT);
    
    % List generated files
    fprintf('Generated files:\n');
    cppFiles = dir(fullfile(CODEGEN_OUTPUT, '**', '*.cpp'));
    hFiles = dir(fullfile(CODEGEN_OUTPUT, '**', '*.h'));
    
    fprintf('  C++ files: %d\n', length(cppFiles));
    fprintf('  Header files: %d\n', length(hFiles));
    
    fprintf('\n===================================================\n');
    fprintf('Next steps:\n');
    fprintf('1. Review code generation report\n');
    fprintf('2. Copy generated code to ROS2 workspace\n');
    fprintf('3. Update ROS2 wrapper with 20-constraint interface\n');
    fprintf('4. Build and test on target platform\n');
    fprintf('===================================================\n');
    
catch ME
    fprintf('\n===================================================\n');
    fprintf('✗ Code generation failed!\n');
    fprintf('===================================================\n');
    fprintf('Error: %s\n', ME.message);
    
    if ~isempty(ME.stack)
        fprintf('\nStack trace:\n');
        for i = 1:length(ME.stack)
            fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
        end
    end
    
    fprintf('\n===================================================\n');
    rethrow(ME);
end
