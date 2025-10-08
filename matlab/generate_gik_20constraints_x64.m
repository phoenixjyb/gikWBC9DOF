%% Generate x64 C++ Code for GIK 20-Constraint Solver
% This script generates x64 C++ code for local Windows testing
% Uses the same wrapper as ARM64 version
% Output: codegen/gik9dof_x64_20constraints/

clearvars; clc;

% Add paths
addpath(genpath(fileparts(mfilename('fullpath'))));

% Import the namespace
import gik9dof.codegen_inuse.*

%% Input Types
qCurrent_type = coder.typeof(zeros(9,1));
targetPose_type = coder.typeof(eye(4));
distBodyIndices_type = coder.typeof(int32(zeros(20,1)));
distRefBodyIndices_type = coder.typeof(int32(zeros(20,1)));
distBoundsLower_type = coder.typeof(zeros(20,1));
distBoundsUpper_type = coder.typeof(zeros(20,1));
distWeights_type = coder.typeof(zeros(20,1));

%% Code Configuration
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.GenerateReport = true;

%% Output Directory
outputDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'codegen', 'gik9dof_x64_20constraints');

% Clean previous build
if exist(outputDir, 'dir')
    fprintf('Cleaning previous build...\n');
    rmdir(outputDir, 's');
end

%% Generate Code
fprintf('===================================================\n');
fprintf('Generating C++ code for 20-Constraint GIK Solver (x64)...\n');
fprintf('===================================================\n');
fprintf('Target function: gik9dof.codegen_inuse.solveGIKStepWrapper\n');
fprintf('Output directory: %s\n', outputDir);
fprintf('Configuration:\n');
fprintf('  - Language: C++\n');
fprintf('  - Platform: x86-64 (Windows64)\n');
fprintf('  - Distance Constraints: 20 (fixed array)\n');
fprintf('===================================================\n\n');

try
    % Generate code with 20-constraint interface
    codegen('-config', cfg, ...
        'gik9dof.codegen_inuse.solveGIKStepWrapper', ...
        '-args', {qCurrent_type, targetPose_type, ...
                  distBodyIndices_type, distRefBodyIndices_type, ...
                  distBoundsLower_type, distBoundsUpper_type, distWeights_type}, ...
        '-d', outputDir, ...
        '-report');
    
    fprintf('\n===================================================\n');
    fprintf('✅ x64 Code generation SUCCESSFUL!\n');
    fprintf('===================================================\n');
    fprintf('Generated files: %s\n', outputDir);
    fprintf('\nKey files:\n');
    fprintf('  - solveGIKStepWrapper.h\n');
    fprintf('  - solveGIKStepWrapper.cpp\n');
    fprintf('  - Complete robot model and solver implementation\n');
    
catch ME
    fprintf('\n===================================================\n');
    fprintf('❌ Code generation FAILED!\n');
    fprintf('===================================================\n');
    fprintf('Error: %s\n', ME.message);
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    rethrow(ME);
end

fprintf('\n📊 Code Generation Summary:\n');
fprintf('  Platform: x64 (Windows64)\n');
fprintf('  Constraints: 20 distance constraints\n');
fprintf('  Interface: 7 parameters\n');
fprintf('  Next step: Build standalone C++ test\n');
