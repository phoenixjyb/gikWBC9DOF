%% Generate Standalone C++ Code with All Dependencies (packNGo)
% This script uses MATLAB Coder's packNGo feature to create a complete
% standalone package including all external dependencies from Robotics
% System Toolbox.
%
% Run this in MATLAB R2024b on Windows

clear; clc;

%% Setup paths
PROJECT_ROOT = fileparts(mfilename('fullpath'));
CODEGEN_OUTPUT = fullfile(PROJECT_ROOT, 'codegen', 'arm64_packNGo');

% Add MATLAB code to path
addpath(fullfile(PROJECT_ROOT, 'matlab'));

fprintf('===================================================\n');
fprintf('Generating Standalone C++ Package with packNGo\n');
fprintf('===================================================\n');
fprintf('This will include ALL external dependencies\n');
fprintf('from Robotics System Toolbox\n');
fprintf('===================================================\n\n');

%% Create code generation config
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof';
cfg.GenerateExampleMain = 'DoNotGenerate';
cfg.PackageType = 'Compressed';  % Create ZIP package
cfg.ZipFileName = 'gik9dof_standalone';

% Code generation settings
cfg.GenerateReport = true;
cfg.LaunchReport = false;
cfg.BuildConfiguration = 'Faster Runs';

% Enable dynamic memory (required for IK constraints)
cfg.EnableDynamicMemoryAllocation = true;
cfg.DynamicMemoryAllocationThreshold = 65536;

% Enable OpenMP
cfg.EnableOpenMP = true;

% C++ settings
cfg.CppInterfaceStyle = 'Methods';
cfg.CppInterfaceClassName = 'GIKSolver';

%% Define example inputs for code generation
qCurrent = zeros(9, 1);
targetPose = eye(4);
distanceLower = 0.5;
distanceWeight = 1.0;

%% Generate code
fprintf('Generating code...\n');
codegen('-config', cfg, ...
    'gik9dof.codegen_realtime.solveGIKStepWrapper', ...
    '-args', {qCurrent, targetPose, distanceLower, distanceWeight}, ...
    '-d', CODEGEN_OUTPUT, ...
    '-report');

fprintf('\n✓ Code generation complete!\n');
fprintf('Output directory: %s\n', CODEGEN_OUTPUT);

%% Use packNGo to create standalone package
fprintf('\n===================================================\n');
fprintf('Creating standalone package with packNGo...\n');
fprintf('===================================================\n');

% Navigate to codegen directory
cd(CODEGEN_OUTPUT);

% Create packNGo package
fprintf('Running packNGo...\n');
packNGo_opts = {};  % Default options include all dependencies

try
    % packNGo automatically includes all external dependencies
    packNGo(packNGo_opts);
    
    fprintf('\n✓ packNGo complete!\n');
    fprintf('Standalone package created in: %s\n', CODEGEN_OUTPUT);
    fprintf('Look for packNGo directory or ZIP file\n');
    
    % List generated files
    fprintf('\nGenerated files:\n');
    dir_listing = dir(CODEGEN_OUTPUT);
    for i = 1:length(dir_listing)
        if ~dir_listing(i).isdir
            fprintf('  - %s\n', dir_listing(i).name);
        end
    end
    
catch ME
    fprintf('\n⚠ packNGo failed: %s\n', ME.message);
    fprintf('This may happen if external dependencies cannot be packaged\n');
    fprintf('Trying alternative approach...\n');
end

% Return to project root
cd(PROJECT_ROOT);

fprintf('\n===================================================\n');
fprintf('Next Steps:\n');
fprintf('===================================================\n');
fprintf('1. Check %s/packNGo/ directory\n', CODEGEN_OUTPUT);
fprintf('2. If packNGo succeeded, copy packNGo/* to WSL\n');
fprintf('3. If packNGo failed, we need a different approach\n');
fprintf('   (see generate_code_arm64_noCollision.m)\n');
fprintf('===================================================\n');
