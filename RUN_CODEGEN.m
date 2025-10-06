%% RUN_CODEGEN.m
% Complete code generation workflow for ARM64 deployment
% Run this after RUN_VALIDATION.m passes

clear; clc;

fprintf('========================================\n');
fprintf('CODEGENCC45 Code Generation Workflow\n');
fprintf('========================================\n\n');

%% Configuration
projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'matlab')));

% Output directory
codegenOutput = fullfile(projectRoot, 'codegen', 'arm64_realtime');
ros2Target = fullfile(projectRoot, 'ros2', 'gik9dof_solver', 'matlab_codegen');

%% Step 1: Pre-generation validation
fprintf('Step 1: Pre-generation validation...\n');
try
    robot = gik9dof.codegen_realtime.buildRobotForCodegen();
    fprintf('  ✓ Robot builds successfully\n');
catch ME
    error('Robot builder failed: %s', ME.message);
end

try
    q0 = zeros(9, 1);
    T = eye(4); T(1:3, 4) = [0.5; 0.2; 0.8];
    [~, info] = gik9dof.codegen_realtime.solveGIKStepWrapper(q0, T, 0.1, 1.0);
    if isfield(info, 'Status')
        fprintf('  ✓ Solver works (status: %s)\n\n', info.Status);
    else
        fprintf('  ✓ Solver works\n\n');
    end
catch ME
    error('Solver failed: %s', ME.message);
end

%% Step 2: Run code generation
fprintf('Step 2: Generating C++ code for ARM64...\n');
fprintf('This may take 5-15 minutes...\n\n');

tic;
try
    % Run the code generation script at project root
    generate_code_arm64;
    genTime = toc;
    fprintf('\n  ✓ Code generation completed in %.1f seconds\n\n', genTime);
catch ME
    error('Code generation failed: %s\nCheck codegen/arm64_realtime/html/report.mldatx', ME.message);
end

%% Step 3: Verify generated files
fprintf('Step 3: Verifying generated files...\n');

requiredFiles = {
    fullfile(codegenOutput, 'solveGIKStepWrapper.h')
    fullfile(codegenOutput, 'solveGIKStepWrapper.cpp')
};

allPresent = true;
for i = 1:length(requiredFiles)
    if isfile(requiredFiles{i})
        fprintf('  ✓ %s\n', requiredFiles{i});
    else
        fprintf('  ✗ Missing: %s\n', requiredFiles{i});
        allPresent = false;
    end
end

if ~allPresent
    error('Some required files were not generated. Check codegen report.');
end

% Count total generated files
allFiles = dir(fullfile(codegenOutput, '**', '*.*'));
allFiles = allFiles(~[allFiles.isdir]);
fprintf('  ✓ Total files generated: %d\n\n', length(allFiles));

%% Step 4: Prepare files for ROS2
fprintf('Step 4: Preparing files for ROS2 deployment...\n');

% Create target directories
includeDir = fullfile(ros2Target, 'include');
libDir = fullfile(ros2Target, 'lib');

if ~exist(includeDir, 'dir'), mkdir(includeDir); end
if ~exist(libDir, 'dir'), mkdir(libDir); end

% Copy headers
headerFiles = dir(fullfile(codegenOutput, '*.h'));
for i = 1:length(headerFiles)
    src = fullfile(headerFiles(i).folder, headerFiles(i).name);
    dst = fullfile(includeDir, headerFiles(i).name);
    copyfile(src, dst);
end
fprintf('  ✓ Copied %d header files\n', length(headerFiles));

% Copy C++ files (for reference, may not be needed if using .a library)
cppFiles = dir(fullfile(codegenOutput, '*.cpp'));
for i = 1:length(cppFiles)
    src = fullfile(cppFiles(i).folder, cppFiles(i).name);
    dst = fullfile(includeDir, cppFiles(i).name);
    copyfile(src, dst);
end
fprintf('  ✓ Copied %d C++ source files\n', length(cppFiles));

% Copy static library if exists
libFiles = dir(fullfile(codegenOutput, '*.a'));
if ~isempty(libFiles)
    for i = 1:length(libFiles)
        src = fullfile(libFiles(i).folder, libFiles(i).name);
        dst = fullfile(libDir, libFiles(i).name);
        copyfile(src, dst);
    end
    fprintf('  ✓ Copied %d library files\n', length(libFiles));
else
    fprintf('  ⚠ No .a library files found (may be generated separately)\n');
end

fprintf('\n');

%% Step 5: Create deployment package
fprintf('Step 5: Creating deployment package...\n');

deployDir = fullfile(projectRoot, 'deployment_package');
if ~exist(deployDir, 'dir'), mkdir(deployDir); end

% Zip the matlab_codegen directory
zipFile = fullfile(deployDir, sprintf('gik_codegen_%s.zip', datestr(now, 'yyyymmdd_HHMMSS')));
zip(zipFile, ros2Target);

fprintf('  ✓ Created deployment package: %s\n', zipFile);
fprintf('  Size: %.2f MB\n\n', dir(zipFile).bytes / 1024 / 1024);

%% Summary
fprintf('========================================\n');
fprintf('✓ CODE GENERATION COMPLETE\n');
fprintf('========================================\n\n');

fprintf('Generated files location:\n');
fprintf('  - Headers: %s\n', includeDir);
fprintf('  - Libraries: %s\n', libDir);
fprintf('  - Deployment package: %s\n\n', zipFile);

fprintf('Next steps:\n');
fprintf('1. Transfer deployment package to AGX Orin:\n');
fprintf('   scp %s nvidia@<orin-ip>:~/\n\n', zipFile);

fprintf('2. On AGX Orin, extract and build:\n');
fprintf('   cd ~/gikWBC9DOF/ros2\n');
fprintf('   unzip ~/gik_codegen_*.zip\n');
fprintf('   colcon build --packages-select gik9dof_msgs gik9dof_solver\n\n');

fprintf('3. Test the solver:\n');
fprintf('   source install/setup.bash\n');
fprintf('   ros2 launch gik9dof_solver test_solver.launch.py\n\n');

fprintf('See FAST_TRACK_2DAY.md for detailed instructions.\n');
fprintf('========================================\n');
