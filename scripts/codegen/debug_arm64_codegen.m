%% Debug ARM64 Codegen Issue
% Simplified ARM64 codegen to identify the problem

fprintf('Pure Pursuit ARM64 Debug - Step by Step\n');
fprintf('=========================================\n\n');

cd('matlab');
addpath(genpath('.'));

fprintf('Step 1: Define input types...\n');

% Scalar inputs
refX_type = coder.typeof(0.0);
refY_type = coder.typeof(0.0);
refTheta_type = coder.typeof(0.0);
refTime_type = coder.typeof(0.0);
estX_type = coder.typeof(0.0);
estY_type = coder.typeof(0.0);
estYaw_type = coder.typeof(0.0);

% Parameters struct
params_type = struct();
params_type.lookaheadBase = coder.typeof(0.0);
params_type.lookaheadVelGain = coder.typeof(0.0);
params_type.lookaheadTimeGain = coder.typeof(0.0);
params_type.vxNominal = coder.typeof(0.0);
params_type.vxMax = coder.typeof(0.0);
params_type.vxMin = coder.typeof(0.0);
params_type.wzMax = coder.typeof(0.0);
params_type.track = coder.typeof(0.0);
params_type.vwheelMax = coder.typeof(0.0);
params_type.waypointSpacing = coder.typeof(0.0);
params_type.pathBufferSize = coder.typeof(0.0);
params_type.goalTolerance = coder.typeof(0.0);
params_type.interpSpacing = coder.typeof(0.0);

% State struct
state_type = struct();
state_type.pathX = coder.typeof(zeros(1, 30));
state_type.pathY = coder.typeof(zeros(1, 30));
state_type.pathTheta = coder.typeof(zeros(1, 30));
state_type.pathTime = coder.typeof(zeros(1, 30));
state_type.numWaypoints = coder.typeof(0);
state_type.prevVx = coder.typeof(0.0);
state_type.prevWz = coder.typeof(0.0);
state_type.prevPoseX = coder.typeof(0.0);
state_type.prevPoseY = coder.typeof(0.0);
state_type.prevPoseYaw = coder.typeof(0.0);
state_type.lastRefTime = coder.typeof(0.0);

fprintf('✓ Types defined\n\n');

%% Try 1: Most basic ARM64 config
fprintf('Try 1: Basic ARM64 configuration (no optimizations)...\n');

cfg1 = coder.config('lib');
cfg1.TargetLang = 'C++';
cfg1.CppNamespace = 'gik9dof_purepursuit';
cfg1.GenCodeOnly = true;
cfg1.GenerateReport = true;

try
    codegen -config cfg1 purePursuitVelocityController ...
        -args {refX_type, refY_type, refTheta_type, refTime_type, ...
               estX_type, estY_type, estYaw_type, ...
               coder.typeof(params_type), coder.typeof(state_type)} ...
        -d codegen/debug_basic -report
    
    fprintf('✓ SUCCESS with basic config!\n\n');
    success1 = true;
catch ME
    fprintf('✗ FAILED with basic config:\n');
    fprintf('  Error: %s\n', ME.message);
    if ~isempty(ME.stack)
        for i = 1:min(3, length(ME.stack))
            fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
        end
    end
    fprintf('\n');
    success1 = false;
end

if ~success1
    fprintf('Basic config failed - stopping here.\n');
    fprintf('This suggests a fundamental issue with the function.\n');
    return;
end

%% Try 2: Add ARM-specific hardware
fprintf('Try 2: Adding ARM Cortex-A hardware target...\n');

cfg2 = coder.config('lib');
cfg2.TargetLang = 'C++';
cfg2.CppNamespace = 'gik9dof_purepursuit';
cfg2.GenCodeOnly = true;
cfg2.GenerateReport = true;
cfg2.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';

try
    codegen -config cfg2 purePursuitVelocityController ...
        -args {refX_type, refY_type, refTheta_type, refTime_type, ...
               estX_type, estY_type, estYaw_type, ...
               coder.typeof(params_type), coder.typeof(state_type)} ...
        -d codegen/debug_arm_hw -report
    
    fprintf('✓ SUCCESS with ARM hardware target!\n\n');
    success2 = true;
catch ME
    fprintf('✗ FAILED with ARM hardware:\n');
    fprintf('  Error: %s\n', ME.message);
    if ~isempty(ME.stack)
        for i = 1:min(3, length(ME.stack))
            fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
        end
    end
    fprintf('\n');
    success2 = false;
end

%% Try 3: Add toolchain
if success2
    fprintf('Try 3: Adding CMake toolchain...\n');
    
    cfg3 = coder.config('lib');
    cfg3.TargetLang = 'C++';
    cfg3.CppNamespace = 'gik9dof_purepursuit';
    cfg3.GenCodeOnly = true;
    cfg3.GenerateReport = true;
    cfg3.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
    cfg3.Toolchain = 'CMake';
    
    try
        codegen -config cfg3 purePursuitVelocityController ...
            -args {refX_type, refY_type, refTheta_type, refTime_type, ...
                   estX_type, estY_type, estYaw_type, ...
                   coder.typeof(params_type), coder.typeof(state_type)} ...
            -d codegen/debug_arm_cmake -report
        
        fprintf('✓ SUCCESS with CMake toolchain!\n\n');
        success3 = true;
    catch ME
        fprintf('✗ FAILED with CMake toolchain:\n');
        fprintf('  Error: %s\n', ME.message);
        if ~isempty(ME.stack)
            for i = 1:min(3, length(ME.stack))
                fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
            end
        end
        fprintf('\n');
        success3 = false;
    end
end

%% Try 4: Full ARM64 config (as in original script)
if success2 && success3
    fprintf('Try 4: Full ARM64 configuration (all optimizations)...\n');
    
    cfg4 = coder.config('lib');
    cfg4.TargetLang = 'C++';
    cfg4.CppNamespace = 'gik9dof_purepursuit';
    cfg4.GenCodeOnly = true;
    cfg4.GenerateReport = true;
    cfg4.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-A';
    cfg4.Toolchain = 'CMake';
    cfg4.BuildConfiguration = 'Faster Runs';
    cfg4.EnableVariableSizing = false;
    cfg4.DynamicMemoryAllocation = 'Off';
    cfg4.EnableOpenMP = false;
    
    try
        codegen -config cfg4 purePursuitVelocityController ...
            -args {refX_type, refY_type, refTheta_type, refTime_type, ...
                   estX_type, estY_type, estYaw_type, ...
                   coder.typeof(params_type), coder.typeof(state_type)} ...
            -d codegen/purepursuit_arm64 -report
        
        fprintf('✓ SUCCESS with full ARM64 config!\n\n');
        fprintf('=========================================\n');
        fprintf('ARM64 codegen working! 🎉\n');
        fprintf('Output: codegen/purepursuit_arm64/\n');
        
    catch ME
        fprintf('✗ FAILED with full config:\n');
        fprintf('  Error: %s\n', ME.message);
        if ~isempty(ME.stack)
            for i = 1:min(3, length(ME.stack))
                fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
            end
        end
        
        fprintf('\n=========================================\n');
        fprintf('Issue found in optimization settings.\n');
        fprintf('Successful config: cfg3 (ARM + CMake)\n');
        fprintf('Use that configuration instead.\n');
    end
end

fprintf('\n=========================================\n');
fprintf('Debug session complete\n');
fprintf('=========================================\n');
