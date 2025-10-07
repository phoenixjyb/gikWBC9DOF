% Test x64 codegen only (simpler than ARM64)
% This isolates whether the issue is ARM64-specific or broader

clear; clc;

% Navigate to correct directory
originalDir = pwd;
cd('c:\Users\yanbo\wSpace\codegenGIKsample\Trial\gikWBC9DOF\matlab');

fprintf('=== Testing x64 Codegen Only ===\n\n');

% Define all input types - ALL SCALARS (single waypoint at a time)
refX_type = coder.typeof(0.0);
refY_type = coder.typeof(0.0);
refTheta_type = coder.typeof(0.0);
refTime_type = coder.typeof(0.0);
estX_type = coder.typeof(0.0);
estY_type = coder.typeof(0.0);
estYaw_type = coder.typeof(0.0);

% Parameters struct - ALL 13 parameters
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

% State struct - Fixed-size arrays for 30 waypoints
state_type = struct();
state_type.pathX = coder.typeof(zeros(1, 30));
state_type.pathY = coder.typeof(zeros(1, 30));
state_type.pathTheta = coder.typeof(zeros(1, 30));
state_type.pathTime = coder.typeof(zeros(1, 30));
state_type.numWaypoints = coder.typeof(uint32(0));  % MUST be uint32
state_type.prevVx = coder.typeof(0.0);
state_type.prevWz = coder.typeof(0.0);
state_type.prevPoseX = coder.typeof(0.0);
state_type.prevPoseY = coder.typeof(0.0);
state_type.prevPoseYaw = coder.typeof(0.0);
state_type.lastRefTime = coder.typeof(0.0);

% Configure for x64 (simpler than ARM64)
cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof_purepursuit';
cfg.GenCodeOnly = true;
cfg.BuildConfiguration = 'Faster Runs';
cfg.EnableVariableSizing = false;  % No variable-sized arrays
cfg.DynamicMemoryAllocation = 'Off';

fprintf('x64 Config:\n');
fprintf('  Target: %s\n', cfg.TargetLang);
fprintf('  Namespace: %s\n', cfg.CppNamespace);
fprintf('\nStarting x64 codegen...\n');

try
    codegen -config cfg purePursuitVelocityController -args {refX_type, refY_type, refTheta_type, refTime_type, estX_type, estY_type, estYaw_type, params_type, state_type} -d codegen/purepursuit_x64_test -report
    fprintf('✓ x64 code generation SUCCESS!\n');
catch ME
    fprintf('✗ x64 code generation FAILED:\n');
    fprintf('  Error: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('  Location: %s (line %d)\n', ME.stack(1).file, ME.stack(1).line);
    end
end

cd(originalDir);
fprintf('\n=== Test Complete ===\n');
