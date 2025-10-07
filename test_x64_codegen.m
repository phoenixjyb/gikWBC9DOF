%% Generate x64 Only - For Testing
% Quick test to verify bidirectional logic generates correctly

fprintf('Pure Pursuit Code Generation - x64 ONLY (Test)\n');
fprintf('===============================================\n\n');

cd('matlab');
addpath(genpath('.'));

fprintf('Working directory: %s\n\n', pwd);

% Define input types
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
params_type.vxMin = coder.typeof(0.0);  % BIDIRECTIONAL
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

fprintf('Input types defined\n\n');

%% x86_64 Code Generation
fprintf('Generating x86_64 code...\n');

cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'gik9dof_purepursuit';
cfg.GenCodeOnly = true;
cfg.GenerateReport = true;
cfg.EnableVariableSizing = false;
cfg.DynamicMemoryAllocation = 'Off';

try
    codegen -config cfg purePursuitVelocityController ...
        -args {refX_type, refY_type, refTheta_type, refTime_type, ...
               estX_type, estY_type, estYaw_type, ...
               coder.typeof(params_type), coder.typeof(state_type)} ...
        -d codegen/purepursuit_x64_test -report
    
    fprintf('\n✓ SUCCESS! x64 code generated\n');
    fprintf('  Output: codegen/purepursuit_x64_test/\n\n');
    
    % Check for bidirectional code
    cppFile = fullfile('codegen', 'purepursuit_x64_test', 'purePursuitVelocityController.cpp');
    if exist(cppFile, 'file')
        content = fileread(cppFile);
        if contains(content, 'vx_direction') || contains(content, 'BIDIRECTIONAL')
            fprintf('✓ Bidirectional support found in generated C++!\n');
        else
            fprintf('⚠ WARNING: Bidirectional support NOT found in generated C++\n');
        end
    end
    
catch ME
    fprintf('\n✗ FAILED:\n');
    fprintf('  %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
    rethrow(ME);
end
