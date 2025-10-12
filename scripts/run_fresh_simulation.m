%% Run Fresh Simulation and Generate Animation
% This script runs a new simulation with baseline parameters,
% then generates animation with all three fixes applied.

clear; clc; close all;
addpath(genpath('matlab'));

fprintf('=== Running Fresh Simulation with Animation ===\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% Configuration
fprintf('--- Configuration ---\n');
config = struct();
config.name = 'Baseline_Fresh';
config.SafetyMargin = 0.10;
config.LambdaCusp = 1.0;
config.MaxRSIterations = 200;
config.AllowReverse = true;
config.ClothoidDiscretization = 0.08;
config.Lookahead = 0.80;
config.AccelLimit = 0.80;
config.HeadingKp = 1.0;

fprintf('  Safety Margin: %.2f\n', config.SafetyMargin);
fprintf('  Lambda Cusp: %.1f\n', config.LambdaCusp);
fprintf('  Max RS Iterations: %d\n', config.MaxRSIterations);
fprintf('  Allow Reverse: %d\n', config.AllowReverse);
fprintf('  Clothoid Discretization: %.2f\n', config.ClothoidDiscretization);
fprintf('  Lookahead: %.2f\n', config.Lookahead);
fprintf('  Accel Limit: %.2f\n', config.AccelLimit);
fprintf('  Heading Kp: %.1f\n', config.HeadingKp);

%% Load trajectory and environment
fprintf('\n--- Loading Reference Trajectory ---\n');
jsonFile = 'refEETrajs/1_pull_world_scaled.json';
if ~isfile(jsonFile)
    error('JSON file not found: %s', jsonFile);
end
refData = jsondecode(fileread(jsonFile));
fprintf('  File: %s\n', jsonFile);
fprintf('  Waypoints: %d\n', length(refData.trajectory));

%% Create robot model
fprintf('\n--- Creating Robot Model ---\n');
robot = gik9dof.createRobotModel();
fprintf('  Robot created: %d bodies\n', numel(robot.Bodies));

%% Load floor obstacles
fprintf('\n--- Loading Floor Obstacles ---\n');
floorFile = 'config/floor_discs.mat';
if isfile(floorFile)
    floorData = load(floorFile);
    floorDiscs = floorData.discs;
    fprintf('  Floor obstacles: %d discs\n', numel(floorDiscs));
else
    fprintf('  No floor obstacles file found, using default\n');
    floorDiscs = [];
end

%% Run simulation
fprintf('\n--- Running Simulation ---\n');
fprintf('Please wait...\n\n');

tic;
try
    % Run staged trajectory control
    log = gik9dof.runTrajectoryControl(refData, ...
        'Robot', robot, ...
        'FloorDiscs', floorDiscs, ...
        'SafetyMargin', config.SafetyMargin, ...
        'LambdaCusp', config.LambdaCusp, ...
        'MaxRSIterations', config.MaxRSIterations, ...
        'AllowReverse', config.AllowReverse, ...
        'ClothoidDiscretization', config.ClothoidDiscretization, ...
        'Lookahead', config.Lookahead, ...
        'AccelLimit', config.AccelLimit, ...
        'HeadingKp', config.HeadingKp);
    
    simTime = toc;
    fprintf('✓ Simulation complete: %.1f seconds\n', simTime);
    
    % Check simulation results
    if isfield(log, 'qTraj')
        fprintf('  Total frames: %d\n', size(log.qTraj, 2));
    end
    
    if isfield(log, 'stageLogs')
        if isfield(log.stageLogs, 'stageA')
            fprintf('  Stage A frames: %d\n', size(log.stageLogs.stageA.qTraj, 2));
        end
        if isfield(log.stageLogs, 'stageB')
            fprintf('  Stage B frames: %d\n', size(log.stageLogs.stageB.qTraj, 2));
        end
        if isfield(log.stageLogs, 'stageC')
            fprintf('  Stage C frames: %d\n', size(log.stageLogs.stageC.qTraj, 2));
            
            % Check tracking error
            if isfield(log.stageLogs.stageC, 'positionError')
                errors = vecnorm(log.stageLogs.stageC.positionError, 2, 1);
                fprintf('  Stage C EE Error: mean=%.4f m, max=%.4f m\n', ...
                    mean(errors), max(errors));
            end
        end
    end
    
catch ME
    simTime = toc;
    fprintf('❌ Simulation failed after %.1f seconds:\n', simTime);
    fprintf('   %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('   at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
    return;
end

%% Save log
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
logDir = sprintf('results/%s_fresh_sim', timestamp);
if ~exist(logDir, 'dir')
    mkdir(logDir);
end
logFile = fullfile(logDir, 'simulation_log.mat');
save(logFile, 'log', 'config', '-v7.3');
fprintf('\n✓ Log saved: %s\n', logFile);

%% Generate animation
fprintf('\n--- Generating Animation ---\n');
videoFile = fullfile(logDir, sprintf('%s_animation.mp4', config.name));

fprintf('  SampleStep: 2 (50%% frame reduction)\n');
fprintf('  FrameRate: 20 fps\n');
fprintf('  Output: %s\n', videoFile);

% Calculate expected stage structure
stageA_raw = size(log.stageLogs.stageA.qTraj, 2);
stageB_raw = size(log.stageLogs.stageB.qTraj, 2);
stageC_raw = size(log.stageLogs.stageC.qTraj, 2);
stageA_frames = ceil(stageA_raw / 2);
stageB_frames = ceil(stageB_raw / 2);
stageC_frames = ceil(stageC_raw / 2);

fprintf('\n  Expected stage structure:\n');
fprintf('    Stage A: frames 1-%d (%.1f sec @ 20fps)\n', ...
    stageA_frames, stageA_frames/20);
fprintf('    Stage B: frames %d-%d (%.1f sec @ 20fps)\n', ...
    stageA_frames+1, stageA_frames+stageB_frames, stageB_frames/20);
fprintf('    Stage C: frames %d-%d (%.1f sec @ 20fps)\n', ...
    stageA_frames+stageB_frames+1, stageA_frames+stageB_frames+stageC_frames, stageC_frames/20);
fprintf('    Total: %d frames (%.1f sec)\n', ...
    stageA_frames+stageB_frames+stageC_frames, (stageA_frames+stageB_frames+stageC_frames)/20);

fprintf('\nGenerating... (please wait ~2 minutes)\n');

tic;
try
    gik9dof.animateStagedWithHelper(log, ...
        'SampleStep', 2, ...
        'FrameRate', 20, ...
        'ExportVideo', videoFile);
    
    animTime = toc;
    fprintf('\n✅ Animation Generated Successfully!\n');
    fprintf('   Time elapsed: %.1f seconds (%.1f minutes)\n', animTime, animTime/60);
    
    % Check file
    fileInfo = dir(videoFile);
    if ~isempty(fileInfo) && fileInfo.bytes > 0
        fprintf('   File: %s\n', videoFile);
        fprintf('   Size: %.2f MB\n', fileInfo.bytes / 1024 / 1024);
        
        fprintf('\n=== VERIFICATION CHECKLIST ===\n');
        fprintf('Please review the animation:\n\n');
        fprintf('Stage Labels:\n');
        fprintf('  [ ] Stage A, B, C labels appear at correct times\n');
        fprintf('  [ ] Transitions at frames %d and %d\n', stageA_frames, stageA_frames+stageB_frames);
        
        fprintf('\nRed Dot (Stage C Reference EE):\n');
        fprintf('  [ ] Red dot hidden during Stages A & B\n');
        fprintf('  [ ] Red dot appears at frame %d (Stage C start)\n', stageA_frames+stageB_frames+1);
        fprintf('  [ ] Red dot shows Stage C desired trajectory (NOT from homebase)\n');
        fprintf('  [ ] Red dot moves synchronously with robot\n');
        fprintf('  [ ] Red dot is above chassis (Z ≈ 0.86 m)\n');
        
        fprintf('\nOverall:\n');
        fprintf('  [ ] Stage transitions smooth\n');
        fprintf('  [ ] Green square tracks red dot in Stage C\n');
        fprintf('  [ ] Total duration: %.1f seconds\n', (stageA_frames+stageB_frames+stageC_frames)/20);
        
        fprintf('\n=== All Three Fixes Applied ===\n');
        fprintf('✓ Fix 1: Stage boundaries correctly adjusted for sampling\n');
        fprintf('✓ Fix 2: Red dot uses stage-relative indexing\n');
        fprintf('✓ Fix 3: Red dot shows Stage C desired trajectory\n');
        
    else
        fprintf('\n⚠ WARNING: Video file is empty or not found!\n');
    end
    
catch ME
    animTime = toc;
    fprintf('\n❌ Animation generation failed after %.1f seconds:\n', animTime);
    fprintf('   %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('   at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
end

fprintf('\n=== Summary ===\n');
fprintf('Simulation time: %.1f seconds\n', simTime);
if exist('animTime', 'var')
    fprintf('Animation time: %.1f seconds\n', animTime);
    fprintf('Total time: %.1f seconds (%.1f minutes)\n', simTime+animTime, (simTime+animTime)/60);
end
fprintf('Output directory: %s\n', logDir);

fprintf('\n=== Complete ===\n');
