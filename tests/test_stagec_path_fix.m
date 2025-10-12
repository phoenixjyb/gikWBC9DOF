%% Test Stage C EE Reference Path Fix
clear; clc; close all;
addpath(genpath('matlab'));

fprintf('=== Testing Stage C EE Reference Path Fix ===\n\n');

load('results/parametric_study_20251011_085252/parametric_study_results.mat');
log = results(1).log;

fprintf('Configuration: %s\n\n', results(1).config.name);

% Check what will be used
fprintf('--- Data Source Priority Check ---\n');
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageC')
    stageC = log.stageLogs.stageC;
    
    if isfield(stageC, 'referenceInitialIk') && ...
            isfield(stageC.referenceInitialIk, 'eePositions') && ...
            ~isempty(stageC.referenceInitialIk.eePositions)
        fprintf('✓ Will use: stageC.referenceInitialIk.eePositions\n');
        source = stageC.referenceInitialIk.eePositions;
    elseif isfield(stageC, 'targetPositions') && ~isempty(stageC.targetPositions)
        fprintf('✓ Will use: stageC.targetPositions (JSON desired trajectory)\n');
        source = stageC.targetPositions;
        fprintf('  This is the CORRECT source!\n');
    elseif isfield(stageC, 'eePositions') && ~isempty(stageC.eePositions)
        fprintf('⚠ Will use: stageC.eePositions (actual, not reference)\n');
        source = stageC.eePositions;
    else
        fprintf('❌ No Stage C data found, will fall back\n');
        source = [];
    end
    
    if ~isempty(source)
        fprintf('\nSelected data:\n');
        fprintf('  Size: %d × %d\n', size(source, 1), size(source, 2));
        fprintf('  First point: [%.4f, %.4f, %.4f]\n', source(1,1), source(2,1), source(3,1));
        fprintf('  Last point:  [%.4f, %.4f, %.4f]\n', source(1,end), source(2,end), source(3,end));
        
        % Compare with full trajectory reference
        if isfield(log, 'referenceTrajectory') && ...
                isfield(log.referenceTrajectory, 'EndEffectorPositions')
            refTraj = log.referenceTrajectory.EndEffectorPositions;
            fprintf('\nComparison with referenceTrajectory.EndEffectorPositions:\n');
            fprintf('  Match: %s\n', mat2str(isequal(source, refTraj)));
            if isequal(source, refTraj)
                fprintf('  ✓ Confirmed: Using Stage C desired trajectory!\n');
            end
        end
    end
end

% Check home position (just show coordinates for comparison)
fprintf('\n--- Home vs Stage C First Waypoint ---\n');
fprintf('Stage C first waypoint: [%.4f, %.4f, %.4f]\n', source(1,1), source(2,1), source(3,1));
fprintf('  This is where red dot should START (at frame 97)\n');
fprintf('  NOT at homebase!\n');

fprintf('\n--- Generating Test Animation ---\n');
videoFile = 'TEST_STAGEC_PATH_FIX.mp4';

try
    gik9dof.animateStagedWithHelper(log, ...
        'SampleStep', 2, ...
        'FrameRate', 20, ...
        'ExportVideo', videoFile);
    
    fprintf('\n✅ Animation generated: %s\n', videoFile);
    
    fileInfo = dir(videoFile);
    if ~isempty(fileInfo) && fileInfo.bytes > 0
        fprintf('File size: %.2f MB\n', fileInfo.bytes / 1024 / 1024);
        
        fprintf('\n=== VERIFICATION CHECKLIST ===\n');
        fprintf('Please verify in the animation:\n\n');
        fprintf('[ ] 1. Red dot appears at frame 97 (Stage C start)\n');
        fprintf('[ ] 2. Red dot starts at [%.2f, %.2f, %.2f]\n', source(1,1), source(2,1), source(3,1));
        fprintf('[ ] 3. Red dot does NOT start from homebase\n');
        fprintf('[ ] 4. Red dot tracks the JSON desired trajectory\n');
        fprintf('[ ] 5. Red dot is clearly different from Stage B chassis path\n');
        fprintf('[ ] 6. Green square (actual EE) tracks red dot closely\n');
        
        fprintf('\nExpected behavior:\n');
        fprintf('  - Red dot appears when Stage C label appears\n');
        fprintf('  - Red dot starts at first Stage C waypoint (%.2f, %.2f, %.2f)\n', ...
            source(1,1), source(2,1), source(3,1));
        fprintf('  - Red dot moves along Stage C desired trajectory\n');
        fprintf('  - Red dot is ABOVE the chassis (Z ≈ %.2f m)\n', source(3,1));
    else
        fprintf('\n⚠ WARNING: Video file is empty or not found!\n');
    end
    
catch ME
    fprintf('\n❌ ERROR during animation:\n');
    fprintf('   %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('   at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
end

fprintf('\n=== Test Complete ===\n');
