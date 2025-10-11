%% Debug Stage C EE Reference Path Source
clear; clc;
addpath(genpath('matlab'));

fprintf('=== Investigating Stage C EE Reference Path ===\n\n');

load('results/parametric_study_20251011_085252/parametric_study_results.mat');
log = results(1).log;

fprintf('Configuration: %s\n\n', results(1).config.name);

% Check what data exists
fprintf('--- Available Stage C Data ---\n');

if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageC')
    stageC = log.stageLogs.stageC;
    
    % Check referenceInitialIk
    if isfield(stageC, 'referenceInitialIk')
        fprintf('✓ stageC.referenceInitialIk EXISTS\n');
        if isfield(stageC.referenceInitialIk, 'eePositions')
            eeRef = stageC.referenceInitialIk.eePositions;
            fprintf('  ✓ eePositions: %d × %d\n', size(eeRef, 1), size(eeRef, 2));
            fprintf('  First point: [%.4f, %.4f, %.4f]\n', eeRef(1,1), eeRef(2,1), eeRef(3,1));
            fprintf('  Last point:  [%.4f, %.4f, %.4f]\n', eeRef(1,end), eeRef(2,end), eeRef(3,end));
        else
            fprintf('  ❌ eePositions field NOT found\n');
        end
    else
        fprintf('❌ stageC.referenceInitialIk NOT found\n');
    end
    
    % Check eePositions directly
    if isfield(stageC, 'eePositions')
        eePos = stageC.eePositions;
        fprintf('✓ stageC.eePositions EXISTS: %d × %d\n', size(eePos, 1), size(eePos, 2));
        fprintf('  First point: [%.4f, %.4f, %.4f]\n', eePos(1,1), eePos(2,1), eePos(3,1));
        fprintf('  Last point:  [%.4f, %.4f, %.4f]\n', eePos(1,end), eePos(2,end), eePos(3,end));
    end
    
    % Check targetPositions (desired trajectory from JSON)
    if isfield(stageC, 'targetPositions')
        target = stageC.targetPositions;
        fprintf('✓ stageC.targetPositions EXISTS: %d × %d\n', size(target, 1), size(target, 2));
        fprintf('  First point: [%.4f, %.4f, %.4f]\n', target(1,1), target(2,1), target(3,1));
        fprintf('  Last point:  [%.4f, %.4f, %.4f]\n', target(1,end), target(2,end), target(3,end));
    end
end

% Check full trajectory reference
fprintf('\n--- Full Trajectory Reference ---\n');
if isfield(log, 'referenceTrajectory')
    refTraj = log.referenceTrajectory;
    if isfield(refTraj, 'EndEffectorPositions')
        eeDesired = refTraj.EndEffectorPositions;
        fprintf('✓ referenceTrajectory.EndEffectorPositions: %d × %d\n', size(eeDesired, 1), size(eeDesired, 2));
        fprintf('  First point: [%.4f, %.4f, %.4f]\n', eeDesired(1,1), eeDesired(2,1), eeDesired(3,1));
        fprintf('  Last point:  [%.4f, %.4f, %.4f]\n', eeDesired(1,end), eeDesired(2,end), eeDesired(3,end));
    end
end

% Check what computeEEPath would produce
fprintf('\n--- What computeEEPath Produces ---\n');
robot = gik9dof.createRobotModel();
qTraj = log.qTraj;

% Full trajectory EE path
eePathFull = computeEEPath(robot, qTraj);
fprintf('computeEEPath(full qTraj): %d × %d\n', size(eePathFull, 1), size(eePathFull, 2));
fprintf('  First point: [%.4f, %.4f, %.4f]\n', eePathFull(1,1), eePathFull(1,2), eePathFull(1,3));
fprintf('  Last point:  [%.4f, %.4f, %.4f]\n', eePathFull(end,1), eePathFull(end,2), eePathFull(end,3));

% Stage C only
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageC')
    qTrajC = log.stageLogs.stageC.qTraj;
    eePathC = computeEEPath(robot, qTrajC);
    fprintf('\ncomputeEEPath(stageC.qTraj only): %d × %d\n', size(eePathC, 1), size(eePathC, 2));
    fprintf('  First point: [%.4f, %.4f, %.4f]\n', eePathC(1,1), eePathC(1,2), eePathC(1,3));
    fprintf('  Last point:  [%.4f, %.4f, %.4f]\n', eePathC(end,1), eePathC(end,2), eePathC(end,3));
end

fprintf('\n=== THE PROBLEM ===\n');
fprintf('Line 51-53 in animateStagedWithHelper.m:\n');
fprintf('  if isempty(eePathStageCRef)\n');
fprintf('      eePathStageCRef = computeEEPath(robot, qTraj(:, 1:opts.SampleStep:end))\n');
fprintf('\nThis computes EE path from FULL qTraj (Stages A+B+C),\n');
fprintf('so red dot starts at homebase and follows entire robot motion!\n');
fprintf('\nIt should use Stage C qTraj only:\n');
fprintf('  eePathStageCRef = computeEEPath(robot, stageC.qTraj(:, 1:opts.SampleStep:end))\n');

fprintf('\n=== CORRECT DATA SOURCE ===\n');
if isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageC')
    stageC = log.stageLogs.stageC;
    if isfield(stageC, 'targetPositions')
        fprintf('✓ Use stageC.targetPositions (desired EE trajectory from JSON)\n');
        fprintf('  This is what Stage C should track!\n');
    elseif isfield(stageC, 'eePositions')
        fprintf('⚠ Use stageC.eePositions (actual Stage C EE positions)\n');
        fprintf('  This shows what was achieved, not the reference\n');
    end
end

function eePath = computeEEPath(robot, qTraj)
    % Compute end-effector path from joint trajectory
    eeName = 'end_effector';
    nFrames = size(qTraj, 2);
    eePath = zeros(nFrames, 3);
    
    for i = 1:nFrames
        q = qTraj(:, i);
        config = struct();
        for j = 1:numel(robot.Bodies)
            jname = robot.Bodies{j}.Joint.Name;
            if ~strcmp(jname, robot.BaseName)
                idx = find(strcmp({robot.Bodies.Name}, robot.Bodies{j}.Name));
                if idx <= numel(q)
                    config.(jname) = q(idx);
                end
            end
        end
        try
            T = getTransform(robot, config, eeName, robot.BaseName);
            eePath(i, :) = T(1:3, 4)';
        catch
            eePath(i, :) = [NaN NaN NaN];
        end
    end
end
