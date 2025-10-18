%% Minimal Method 6 test
clear; clc;
addpath(genpath('matlab'));

try
    fprintf('Loading environment...\n');
    env = gik9dof.environmentConfig();
    fprintf('✓ Environment loaded\n');
    
    fprintf('Creating robot...\n');
    robot = gik9dof.createRobotModel();
    fprintf('✓ Robot created\n');
    
    fprintf('Loading trajectory...\n');
    jsonFile = '1_pull_world_scaled.json';
    traj = gik9dof.loadJsonTrajectory(jsonFile);
    fprintf('✓ Trajectory loaded (%d waypoints)\n', size(traj.Poses, 3));
    
    fprintf('Getting initial config...\n');
    config = gik9dof.configTools.homeConfig();
    qStart = gik9dof.configTools.column(config);
    fprintf('✓ Initial config: [%.3f, %.3f, %.3f, ...]\n', qStart(1), qStart(2), qStart(3));
    
    fprintf('Calling trackReferenceTrajectory with Method 6...\n');
    log = gik9dof.trackReferenceTrajectory(...
        'Mode', 'staged', ...
        'ExecutionMode', 'alternating', ...
        'RateHz', 10, ...
        'Verbose', true);
    
    fprintf('✅ SUCCESS: Method 6 completed\n');
    fprintf('   Steps: %d, Time: %.1fs\n', size(log.qTraj, 2), log.time(end));
    
catch ME
    fprintf('❌ ERROR:\n');
    fprintf('   %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('   at %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
end
