%% Pre-Test Dependency Check
% Quick verification that all required functions and files exist
% Run this before test_method4_phase1_improvements.m

clc;
fprintf('====================================================\n');
fprintf('  Phase 1 Pre-Test Dependency Check\n');
fprintf('====================================================\n\n');

allOk = true;

%% 1. Check MATLAB functions
fprintf('Checking MATLAB dependencies...\n');

try
    robot = gik9dof.createRobotModel();
    fprintf('  ✓ gik9dof.createRobotModel()\n');
catch ME
    fprintf('  ✗ gik9dof.createRobotModel() FAILED: %s\n', ME.message);
    allOk = false;
    robot = [];
end

try
    chassisParams = gik9dof.control.loadChassisProfile('wide_track');
    fprintf('  ✓ gik9dof.control.loadChassisProfile()\n');
catch ME
    fprintf('  ✗ gik9dof.control.loadChassisProfile() FAILED: %s\n', ME.message);
    allOk = false;
end

%% 2. Check trajectory file
fprintf('\nChecking trajectory file...\n');

trajFile = '1_pull_world_scaled.json';
if isfile(trajFile)
    fprintf('  ✓ %s exists\n', trajFile);
    
    % Try to load it
    try
        fid = fopen(trajFile, 'r');
        raw = fread(fid, inf);
        str = char(raw');
        fclose(fid);
        trajData = jsondecode(str);
        
        if isfield(trajData, 'poses')
            nPoses = numel(trajData.poses);
            fprintf('  ✓ Trajectory parsed: %d poses\n', nPoses);
        else
            fprintf('  ✗ Trajectory missing "poses" field\n');
            allOk = false;
        end
    catch ME
        fprintf('  ✗ Failed to parse trajectory: %s\n', ME.message);
        allOk = false;
    end
else
    fprintf('  ✗ %s NOT FOUND\n', trajFile);
    allOk = false;
end

%% 3. Check new functions
fprintf('\nChecking Phase 1 implementation files...\n');

if exist('gik9dof.runStageCPPFirst_enhanced', 'file') == 2
    fprintf('  ✓ runStageCPPFirst_enhanced.m\n');
else
    fprintf('  ✗ runStageCPPFirst_enhanced.m NOT FOUND\n');
    allOk = false;
end

%% 4. Check other dependencies
fprintf('\nChecking other helper functions...\n');

helpers = {
    'gik9dof.baseSeedFromEE',
    'gik9dof.initPPFromBasePath',
    'gik9dof.createGikSolver',
    'gik9dof.solveArmOnlyIK',
    'gik9dof.updateBaseJointBounds'
};

for i = 1:length(helpers)
    if exist(helpers{i}, 'file') == 2
        fprintf('  ✓ %s\n', helpers{i});
    else
        fprintf('  ✗ %s NOT FOUND\n', helpers{i});
        allOk = false;
    end
end

%% 6. Check robot model if available
if ~isempty(robot)
    fprintf('\nChecking robot model...\n');
    try
        nJoints = numel(robot.homeConfiguration);
        fprintf('  ✓ Robot has %d joints\n', nJoints);
        
        if nJoints == 9
            fprintf('  ✓ Correct DOF (9 = 3 base + 6 arm)\n');
        else
            fprintf('  ⚠ Expected 9 DOF, got %d\n', nJoints);
        end
        
        % Check for expected end-effector
        if ~isempty(robot.getBody('left_gripper_link'))
            fprintf('  ✓ End-effector "left_gripper_link" found\n');
        else
            fprintf('  ⚠ End-effector "left_gripper_link" not found\n');
        end
    catch ME
        fprintf('  ⚠ Could not verify robot model: %s\n', ME.message);
    end
end

%% Summary
fprintf('\n====================================================\n');
if allOk
    fprintf('  ✅ ALL CHECKS PASSED\n');
    fprintf('  Ready to run: test_method4_phase1_improvements\n');
    fprintf('====================================================\n\n');
    fprintf('Next step:\n');
    fprintf('  >> test_method4_phase1_improvements\n\n');
else
    fprintf('  ❌ SOME CHECKS FAILED\n');
    fprintf('  Please fix issues above before testing\n');
    fprintf('====================================================\n');
end
