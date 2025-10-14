%% Debug script to verify Phase 2A parameters are being passed correctly
% This script traces the parameter passing through the call chain

clear; clc;

% Load the reference trajectory and configuration
refTraj = gik9dof.loadReferenceTrajectory('1_pull_world_scaled.json');
robot = gik9dof.internal.getRobotInstance();

% Create minimal options for testing
testOptions = struct();
testOptions.ExecutionMode = "ppFirst";
testOptions.RateHz = 10;
testOptions.Verbose = false;

% Directly call executeStageCPPFirst with test parameters
% We'll inject a modified version that prints diagnostics

fprintf('Testing parameter passing to runStageCPPFirst_enhanced...\n\n');

% Create a test call that mimics what runStagedTrajectory does
qStart = homeConfiguration(robot);
trajStruct = refTraj;
baseIdx = [1 2 3];
armIdx = [4 5 6 7 8 9];
velLimits = struct('linear', 0.2, 'angular', 1.0);

% Load chassis config
configFile = fullfile('config', 'chassis_profiles.yaml');
chassisCfg = yaml.loadFile(configFile, "ConvertToArray", true);
chassisParams = chassisCfg.profiles{1};

% Call runStageCPPFirst_enhanced directly with Phase 2A parameters
fprintf('Calling runStageCPPFirst_enhanced with UseOrientationZNominal=true\n');
fprintf('Expected: Orientation+Z priority nominal pose generation\n\n');

rawLog = gik9dof.runStageCPPFirst_enhanced(robot, trajStruct, qStart, ...
    'ChassisParams', chassisParams, ...
    'MaxIterations', 1000, ...
    'SampleTime', 0.1, ...
    'BaseIndices', baseIdx, ...
    'ArmIndices', armIdx, ...
    'EndEffector', 'end_effector', ...
    'VerboseLevel', 2, ...  % Enable verbose output
    'UseOrientationZNominal', true, ...  % KEY PARAMETER!
    'OrientationWeight', 1.0, ...
    'PositionWeightXY', 0.1, ...
    'PositionWeightZ', 1.0);

fprintf('\n=== RESULTS ===\n');
fprintf('Mean EE Error: %.1f mm\n', mean(rawLog.positionErrorNorm) * 1000);
fprintf('Max EE Error: %.1f mm\n', max(rawLog.positionErrorNorm) * 1000);
fprintf('Fallback Rate: %.1f%%\n', (1 - mean(rawLog.successMask)) * 100);

% Expected: ~1.2mm mean error if Phase 2A is working
% If getting ~500-1000mm, Phase 2A is NOT being applied

if mean(rawLog.positionErrorNorm) * 1000 < 5
    fprintf('\n✅ SUCCESS: Phase 2A is working correctly!\n');
elseif mean(rawLog.positionErrorNorm) * 1000 < 50
    fprintf('\n⚠️  PARTIAL: Some improvement but not full Phase 2A performance\n');
else
    fprintf('\n❌ FAILURE: Phase 2A not being applied!\n');
    fprintf('   This suggests UseOrientationZNominal is not reaching baseSeedFromEE\n');
end
