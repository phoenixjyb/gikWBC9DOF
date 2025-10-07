function pipeline = run_stageb_purehyb_smoke()
%RUN_STAGEB_PUREHYB_SMOKE Quick staged-run smoke test for pure-hybrid mode.
%   pipeline = run_stageb_purehyb_smoke() builds a reduced reference
%   trajectory (first five waypoints), executes gik9dof.runStagedTrajectory
%   with Stage B set to pure-hybrid mode, and returns the resulting pipeline
%   struct. The function saves the log to results/pureHyb_stage_smoke.mat
%   for inspection.

robot = gik9dof.createRobotModel("Validate", true);
configTools = gik9dof.configurationTools(robot);
env = gik9dof.environmentConfig();

homeStruct = configTools.struct(configTools.homeConfig());
if isfield(env, "BaseHome") && numel(env.BaseHome) == 3
    baseMap = struct('joint_x', env.BaseHome(1), ...
                     'joint_y', env.BaseHome(2), ...
                     'joint_theta', env.BaseHome(3));
    for idx = 1:numel(homeStruct)
        jointName = homeStruct(idx).JointName;
        if isfield(baseMap, jointName)
            homeStruct(idx).JointPosition = baseMap.(jointName);
        end
    end
end
q0 = configTools.column(homeStruct);

trajStruct = createTrimmedTrajectory(5);

rateHz = 25;

pipeline = gik9dof.runStagedTrajectory(robot, trajStruct, ...
    'InitialConfiguration', q0, ...
    'ConfigTools', configTools, ...
    'DistanceSpecs', struct([]), ...
    'DistanceWeight', env.DistanceWeight, ...
    'RateHz', rateHz, ...
    'Verbose', false, ...
    'FloorDiscs', env.FloorDiscs, ...
    'EnvironmentConfig', env, ...
    'UseStageBHybridAStar', true, ...
    'StageBMode', 'pureHyb', ...
    'StageBHybridResolution', 0.1, ...
    'StageBHybridSafetyMargin', 0.15, ...
    'StageBHybridMinTurningRadius', 0.5, ...
    'StageBHybridMotionPrimitiveLength', 0.5, ...
    'StageBLookaheadDistance', 0.6, ...
    'StageBDesiredLinearVelocity', 0.5, ...
    'StageBMaxAngularVelocity', 2.0);

resultsDir = fullfile('results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

save(fullfile(resultsDir, 'pureHyb_stage_smoke.mat'), 'pipeline');

end

function traj = createTrimmedTrajectory(numWaypoints)
jsonPath = gik9dof.internal.resolvePath("1_pull_world_scaled.json");
raw = jsondecode(fileread(jsonPath));

numWaypoints = min(numWaypoints, numel(raw.poses));
poses = repmat(eye(4), 1, 1, numWaypoints);
posXYZ = zeros(3, numWaypoints);

for k = 1:numWaypoints
    entry = raw.poses(k);
    position = reshape(entry.position, [], 1);
    quatXYZW = reshape(entry.orientation, 1, []);
    quatWXYZ = [quatXYZW(4), quatXYZW(1:3)];
    quatWXYZ = quatWXYZ ./ norm(quatWXYZ);

    T = quat2tform(quatWXYZ);
    T(1:3,4) = position;

    poses(:,:,k) = T;
    posXYZ(:,k) = position;
end

traj = struct();
traj.Poses = poses;
traj.EndEffectorPositions = posXYZ;
traj.EndEffectorName = "left_gripper_link";
end
