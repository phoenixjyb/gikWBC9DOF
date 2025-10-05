function out = unified_chassis_replay(logPath, mode, params)
%UNIFIED_CHASSIS_REPLAY Convert trajectory logs into unified base commands.
%   out = unified_chassis_replay(logPath, mode) loads the given MAT-file log
%   (or accepts a struct) and produces a table of unified base commands using
%   gik9dof.control.unifiedChassisCtrl. The returned struct contains:
%       .cmds   - table with columns time, Vx, Wz, YawLimitWheel, YawLimitCmd
%       .state  - final controller state
%       .params - parameters used
%
%   Example:
%       out = unified_chassis_replay('results/log_holistic_compare.mat', 'holistic');
%
%   For staged logs, set mode to 'staged-C' or 'staged-B'. When replaying a
%   staged log the helper automatically splits Stage-B and Stage-C segments.
%
%   See also gik9dof.control.unifiedChassisCtrl, gik9dof.control.defaultUnifiedParams.

arguments
    logPath
    mode (1,1) string {mustBeMember(mode, ["holistic","staged","staged-B","staged-C"])}
    params struct = gik9dof.control.defaultUnifiedParams()
end

logStruct = resolveLog(logPath);

if mode == "staged"
    error("unified_chassis_replay:AmbiguousMode", "Specify 'staged-B' or 'staged-C'.");
end

state = struct();
cmdList = struct([]);

switch mode
    case "holistic"
        samples = extractHolisticSamples(logStruct);
        for k = 1:numel(samples)
            ref = samples(k);
            estPose = [ref.x, ref.y, ref.theta];
            [cmd, state] = gik9dof.control.unifiedChassisCtrl("holistic", ref, estPose, state, params);
            cmdList = appendCmd(cmdList, cmd);
        end
    case "staged-C"
        samples = extractStageCSamples(logStruct);
        for k = 1:numel(samples)
            ref = samples(k);
            estPose = [ref.x, ref.y, ref.theta];
            [cmd, state] = gik9dof.control.unifiedChassisCtrl("staged-C", ref, estPose, state, params);
            cmdList = appendCmd(cmdList, cmd);
        end
    case "staged-B"
        samples = extractStageBSamples(logStruct);
        for k = 1:numel(samples)
            ref = samples(k);
            estPose = ref.pose;
            [cmd, state] = gik9dof.control.unifiedChassisCtrl("staged-B", ref, estPose, state, params);
            cmdList = appendCmd(cmdList, cmd);
        end
end

if isempty(cmdList)
    tbl = table();
else
    rows = arrayfun(@(c) struct('time', c.time, ...
        'Vx', c.base.Vx, 'Wz', c.base.Wz, ...
        'YawLimitWheel', c.base.YawLimitWheel, ...
        'YawLimitCmd', c.base.YawLimitCmd), cmdList);
    tbl = struct2table(rows);
end

outStruct = struct();
outStruct.cmds = tbl;
outStruct.state = state;
outStruct.params = params;

if ~isempty(tbl)
    outStruct.summary = struct('mode', mode, ...
        'numSamples', height(tbl), ...
        'maxVx', max(abs(tbl.Vx)), ...
        'maxWz', max(abs(tbl.Wz)));
else
    outStruct.summary = struct('mode', mode, 'numSamples', 0, 'maxVx', 0, 'maxWz', 0);
end

if isfield(logStruct, 'environment')
    outStruct.environment = logStruct.environment;
else
    outStruct.environment = struct();
end

outStruct.cmdStructs = cmdList;

if nargout == 0
    disp(outStruct.summary);
    assignin('base', 'unifiedReplay', outStruct);
    clear outStruct
else
    out = outStruct;
end
end

function cmdList = appendCmd(cmdList, cmd)
if isempty(cmdList)
    cmdList = cmd;
else
    cmdList(end+1) = cmd; %#ok<AGROW>
end
end

function log = resolveLog(input)
if isstruct(input)
    log = input;
    return
end

path = gik9dof.internal.resolvePath(string(input));
loaded = load(path);
if isfield(loaded, 'log')
    log = loaded.log;
elseif isfield(loaded, 'logH')
    log = loaded.logH;
elseif isfield(loaded, 'logS')
    log = loaded.logS;
else
    error("unified_chassis_replay:MissingLog", ...
        "MAT file %s did not contain log/logH/logS.", path);
end
end

function samples = extractHolisticSamples(log)
if ~isfield(log, 'qTraj') || ~isfield(log, 'timestamps')
    error("unified_chassis_replay:InvalidHolisticLog", "Missing qTraj or timestamps.");
end
baseIdx = findBaseIndices(log);
if isempty(baseIdx)
    error("unified_chassis_replay:MissingBaseIndices", "Base joints not found in log.");
end
poses = log.qTraj(baseIdx, :);
numSamples = size(poses, 2);

samples(1,numSamples) = struct('x',0,'y',0,'theta',0,'t',0,'arm_qdot',[]); %#ok<AGROW>
for k = 1:numSamples
    samples(k).x = poses(1,k);
    samples(k).y = poses(2,k);
    samples(k).theta = poses(3,k);
    if k == 1
        samples(k).t = 0;
    else
        samples(k).t = log.timestamps(k-1);
    end
    if isfield(log, 'armVelocities') && ~isempty(log.armVelocities)
        samples(k).arm_qdot = log.armVelocities(:,k);
    end
end
end

function samples = extractStageCSamples(log)
if ~isfield(log, 'stageLogs')
    error("unified_chassis_replay:InvalidStagedLog", "Missing stageLogs.");
end
stageC = log.stageLogs.stageC;
if isempty(stageC) || ~isfield(stageC, 'qTraj')
    samples = struct([]);
    return
end
baseIdx = findBaseIndices(stageC);
poses = stageC.qTraj(baseIdx, :);
numSamples = size(poses, 2);

samples(1,numSamples) = struct('x',0,'y',0,'theta',0,'t',0,'arm_qdot',[]); %#ok<AGROW>
for k = 1:numSamples
    samples(k).x = poses(1,k);
    samples(k).y = poses(2,k);
    samples(k).theta = poses(3,k);
    if k == 1
        samples(k).t = 0;
    else
        samples(k).t = stageC.timestamps(k-1);
    end
    samples(k).arm_qdot = [];
end
end

function samples = extractStageBSamples(log)
if ~isfield(log, 'stageLogs') || ~isfield(log.stageLogs, 'stageB')
    samples = struct([]);
    return
end
stageB = log.stageLogs.stageB;
if ~isfield(stageB, 'qTraj') || isempty(stageB.qTraj)
    samples = struct([]);
    return
end

baseIdx = findBaseIndices(stageB);
if isempty(baseIdx)
    samples = struct([]);
    return
end

timestamps = stageB.timestamps;
if isempty(timestamps)
    timestamps = linspace(0, (size(stageB.qTraj,2)-1)*0.01, size(stageB.qTraj,2));
end

if isfield(stageB, 'pathStates') && ~isempty(stageB.pathStates)
    follower = gik9dof.control.purePursuitFollower(stageB.pathStates, ...
        'DesiredLinearVelocity', 0.6);
else
    follower = [];
end

numSamples = numel(timestamps);
samples = repmat(struct('v',0,'w',0,'t',0,'pose',[0 0 0]), 1, numSamples);
for k = 1:numSamples
    pose = [stageB.qTraj(baseIdx(1),k), stageB.qTraj(baseIdx(2),k), stageB.qTraj(baseIdx(3),k)];
    if ~isempty(follower)
        [v,w,~] = follower.step(pose);
    else
        if k == 1
            dt = 1e-3;
        else
            dt = max(1e-3, timestamps(k) - timestamps(k-1));
        end
        if k == 1
            v = 0;
            w = 0;
        else
            prevPose = [stageB.qTraj(baseIdx(1),k-1), stageB.qTraj(baseIdx(2),k-1), stageB.qTraj(baseIdx(3),k-1)];
            dx = pose(1) - prevPose(1);
            dy = pose(2) - prevPose(2);
            v = hypot(dx, dy) / dt;
            dtheta = wrapToPi(pose(3) - prevPose(3));
            w = dtheta / dt;
        end
    end
    samples(k).v = v;
    samples(k).w = w;
    samples(k).t = timestamps(k);
    samples(k).pose = pose;
end
end

function idx = findBaseIndices(log)
if isfield(log, 'baseIndices') && ~isempty(log.baseIndices)
    idx = log.baseIndices;
    return
end

if isfield(log, 'jointNames')
    jointNames = string(log.jointNames);
else
    jointNames = strings(0,1);
end

idx = zeros(1,3);
idx(1) = find(jointNames == "joint_x", 1);
idx(2) = find(jointNames == "joint_y", 1);
idx(3) = find(jointNames == "joint_theta", 1);
idx(idx==0) = [];
end
