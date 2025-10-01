function outputs = generateExternalPlots(logInput, options)
%GENERATEEXTERNALPLOTS Reuse legacy plotting utilities for trajectory data.
%   outputs = gik9dof.generateExternalPlots(logInput) loads the supplied log
%   (struct or MAT-file), adds the external mobile manipulator package to the
%   path, computes stage metrics using computeStageMetrics_row, and produces
%   top-view/3D/error plots. Figures are optionally saved to the provided
%   directory. The helper is a thin compatibility wrapper so new pipelines can
%   leverage the existing plotting stack from mobile_manip_3stage_package.
%
%   Name-value options:
%       ExternalPackagePath - Location of mobile_manip_3stage_package (default
%                             '~/Projects/mobile_manip_3stage_package').
%       OutputDir           - Directory to save figures (optional).
%       World               - Cell array of collision objects. When empty the
%                             helper attempts to build discs from log.floorDiscs.
%       SafetyMargin        - Additional inflation margin passed to legacy
%                             utilities (default 0.0).
%       EndEffector         - End-effector name (default 'left_gripper_link').
%
%   outputs contains figure handles and any exported file paths.

arguments
    logInput
    options.ExternalPackagePath (1,1) string = "~/Projects/mobile_manip_3stage_package"
    options.OutputDir (1,1) string = ""
    options.World = {}
    options.SafetyMargin (1,1) double {mustBeNonnegative} = 0.0
    options.EndEffector (1,1) string = "left_gripper_link"
end

log = resolveLog(logInput);
externalPath = char(expanduser(options.ExternalPackagePath));
if ~isfolder(externalPath)
    error("gik9dof:generateExternalPlots:MissingExternalPackage", ...
        "External package not found at %s", externalPath);
end

oldPath = addpath(externalPath);
cleanupPath = onCleanup(@() path(oldPath));

robot = gik9dof.createRobotModel("DataFormat","row");
qTraj = log.qTraj;
if isempty(qTraj)
    error("gik9dof:generateExternalPlots:EmptyTrajectory", ...
        "Log does not contain qTraj data.");
end

% Align samples with target poses (drop initial configuration column).
qRow = qTraj(:,2:end).';
Tref = log.targetPoses;

world = options.World;
if isempty(world) && isfield(log, 'floorDiscs') && ~isempty(log.floorDiscs) && isfield(log.floorDiscs, 'Center')
    world = discsToCollisionWorld(log.floorDiscs);
end

if size(Tref,3) >= 1
    T1 = Tref(:,:,1);
else
    T1 = eye(4);
end

MA = computeStageMetricsLocal(robot, char(options.EndEffector), qRow, 'A', T1, Tref);
MB = computeStageMetricsLocal(robot, char(options.EndEffector), qRow, 'B', T1, Tref);
MC = computeStageMetricsLocal(robot, char(options.EndEffector), qRow, 'C', T1, Tref);

figTop = plotStageTopView_adapted(MA, MB, MC, Tref, world);
fig3D  = plotStage3D_adapted(MA, MB, MC, Tref);
figErr = plotStageErrors_adapted(MA, MB, MC);

files.topView = "";
files.threeD  = "";
files.errors  = "";

if strlength(options.OutputDir) > 0
    outDir = gik9dof.internal.resolvePath(options.OutputDir, true);
    if ~isfolder(outDir)
        mkdir(outDir);
    end
    files.topView = fullfile(outDir, 'external_top_view.png');
    files.threeD  = fullfile(outDir, 'external_3d_view.png');
    files.errors  = fullfile(outDir, 'external_errors.png');
    exportgraphics(figTop, files.topView, 'Resolution', 150);
    exportgraphics(fig3D,  files.threeD, 'Resolution', 150);
    exportgraphics(figErr, files.errors, 'Resolution', 150);
end

outputs = struct('topViewFigure', figTop, ...
    'threeDFigure', fig3D, ...
    'errorFigure', figErr, ...
    'files', files, ...
    'world', {world});
end

function world = discsToCollisionWorld(discs)
world = cell(numel(discs), 1);
for i = 1:numel(discs)
    disc = discs(i);
    radius = disc.Radius + disc.SafetyMargin;
    cyl = collisionCylinder(radius, 0.05);
    pose = eye(4);
    pose(1:2,4) = disc.Center(:);
    cyl.Pose = pose;
    world{i} = cyl;
end
end

function out = resolveLog(input)
if isstruct(input)
    out = input;
    return
end

if ~(ischar(input) || (isstring(input) && isscalar(input)))
    error("gik9dof:generateExternalPlots:InvalidInput", ...
        "Input must be a log struct or path to a MAT file.");
end

path = gik9dof.internal.resolvePath(string(input));
if ~isfile(path)
    error("gik9dof:generateExternalPlots:MissingFile", ...
        "Could not find log file: %s", path);
end

loaded = load(path, 'log');
if ~isfield(loaded, 'log')
    error("gik9dof:generateExternalPlots:MissingVariable", ...
        "MAT file %s does not contain a variable named 'log'.", path);
end

out = loaded.log;
end

function mustBeNonnegative(x)
if any(x < 0)
    error("gik9dof:generateExternalPlots:NegativeValue", ...
        "Value must be non-negative.");
end
end

function full = expanduser(pathStr)
if startsWith(pathStr, "~")
    homeDir = char(java.lang.System.getProperty('user.home'));
    rest = char(extractAfter(pathStr, 1));
    if ~isempty(rest) && (rest(1) == filesep)
        rest = rest(2:end);
    end
    full = fullfile(homeDir, rest);
else
    full = pathStr;
end
end

function fig = plotStageTopView_adapted(MA, MB, MC, Tref, world)
% Adapted from mobile_manip_3stage_package/mm_stage_plots.m (plotStageTopView_row).
fig = figure('Name','External Top view (XY): EE & Base paths');
clf(fig);
ax = axes('Parent', fig);
hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
xlabel(ax,'X [m]'); ylabel(ax,'Y [m]');
title(ax,'Top view: EE (solid) & Base (dashed)');

drawDiscFootprints(world, ax);

refXY = squeeze(Tref(1:2,4,:)).';
plot(ax, refXY(:,1), refXY(:,2), ':', 'LineWidth', 1.2, 'DisplayName','Desired EE (XY)');

plot(ax, MA.pos(:,1), MA.pos(:,2), '-',  'LineWidth',1.8, 'DisplayName','Stage A: EE');
plot(ax, MA.baseXY(:,1), MA.baseXY(:,2), '--', 'LineWidth',1.2, 'DisplayName','Stage A: Base');

plot(ax, MB.pos(:,1), MB.pos(:,2), '-',  'LineWidth',1.8, 'DisplayName','Stage B: EE');
plot(ax, MB.baseXY(:,1), MB.baseXY(:,2), '--', 'LineWidth',1.2, 'DisplayName','Stage B: Base');

plot(ax, MC.pos(:,1), MC.pos(:,2), '-',  'LineWidth',1.8, 'DisplayName','Stage C: EE');
plot(ax, MC.baseXY(:,1), MC.baseXY(:,2), '--', 'LineWidth',1.2, 'DisplayName','Stage C: Base');

legend(ax,'Location','bestoutside'); hold(ax,'off');
end

function fig = plotStage3D_adapted(MA, MB, MC, Tref)
% Adapted from mobile_manip_3stage_package/mm_stage_plots.m (plotStage3D_row).
fig = figure('Name','External 3D Overview: EE vs Desired');
clf(fig);
ax = axes('Parent', fig);
hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
title(ax,'3D EE vs Desired');
refP = squeeze(Tref(1:3,4,:)).';
plot3(ax, refP(:,1), refP(:,2), refP(:,3), ':', 'LineWidth', 1.2, 'DisplayName','Desired EE');
plot3(ax, MA.pos(:,1), MA.pos(:,2), MA.pos(:,3), '-', 'LineWidth',1.8, 'DisplayName','Stage A: EE');
plot3(ax, MB.pos(:,1), MB.pos(:,2), MB.pos(:,3), '-', 'LineWidth',1.8, 'DisplayName','Stage B: EE');
plot3(ax, MC.pos(:,1), MC.pos(:,2), MC.pos(:,3), '-', 'LineWidth',1.8, 'DisplayName','Stage C: EE');
legend(ax,'Location','bestoutside'); hold(ax,'off');
end

function fig = plotStageErrors_adapted(MA, MB, MC)
% Adapted from mobile_manip_3stage_package/mm_stage_plots.m (plotStageErrors_row).
fig = figure('Name','External Tracking Errors by Stage');
clf(fig);

subplot(3,1,1);
yyaxis left;  plot(MA.errZ,      'LineWidth',1.6); ylabel('z error [m]');
yyaxis right; plot(MA.errOriDeg, 'LineWidth',1.6); ylabel('ori error [deg]');
grid on; title('Stage A (arm-only): z & orientation errors'); xlabel('step');

subplot(3,1,2);
if isfield(MB,'errXY')
    plot(MB.errXY, 'LineWidth',1.6);
else
    plot(vecnorm(MB.pos(:,1:2) - MB.pos(1,1:2), 2, 2), 'LineWidth',1.6);
end
grid on; title('Stage B (base-only): XY error'); ylabel('XY error [m]'); xlabel('step');

subplot(3,1,3);
yyaxis left;
if isfield(MC,'errPosNorm')
    plot(MC.errPosNorm, 'LineWidth',1.6);
else
    plot(vecnorm(MC.pos - MC.pos(1,:), 2, 2), 'LineWidth',1.6);
end
ylabel('pos error [m]');
yyaxis right;
plot(MC.errOriDeg, 'LineWidth',1.6); ylabel('ori error [deg]');
grid on; title('Stage C (full): position & orientation errors'); xlabel('sample');
end

function drawDiscFootprints(world, ax)
if nargin < 2 || isempty(ax)
    ax = gca;
end
if isempty(world), return; end
for i = 1:numel(world)
    c = world{i};
    if isa(c,'collisionCylinder')
        T = c.Pose; r = c.Radius;
        th = linspace(0,2*pi,100);
        cx = T(1,4); cy = T(2,4);
        plot(ax, cx + r*cos(th), cy + r*sin(th), '-', 'LineWidth', 1.5);
    end
end
end

function M = computeStageMetricsLocal(robot, ee, qSeq, stage, T1, Tref)
if nargin < 6 || isempty(Tref)
    Tref = repmat(eye(4),1,1,size(qSeq,1));
end
if nargin < 5 || isempty(T1)
    T1 = eye(4);
end

n = size(qSeq,1);
M = struct;
M.stage = char(stage);
M.n = n;
M.pos = zeros(n,3);
M.quat = zeros(n,4);
M.baseXY = nan(n,2);

map = jointIndexMapLocal(robot);
idxX = mapKey(map, "joint_x");
idxY = mapKey(map, "joint_y");

for i = 1:n
    cfg = qSeq(i,:);
    T = getTransform(robot, cfg, ee);
    M.pos(i,:) = tform2trvec(T);
    M.quat(i,:) = tform2quat(T);
    if idxX > 0 && idxY > 0
        M.baseXY(i,:) = [cfg(idxX), cfg(idxY)];
    end
end

switch upper(M.stage)
    case 'A'
        zDes = T1(3,4);
        qDes = tform2quat(T1);
        M.errZ = M.pos(:,3) - zDes;
        M.errOriDeg = quatAngleErrorDegLocal(M.quat, repmat(qDes, n, 1));
        M.errXY = hypot(M.pos(:,1) - T1(1,4), M.pos(:,2) - T1(2,4));
    case 'B'
        M.errXY = hypot(M.pos(:,1) - T1(1,4), M.pos(:,2) - T1(2,4));
        M.errZ = M.pos(:,3) - T1(3,4);
        qDes = tform2quat(T1);
        M.errOriDeg = quatAngleErrorDegLocal(M.quat, repmat(qDes, n, 1));
    otherwise
        K = size(Tref,3);
        refPos = squeeze(Tref(1:3,4,:)).';
        refQuat = zeros(K,4);
        for k = 1:K
            refQuat(k,:) = tform2quat(Tref(:,:,k));
        end
        idx = min((1:n)', K);
        M.refPosAtSample = refPos(idx,:);
        M.refQuatAtSample = refQuat(idx,:);
        M.errPosVec = M.pos - M.refPosAtSample;
        M.errPosNorm = vecnorm(M.errPosVec, 2, 2);
        M.errOriDeg = quatAngleErrorDegLocal(M.quat, M.refQuatAtSample);
        M.refIndex = idx;
end
end

function idx = mapKey(map, name)
if isKey(map, name)
    idx = map(name);
else
    idx = -1;
end
end

function map = jointIndexMapLocal(robot)
originalFormat = robot.DataFormat;
robot.DataFormat = "struct";
homeStruct = homeConfiguration(robot);
robot.DataFormat = originalFormat;
map = containers.Map;
for k = 1:numel(homeStruct)
    map(string(homeStruct(k).JointName)) = k;
end
end

function deg = quatAngleErrorDegLocal(qEst, qDes)
if size(qDes,1) == 1
    qDes = repmat(qDes, size(qEst,1), 1);
end
d = abs(sum(qEst .* qDes, 2));
d = min(max(d, -1), 1);
deg = 2*acos(d) * 180/pi;
end
