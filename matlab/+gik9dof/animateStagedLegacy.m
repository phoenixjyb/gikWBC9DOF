function animateStagedLegacy(logStaged, trajectories, varargin)
%ANIMATESTAGEDLEGACY Render staged control animation matching legacy demo.
%   gik9dof.animateStagedLegacy(logStaged) replays the staged run with robot
%   meshes, floor-disc obstacles, and desired vs. actual EE paths using the
%   style from mobile_manip_3stage_demo.m.
%
%   gik9dof.animateStagedLegacy(logStaged, trajectories) lets you override the
%   target pose stack via a struct field 'staged'.
%
%   Name-value options:
%       'Robot'      - rigidBodyTree to render (default from URDF)
%       'SampleStep' - Render every Nth frame (default 2)
%       'FrameRate'  - Video frame rate (default 30)
%       'ExportVideo'- Optional MP4 path
%
parser = inputParser;
parser.FunctionName = mfilename;
addParameter(parser, 'Robot', gik9dof.createRobotModel(), @(x) isa(x,'rigidBodyTree'));
addParameter(parser, 'SampleStep', 2, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(parser, 'FrameRate', 30, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'ExportVideo', "", @(x) ischar(x) || (isstring(x) && isscalar(x)));
addParameter(parser, 'Title', "Staged Control", @(x) ischar(x) || (isstring(x) && isscalar(x)));
parse(parser, varargin{:});
opts = parser.Results;

robot = opts.Robot;
assert(strcmp(robot.DataFormat, 'column'), 'Robot DataFormat must be column for animation.');

posesStaged = getPoseStack(logStaged, trajectories, "staged");
actualStaged = computeEEPath(robot, logStaged.qTraj);

stageInfo = extractStageInfo(robot, logStaged);

fig = figure('Name','Staged Control Animation','NumberTitle','off','Color','w');
set(fig, 'Units','pixels', 'Position', [120 120 1120 840], 'Resize','off');
ax = axes('Parent', fig);
bounds = computeBounds(posesStaged, actualStaged, stageInfo);

panel = struct('ax', ax, 'title', opts.Title, 'log', logStaged, ...
    'robot', robot, 'desiredPath', squeeze(posesStaged(1:3,4,:)).', ...
    'actualPath', actualStaged, 'stageInfo', stageInfo, 'bounds', bounds);

drawPanel(panel, logStaged.qTraj(:,1), 1);

frames = 1:opts.SampleStep:size(logStaged.qTraj, 2);
numFrames = numel(frames);
movieFrames(numFrames) = struct('cdata', [], 'colormap', []); %#ok<NASGU>

for k = 1:numFrames
    idx = frames(k);
    drawPanel(panel, logStaged.qTraj(:, idx), idx);
    drawnow limitrate;
    movieFrames(k) = getframe(fig); %#ok<AGROW>
end

if strlength(opts.ExportVideo) > 0
    exportPath = gik9dof.internal.resolvePath(opts.ExportVideo);
    writer = VideoWriter(exportPath, 'MPEG-4');
    writer.FrameRate = opts.FrameRate;
    open(writer);
    writeVideo(writer, movieFrames);
    close(writer);
    fprintf('Exported staged animation to %s\n', exportPath);
end
end

function poses = getPoseStack(logStruct, trajectories, fieldName)
if nargin >= 3 && isfield(trajectories, fieldName) && ~isempty(trajectories.(fieldName))
    poses = trajectories.(fieldName);
else
    poses = logStruct.targetPoses;
end
end

function eePath = computeEEPath(robot, qSeries)
numFrames = size(qSeries, 2);
eePath = zeros(numFrames, 3);
for i = 1:numFrames
    T = getTransform(robot, qSeries(:, i), 'left_gripper_link');
    eePath(i, :) = tform2trvec(T);
end
end

function stageInfo = extractStageInfo(robot, logStaged)
stageInfo = struct('available', false, 'paths', {{}}, 'colors', {{}}, 'goals', {{}});
if ~isfield(logStaged, 'stageLogs')
    return;
end
fields = {'stageA','stageB','stageC'};
colors = {[0.95 0.55 0.05], [0.05 0.40 0.90], [0.05 0.65 0.25]};
paths = cell(1, numel(fields));
goals = cell(1, numel(fields));
for i = 1:numel(fields)
    if isfield(logStaged.stageLogs, fields{i}) && isfield(logStaged.stageLogs.(fields{i}), 'qTraj')
        qStage = logStaged.stageLogs.(fields{i}).qTraj;
        paths{i} = computeEEPath(robot, qStage);
        if ~isempty(paths{i})
            goals{i} = paths{i}(end, :);
        else
            goals{i} = [];
        end
    else
        paths{i} = [];
        goals{i} = [];
    end
end
stageInfo.available = true;
stageInfo.paths = paths;
stageInfo.colors = colors;
stageInfo.goals = goals;
end

function bounds = computeBounds(poses, actualPath, stageInfo)
desired = squeeze(poses(1:3,4,:)).';
points = [desired; actualPath];
if stageInfo.available
    for i = 1:numel(stageInfo.paths)
        pts = stageInfo.paths{i};
        if ~isempty(pts)
            points = [points; pts]; %#ok<AGROW>
        end
    end
end
if isempty(points)
    points = zeros(1,3);
end
margin = 0.3;
bounds.x = [min(points(:,1))-margin, max(points(:,1))+margin];
bounds.y = [min(points(:,2))-margin, max(points(:,2))+margin];
bounds.z = [0, max(points(:,3))+margin];
end

function setupAxes(ax, titleText, bounds)
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
viewAz = 45; viewEl = 25;
view(ax, [viewAz viewEl]);
axis(ax, [bounds.x(1) bounds.x(2) bounds.y(1) bounds.y(2) bounds.z(1) bounds.z(2)]);
axis(ax,'vis3d');
camproj(ax,'perspective');
center = [mean(bounds.x) mean(bounds.y) mean(bounds.z)];
range = max([diff(bounds.x), diff(bounds.y), diff(bounds.z)]);
range = max(range, 0.6);
camDist = range * 2.0;
azRad = deg2rad(viewAz);
elRad = deg2rad(viewEl);
camOffset = camDist * [cos(elRad)*cos(azRad), cos(elRad)*sin(azRad), sin(elRad)];
set(ax, 'CameraTarget', center, 'CameraPosition', center + camOffset, ...
    'CameraUpVector', [0 0 1], 'CameraViewAngleMode','manual', 'CameraViewAngle', 20);
set(ax, 'CameraPositionMode','manual', 'CameraTargetMode','manual', 'CameraUpVectorMode','manual');
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
title(ax, titleText);
hold(ax,'off');
end

function hDesired = plotDesired(ax, poses)
hold(ax,'on');
desired = squeeze(poses(1:3,4,:)).';
hDesired = plot3(ax, desired(:,1), desired(:,2), desired(:,3), 'LineStyle','-.', 'Color',[0.95 0.15 0.10], 'LineWidth',2.0, 'DisplayName','Desired EE path');
hold(ax,'off');
end

function handles = plotStagePaths(ax, stageInfo, handles)
if ~stageInfo.available
    handles.stagePaths = [];
    return;
end
labels = {'Stage A EE','Stage B EE','Stage C EE'};
hold(ax,'on');
stageHandles = gobjects(1, numel(stageInfo.paths));
for i = 1:numel(stageInfo.paths)
    pts = stageInfo.paths{i};
    if isempty(pts)
        continue;
    end
    stageHandles(i) = plot3(ax, pts(:,1), pts(:,2), pts(:,3), '-', 'Color', stageInfo.colors{i}, 'LineWidth',2.0, 'DisplayName', labels{i});
end
hold(ax,'off');
handles.stagePaths = stageHandles;
end

function handles = plotStageGoals(ax, stageInfo, handles)
if ~stageInfo.available
    handles.stageGoals = [];
    return;
end
shapes = {'^','s','o'};
labels = {'Stage A goal','Stage B goal','Stage C goal'};
hold(ax,'on');
goalHandles = gobjects(1, numel(stageInfo.goals));
for i = 1:numel(stageInfo.goals)
    goal = stageInfo.goals{i};
    if isempty(goal)
        continue;
    end
    goalHandles(i) = plot3(ax, goal(1), goal(2), goal(3), shapes{min(i,numel(shapes))}, 'MarkerSize',12, 'MarkerFaceColor', stageInfo.colors{i}, 'MarkerEdgeColor','k', 'LineStyle','none', 'DisplayName', labels{i});
end
hold(ax,'off');
handles.stageGoals = goalHandles;
end

function plotObstacles(ax, logStruct)
if isfield(logStruct, 'floorDiscs') && ~isempty(logStruct.floorDiscs)
    discs = logStruct.floorDiscs;
    hold(ax,'on');
    for k = 1:numel(discs)
        d = discs(k);
        radius = d.Radius + d.SafetyMargin;
        [X, Y, Z] = cylinder(radius, 60);
        Z = Z * 0.05;
        surf(ax, X + d.Center(1), Y + d.Center(2), Z, 'FaceAlpha', 0.2, 'FaceColor',[1.0 0.2 0.2], 'EdgeColor','none', 'DisplayName','Obstacle');
    end
    hold(ax,'off');
end
end

function shape = stageMarkerShape(idx)
markers = {'^','s','o'};
shape = markers{min(idx, numel(markers))};
end

function drawPanel(panel, qColumn, idx)
ax = panel.ax;
setupAxes(ax, panel.title, panel.bounds);
plotObstacles(ax, panel.log);

hold(ax,'on');
plot3(ax, panel.desiredPath(:,1), panel.desiredPath(:,2), panel.desiredPath(:,3), 'k-.', 'LineWidth',2.0, 'DisplayName','Desired EE path');
plot3(ax, panel.actualPath(1:idx,1), panel.actualPath(1:idx,2), panel.actualPath(1:idx,3), '-', 'Color',[0.05 0.40 0.90], 'LineWidth',2.0, 'DisplayName','EE path');
plot3(ax, panel.actualPath(idx,1), panel.actualPath(idx,2), panel.actualPath(idx,3), 'o', 'MarkerSize',10, 'MarkerFaceColor',[0.85 0.33 0.10], 'MarkerEdgeColor','k', 'LineStyle','none', 'DisplayName','Current EE');

if panel.stageInfo.available
    labels = {'Stage A EE','Stage B EE','Stage C EE'};
    for i = 1:numel(panel.stageInfo.paths)
        pts = panel.stageInfo.paths{i};
        if isempty(pts)
            continue;
        end
        plot3(ax, pts(:,1), pts(:,2), pts(:,3), '-', 'Color', panel.stageInfo.colors{i}, 'LineWidth',2.0, 'DisplayName', labels{i});
        goal = panel.stageInfo.goals{i};
        if ~isempty(goal)
            plot3(ax, goal(1), goal(2), goal(3), stageMarkerShape(i), 'MarkerSize',12, 'MarkerFaceColor', panel.stageInfo.colors{i}, 'MarkerEdgeColor','k', 'LineStyle','none', 'DisplayName',[labels{i} ' goal']);
        end
    end
end

show(panel.robot, qColumn, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true, 'Visuals','on', 'Frames','off');
legend(ax,'Location','bestoutside','AutoUpdate','off');
hold(ax,'off');
end
