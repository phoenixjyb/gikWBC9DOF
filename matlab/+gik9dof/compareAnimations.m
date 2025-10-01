function compareAnimations(logHolistic, logStaged, trajectories, varargin)
%COMPAREANIMATIONS Visualise holistic vs. staged runs side-by-side.
%   gik9dof.compareAnimations(logHolistic, logStaged) animates both control
%   strategies in a 1x2 subplot window using logged joint trajectories and
%   target poses. Floor-disc obstacles are rendered when present in the logs.
%
%   gik9dof.compareAnimations(..., trajectories) lets you override target pose
%   stacks with a struct containing fields 'holistic' and/or 'staged'.
%
%   Name-value options:
%       'Robot'         - rigidBodyTree (default from URDF)
%       'SampleStep'    - Visualise every Nth frame (default 5)
%       'FrameRate'     - Export framerate (default 30)
%       'ExportVideo'   - Optional MP4 output path
%       'TitleHolistic' - Holistic subplot title
%       'TitleStaged'   - Staged subplot title

parser = inputParser;
parser.FunctionName = mfilename;
addParameter(parser, 'Robot', gik9dof.createRobotModel(), @(x) isa(x,'rigidBodyTree'));
addParameter(parser, 'SampleStep', 5, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(parser, 'FrameRate', 30, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'ExportVideo', "", @(x) ischar(x) || (isstring(x) && isscalar(x)));
addParameter(parser, 'TitleHolistic', "Holistic Control", @(x) ischar(x) || (isstring(x) && isscalar(x)));
addParameter(parser, 'TitleStaged', "Staged Control", @(x) ischar(x) || (isstring(x) && isscalar(x)));
parse(parser, varargin{:});
opts = parser.Results;

robot = opts.Robot;
assert(strcmp(robot.DataFormat, 'column'), 'Robot DataFormat must be column for animation.');

posesHolistic = getPoseStack(logHolistic, trajectories, "holistic");
posesStaged   = getPoseStack(logStaged, trajectories, "staged");

actualHolistic = computeEEPath(robot, logHolistic.qTraj);
actualStaged   = computeEEPath(robot, logStaged.qTraj);

stageInfo = extractStageInfo(robot, logStaged);

fig = figure('Name','Holistic vs Staged Animation','NumberTitle','off','Color','w');
set(fig, 'Position', [100 100 1280 640]);
axHolistic = subplot(1,2,1, 'Parent', fig);
axStaged   = subplot(1,2,2, 'Parent', fig);

panelHolistic = struct('ax', axHolistic, 'title', opts.TitleHolistic, ...
    'log', logHolistic, 'robot', robot, ...
    'desiredPath', squeeze(posesHolistic(1:3,4,:)).', ...
    'actualPath', actualHolistic, 'stageInfo', struct('available', false));
panelStaged = struct('ax', axStaged, 'title', opts.TitleStaged, ...
    'log', logStaged, 'robot', robot, ...
    'desiredPath', squeeze(posesStaged(1:3,4,:)).', ...
    'actualPath', actualStaged, 'stageInfo', stageInfo);

framesHolistic = 1:opts.SampleStep:size(logHolistic.qTraj, 2);
framesStaged   = 1:opts.SampleStep:size(logStaged.qTraj, 2);
numFrames = max(numel(framesHolistic), numel(framesStaged));

movieFrames(numFrames) = struct('cdata', [], 'colormap', []); %#ok<NASGU>

for k = 1:numFrames
    idxH = framesHolistic(min(k, numel(framesHolistic)));
    idxS = framesStaged(min(k, numel(framesStaged)));

    drawPanel(panelHolistic, logHolistic.qTraj(:, idxH), idxH);
    drawPanel(panelStaged,   logStaged.qTraj(:, idxS),   idxS);

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
    fprintf('Exported comparison animation to %s\n', exportPath);
end
end

function drawPanel(panel, qColumn, idx)
ax = panel.ax;
setupAxes(ax, panel.title);
plotObstacles(ax, panel.log);
hold(ax, 'on');
plot3(ax, panel.desiredPath(:,1), panel.desiredPath(:,2), panel.desiredPath(:,3), 'k-.', 'LineWidth',1.6, 'DisplayName','Desired EE');
plot3(ax, panel.actualPath(1:idx,1), panel.actualPath(1:idx,2), panel.actualPath(1:idx,3), 'Color',[0.15 0.45 0.80], 'LineWidth',1.8, 'DisplayName','Actual EE');
plot3(ax, panel.actualPath(idx,1), panel.actualPath(idx,2), panel.actualPath(idx,3), 'o', 'MarkerSize',8, 'MarkerFaceColor',[0.85 0.33 0.10], 'MarkerEdgeColor','k', 'LineStyle','none', 'DisplayName','Current EE');

if panel.stageInfo.available
    labels = {'Stage A','Stage B','Stage C'};
    for i = 1:numel(panel.stageInfo.paths)
        pts = panel.stageInfo.paths{i};
        if isempty(pts)
            continue;
        end
        plot3(ax, pts(:,1), pts(:,2), pts(:,3), 'Color', panel.stageInfo.colors{i}, 'LineWidth',1.8, 'DisplayName',[labels{i} ' path']);
        plot3(ax, pts(end,1), pts(end,2), pts(end,3), stageMarkerShape(i), 'MarkerSize',10, 'MarkerFaceColor', panel.stageInfo.colors{i}, 'MarkerEdgeColor','k', 'LineStyle','none', 'DisplayName',[labels{i} ' goal']);
    end
end

show(panel.robot, qColumn, 'Parent', ax, 'PreservePlot', true, 'Frames','off', 'Visuals','on');
hold(ax, 'off');
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
stageInfo = struct('available', false, 'paths', {{}}, 'colors', {{}}, 'stageLengths', []);
if ~isfield(logStaged, 'stageLogs')
    return;
end
fields = {'stageA','stageB','stageC'};
colors = {[0.95 0.55 0.05], [0.05 0.40 0.90], [0.05 0.65 0.25]};
paths = cell(1, numel(fields));
lengths = zeros(1, numel(fields));
for i = 1:numel(fields)
    if isfield(logStaged.stageLogs, fields{i}) && isfield(logStaged.stageLogs.(fields{i}), 'qTraj')
        qStage = logStaged.stageLogs.(fields{i}).qTraj;
        paths{i} = computeEEPath(robot, qStage);
        lengths(i) = size(qStage, 2);
    else
        paths{i} = [];
        lengths(i) = 0;
    end
end
stageInfo.available = true;
stageInfo.paths = paths;
stageInfo.colors = colors;
stageInfo.stageLengths = lengths;
end

function setupAxes(ax, titleText)
cla(ax);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
axis(ax, [-3 3 -3 3 0 2]);
view(ax,[45 25]);
axis(ax,'vis3d');
camproj(ax,'perspective');
xlabel(ax,'X [m]');
ylabel(ax,'Y [m]');
zlabel(ax,'Z [m]');
title(ax, titleText);
hold(ax,'off');
end

function plotObstacles(ax, logStruct)
if isfield(logStruct, 'floorDiscs') && ~isempty(logStruct.floorDiscs)
    discs = logStruct.floorDiscs;
    hold(ax,'on');
    for k = 1:numel(discs)
        d = discs(k);
        radius = d.Radius + d.SafetyMargin;
        [X, Y, Z] = cylinder(radius, 40);
        Z = Z * 0.05;
        surf(ax, X + d.Center(1), Y + d.Center(2), Z, 'FaceAlpha', 0.25, 'FaceColor',[1.0 0.2 0.2], 'EdgeColor','none', 'DisplayName','Obstacle');
    end
    hold(ax,'off');
end
end

function shape = stageMarkerShape(idx)
markers = {'^','s','o'};
shape = markers{min(idx, numel(markers))};
end
