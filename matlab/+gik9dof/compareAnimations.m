function compareAnimations(logHolistic, logStaged, trajectories, varargin)
%COMPAREANIMATIONS Visualise holistic vs. staged runs side-by-side.
%   gik9dof.compareAnimations(logHolistic, logStaged) animates both control
%   strategies in a 1x2 subplot window using their logged joint trajectories
%   (`qTraj`) and target poses. Obstacles (floor discs) are rendered when stored
%   in each log.
%
%   gik9dof.compareAnimations(..., trajectories) lets you override the target
%   pose stacks with a struct containing fields 'holistic' and/or 'staged'.
%
%   Name-value options:
%       'Robot'         - Preconstructed rigidBodyTree (default from URDF)
%       'SampleStep'    - Visualise every Nth frame (default 5)
%       'FrameRate'     - Export framerate (default 30)
%       'ExportVideo'   - Optional MP4 output path
%       'TitleHolistic' - Title for the holistic subplot
%       'TitleStaged'   - Title for the staged subplot
%
%   Example:
%       gik9dof.compareAnimations(logH, logS, struct(), 'ExportVideo', 'results/comp.mp4');

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

panelHolistic = initPanel(axHolistic, opts.TitleHolistic, logHolistic, robot, logHolistic.qTraj(:,1), posesHolistic, actualHolistic, struct('available', false));
panelStaged   = initPanel(axStaged,   opts.TitleStaged,   logStaged,   robot, logStaged.qTraj(:,1),   posesStaged,   actualStaged,   stageInfo);

framesHolistic = 1:opts.SampleStep:size(logHolistic.qTraj, 2);
framesStaged   = 1:opts.SampleStep:size(logStaged.qTraj, 2);
numFrames = max(numel(framesHolistic), numel(framesStaged));

movieFrames(numFrames) = struct('cdata', [], 'colormap', []); %#ok<NASGU>

for k = 1:numFrames
    idxH = framesHolistic(min(k, numel(framesHolistic)));
    idxS = framesStaged(min(k, numel(framesStaged)));

    updatePanel(panelHolistic, robot, logHolistic.qTraj(:, idxH), actualHolistic, idxH);
    updatePanel(panelStaged,   robot, logStaged.qTraj(:, idxS),   actualStaged,   idxS);

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

function panel = initPanel(ax, titleText, logStruct, robot, qInitial, poses, actualPath, stageInfo)
setupAxes(ax, titleText);
plotObstacles(ax, logStruct);

hold(ax, 'on');
desiredPath = squeeze(poses(1:3,4,:)).';
panel.hDesired = plot3(ax, desiredPath(:,1), desiredPath(:,2), desiredPath(:,3), 'k-.', 'LineWidth',1.6, 'DisplayName','Desired EE');
panel.hActual  = plot3(ax, actualPath(1,1), actualPath(1,2), actualPath(1,3), 'Color',[0.15 0.45 0.80], 'LineWidth',1.8, 'DisplayName','Actual EE');
panel.hCurrent = scatter3(ax, actualPath(1,1), actualPath(1,2), actualPath(1,3), 90, 'filled', 'MarkerFaceColor',[0.85 0.33 0.10], 'DisplayName','Current EE');

if stageInfo.available
    labels = {'Stage A','Stage B','Stage C'};
    panel.stageHandles = gobjects(1, numel(stageInfo.paths));
    panel.stageMarkers = gobjects(1, numel(stageInfo.paths));
    for i = 1:numel(stageInfo.paths)
        pts = stageInfo.paths{i};
        if isempty(pts)
            continue;
        end
        panel.stageHandles(i) = plot3(ax, pts(:,1), pts(:,2), pts(:,3), 'Color', stageInfo.colors{i}, 'LineWidth',1.8, 'LineStyle','-', 'DisplayName',[labels{i} ' path']);
        panel.stageMarkers(i) = scatter3(ax, pts(end,1), pts(end,2), pts(end,3), 120, stageMarkerShape(i), 'filled', 'MarkerFaceColor', stageInfo.colors{i}, 'MarkerEdgeColor','k', 'DisplayName',[labels{i} ' goal']);
    end
else
    panel.stageHandles = gobjects(0);
    panel.stageMarkers = gobjects(0);
end

panel.robotAxes = ax;
panel.robot = robot;
show(robot, qInitial, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true, 'Frames','off', 'Visuals','on');
end

function updatePanel(panel, robot, qColumn, actualPath, idx)
set(panel.hActual, 'XData', actualPath(1:idx,1), 'YData', actualPath(1:idx,2), 'ZData', actualPath(1:idx,3));
set(panel.hCurrent, 'XData', actualPath(idx,1), 'YData', actualPath(idx,2), 'ZData', actualPath(idx,3));
show(robot, qColumn, 'Parent', panel.robotAxes, 'PreservePlot', false, 'FastUpdate', true, 'Frames','off', 'Visuals','on');
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
stageInfo = struct('available', false);
if ~isfield(logStaged, 'stageLogs')
    stageInfo.paths = {};
    stageInfo.colors = {};
    return;
end
fields = {'stageA','stageB','stageC'};
colors = {[0.95 0.55 0.05], [0.05 0.40 0.90], [0.05 0.65 0.25]};
paths = cell(1, numel(fields));
for i = 1:numel(fields)
    if isfield(logStaged.stageLogs, fields{i}) && ~isempty(logStaged.stageLogs.(fields{i}).qTraj)
        paths{i} = computeEEPath(robot, logStaged.stageLogs.(fields{i}).qTraj);
    else
        paths{i} = [];
    end
end
stageInfo.available = true;
stageInfo.paths = paths;
stageInfo.colors = colors;
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
end

function plotObstacles(ax, logStruct)
if isfield(logStruct, 'floorDiscs') && ~isempty(logStruct.floorDiscs)
    discs = logStruct.floorDiscs;
    for k = 1:numel(discs)
        d = discs(k);
        radius = d.Radius + d.SafetyMargin;
        [X, Y, Z] = cylinder(radius, 40);
        Z = Z * 0.05;
        surf(ax, X + d.Center(1), Y + d.Center(2), Z, 'FaceAlpha', 0.25, 'FaceColor',[1.0 0.2 0.2], 'EdgeColor','none', 'DisplayName','Obstacle');
    end
end
end

function marker = stageMarkerShape(idx)
shapes = {'^','s','o'};
marker = shapes{min(idx, numel(shapes))};
end
