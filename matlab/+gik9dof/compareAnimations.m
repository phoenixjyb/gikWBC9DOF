function compareAnimations(logHolistic, logStaged, trajectories, varargin)
%COMPAREANIMATIONS Side-by-side animation for holistic vs. staged runs.
%   gik9dof.compareAnimations(logHolistic, logStaged) replays the robot motion
%   for both control strategies in a 1x2 subplot (left = holistic, right = staged)
%   using logged joint trajectories and target poses. Obstacles (floor discs)
%   are rendered if available in the logs.
%
%   gik9dof.compareAnimations(logHolistic, logStaged, trajectories) lets you
%   override the target pose stacks with a struct containing fields
%   'holistic' and/or 'staged'.
%
%   Additional name-value options:
%       'Robot'         - Preconstructed rigidBodyTree (default from URDF)
%       'SampleStep'    - Visualise every Nth frame (default 5)
%       'FrameRate'     - Export framerate (default 30)
%       'ExportVideo'   - MP4 output path (optional)
%       'TitleHolistic' - Left subplot title
%       'TitleStaged'   - Right subplot title
%
%   Example:
%       compareAnimations(logH, logS, struct(), 'ExportVideo', 'results/comp.mp4');

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

% Extract pose stacks (override when provided)
posesHolistic = getPoseStack(logHolistic, trajectories, "holistic");
posesStaged   = getPoseStack(logStaged, trajectories, "staged");

% Compute EE paths
actualHolistic = computeEEPath(robot, logHolistic.qTraj);
actualStaged   = computeEEPath(robot, logStaged.qTraj);

% Stage breakdown for staged log (if available)
stageInfo = extractStageInfo(robot, logStaged);

% Desired path points
desiredHolistic = squeeze(posesHolistic(1:3,4,:)).';
desiredStaged   = squeeze(posesStaged(1:3,4,:)).';

framesHolistic = 1:opts.SampleStep:size(logHolistic.qTraj, 2);
framesStaged   = 1:opts.SampleStep:size(logStaged.qTraj, 2);
numFrames = max(numel(framesHolistic), numel(framesStaged));

fig = figure('Name','Holistic vs Staged Animation','NumberTitle','off');
set(fig, 'Position', [100 100 1280 640], 'Color','w');
axHolistic = subplot(1,2,1, 'Parent', fig);
axStaged   = subplot(1,2,2, 'Parent', fig);

setupAxes(axHolistic, opts.TitleHolistic, logHolistic);
setupAxes(axStaged,   opts.TitleStaged,   logStaged);

legend(axHolistic, 'Location','bestoutside');
legend(axStaged,   'Location','bestoutside');

movieFrames(numFrames) = struct('cdata', [], 'colormap', []); %#ok<NASGU>

for k = 1:numFrames
    idxH = framesHolistic(min(k, numel(framesHolistic)));
    idxS = framesStaged(min(k, numel(framesStaged)));

    drawScene(axHolistic, robot, logHolistic.qTraj(:, idxH), desiredHolistic, actualHolistic, idxH, logHolistic);
    drawScene(axStaged,   robot, logStaged.qTraj(:, idxS),   desiredStaged,   actualStaged,   idxS, logStaged, stageInfo);

    drawnow;
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
if isfield(logStaged, 'stageLogs')
    fields = {'stageA','stageB','stageC'};
    colors = {[0.95 0.55 0.05], [0.05 0.40 0.90], [0.05 0.65 0.25]};
    stagePaths = cell(1, numel(fields));
    stageLengths = zeros(1, numel(fields));
    for i = 1:numel(fields)
        if isfield(logStaged.stageLogs, fields{i})
            q = logStaged.stageLogs.(fields{i}).qTraj;
            stagePaths{i} = computeEEPath(robot, q);
            stageLengths(i) = size(q, 2);
        else
            stagePaths{i} = [];
            stageLengths(i) = 0;
        end
    end
    stageInfo.available = true;
    stageInfo.paths = stagePaths;
    stageInfo.lengths = stageLengths;
    stageInfo.colors = colors;
end
end

function setupAxes(ax, titleText, logStruct)
cla(ax);
hold(ax, 'on');
axis(ax, 'equal');
view(ax, [45 25]);
axis(ax, [-3 3 -3 3 0 2]);
box(ax, 'on');
set(ax, 'Color','w');
xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
zlabel(ax, 'Z [m]');
title(ax, titleText);
plotObstacles(ax, logStruct);
end

function drawScene(ax, robot, q, desiredPath, actualPath, idx, logStruct, stageInfo)
if nargin < 8
    stageInfo = struct('available', false);
end

cla(ax);
setupAxes(ax, ax.Title.String, logStruct);

plot3(ax, desiredPath(:,1), desiredPath(:,2), desiredPath(:,3), 'k-.', 'LineWidth', 1.5, 'DisplayName','Desired EE');
plot3(ax, actualPath(:,1), actualPath(:,2), actualPath(:,3), 'Color',[0.15 0.45 0.80], 'LineWidth', 1.8, 'DisplayName','Actual EE');
scatter3(ax, actualPath(idx,1), actualPath(idx,2), actualPath(idx,3), 80, 'filled', 'MarkerFaceColor',[0.85 0.33 0.10], 'DisplayName','Current EE');

if stageInfo.available
    offset = 0;
    for i = 1:numel(stageInfo.paths)
        pts = stageInfo.paths{i};
        if isempty(pts)
            continue;
        end
        plot3(ax, pts(:,1), pts(:,2), pts(:,3), 'Color', stageInfo.colors{i}, ...
            'LineWidth', 1.8, 'DisplayName', sprintf('Stage %c', 'A'+i-1));
        offset = offset + size(pts,1);
    end
end

show(robot, q, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
end

function plotObstacles(ax, logStruct)
if isfield(logStruct, 'floorDiscs') && ~isempty(logStruct.floorDiscs)
    discs = logStruct.floorDiscs;
    for k = 1:numel(discs)
        d = discs(k);
        radius = d.Radius + d.SafetyMargin;
        [X, Y, Z] = cylinder(radius, 40);
        Z = Z * 0.05;
        surf(ax, X + d.Center(1), Y + d.Center(2), Z, 'FaceAlpha', 0.2, ...
            'EdgeColor', 'none', 'FaceColor', [1.0 0.2 0.2], 'DisplayName','Obstacle');
    end
end
end
