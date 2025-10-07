function animatePurePursuitSimulation(targetPath, ikPath, simResult, discs, options)
%ANIMATEPUREPURSUITSIMULATION Visualise reference vs simulated chassis path.
%   gik9dof.viz.animatePurePursuitSimulation(targetPath, ikPath, simResult, discs)
%   draws a top-view animation comparing the target reference, IK result,
%   and the path executed by the pure-pursuit simulation.
%
%   targetPath : Nx3 [x y yaw] desired path.
%   ikPath     : optional Nx3 actual IK path (empty to skip).
%   simResult  : struct returned by simulatePurePursuitExecution.
%   discs      : struct array with fields Center, Radius, SafetyMargin,
%                DistanceMargin (optional) used for drawing safety zones.
%   options    : name-value pairs
%       VideoFile    - path to mp4 (default "")
%       FrameRate    - 20 fps default
%       FigureScale  - 1.0 default
%       Background   - [0 0 0]
%
%   The animation shows the simulated chassis orientation and safety halos.

arguments
    targetPath (:,3) double
    ikPath (:,3) double
    simResult struct
    discs struct = struct([])
    options.VideoFile (1,1) string = ""
    options.FrameRate (1,1) double {mustBePositive} = 20
    options.FigureScale (1,1) double {mustBePositive} = 1.0
    options.Background (1,3) double = [0 0 0]
end

if any(options.Background < 0) || any(options.Background > 1)
    error('animatePurePursuitSimulation:BackgroundRange', ...
        'Background must be within [0,1].');
end

numSteps = size(simResult.poses,1);
if numSteps == 0
    warning('animatePurePursuitSimulation:EmptySimulation', 'Simulation results empty; nothing to animate.');
    return
end

scale = options.FigureScale;
fig = figure('Color', options.Background, 'Position', round([100 100 800 600]*scale));
ax = axes('Parent', fig);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');

plot(ax, targetPath(:,1), targetPath(:,2), 'g-', 'LineWidth', 1.2, 'DisplayName', 'Reference');
if ~isempty(ikPath)
    plot(ax, ikPath(:,1), ikPath(:,2), 'Color', [0.85 0.6 0.1], 'LineWidth', 1.2, 'DisplayName', 'IK Path');
end
plot(ax, simResult.poses(:,1), simResult.poses(:,2), 'c--', 'LineWidth', 1.4, 'DisplayName', 'Simulated');

theta = linspace(0, 2*pi, 100);
for d = 1:numel(discs)
    center = double(discs(d).Center(:)');
    radius = discs(d).Radius;
    safety = 0;
    if isfield(discs(d), 'SafetyMargin'), safety = safety + discs(d).SafetyMargin; end
    if isfield(discs(d), 'DistanceMargin'), safety = safety + discs(d).DistanceMargin; end
    plot(ax, center(1) + radius*cos(theta), center(2) + radius*sin(theta), ...
        'Color', [1 0.4 0.2], 'LineStyle', '-', 'LineWidth', 1);
    plot(ax, center(1) + (radius+safety)*cos(theta), center(2) + (radius+safety)*sin(theta), ...
        'Color', [1 0.4 0.2], 'LineStyle', '--', 'LineWidth', 1);
end

legend(ax, 'Location', 'bestoutside');

bodyWidth = 0.57;  % approximate chassis width
bodyLength = 0.8;  % approximate chassis length
chassisPatch = patch(ax, 'XData', [], 'YData', [], 'FaceColor', [0.2 0.6 1], ...
    'FaceAlpha', 0.7, 'EdgeColor', [0.1 0.3 0.8], 'LineWidth', 1.2);

lookaheadMarker = plot(ax, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Lookahead');

if strlength(options.VideoFile) > 0
    writer = VideoWriter(char(options.VideoFile), 'MPEG-4');
    writer.FrameRate = options.FrameRate;
    open(writer);
    cleanup = onCleanup(@() close(writer));
else
    writer = [];
    cleanup = onCleanup(@() []);
end

axis(ax, [min(targetPath(:,1))-1, max(targetPath(:,1))+1, ...
    min(targetPath(:,2))-1, max(targetPath(:,2))+1]);

for k = 1:numSteps
    pose = simResult.poses(k,:);
    lookaheadIdx = simResult.status(k).lookaheadIndex;
    if ~isnan(lookaheadIdx) && lookaheadIdx >= 1 && lookaheadIdx <= size(targetPath,1)
        laPt = targetPath(lookaheadIdx,1:2);
        set(lookaheadMarker, 'XData', laPt(1), 'YData', laPt(2));
    end

    R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
    corners = [ bodyLength/2,  bodyWidth/2;
                bodyLength/2, -bodyWidth/2;
               -bodyLength/2, -bodyWidth/2;
               -bodyLength/2,  bodyWidth/2]';
    worldCorners = R * corners + pose(1:2)';
    set(chassisPatch, 'XData', [worldCorners(1,:), worldCorners(1,1)], ...
        'YData', [worldCorners(2,:), worldCorners(2,1)]);

    drawnow;
    if ~isempty(writer)
        frame = getframe(fig);
        writeVideo(writer, frame);
    end
end

delete(cleanup);
end
