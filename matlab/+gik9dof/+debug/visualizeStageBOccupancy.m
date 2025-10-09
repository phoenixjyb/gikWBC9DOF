function visualizeStageBOccupancy(startBase, goalBase, floorDiscs, options)
%VISUALIZESTAGEBOCCUPANCY Plot the Stage-B Hybrid A* occupancy map.
%   gik9dof.debug.visualizeStageBOccupancy(startBase, goalBase, floorDiscs)
%   builds the same inflated-disc occupancy map used during Stage B
%   planning and displays it with the start/goal markers.  STARTBASE and
%   GOALBASE are 1x3 [x y yaw] states.  FLOORDISCS is the struct array
%   returned by gik9dof.environmentConfig (fields Center, Radius,
%   SafetyMargin, etc.).
%
%   gik9dof.debug.visualizeStageBOccupancy(..., Name, Value) supports:
%       Path                - Nx3 array of reference path samples to overlay.
%       Resolution          - Map resolution (meters/cell, default 0.1).
%       SafetyMargin        - Extra inflation applied to each disc (default 0.15).
%       AxisLimits          - [xmin xmax ymin ymax] overrides auto bounds.
%       ShowGrid            - true to display grid lines (default false).
%       FigureHandle        - Existing figure to plot into.
%
%   Example:
%       env = gik9dof.environmentConfig();
%       gik9dof.debug.visualizeStageBOccupancy([-2 -2 0],[1.2 0.2 0], env.FloorDiscs);

arguments
    startBase (1,3) double
    goalBase (1,3) double
    floorDiscs (1,:) struct = struct([])
    options.Path double = []
    options.Resolution (1,1) double {mustBePositive} = 0.1
    options.SafetyMargin (1,1) double {mustBeNonnegative} = 0.15
    options.AxisLimits (1,4) double = []
    options.ShowGrid (1,1) logical = false
    options.FigureHandle = []
end

points = [startBase(1:2); goalBase(1:2)];
if ~isempty(options.Path)
    points = [points; options.Path(:,1:2)];
end
if ~isempty(floorDiscs)
    centers = reshape([floorDiscs.Center], 2, []).';
    points = [points; centers];
    discRadii = arrayfun(@(d) d.Radius + d.SafetyMargin + options.SafetyMargin, floorDiscs);
else
    discRadii = [];
end

if isempty(points)
    points = zeros(1,2);
end

minXY = min(points, [], 1);
maxXY = max(points, [], 1);

padding = max(1.0, max(discRadii, [], 'all', 'omitnan'));
if isnan(padding), padding = 1.0; end
minXY = minXY - padding;
maxXY = maxXY + padding;

width = maxXY(1) - minXY(1);
height = maxXY(2) - minXY(2);
width = max(width, options.Resolution);
height = max(height, options.Resolution);

cellsPerMeter = 1 / options.Resolution;
occMap = occupancyMap(width, height, cellsPerMeter);
occMap.GridLocationInWorld = minXY;
setOccupancy(occMap, zeros(occMap.GridSize));

for disc = reshape(floorDiscs,1,[])
    inflatedRadius = disc.Radius + disc.SafetyMargin + options.SafetyMargin;
    if inflatedRadius <= 0
        continue
    end
    markDiscOnMap(occMap, disc.Center, inflatedRadius);
end

if isempty(options.FigureHandle) || ~ishandle(options.FigureHandle)
    fig = figure('Name', 'Stage B Occupancy Map');
else
    fig = figure(options.FigureHandle);
    clf(fig);
end

ax = axes(fig);
show(occMap, 'Parent', ax);
hold(ax, 'on');
if options.ShowGrid
    grid(ax, 'on');
else
    grid(ax, 'off');
end

plot(ax, startBase(1), startBase(2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'StageB Start');
plot(ax, goalBase(1), goalBase(2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'StageB Goal');

if ~isempty(options.Path)
    plot(ax, options.Path(:,1), options.Path(:,2), 'c-', 'LineWidth', 1.2, 'DisplayName', 'Reference path');
end

if ~isempty(floorDiscs)
    theta = linspace(0, 2*pi, 200);
    for disc = reshape(floorDiscs,1,[])
        inflatedRadius = disc.Radius + disc.SafetyMargin + options.SafetyMargin;
        circleX = disc.Center(1) + inflatedRadius * cos(theta);
        circleY = disc.Center(2) + inflatedRadius * sin(theta);
        plot(ax, circleX, circleY, 'm--', 'LineWidth', 1.0, 'DisplayName', sprintf('%s (inflated)', char(disc.Name)));
    end
end

axis(ax, 'equal');
if ~isempty(options.AxisLimits)
    axis(ax, options.AxisLimits);
else
    axis(ax, 'tight');
end
title(ax, 'Stage B Hybrid A* Occupancy');
legend(ax, 'Location', 'bestoutside');
hold(ax, 'off');
end

function markDiscOnMap(map, center, radius)
if radius <= 0
    return
end
xRange = center(1) + [-radius radius];
yRange = center(2) + [-radius radius];

idxMin = world2grid(map, [xRange(1) yRange(1)]);
idxMax = world2grid(map, [xRange(2) yRange(2)]);
if any(~isfinite([idxMin idxMax]))
    return
end

rows = sort([idxMin(1), idxMax(1)]);
cols = sort([idxMin(2), idxMax(2)]);

rowLo = max(1, floor(rows(1)));
rowHi = min(map.GridSize(1), ceil(rows(2)));
colLo = max(1, floor(cols(1)));
colHi = min(map.GridSize(2), ceil(cols(2)));

if rowLo > rowHi || colLo > colHi
    return
end

[rowGrid, colGrid] = ndgrid(rowLo:rowHi, colLo:colHi);
worldPts = grid2world(map, [rowGrid(:), colGrid(:)]);
dx = worldPts(:,1) - center(1);
dy = worldPts(:,2) - center(2);
cellPadding = 0.5 / map.Resolution;
mask = (dx.^2 + dy.^2) <= (radius + cellPadding)^2;

if any(mask)
    setOccupancy(map, worldPts(mask,:), 1, "world");
end
end
