function [pathOut, info] = rsRefinePath(mapData, resolution, pathIn, params)
%RSREFINEPATH Reeds-Shepp shortcutting with cusp penalty (codegen-ready).
%   [pathOut, info] = gik9dof.control.rsRefinePath(mapData, resolution, pathIn, params)
%   refines the seed path PATHIN using random RS shortcuts that are collision
%   checked against MAPDATA.  INFO returns basic statistics describing the
%   refinement (path length, cusp count, number of accepted shortcuts, etc.).
%
%   mapData        - logical/double occupancy grid (1 = occupied). Size MxN.
%   resolution     - cells per meter scalar.
%   pathIn         - Nx3 [x y yaw] poses (N >= 2).
%   params struct fields (defaults shown):
%       Rmin                - min turning radius (meters)              (required > 0)
%       reverseCost         - cost multiplier for reverse motions      (default 2.0)
%       inflationRadius     - inflate map (meters)                     (default 0.0)
%       validationDistance  - validator sampling step (meters)         (default 0.05)
%       step                - sampling step along RS segment (meters)  (default 0.10)
%       iters               - maximum iterations (int32)               (default 200)
%       maxSpan             - maximum index span (int32)               (default 80)
%       lambdaCusp          - penalty per cusp (meters)                (default 1.0)
%       seed                - optional uint32 RNG seed (default 0)
%
%#codegen

arguments
    mapData {mustBeNumeric, mustBeNonempty}
    resolution (1,1) double {mustBePositive}
    pathIn (:,3) double
    params struct
end

N = size(pathIn,1);
if N < 2
    pathOut = pathIn;
    if nargout > 1
        info = defaultInfoStruct();
    end
    return
end

if ~isfield(params, 'Rmin') || params.Rmin <= 0
    error('gik9dof:rsRefinePath:InvalidRmin', ...
        'params.Rmin must be provided and strictly positive.');
end

if isfield(params, 'seed')
    rng(double(params.seed), 'twister');
else
    rng('shuffle');
end

map = binaryOccupancyMap(mapData, resolution);
inflationRadius = getFieldWithDefault(params, 'inflationRadius', 0.0);
if inflationRadius > 0
    inflate(map, inflationRadius);
end

ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
sv = validatorOccupancyMap(ss, 'Map', map);
sv.ValidationDistance = getFieldWithDefault(params, 'validationDistance', 0.05);

conn = reedsSheppConnection('MinTurningRadius', params.Rmin);
conn.ReverseCost = getFieldWithDefault(params, 'reverseCost', 2.0);

maxIter = int32(getFieldWithDefault(params, 'iters', int32(200)));
maxSpan = int32(getFieldWithDefault(params, 'maxSpan', int32(80)));
lambdaCusp = getFieldWithDefault(params, 'lambdaCusp', 1.0);
step = getFieldWithDefault(params, 'step', 0.1);

pathOut = pathIn;
N = size(pathOut,1);
improvementCount = int32(0);
initialSampleCount = int32(N);
lastIter = int32(0);

initialMetrics = computePathMetrics(conn, pathOut, lambdaCusp);

for iter = int32(1):maxIter
    if N < 3
        lastIter = int32(max(0, double(iter) - 1));
        break
    end
    lastIter = iter;

    i = randi([1, N-2]);
    spanMax = int32(min(double(maxSpan), N - i));
    if spanMax < 2
        continue
    end

    j = i + randi([2, double(spanMax)]);

    segNew = bestRS(conn, pathOut(i,:), pathOut(j,:));
    if isempty(segNew)
        continue
    end

    if ~isRSConnectionValid(sv, segNew, step)
        continue
    end

    [oldCost, feasibleOld] = evaluatePathSegment(conn, sv, pathOut, i, j, lambdaCusp, step);
    if ~feasibleOld
        continue
    end

    newCost = segNew.Length + lambdaCusp * countCusps(segNew);

    if newCost + 1e-9 < oldCost
        posesNew = interpolateRS(segNew, step);
        if size(posesNew,1) >= 2
            pathOut = [pathOut(1:i,:); posesNew(2:end-1,:); pathOut(j:end,:)]; %#ok<AGROW>
        else
            pathOut = [pathOut(1:i,:); pathOut(j:end,:)]; %#ok<AGROW>
        end
        N = size(pathOut,1);
        improvementCount = improvementCount + 1;
    end
end

finalMetrics = computePathMetrics(conn, pathOut, lambdaCusp);
finalSampleCount = int32(size(pathOut,1));

if nargout > 1
    info = struct( ...
        'initialLength', initialMetrics.length, ...
        'finalLength', finalMetrics.length, ...
        'initialCost', initialMetrics.cost, ...
        'finalCost', finalMetrics.cost, ...
        'initialCusps', initialMetrics.cusps, ...
        'finalCusps', finalMetrics.cusps, ...
        'initialSamples', initialSampleCount, ...
        'finalSamples', finalSampleCount, ...
        'iterationsPlanned', maxIter, ...
        'iterationsExecuted', lastIter, ...
        'improvements', improvementCount, ...
        'lambdaCusp', lambdaCusp, ...
        'params', params);
end
end

% -------------------------------------------------------------------------
function seg = bestRS(conn, poseA, poseB)
seg = [];
try
    [segments, costs] = connect(conn, poseA, poseB);
catch
    segments = {};
    costs = [];
end
if isempty(segments)
    return
end
[~, idx] = min(costs);
seg = segments{idx};
end

function tf = isRSConnectionValid(validator, seg, step)
L = seg.Length;
numSamples = max(2, ceil(L / step) + 1);
s = linspace(0, L, numSamples);
poses = interpolate(seg, s);
tf = true;
for idx = 1:(size(poses,1)-1)
    [ok, ~] = isMotionValid(validator, poses(idx,:), poses(idx+1,:));
    if ~ok
        tf = false;
        return
    end
end
end

function poses = interpolateRS(seg, step)
L = seg.Length;
numSamples = max(2, ceil(L / step) + 1);
s = linspace(0, L, numSamples);
poses = interpolate(seg, s);
end

function [cost, feasible] = evaluatePathSegment(conn, validator, path, i, j, lambdaCusp, step)
cost = 0.0;
feasible = true;
for k = i:(j-1)
    seg = bestRS(conn, path(k,:), path(k+1,:));
    if isempty(seg) || ~isRSConnectionValid(validator, seg, step)
        feasible = false;
        cost = inf;
        return
    end
    cost = cost + seg.Length + lambdaCusp * countCusps(seg);
end
end

function metrics = computePathMetrics(conn, path, lambdaCusp)
lengthSum = 0.0;
cuspSum = 0;
for k = 1:(size(path,1)-1)
    seg = bestRS(conn, path(k,:), path(k+1,:));
    if isempty(seg)
        % fallback: straight-line estimate
        lengthSum = lengthSum + norm(path(k+1,1:2) - path(k,1:2));
    else
        lengthSum = lengthSum + seg.Length;
        cuspSum = cuspSum + countCusps(seg);
    end
end
metrics = struct('length', lengthSum, ...
    'cusps', cuspSum, ...
    'cost', lengthSum + lambdaCusp * double(cuspSum));
end

function c = countCusps(seg)
dirs = seg.MotionDirections(:).';
types = seg.MotionTypes;
validCount = 0;
for t = 1:numel(types)
    if types{t} ~= "N"
        validCount = validCount + 1;
    end
end
dirs = dirs(1:max(1, validCount));
if numel(dirs) < 2
    c = 0;
else
    c = sum(abs(diff(sign(dirs))) > 0);
end
end

function val = getFieldWithDefault(s, name, defaultValue)
if isfield(s, name) && ~isempty(s.(name))
    val = s.(name);
else
    val = defaultValue;
end
end

function info = defaultInfoStruct()
info = struct('initialLength', 0, 'finalLength', 0, ...
    'initialCost', 0, 'finalCost', 0, ...
    'initialCusps', 0, 'finalCusps', 0, ...
    'initialSamples', int32(0), 'finalSamples', int32(0), ...
    'iterationsPlanned', int32(0), 'iterationsExecuted', int32(0), ...
    'improvements', int32(0), ...
    'lambdaCusp', 0, 'params', struct());
end
