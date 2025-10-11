function [pathOut, info] = rsClothoidRefine(pathIn, params)
%RSCLOTHOIDREFINE Fit clothoid splines to an SE(2) path (codegen-ready).
%   [pathOut, info] = gik9dof.control.rsClothoidRefine(pathIn, params)
%   splits PATHIN at gear changes, fits a referencePathFrenet segment to
%   each forward/reverse portion, and returns the smoothed poses PATHOUT.
%   INFO describes the fitting process (lengths, curvature, segment counts).
%
%   params fields (defaults shown):
%       discretizationDistance  - Output spacing along the clothoid (0.05 m)
%       maxNumWaypoints         - Max waypoints for referencePathFrenet (0 => auto)
%
%#codegen

arguments
    pathIn (:,3) double
    params struct
end

nStates = size(pathIn, 1);
if nStates < 2
    pathOut = pathIn;
    info = defaultInfoStruct(params, false, 0.0, 0.0, 0, 0, 0);
    return
end

    disc = getFieldWithDefault(params, 'discretizationDistance', 0.08);
if ~isfinite(disc) || disc <= 0
    disc = 0.05;
end
maxNumWaypointsParam = getFieldWithDefault(params, 'maxNumWaypoints', int32(0));

dirs = estimateDirections(pathIn);
segmentBounds = computeSegmentBounds(dirs);

pathOut = zeros(0,3);
pathLengthOriginal = sum(vecnorm(diff(pathIn(:,1:2).'), 2, 1));
pathLengthSmoothed = 0.0;
maxCurvature = 0.0;
segmentCount = int32(0);
fittedSegments = int32(0);
appliedFlag = false;

for segIdx = 1:size(segmentBounds,1)
    idxStart = segmentBounds(segIdx,1);
    idxEnd = segmentBounds(segIdx,2);
    if idxEnd <= idxStart
        continue
    end

    segmentCount = segmentCount + int32(1);
    xySegment = pathIn(idxStart:idxEnd, 1:2);
    segmentOriginalLength = sum(vecnorm(diff(xySegment.'), 2, 1));

    % Auto-select waypoint cap if not provided.
    if maxNumWaypointsParam <= 0
        maxNumWaypointsLocal = max(2, size(xySegment,1) * 4);
    else
        maxNumWaypointsLocal = double(maxNumWaypointsParam);
    end

    fitSucceeded = false;
    try
        ref = referencePathFrenet(xySegment, ...
            'DiscretizationDistance', disc, ...
            'MaxNumWaypoints', maxNumWaypointsLocal);
        numSamples = max(2, ceil(ref.PathLength / disc) + 1);
        s = linspace(0, ref.PathLength, numSamples).';
        xySmooth = position(ref, s);
        thetaSmooth = wrapToPi(tangentAngle(ref, s));
        segmentPath = [xySmooth, thetaSmooth];
        curv = curvature(ref, s);
        maxCurvature = max(maxCurvature, max(abs(curv)));
        pathLengthSmoothed = pathLengthSmoothed + ref.PathLength;
        fitSucceeded = true;
    catch
        % Fallback to original segment if fitting fails.
        segmentPath = pathIn(idxStart:idxEnd, :);
        pathLengthSmoothed = pathLengthSmoothed + segmentOriginalLength;
    end

    % Preserve end orientations from the seed path.
    segmentPath(1,3) = wrapToPi(pathIn(idxStart,3));
    segmentPath(end,3) = wrapToPi(pathIn(idxEnd,3));

    if isempty(pathOut)
        pathOut = segmentPath;
    else
        pathOut = [pathOut; segmentPath(2:end,:)]; %#ok<AGROW>
    end

    if fitSucceeded
        fittedSegments = fittedSegments + int32(1);
        appliedFlag = true;
    end
end

if isempty(pathOut) || size(pathOut,1) < 2
    pathOut = pathIn;
    pathLengthSmoothed = pathLengthOriginal;
    appliedFlag = false;
end

pathOut(1,3) = wrapToPi(pathIn(1,3));
pathOut(end,3) = wrapToPi(pathIn(end,3));

gearChanges = int32(max(size(segmentBounds,1) - 1, 0));

info = defaultInfoStruct(params, appliedFlag, pathLengthOriginal, pathLengthSmoothed, ...
    segmentCount, fittedSegments, gearChanges);
info.maxCurvature = maxCurvature;
info.discretizationDistance = disc;
info.segmentBounds = segmentBounds;

end

% -------------------------------------------------------------------------
function info = defaultInfoStruct(params, appliedFlag, lengthOriginal, lengthSmoothed, ...
    segmentCount, fittedSegments, gearChanges)
info = struct( ...
    'applied', logical(appliedFlag), ...
    'segmentCount', segmentCount, ...
    'fittedSegments', fittedSegments, ...
    'gearChanges', gearChanges, ...
    'pathLengthOriginal', lengthOriginal, ...
    'pathLengthSmoothed', lengthSmoothed, ...
    'maxCurvature', 0.0, ...
    'discretizationDistance', 0.0, ...
    'segmentBounds', zeros(0,2), ...
    'params', params);
end

function dirs = estimateDirections(pathStates)
nSteps = size(pathStates,1) - 1;
dirs = zeros(nSteps,1);
for k = 1:nSteps
    theta = pathStates(k,3);
    delta = pathStates(k+1,1:2) - pathStates(k,1:2);
    forward = delta(1) * cos(theta) + delta(2) * sin(theta);
    if abs(forward) < 1e-6
        dirs(k) = 0;
    else
        dirs(k) = sign(forward);
    end
end

for k = 1:nSteps
    if dirs(k) == 0
        if k == 1
            dirs(k) = 1;
        else
            dirs(k) = dirs(k-1);
        end
    end
end

if nSteps >= 2
    for k = nSteps:-1:1
        if dirs(k) == 0
            if k < nSteps
                dirs(k) = dirs(k+1);
            else
                dirs(k) = 1;
            end
        end
    end
end
end

function bounds = computeSegmentBounds(dirs)
nSteps = numel(dirs);
if nSteps == 0
    bounds = [1, 1];
    return
end

changes = find(abs(diff(dirs)) > 0) + 1;
bounds = [1; changes; nSteps + 1];
bounds = unique(bounds);
if bounds(end) ~= nSteps + 1
    bounds = [bounds; nSteps + 1];
end

segmentCount = numel(bounds) - 1;
segments = zeros(segmentCount, 2);
for idx = 1:segmentCount
    segments(idx,1) = bounds(idx);
    segments(idx,2) = bounds(idx+1);
end

segments(:,1) = max(segments(:,1), 1);
segments(:,2) = min(segments(:,2), nSteps + 1);
segments(:,2) = max(segments(:,2), segments(:,1) + 1);

bounds = segments;
end

function val = getFieldWithDefault(s, name, defaultValue)
if isfield(s, name) && ~isempty(s.(name))
    val = s.(name);
else
    val = defaultValue;
end
end
