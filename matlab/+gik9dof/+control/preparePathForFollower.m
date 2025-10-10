function pathInfo = preparePathForFollower(pathStates, params, options)
%PREPAREPATHFORFOLLOWER Normalise path for chassis followers.
%   pathInfo = gik9dof.control.preparePathForFollower(pathStates, params)
%   returns a struct containing resampled poses, curvature, and diagnostics
%   derived from PATHSTATES. PARAMS must contain fields defining the
%   interpolation spacing and discontinuity thresholds (see chassis profile
%   YAML). Optional name-value options:
%       Validate (default true) - populate diagnostic warnings.
%
arguments
    pathStates double
    params struct
    options.Validate (1,1) logical = true
end

if isempty(pathStates)
    pathInfo = emptyPathInfo();
    return
end

if size(pathStates, 2) < 2
    error("gik9dof:preparePathForFollower:InvalidPath", ...
        "Path must contain at least [x y] columns.");
end

states = pathStates;
states = states(~any(isnan(states),2), :);
if isempty(states)
    pathInfo = emptyPathInfo();
    return
end

% Ensure yaw column present.
if size(states,2) < 3
    states(:,3) = computeHeading(states(:,1:2));
else
    states(:,3) = wrapToPi(states(:,3));
end

% Remove duplicate positions.
diffs = diff(states(:,1:2), 1, 1);
segLen = hypot(diffs(:,1), diffs(:,2));
keepMask = [true; segLen > 1e-6];
states = states(keepMask, :);
if size(states,1) < 2
    pathInfo = emptyPathInfo(states);
    return
end

params = fillDefaultParams(params);
discontinuityThreshold = params.discontinuity_threshold;

diffs = diff(states(:,1:2), 1, 1);
segLen = hypot(diffs(:,1), diffs(:,2));
cumulative = [0; cumsum(segLen)];
discontinuities = find(segLen > discontinuityThreshold);

segments = splitSegments(states, discontinuities);

resampledStates = [];
resampledCurvature = [];
resampledArc = [];
segmentMap = zeros(0,1);

offset = 0;
for sIdx = 1:numel(segments)
    segStates = segments{sIdx};
    [segRes, segCurv, segArc] = resampleSegment(segStates, params);
    if sIdx > 1
        segArc = segArc + resampledArc(end);
    end
    resampledStates = [resampledStates; segRes]; %#ok<AGROW>
    resampledCurvature = [resampledCurvature; segCurv]; %#ok<AGROW>
    resampledArc = [resampledArc; segArc]; %#ok<AGROW>
    segmentMap = [segmentMap; sIdx*ones(size(segRes,1),1)]; %#ok<AGROW>
    offset = offset + size(segRes,1);
end

distanceRemaining = resampledArc(end) - resampledArc;

pathInfo = struct();
pathInfo.States = resampledStates;
pathInfo.Curvature = resampledCurvature;
pathInfo.ArcLength = resampledArc;
pathInfo.DistanceRemaining = distanceRemaining;
pathInfo.SegmentIndex = segmentMap;
pathInfo.Diagnostics = struct( ...
    'discontinuityIdx', discontinuities(:), ...
    'originalPoints', size(pathStates,1), ...
    'resampledPoints', size(resampledStates,1), ...
    'totalLength', resampledArc(end));

if options.Validate
    pathInfo.Diagnostics.warnings = runValidationChecks(pathInfo, params);
else
    pathInfo.Diagnostics.warnings = string.empty(1,0);
end
end

function params = fillDefaultParams(params)
defaults = struct( ...
    'interp_spacing_min', 0.05, ...
    'interp_spacing_max', 0.20, ...
    'discontinuity_threshold', 0.40, ...
    'curvature_slowdown', struct('kappa_threshold', 1.0, 'vx_reduction', 0.6));
fields = fieldnames(defaults);
for k = 1:numel(fields)
    name = fields{k};
    if ~isfield(params, name) || isempty(params.(name))
        params.(name) = defaults.(name);
    elseif isstruct(defaults.(name)) && isstruct(params.(name))
        subFields = fieldnames(defaults.(name));
        for s = 1:numel(subFields)
            subName = subFields{s};
            if ~isfield(params.(name), subName) || isempty(params.(name).(subName))
                params.(name).(subName) = defaults.(name).(subName);
            end
        end
    end
end
end

function warningList = runValidationChecks(info, params)
warningList = string.empty(1,0);

if isempty(info.States)
    warningList(end+1) = "Path is empty after preprocessing.";
    return
end

if any(info.Diagnostics.discontinuityIdx)
    warningList(end+1) = sprintf("Detected %d discontinuities (>%.2fm).", ...
        numel(info.Diagnostics.discontinuityIdx), params.discontinuity_threshold);
end

maxCurv = max(abs(info.Curvature));
if maxCurv > 3.0
    warningList(end+1) = sprintf("High curvature encountered (|kappa|=%.2f 1/m).", maxCurv);
end
end

function segments = splitSegments(states, discontinuities)
breakIdx = unique([0; discontinuities(:); size(states,1)-1]);
segments = cell(numel(breakIdx),1);
for k = 1:numel(breakIdx)
    startIdx = breakIdx(k) + 1;
    if k < numel(breakIdx)
        endIdx = breakIdx(k+1) + 1;
    else
        endIdx = size(states,1);
    end
    segments{k} = states(startIdx:endIdx, :);
end
end

function [segRes, segCurv, segArc] = resampleSegment(states, params)
diffs = diff(states(:,1:2), 1, 1);
segLen = hypot(diffs(:,1), diffs(:,2));
arc = [0; cumsum(segLen)];
totalLen = arc(end);

if totalLen < params.interp_spacing_min
    segRes = states;
    segCurv = computeCurvature(states);
    segArc = arc;
    return
end

% Base coarse grid.
spacingMax = params.interp_spacing_max;
query = 0:spacingMax:totalLen;
if query(end) ~= totalLen
    query(end+1) = totalLen;
end

curvSamples = computeCurvature(states);
% Add dense samples around high-curvature regions.
curvatureThreshold = params.curvature_slowdown.kappa_threshold;
highIdx = find(abs(curvSamples) > curvatureThreshold);
for idx = highIdx(:)'
    sCenter = arc(idx);
    denseStart = max(0, sCenter - spacingMax);
    denseEnd = min(totalLen, sCenter + spacingMax);
    denseQuery = denseStart:params.interp_spacing_min:denseEnd;
    query = [query denseQuery]; %#ok<AGROW>
end
query = unique(sort(query));

xInterp = interp1(arc, states(:,1), query, 'linear');
yInterp = interp1(arc, states(:,2), query, 'linear');
yawInterp = unwrap(states(:,3));
yawInterp = interp1(arc, yawInterp, query, 'linear');

segRes = [xInterp(:), yInterp(:), wrapToPi(yawInterp(:))];
segCurv = computeCurvature(segRes);
segArc = query(:);
end

function heading = computeHeading(xy)
diffs = diff(xy,1,1);
heading = atan2([diffs(:,2); diffs(end,2)], [diffs(:,1); diffs(end,1)]);
heading = wrapToPi(heading);
end

function curvature = computeCurvature(states)
numPts = size(states,1);
if numPts < 3
    curvature = zeros(numPts,1);
    return
end
curvature = zeros(numPts,1);
x = states(:,1);
y = states(:,2);

for i = 2:numPts-1
    x1 = x(i-1); y1 = y(i-1);
    x2 = x(i);   y2 = y(i);
    x3 = x(i+1); y3 = y(i+1);
    % Using circle through three points.
    denom = ((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1));
    if abs(denom) < 1e-8
        curvature(i) = 0;
    else
        a = hypot(x2-x3, y2-y3);
        b = hypot(x1-x3, y1-y3);
        c = hypot(x1-x2, y1-y2);
        s = 0.5*(a+b+c);
        area = sqrt(max(s*(s-a)*(s-b)*(s-c), 0));
        if area < 1e-10
            curvature(i) = 0;
        else
            curvature(i) = 4*area/(a*b*c);
            if denom < 0
                curvature(i) = -curvature(i);
            end
        end
    end
end
curvature(1) = curvature(2);
curvature(end) = curvature(end-1);
end

function pathInfo = emptyPathInfo(states)
if nargin == 0
    states = zeros(0,3);
end
pathInfo = struct('States', states, ...
    'Curvature', zeros(size(states,1),1), ...
    'ArcLength', zeros(size(states,1),1), ...
    'DistanceRemaining', zeros(size(states,1),1), ...
    'SegmentIndex', zeros(size(states,1),1), ...
    'Diagnostics', struct('discontinuityIdx', [], 'originalPoints', 0, ...
        'resampledPoints', size(states,1), 'totalLength', 0, 'warnings', strings(1,0)));
end
