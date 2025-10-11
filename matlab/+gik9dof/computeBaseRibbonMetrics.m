function metrics = computeBaseRibbonMetrics(baseStates, options)
%COMPUTEBASERIBBON METRICS Compute curvature and smoothness metrics for SE(2) path.
%   metrics = computeBaseRibbonMetrics(baseStates) analyzes an Nx3 array of
%   SE(2) base poses [x, y, yaw] and returns curvature distribution, cusp
%   locations, and path smoothness metrics.
%
%   Inputs:
%       baseStates  - Nx3 array of [x, y, yaw] poses
%       options     - (optional) struct with fields:
%                     .CuspThreshold (default 0.01): min distance to consider cusp
%                     .CurvatureBins (default [0.5, 1.0, 2.0]): bin edges in [1/m]
%
%   Outputs:
%       metrics.curvature           - Nx1 array of signed curvature [1/m]
%       metrics.curvatureAbs        - Nx1 array of absolute curvature [1/m]
%       metrics.curvatureHistogram  - Struct with counts per bin
%                                     .low (<0.5), .medium (0.5-1.0),
%                                     .high (1.0-2.0), .veryHigh (>2.0)
%       metrics.cuspLocations       - Kx1 indices where cusps occur
%       metrics.cuspCount           - Total number of cusps
%       metrics.pathSmoothness      - Std dev of absolute curvature
%       metrics.maxCurvature        - Maximum absolute curvature [1/m]
%       metrics.meanCurvature       - Mean absolute curvature [1/m]
%
%   Example:
%       baseStates = [...];  % Nx3 path from planner
%       metrics = gik9dof.computeBaseRibbonMetrics(baseStates);
%       fprintf('Path smoothness: %.3f\n', metrics.pathSmoothness);
%       fprintf('Cusp count: %d\n', metrics.cuspCount);

arguments
    baseStates (:,3) double
    options.CuspThreshold (1,1) double = 0.01
    options.CurvatureBins (1,:) double = [0.5, 1.0, 2.0]
end

% Initialize output
metrics = struct();
N = size(baseStates, 1);

if N < 3
    % Not enough points for curvature estimation
    metrics.curvature = zeros(N, 1);
    metrics.curvatureAbs = zeros(N, 1);
    metrics.curvatureHistogram = struct('low', 0, 'medium', 0, 'high', 0, 'veryHigh', 0);
    metrics.cuspLocations = [];
    metrics.cuspCount = 0;
    metrics.pathSmoothness = 0;
    metrics.maxCurvature = 0;
    metrics.meanCurvature = 0;
    return;
end

% Extract positions and compute tangent directions
x = baseStates(:, 1);
y = baseStates(:, 2);

% Compute curvature using finite differences
% Formula: κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
dx = gradient(x);
dy = gradient(y);
ddx = gradient(dx);
ddy = gradient(dy);

speed = sqrt(dx.^2 + dy.^2);
curvature = zeros(N, 1);

% Avoid division by zero
validIdx = speed > 1e-6;
curvature(validIdx) = (dx(validIdx) .* ddy(validIdx) - dy(validIdx) .* ddx(validIdx)) ./ ...
                       (speed(validIdx).^3);

% Cap extreme values (numerical artifacts)
curvature = max(min(curvature, 10), -10);

curvatureAbs = abs(curvature);

% Histogram: <0.5, 0.5-1.0, 1.0-2.0, >2.0 [1/m]
bins = options.CurvatureBins;
histogram = struct();
histogram.low = sum(curvatureAbs < bins(1));
histogram.medium = sum(curvatureAbs >= bins(1) & curvatureAbs < bins(2));
histogram.high = sum(curvatureAbs >= bins(2) & curvatureAbs < bins(3));
histogram.veryHigh = sum(curvatureAbs >= bins(3));

% Detect cusps: sudden direction reversals (very small displacement)
cuspLocations = [];
for k = 2:N-1
    % Check if point k is a cusp (distance between k-1 and k+1 is small)
    disp_prev = norm(baseStates(k, 1:2) - baseStates(k-1, 1:2));
    disp_next = norm(baseStates(k+1, 1:2) - baseStates(k, 1:2));
    
    if disp_prev < options.CuspThreshold && disp_next < options.CuspThreshold
        cuspLocations = [cuspLocations; k]; %#ok<AGROW>
    end
end
cuspCount = numel(cuspLocations);

% Path smoothness: std dev of curvature (lower = smoother)
pathSmoothness = std(curvatureAbs);

% Summary statistics
maxCurvature = max(curvatureAbs);
meanCurvature = mean(curvatureAbs);

% Populate output struct
metrics.curvature = curvature;
metrics.curvatureAbs = curvatureAbs;
metrics.curvatureHistogram = histogram;
metrics.cuspLocations = cuspLocations;
metrics.cuspCount = cuspCount;
metrics.pathSmoothness = pathSmoothness;
metrics.maxCurvature = maxCurvature;
metrics.meanCurvature = meanCurvature;

end
