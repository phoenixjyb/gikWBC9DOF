function metrics = evaluateCollisionIntrusion(baseStates, obstacleDiscs, safetyMargin)
%EVALUATECOLLISIONINTRUSION Check actual disc intrusion vs margin intrusion.
%   metrics = evaluateCollisionIntrusion(baseStates, obstacleDiscs, safetyMargin)
%   distinguishes between:
%   - Actual disc intrusion (penetration into physical obstacle)
%   - Margin intrusion (violates safety margin but not physical disc)
%
%   Inputs:
%       baseStates     - Nx3 array of [x, y, yaw] poses
%       obstacleDiscs  - Mx3 array of [x, y, radius] obstacle discs
%       safetyMargin   - Safety margin distance [m]
%
%   Outputs:
%       metrics.actualIntrusions      - Count of actual disc collisions
%       metrics.marginIntrusions      - Count of margin violations (not actual)
%       metrics.actualIntrusionIndices - Waypoint indices with actual intrusion
%       metrics.marginIntrusionIndices - Waypoint indices with margin intrusion
%       metrics.maxActualPenetration   - Deepest penetration into disc [m]
%       metrics.maxMarginViolation     - Deepest margin violation [m]
%       metrics.minClearance           - Closest approach to any obstacle [m]
%       metrics.clearanceProfile       - Nx1 min distance to obstacles [m]

arguments
    baseStates (:,3) double
    obstacleDiscs (:,3) double = []
    safetyMargin (1,1) double = 0.1
end

N = size(baseStates, 1);
M = size(obstacleDiscs, 1);

metrics = struct();

if M == 0 || N == 0
    % No obstacles or no path
    metrics.actualIntrusions = 0;
    metrics.marginIntrusions = 0;
    metrics.actualIntrusionIndices = [];
    metrics.marginIntrusionIndices = [];
    metrics.maxActualPenetration = 0;
    metrics.maxMarginViolation = 0;
    metrics.minClearance = Inf;
    metrics.clearanceProfile = Inf(N, 1);
    return;
end

% Compute clearance to all obstacles for each pose
clearanceProfile = zeros(N, 1);
actualIntrusionIndices = [];
marginIntrusionIndices = [];
maxActualPenetration = 0;
maxMarginViolation = 0;

for i = 1:N
    pose = baseStates(i, 1:2);
    
    % Find minimum distance to any obstacle
    distances = zeros(M, 1);
    for j = 1:M
        obstacleCenter = obstacleDiscs(j, 1:2);
        obstacleRadius = obstacleDiscs(j, 3);
        
        % Distance from pose to obstacle surface
        distToCenter = norm(pose - obstacleCenter);
        distToSurface = distToCenter - obstacleRadius;
        distances(j) = distToSurface;
    end
    
    minDist = min(distances);
    clearanceProfile(i) = minDist;
    
    % Check for intrusions
    if minDist < 0
        % Actual disc intrusion
        actualIntrusionIndices = [actualIntrusionIndices; i]; %#ok<AGROW>
        penetration = abs(minDist);
        maxActualPenetration = max(maxActualPenetration, penetration);
    elseif minDist < safetyMargin
        % Margin intrusion (close but not touching disc)
        marginIntrusionIndices = [marginIntrusionIndices; i]; %#ok<AGROW>
        violation = safetyMargin - minDist;
        maxMarginViolation = max(maxMarginViolation, violation);
    end
end

% Populate output
metrics.actualIntrusions = length(actualIntrusionIndices);
metrics.marginIntrusions = length(marginIntrusionIndices);
metrics.actualIntrusionIndices = actualIntrusionIndices;
metrics.marginIntrusionIndices = marginIntrusionIndices;
metrics.maxActualPenetration = maxActualPenetration;
metrics.maxMarginViolation = maxMarginViolation;
metrics.minClearance = min(clearanceProfile);
metrics.clearanceProfile = clearanceProfile;

end
