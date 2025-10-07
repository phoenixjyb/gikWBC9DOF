function [vx, wz, stateOut] = purePursuitVelocityController(...
    refX, refY, refTheta, refTime, ...
    estX, estY, estYaw, ...
    params, ...
    stateIn)
%PUREPURSUITVELOCITYCONTROLLER Pure Pursuit path following controller
%
% Inputs:
%   refX, refY, refTheta, refTime - Current position reference
%   estX, estY, estYaw - Current robot pose estimate
%   params - Controller parameters struct
%   stateIn - Controller state struct (optional, has default)
%
% Outputs:
%   vx - Forward velocity command (m/s)
%   wz - Angular velocity command (rad/s)
%   stateOut - Updated controller state
%
% Design:
%   - Path buffer: 30 waypoints max
%   - Update rate: 100 Hz (dt = 0.01s)
%   - Max speed: 1.5 m/s
%   - Adaptive lookahead: L = L_base + k_v * vx + k_t * dt_since_ref
%   - Interpolation: Linear between waypoints
%   - Continuous reference acceptance (no goal stop)

% Copyright 2025, Mobile Manipulator Team

%% Default state initialization
if nargin < 8 || isempty(stateIn)
    stateOut = initializeState();
else
    stateOut = stateIn;
end

%% Extract parameters
lookaheadBase = params.lookaheadBase;      % Base lookahead distance (m)
lookaheadVelGain = params.lookaheadVelGain; % Velocity-dependent gain
lookaheadTimeGain = params.lookaheadTimeGain; % Time-dependent gain
vxNominal = params.vxNominal;              % Nominal forward speed (m/s)
vxMax = params.vxMax;                      % Max forward speed (m/s)
wzMax = params.wzMax;                      % Max angular rate (rad/s)
track = params.track;                      % Wheel track width (m)
vwheelMax = params.vwheelMax;              % Max wheel speed (m/s)
waypointSpacing = params.waypointSpacing;  % Min spacing between waypoints (m)
pathBufferSize = params.pathBufferSize;    % Max waypoints to keep
goalTolerance = params.goalTolerance;      % Distance to consider waypoint reached (m)
interpSpacing = params.interpSpacing;      % Interpolation spacing (m)

%% Update path buffer with new reference
% Check if new waypoint is sufficiently different from last stored point
addNewWaypoint = false;

if stateOut.numWaypoints == 0
    % First waypoint
    addNewWaypoint = true;
else
    % Check distance from last waypoint
    lastIdx = stateOut.numWaypoints;
    dx = refX - stateOut.pathX(lastIdx);
    dy = refY - stateOut.pathY(lastIdx);
    distFromLast = sqrt(dx*dx + dy*dy);
    
    if distFromLast >= waypointSpacing
        addNewWaypoint = true;
    end
end

if addNewWaypoint
    if stateOut.numWaypoints < pathBufferSize
        % Add to buffer
        stateOut.numWaypoints = stateOut.numWaypoints + 1;
        idx = stateOut.numWaypoints;
    else
        % Buffer full, shift left and add at end
        for i = 1:(pathBufferSize-1)
            stateOut.pathX(i) = stateOut.pathX(i+1);
            stateOut.pathY(i) = stateOut.pathY(i+1);
            stateOut.pathTheta(i) = stateOut.pathTheta(i+1);
            stateOut.pathTime(i) = stateOut.pathTime(i+1);
        end
        idx = pathBufferSize;
    end
    
    stateOut.pathX(idx) = refX;
    stateOut.pathY(idx) = refY;
    stateOut.pathTheta(idx) = refTheta;
    stateOut.pathTime(idx) = refTime;
    stateOut.lastRefTime = refTime;
end

%% Remove passed waypoints
% Remove waypoints that are behind the robot
if stateOut.numWaypoints > 1
    numToRemove = 0;
    
    for i = 1:stateOut.numWaypoints
        % Transform waypoint to robot frame
        dx = stateOut.pathX(i) - estX;
        dy = stateOut.pathY(i) - estY;
        
        % Rotate to robot frame
        xRobot = dx * cos(-estYaw) - dy * sin(-estYaw);
        
        % If waypoint is behind robot (x < -goalTolerance), mark for removal
        if xRobot < -goalTolerance
            numToRemove = i;
        else
            break; % Stop at first waypoint ahead
        end
    end
    
    % Shift path buffer to remove passed waypoints
    if numToRemove > 0
        newNum = stateOut.numWaypoints - numToRemove;
        for i = 1:newNum
            stateOut.pathX(i) = stateOut.pathX(i + numToRemove);
            stateOut.pathY(i) = stateOut.pathY(i + numToRemove);
            stateOut.pathTheta(i) = stateOut.pathTheta(i + numToRemove);
            stateOut.pathTime(i) = stateOut.pathTime(i + numToRemove);
        end
        stateOut.numWaypoints = newNum;
    end
end

%% Handle insufficient waypoints
if stateOut.numWaypoints < 1
    % No path, stop
    vx = 0.0;
    wz = 0.0;
    return;
end

%% Calculate adaptive lookahead distance
% Time since last reference update
if stateOut.lastRefTime > 0
    dtSinceRef = refTime - stateOut.lastRefTime;
else
    dtSinceRef = 0.0;
end

% Current forward velocity estimate (use previous command as estimate)
vxCurrent = stateOut.prevVx;

% Adaptive lookahead: L = L_base + k_v * vx + k_t * dt
lookaheadDist = lookaheadBase + lookaheadVelGain * abs(vxCurrent) + ...
                lookaheadTimeGain * dtSinceRef;

% Clamp lookahead
lookaheadDist = max(0.3, min(2.5, lookaheadDist));

%% Interpolate path for smooth following
% Create interpolated points along the path
interpX = zeros(1, 200); % Pre-allocate max interpolated points
interpY = zeros(1, 200);
numInterpPoints = 0;

if stateOut.numWaypoints == 1
    % Single waypoint, just use it
    numInterpPoints = 1;
    interpX(1) = stateOut.pathX(1);
    interpY(1) = stateOut.pathY(1);
else
    % Interpolate between waypoints
    for i = 1:(stateOut.numWaypoints - 1)
        x1 = stateOut.pathX(i);
        y1 = stateOut.pathY(i);
        x2 = stateOut.pathX(i + 1);
        y2 = stateOut.pathY(i + 1);
        
        segmentDist = sqrt((x2 - x1)^2 + (y2 - y1)^2);
        
        if segmentDist > 1e-6
            numSegPoints = max(2, ceil(segmentDist / interpSpacing));
            
            for j = 0:(numSegPoints - 1)
                if numInterpPoints < 200
                    alpha = j / (numSegPoints - 1);
                    numInterpPoints = numInterpPoints + 1;
                    interpX(numInterpPoints) = x1 + alpha * (x2 - x1);
                    interpY(numInterpPoints) = y1 + alpha * (y2 - y1);
                end
            end
        end
    end
    
    % Add last waypoint if space available
    if numInterpPoints < 200
        numInterpPoints = numInterpPoints + 1;
        interpX(numInterpPoints) = stateOut.pathX(stateOut.numWaypoints);
        interpY(numInterpPoints) = stateOut.pathY(stateOut.numWaypoints);
    end
end

if numInterpPoints == 0
    % No interpolated points, stop
    vx = 0.0;
    wz = 0.0;
    return;
end

%% Find lookahead point
% Find closest point on path to robot
minDist = inf;
closestIdx = 1;

for i = 1:numInterpPoints
    dx = interpX(i) - estX;
    dy = interpY(i) - estY;
    dist = sqrt(dx*dx + dy*dy);
    
    if dist < minDist
        minDist = dist;
        closestIdx = i;
    end
end

% Search forward from closest point for lookahead distance along path
lookaheadX = interpX(numInterpPoints);
lookaheadY = interpY(numInterpPoints);
accDist = 0.0;

for i = closestIdx:numInterpPoints
    if i > closestIdx
        % Accumulate distance along path segments
        dxSeg = interpX(i) - interpX(i-1);
        dySeg = interpY(i) - interpY(i-1);
        segDist = sqrt(dxSeg*dxSeg + dySeg*dySeg);
        accDist = accDist + segDist;
    end
    
    % Check if accumulated distance along path >= lookahead distance
    if accDist >= lookaheadDist
        lookaheadX = interpX(i);
        lookaheadY = interpY(i);
        break;
    end
end

% If we didn't find a point at lookahead distance, use the farthest point
if accDist < lookaheadDist
    lookaheadX = interpX(numInterpPoints);
    lookaheadY = interpY(numInterpPoints);
end

%% Transform lookahead point to robot frame
dx = lookaheadX - estX;
dy = lookaheadY - estY;

% Rotate to robot frame
xLookahead = dx * cos(-estYaw) - dy * sin(-estYaw);
yLookahead = dx * sin(-estYaw) + dy * cos(-estYaw);

%% Pure Pursuit curvature calculation
% Curvature: κ = 2 * y / L²
% Where y is lateral distance to lookahead point
% L is lookahead distance

L_actual = sqrt(xLookahead*xLookahead + yLookahead*yLookahead);

if L_actual < 0.05
    % Too close to lookahead point
    curvature = 0.0;
else
    curvature = 2.0 * yLookahead / (L_actual * L_actual);
end

%% Calculate velocities
% Forward velocity: nominal speed
vx = vxNominal;

% Reduce speed in sharp turns
curvatureMag = abs(curvature);
if curvatureMag > 0.5
    % Sharp turn, reduce speed
    vx = vxNominal * 0.5;
elseif curvatureMag > 0.2
    % Moderate turn
    vx = vxNominal * 0.7;
end

% Clamp forward velocity
vx = max(0.0, min(vxMax, vx));

% Angular velocity from curvature
wz = vx * curvature;

% Clamp angular velocity
wz = max(-wzMax, min(wzMax, wz));

%% Apply wheel speed limits
% Differential drive kinematics:
% v_left = vx - (track/2) * wz
% v_right = vx + (track/2) * wz

vLeft = vx - (track / 2.0) * wz;
vRight = vx + (track / 2.0) * wz;

% Check wheel limits
if abs(vLeft) > vwheelMax || abs(vRight) > vwheelMax
    % Scale down to respect limits
    scaleFactor = vwheelMax / max(abs(vLeft), abs(vRight));
    vx = vx * scaleFactor;
    wz = wz * scaleFactor;
end

%% Update state
stateOut.prevVx = vx;
stateOut.prevWz = wz;
stateOut.prevPoseX = estX;
stateOut.prevPoseY = estY;
stateOut.prevPoseYaw = estYaw;

end

%% Helper function to initialize state
function state = initializeState()
    state = struct();
    
    % Path buffer (fixed size arrays for codegen)
    state.pathX = zeros(1, 30);
    state.pathY = zeros(1, 30);
    state.pathTheta = zeros(1, 30);
    state.pathTime = zeros(1, 30);
    state.numWaypoints = 0;
    
    % Previous commands
    state.prevVx = 0.0;
    state.prevWz = 0.0;
    
    % Previous pose
    state.prevPoseX = 0.0;
    state.prevPoseY = 0.0;
    state.prevPoseYaw = 0.0;
    
    % Timing
    state.lastRefTime = 0.0;
end
