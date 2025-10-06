function [states, count, commands, commandCount] = stageBPlanPath(startState, goalState, floorCenters, floorRadii, floorMargins, floorCount, params)
%STAGEBPLANPATH Plan base-only trajectory via Hybrid A* primitives and derive commands.
%#codegen

arguments
    startState (3,1) double
    goalState (3,1) double
    floorCenters (:,2) double
    floorRadii (:,1) double
    floorMargins (:,1) double
    floorCount (1,1) double
    params struct
end

MAX_STAGE_B = int32(200);
MAX_GRID = int32(256);
MAX_THETA = int32(72);
MAX_NODES = int32(12000);
MAX_CELLS = int32(double(MAX_GRID) * double(MAX_GRID) * double(MAX_THETA));

states = zeros(3, MAX_STAGE_B);
commands = zeros(3, MAX_STAGE_B);
count = int32(0);
commandCount = int32(0);

resolution = getFieldOr(params, 'resolution', 0.1);
if ~(isfinite(resolution) && resolution > 0)
    resolution = 0.1;
end
safetyMargin = getFieldOr(params, 'safetyMargin', 0.15);
if ~(isfinite(safetyMargin) && safetyMargin >= 0)
    safetyMargin = 0.15;
end
minTurningRadius = getFieldOr(params, 'minTurningRadius', getFieldOr(params, 'hybridMinTurningRadius', 0.5));
if ~(isfinite(minTurningRadius) && minTurningRadius > 0.05)
    minTurningRadius = 0.5;
end
motionPrimitiveLength = getFieldOr(params, 'motionPrimitiveLength', getFieldOr(params, 'hybridMotionPrimitiveLength', 0.5));
if ~(isfinite(motionPrimitiveLength) && motionPrimitiveLength > 0.05)
    motionPrimitiveLength = 0.5;
end
rotationStep = getFieldOr(params, 'rotationStep', getFieldOr(params, 'hybridRotationStep', 0.35));
if ~(isfinite(rotationStep) && rotationStep > 0.05)
    rotationStep = 0.35;
end
maxPathLength = getFieldOr(params, 'maxPathLength', 25.0);
if ~(isfinite(maxPathLength) && maxPathLength > 0)
    maxPathLength = 25.0;
end
maxWaypoints = int32(round(getFieldOr(params, 'maxWaypoints', double(MAX_STAGE_B))));
if maxWaypoints < 2
    maxWaypoints = int32(2);
elseif maxWaypoints > MAX_STAGE_B
    maxWaypoints = MAX_STAGE_B;
end

rateHz = getFieldOr(params, 'rateHz', getFieldOr(params, 'stageRateHz', 100.0));
if ~(isfinite(rateHz) && rateHz > 0)
    rateHz = 100.0;
end
sampleTime = getFieldOr(params, 'sampleTime', 1.0 / rateHz);
if ~(isfinite(sampleTime) && sampleTime > 0)
    sampleTime = 1.0 / rateHz;
end
maxLinearSpeed = getFieldOr(params, 'maxLinearSpeed', getFieldOr(params, 'maxLinearVel', 1.5));
if ~(isfinite(maxLinearSpeed) && maxLinearSpeed > 0)
    maxLinearSpeed = 1.5;
end
maxYawRate = getFieldOr(params, 'maxYawRate', getFieldOr(params, 'maxAngularVel', 3.0));
if ~(isfinite(maxYawRate) && maxYawRate > 0)
    maxYawRate = 3.0;
end
thetaBins = int32(round(getFieldOr(params, 'headingBins', getFieldOr(params, 'hybridHeadingBins', double(MAX_THETA)))));
if thetaBins < 8
    thetaBins = int32(8);
elseif thetaBins > MAX_THETA
    thetaBins = MAX_THETA;
end

lookaheadDistance = getFieldOr(params, 'ppLookahead', 0.6);
if ~(isfinite(lookaheadDistance) && lookaheadDistance > 0)
    lookaheadDistance = 0.6;
end
desiredSpeed = getFieldOr(params, 'ppDesiredSpeed', maxLinearSpeed);
if ~(isfinite(desiredSpeed) && desiredSpeed > 0)
    desiredSpeed = maxLinearSpeed;
end
goalRadius = getFieldOr(params, 'ppGoalRadius', 0.15);
if ~(isfinite(goalRadius) && goalRadius > 0)
    goalRadius = 0.15;
end
closeOnHeading = getFieldOr(params, 'ppCloseOnHeading', 0) ~= 0;
reverseAllowed = getFieldOr(params, 'ppReverseAllowed', 0) ~= 0;

minXY = [startState(1), startState(2)];
maxXY = minXY;
goalXY = [goalState(1), goalState(2)];
if goalXY(1) < minXY(1)
    minXY(1) = goalXY(1);
elseif goalXY(1) > maxXY(1)
    maxXY(1) = goalXY(1);
end
if goalXY(2) < minXY(2)
    minXY(2) = goalXY(2);
elseif goalXY(2) > maxXY(2)
    maxXY(2) = goalXY(2);
end
numDiscCandidates = int32(size(floorCenters, 1));
numDiscs = int32(min(double(numDiscCandidates), max(0.0, floorCount)));
for k = 1:double(numDiscs)
    cx = floorCenters(k,1);
    cy = floorCenters(k,2);
    if cx < minXY(1)
        minXY(1) = cx;
    elseif cx > maxXY(1)
        maxXY(1) = cx;
    end
    if cy < minXY(2)
        minXY(2) = cy;
    elseif cy > maxXY(2)
        maxXY(2) = cy;
    end
end
padding = 2.0 + safetyMargin;
minXY = minXY - padding;
maxXY = maxXY + padding;
width = maxXY(1) - minXY(1);
height = maxXY(2) - minXY(2);
if width < resolution
    width = resolution;
end
if height < resolution
    height = resolution;
end
nx = int32(min(MAX_GRID, max(2, ceil(width / resolution) + 6)));
ny = int32(min(MAX_GRID, max(2, ceil(height / resolution) + 6)));
maxXY(1) = minXY(1) + double(nx) * resolution;
maxXY(2) = minXY(2) + double(ny) * resolution;

if collisionAtPoint(startState(1), startState(2), floorCenters, floorRadii, floorMargins, numDiscs, safetyMargin)
    return;
end
if collisionAtPoint(goalState(1), goalState(2), floorCenters, floorRadii, floorMargins, numDiscs, safetyMargin)
    return;
end

[startRow, startCol, validStart] = worldToGrid(startState(1), startState(2), minXY, resolution, ny, nx);
[goalRow, goalCol, validGoal] = worldToGrid(goalState(1), goalState(2), minXY, resolution, ny, nx);
if ~(validStart && validGoal)
    return;
end

startThetaIdx = yawToIndex(startState(3), thetaBins);
goalThetaIdx = yawToIndex(goalState(3), thetaBins);
startCell = gridLinearIndex(startRow, startCol, startThetaIdx, ny, nx);
goalCell = gridLinearIndex(goalRow, goalCol, goalThetaIdx, ny, nx);
if startCell < 1 || startCell > MAX_CELLS || goalCell < 1 || goalCell > MAX_CELLS
    return;
end

gridToNode = zeros(double(MAX_CELLS), 1, 'int32');
nodeX = zeros(double(MAX_NODES), 1);
nodeY = zeros(double(MAX_NODES), 1);
nodeYaw = zeros(double(MAX_NODES), 1);
gCost = inf(double(MAX_NODES), 1);
fCost = inf(double(MAX_NODES), 1);
parent = zeros(double(MAX_NODES), 1, 'int32');
nodeCell = zeros(double(MAX_NODES), 1, 'int32');
openSet = false(double(MAX_NODES), 1);
closedSet = false(double(MAX_NODES), 1);

nodeCount = int32(1);
nodeX(1) = startState(1);
nodeY(1) = startState(2);
nodeYaw(1) = wrapToPi(startState(3));
gCost(1) = 0;
fCost(1) = heuristicCost(startState(1), startState(2), startState(3), goalState, minTurningRadius);
parent(1) = int32(0);
nodeCell(1) = startCell;
gridToNode(startCell) = int32(1);
openSet(1) = true;

motionPrims = buildMotionPrimitives(motionPrimitiveLength, minTurningRadius, rotationStep);
sampleStep = min(0.5 * motionPrimitiveLength, max(0.05, 0.25 * resolution));
goalPosTol = max(resolution * 1.5, 0.1);
goalYawTol = max(pi / double(thetaBins), 0.15);

goalNode = int32(0);
for iter = 1:double(MAX_NODES)
    currentNode = selectBestOpen(openSet, fCost, nodeCount);
    if currentNode == 0
        break;
    end
    if reachedGoal(nodeX(currentNode), nodeY(currentNode), nodeYaw(currentNode), goalState, goalPosTol, goalYawTol)
        goalNode = currentNode;
        break;
    end
    openSet(currentNode) = false;
    closedSet(currentNode) = true;

    xCurr = nodeX(currentNode);
    yCurr = nodeY(currentNode);
    yawCurr = nodeYaw(currentNode);
    gCurr = gCost(currentNode);

    for primIdx = 1:size(motionPrims, 1)
        deltaS = motionPrims(primIdx, 1);
        curvature = motionPrims(primIdx, 2);
        deltaYaw = motionPrims(primIdx, 3);
        [xNext, yNext, yawNext, travelDist, validMove] = propagatePrimitive(xCurr, yCurr, yawCurr, deltaS, curvature, deltaYaw, ...
            minXY, maxXY, resolution, sampleStep, floorCenters, floorRadii, floorMargins, numDiscs, safetyMargin, minTurningRadius);
        if ~validMove
            continue;
        end
        newCost = gCurr + travelDist;
        if newCost > maxPathLength + 1e-6
            continue;
        end
        [nRow, nCol, validCell] = worldToGrid(xNext, yNext, minXY, resolution, ny, nx);
        if ~validCell
            continue;
        end
        thetaIdx = yawToIndex(yawNext, thetaBins);
        cellIndex = gridLinearIndex(nRow, nCol, thetaIdx, ny, nx);
        if cellIndex < 1 || cellIndex > MAX_CELLS
            continue;
        end
        if gridToNode(cellIndex) == 0
            if nodeCount >= MAX_NODES
                continue;
            end
            nodeCount = nodeCount + int32(1);
            nodeId = nodeCount;
            gridToNode(cellIndex) = nodeId;
            nodeCell(nodeId) = cellIndex;
        else
            nodeId = gridToNode(cellIndex);
            if closedSet(nodeId) && newCost >= gCost(nodeId) - 1e-9
                continue;
            end
        end

        if ~openSet(nodeId) || newCost < gCost(nodeId)
            nodeX(nodeId) = xNext;
            nodeY(nodeId) = yNext;
            nodeYaw(nodeId) = yawNext;
            gCost(nodeId) = newCost;
            fCost(nodeId) = newCost + heuristicCost(xNext, yNext, yawNext, goalState, minTurningRadius);
            parent(nodeId) = currentNode;
            openSet(nodeId) = true;
            closedSet(nodeId) = false;
        end
    end
end

if goalNode == 0
    return;
end

pathNodes = zeros(double(MAX_NODES), 1, 'int32');
pathCount = int32(0);
nodePtr = goalNode;
while nodePtr ~= 0 && pathCount < MAX_NODES
    pathCount = pathCount + int32(1);
    pathNodes(pathCount) = nodePtr;
    nodePtr = parent(nodePtr);
end
if nodePtr ~= 0
    return;
end

for k = 1:floor(double(pathCount) / 2)
    tmp = pathNodes(k);
    pathNodes(k) = pathNodes(pathCount + 1 - k);
    pathNodes(pathCount + 1 - k) = tmp;
end

densifyLinStep = min(0.5 * motionPrimitiveLength, 0.25);
densifyYawStep = min(rotationStep, 0.5);
stateCount = int32(0);
prevState = zeros(3,1);
for idx = 1:double(pathCount)
    nodeId = pathNodes(idx);
    x = nodeX(nodeId);
    y = nodeY(nodeId);
    yaw = wrapToPi(nodeYaw(nodeId));
    if stateCount == 0
        stateCount = int32(1);
        states(:,1) = [x; y; yaw];
        prevState = [x; y; yaw];
        continue;
    end
    dx = x - prevState(1);
    dy = y - prevState(2);
    linearDist = hypot(dx, dy);
    yawDiff = wrapToPi(yaw - prevState(3));
    segSteps = int32(max(1.0, max(ceil(linearDist / max(densifyLinStep, 1e-6)), ceil(abs(yawDiff) / max(densifyYawStep, 1e-6)))));
    for s = 1:double(segSteps-1)
        if stateCount >= maxWaypoints
            break;
        end
        tau = double(s) / double(segSteps);
        interpX = prevState(1) + dx * tau;
        interpY = prevState(2) + dy * tau;
        interpYaw = wrapToPi(prevState(3) + yawDiff * tau);
        stateCount = stateCount + int32(1);
        states(:, stateCount) = [interpX; interpY; interpYaw];
    end
    if stateCount >= maxWaypoints
        break;
    end
    stateCount = stateCount + int32(1);
    states(:, stateCount) = [x; y; yaw];
    prevState = [x; y; yaw];
end

if stateCount == 0
    return;
end

if stateCount > MAX_STAGE_B
    stateCount = MAX_STAGE_B;
end

states(3, stateCount) = wrapToPi(goalState(3));
count = stateCount;

[commands, commandCount] = computePurePursuitCommands(states, count, sampleTime, lookaheadDistance, desiredSpeed, maxYawRate, goalRadius, closeOnHeading, reverseAllowed);

end

function [cmds, cmdCount] = computePurePursuitCommands(states, stateCount, sampleTime, lookahead, desiredSpeed, maxYawRate, goalRadius, closeOnHeading, reverseAllowed)
maxCommands = size(states, 2);
cmds = zeros(3, maxCommands);
if stateCount <= 0
    cmdCount = int32(0);
    return;
end

path = states(:,1:stateCount)';
cumDist = zeros(stateCount, 1);
for i = 2:stateCount
    dx = path(i,1) - path(i-1,1);
    dy = path(i,2) - path(i-1,2);
    cumDist(i) = cumDist(i-1) + hypot(dx, dy);
end

lastIdx = int32(1);
for i = 1:stateCount
    pose = states(:,i);
    [v, w, lastIdx] = purePursuitStep(pose, path, cumDist, lastIdx, lookahead, desiredSpeed, maxYawRate, goalRadius, closeOnHeading, reverseAllowed);
    cmds(1,i) = v;
    cmds(2,i) = w;
    cmds(3,i) = double(i-1) * sampleTime;
end
cmdCount = int32(stateCount);
end

function [v, w, nextIdx] = purePursuitStep(pose, path, cumDist, lastIdx, lookahead, desiredSpeed, maxYawRate, goalRadius, closeOnHeading, reverseAllowed)
n = size(path, 1);
position = pose(1:2)';
theta = pose(3);

nearestIdx = int32(1);
nearestDist = inf;
startIdx = max(1, min(n, lastIdx));
for idx = startIdx:n
    dx = path(idx,1) - position(1);
    dy = path(idx,2) - position(2);
    d = hypot(dx, dy);
    if d < nearestDist
        nearestDist = d;
        nearestIdx = int32(idx);
    end
end

lookIdx = nearestIdx;
while lookIdx < n && (cumDist(lookIdx+1) - cumDist(nearestIdx)) < lookahead
    lookIdx = lookIdx + 1;
end

nextIdx = lookIdx;
target = path(lookIdx,1:2);
dx = target(1) - position(1);
dy = target(2) - position(2);
cosT = cos(theta);
sinT = sin(theta);
xLook =  cosT * dx + sinT * dy;
yLook = -sinT * dx + cosT * dy;
ld = max(lookahead, 1e-3);
curvature = (2.0 * yLook) / (ld * ld);

v = desiredSpeed;
if reverseAllowed && xLook < 0
    v = -v;
end

w = curvature * v;
if closeOnHeading && lookIdx == n
    desiredHeading = path(n,3);
    headingError = wrapToPi(desiredHeading - theta);
    w = w + 0.5 * headingError;
end

if nearestDist <= goalRadius && lookIdx == n
    v = 0.0;
    w = 0.0;
end

w = max(-maxYawRate, min(maxYawRate, w));
end

function idx = gridLinearIndex(row, col, thetaIdx, ny, nx)
idx = ((thetaIdx - 1) * ny + (row - 1)) * nx + col;
end

function [row, col, valid] = worldToGrid(x, y, minXY, resolution, ny, nx)
col = int32(floor((x - minXY(1)) / resolution) + 1);
row = int32(floor((y - minXY(2)) / resolution) + 1);
valid = (row >= 1 && row <= ny && col >= 1 && col <= nx);
if ~valid
    row = int32(1);
    col = int32(1);
end
end

function thetaIdx = yawToIndex(yaw, thetaBins)
angle = wrapToPi(yaw);
fraction = (angle + pi) / (2*pi);
thetaIdx = int32(floor(fraction * double(thetaBins)) + 1);
if thetaIdx < 1
    thetaIdx = int32(1);
elseif thetaIdx > thetaBins
    thetaIdx = thetaBins;
end
end

function val = heuristicCost(x, y, yaw, goalState, minTurningRadius)
dx = goalState(1) - x;
dy = goalState(2) - y;
posWeight = hypot(dx, dy);
yawError = abs(wrapToPi(goalState(3) - yaw));
yawWeight = minTurningRadius * yawError;
val = posWeight + yawWeight;
end

function [xNext, yNext, yawNext, travel, validMove] = propagatePrimitive(x, y, yaw, deltaS, curvature, deltaYaw, ...
    minXY, maxXY, resolution, sampleStep, floorCenters, floorRadii, floorMargins, numDiscs, safetyMargin, minTurningRadius)
validMove = true;
yawNext = wrapToPi(yaw);
xNext = x;
yNext = y;
travel = 0;

if abs(deltaS) > 1e-9
    sampleDist = max(sampleStep, resolution * 0.5);
    steps = int32(max(1.0, ceil(abs(deltaS) / sampleDist)));
    sStep = deltaS / double(steps);
    yawStep = curvature * sStep;
    xCurr = x;
    yCurr = y;
    yawCurr = yaw;
    for k = 1:double(steps)
        yawPrev = yawCurr;
        yawCurr = wrapToPi(yawCurr + yawStep);
        if abs(curvature) < 1e-6
            xCurr = xCurr + sStep * cos(yawPrev);
            yCurr = yCurr + sStep * sin(yawPrev);
        else
            radius = 1 / curvature;
            xCurr = xCurr + radius * (sin(yawCurr) - sin(yawPrev));
            yCurr = yCurr - radius * (cos(yawCurr) - cos(yawPrev));
        end
        if xCurr < minXY(1) || xCurr > maxXY(1) || yCurr < minXY(2) || yCurr > maxXY(2)
            validMove = false;
            break;
        end
        if collisionAtPoint(xCurr, yCurr, floorCenters, floorRadii, floorMargins, numDiscs, safetyMargin)
            validMove = false;
            break;
        end
    end
    if ~validMove
        return;
    end
    xNext = xCurr;
    yNext = yCurr;
    yawNext = wrapToPi(yawCurr);
    travel = abs(deltaS);
elseif abs(deltaYaw) > 1e-9
    steps = int32(max(1.0, ceil(abs(deltaYaw) / 0.2)));
    yawStep = deltaYaw / double(steps);
    yawCurr = yaw;
    for k = 1:double(steps)
        yawCurr = wrapToPi(yawCurr + yawStep);
    end
    yawNext = yawCurr;
    xNext = x;
    yNext = y;
    travel = abs(deltaYaw) * minTurningRadius;
else
    yawNext = wrapToPi(yaw);
    xNext = x;
    yNext = y;
    travel = 0;
end

if collisionAtPoint(xNext, yNext, floorCenters, floorRadii, floorMargins, numDiscs, safetyMargin)
    validMove = false;
end
end

function prims = buildMotionPrimitives(motionPrimitiveLength, minTurningRadius, rotationStep)
curvature = 1 / max(minTurningRadius, 1e-3);
prims = [
    motionPrimitiveLength, 0, 0;
   -motionPrimitiveLength, 0, 0;
    motionPrimitiveLength,  curvature, 0;
    motionPrimitiveLength, -curvature, 0;
   -motionPrimitiveLength,  curvature, 0;
   -motionPrimitiveLength, -curvature, 0;
    0, 0,  rotationStep;
    0, 0, -rotationStep
];
end

function hit = collisionAtPoint(x, y, floorCenters, floorRadii, floorMargins, numDiscs, safetyMargin)
hit = false;
for idx = 1:double(numDiscs)
    dx = x - floorCenters(idx,1);
    dy = y - floorCenters(idx,2);
    rr = floorRadii(idx) + floorMargins(idx) + safetyMargin;
    if dx*dx + dy*dy <= rr*rr
        hit = true;
        return;
    end
end
end

function reached = reachedGoal(x, y, yaw, goalState, posTol, yawTol)
dPrim = hypot(goalState(1) - x, goalState(2) - y);
if dPrim > posTol
    reached = false;
    return;
end
yawErr = abs(wrapToPi(goalState(3) - yaw));
reached = (yawErr <= yawTol);
end

function nodeIdx = selectBestOpen(openSet, fCost, nodeCount)
nodeIdx = int32(0);
bestCost = inf;
for idx = 1:double(nodeCount)
    if openSet(idx) && fCost(idx) < bestCost
        bestCost = fCost(idx);
        nodeIdx = int32(idx);
    end
end
end

function angle = wrapToPi(angle)
angle = mod(angle + pi, 2*pi);
if angle < 0
    angle = angle + 2*pi;
end
angle = angle - pi;
end

function val = getFieldOr(s, name, defaultValue)
if isfield(s, name)
    val = s.(name);
else
    val = defaultValue;
end
end
