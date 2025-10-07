classdef purePursuitFollower < handle
    %PUREPURSUITFOLLOWER Pure pursuit controller with chassis limits.
    %   follower = gik9dof.control.purePursuitFollower(pathStates, options)
    %   builds a controller that produces differential-drive friendly
    %   (Vx, Wz) commands. The follower supports bidirectional motion and
    %   dynamically modulates the lookahead distance based on velocity.
    %
    %   pathStates : Nx3 matrix [x y yaw] describing the reference path.
    %   options struct/name-value pairs:
    %       SampleTime          - Default 0.1 s (used for lookahead gain)
    %       LookaheadBase       - 0.8 m
    %       LookaheadVelGain    - 0.3 s
    %       LookaheadTimeGain   - 0.1 s^2
    %       VxNominal           - 1.0 m/s
    %       VxMax               - 1.5 m/s
    %       VxMin               - -1.0 m/s
    %       WzMax               - 2.0 rad/s
    %       TrackWidth          - 0.674 m
    %       WheelBase           - 0.36 m (informational)
    %       MaxWheelSpeed       - 2.0 m/s
    %       WaypointSpacing     - 0.15 m
    %       PathBufferSize      - 30.0 m
    %       GoalTolerance       - 0.2 m
    %       InterpSpacing       - 0.05 m
    %       ReverseEnabled      - true
    %
    %   [vx, wz, status] = follower.step(pose, dt)
    %       pose : [x y yaw] current chassis pose (world frame).
    %       dt   : sample time (defaults to options.SampleTime).
    %       status fields: isFinished, lookaheadIndex, distanceToGoal,
    %                      lookaheadDistance, wheelSpeeds (1x2), headingError.
    %
    %   The controller outputs lateral velocity vy=0; callers must integrate
    %   the (vx, wz) commands using differential-drive kinematics.

    properties
        SampleTime (1,1) double {mustBePositive} = 0.1
        LookaheadBase (1,1) double {mustBePositive} = 0.8
        LookaheadVelGain (1,1) double {mustBeNonnegative} = 0.3
        LookaheadTimeGain (1,1) double {mustBeNonnegative} = 0.1
        VxNominal (1,1) double = 1.0
        VxMax (1,1) double {mustBePositive} = 1.5
        VxMin (1,1) double = -1.0
        WzMax (1,1) double {mustBePositive} = 2.0
        TrackWidth (1,1) double {mustBePositive} = 0.674
        WheelBase (1,1) double {mustBePositive} = 0.36
        MaxWheelSpeed (1,1) double {mustBePositive} = 2.0
        WaypointSpacing (1,1) double {mustBePositive} = 0.15
        PathBufferSize (1,1) double {mustBePositive} = 30.0
        GoalTolerance (1,1) double {mustBePositive} = 0.2
        InterpSpacing (1,1) double {mustBePositive} = 0.05
        ReverseEnabled (1,1) logical = true
    end

    properties (Access = private)
        Path double = zeros(0,3)
        CumulativeDistance double = double.empty(1,0)
        CurrentIndex (1,1) double {mustBeInteger, mustBeNonnegative} = 1
        LastVelocity double = 0
    end

    methods
        function obj = purePursuitFollower(pathStates, varargin)
            if nargin < 1
                pathStates = zeros(0,3);
            end

            parser = inputParser;
            addParameter(parser, 'SampleTime', obj.SampleTime, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(parser, 'LookaheadBase', obj.LookaheadBase);
            addParameter(parser, 'LookaheadVelGain', obj.LookaheadVelGain);
            addParameter(parser, 'LookaheadTimeGain', obj.LookaheadTimeGain);
            addParameter(parser, 'VxNominal', obj.VxNominal);
            addParameter(parser, 'VxMax', obj.VxMax);
            addParameter(parser, 'VxMin', obj.VxMin);
            addParameter(parser, 'WzMax', obj.WzMax);
            addParameter(parser, 'TrackWidth', obj.TrackWidth);
            addParameter(parser, 'WheelBase', obj.WheelBase);
            addParameter(parser, 'MaxWheelSpeed', obj.MaxWheelSpeed);
            addParameter(parser, 'WaypointSpacing', obj.WaypointSpacing);
            addParameter(parser, 'PathBufferSize', obj.PathBufferSize);
            addParameter(parser, 'GoalTolerance', obj.GoalTolerance);
            addParameter(parser, 'InterpSpacing', obj.InterpSpacing);
            addParameter(parser, 'ReverseEnabled', obj.ReverseEnabled);
            parse(parser, varargin{:});

            obj.SampleTime = parser.Results.SampleTime;
            obj.LookaheadBase = parser.Results.LookaheadBase;
            obj.LookaheadVelGain = parser.Results.LookaheadVelGain;
            obj.LookaheadTimeGain = parser.Results.LookaheadTimeGain;
            obj.VxNominal = parser.Results.VxNominal;
            obj.VxMax = parser.Results.VxMax;
            obj.VxMin = parser.Results.VxMin;
            obj.WzMax = parser.Results.WzMax;
            obj.TrackWidth = parser.Results.TrackWidth;
            obj.WheelBase = parser.Results.WheelBase;
            obj.MaxWheelSpeed = parser.Results.MaxWheelSpeed;
            obj.WaypointSpacing = parser.Results.WaypointSpacing;
            obj.PathBufferSize = parser.Results.PathBufferSize;
            obj.GoalTolerance = parser.Results.GoalTolerance;
            obj.InterpSpacing = parser.Results.InterpSpacing;
            obj.ReverseEnabled = parser.Results.ReverseEnabled;

            obj.setPath(pathStates);
        end

        function setPath(obj, pathStates)
            if isempty(pathStates)
                obj.Path = zeros(0,3);
                obj.CumulativeDistance = double.empty(1,0);
                obj.CurrentIndex = 1;
                return
            end

            if size(pathStates,2) < 2
                error('purePursuitFollower:InvalidPath', ...
                    'Path must contain at least [x y] columns.');
            end

            if size(pathStates,2) == 2
                heading = computeHeading(pathStates);
                pathStates = [pathStates, heading];
            end

            % Interpolate with uniform spacing to improve curvature estimate
            diffs = diff(pathStates(:,1:2));
            segLen = hypot(diffs(:,1), diffs(:,2));
            cumLen = [0; cumsum(segLen)];
            [cumLen, uniqueIdx] = unique(cumLen, 'stable');
            pathStates = pathStates(uniqueIdx, :);
            if cumLen(end) < obj.InterpSpacing
                interpPath = pathStates;
            else
                query = 0:obj.InterpSpacing:cumLen(end);
                xInterp = interp1(cumLen, pathStates(:,1), query, 'pchip');
                yInterp = interp1(cumLen, pathStates(:,2), query, 'pchip');
                yawInterp = unwrap(pathStates(:,3));
                yawInterp = interp1(cumLen, yawInterp, query, 'linear');
                interpPath = [xInterp(:), yInterp(:), wrapToPi(yawInterp(:))];
            end

            % Apply buffer limit (meters)
            maxPoints = ceil(obj.PathBufferSize / max(obj.WaypointSpacing, obj.InterpSpacing));
            if size(interpPath,1) > maxPoints
                interpPath = interpPath(end-maxPoints+1:end, :);
            end

            obj.Path = interpPath;
            diffs = diff(interpPath(:,1:2));
            segLen = hypot(diffs(:,1), diffs(:,2));
            obj.CumulativeDistance = [0; cumsum(segLen)];
            obj.CurrentIndex = 1;
            obj.LastVelocity = 0;
        end

        function reset(obj)
            obj.CurrentIndex = 1;
            obj.LastVelocity = 0;
        end

        function [vx, wz, status] = step(obj, pose, dt)
            if nargin < 3 || isempty(dt)
                dt = obj.SampleTime;
            end
            if isempty(obj.Path)
                vx = 0; wz = 0;
                status = defaultStatus(true, NaN, NaN, obj.LookaheadBase, [0 0], 0);
                return
            end

            position = pose(1:2);
            theta = pose(3);

            % Find nearest waypoint
            distances = vecnorm(obj.Path(:,1:2) - position, 2, 2);
            [~, nearestIdx] = min(distances);
            obj.CurrentIndex = nearestIdx;

            cumulativeFromCurrent = obj.CumulativeDistance - obj.CumulativeDistance(nearestIdx);

            lookahead = obj.LookaheadBase + obj.LookaheadVelGain * abs(obj.LastVelocity) + ...
                obj.LookaheadTimeGain * abs(obj.LastVelocity) * dt;
            lookahead = max([lookahead, obj.GoalTolerance, obj.InterpSpacing]);

            targetIdx = nearestIdx;
            while targetIdx < size(obj.Path,1) && cumulativeFromCurrent(targetIdx) < lookahead
                targetIdx = targetIdx + 1;
            end
            if targetIdx > size(obj.Path,1)
                targetIdx = size(obj.Path,1);
            end

            targetPoint = obj.Path(targetIdx,1:2);
            delta = targetPoint - position;
            rot = [cos(theta), sin(theta); -sin(theta), cos(theta)];
            deltaBody = rot * delta';
            xLook = deltaBody(1);
            yLook = deltaBody(2);

            ld = max(lookahead, 1e-3);
            curvature = 2 * yLook / (ld^2);

            % Determine heading error at goal
            goalHeading = obj.Path(targetIdx,3);
            headingError = wrapToPi(goalHeading - theta);

            % Forward velocity selection
            direction = 1;
            if obj.ReverseEnabled && xLook < 0
                direction = -1;
            end
            vx = direction * obj.VxNominal;

            % Distance based tapering near goal
            distanceToGoal = distances(end);
            if distanceToGoal < obj.GoalTolerance
                vx = 0;
                curvature = 0;
            end

            % Apply linear velocity saturations
            vx = min(obj.VxMax, max(obj.VxMin, vx));

            wz = curvature * vx + headingError * 0.5;
            wz = min(obj.WzMax, max(-obj.WzMax, wz));

            % Enforce wheel speed limits
            [vx, wz, wheelSpeeds] = enforceWheelLimits(vx, wz, obj.TrackWidth, obj.MaxWheelSpeed);

            obj.LastVelocity = vx;

            finished = distanceToGoal < obj.GoalTolerance && targetIdx == size(obj.Path,1);
            status = defaultStatus(finished, targetIdx, distanceToGoal, lookahead, wheelSpeeds, headingError);
        end
    end
end

%% Helper functions
function heading = computeHeading(path)
numPts = size(path,1);
d_heading = atan2(diff([path(:,2); path(end,2)]), diff([path(:,1); path(end,1)]));
heading = wrapToPi(d_heading(1:numPts));
end

function status = defaultStatus(isFinished, idx, distGoal, lookahead, wheelSpeeds, headingError)
status = struct('isFinished', logical(isFinished), ...
    'lookaheadIndex', idx, ...
    'distanceToGoal', distGoal, ...
    'lookaheadDistance', lookahead, ...
    'wheelSpeeds', wheelSpeeds, ...
    'headingError', headingError);
end

function [vx, wz, wheelSpeeds] = enforceWheelLimits(vx, wz, trackWidth, maxWheelSpeed)
vl = vx - 0.5 * trackWidth * wz;
vr = vx + 0.5 * trackWidth * wz;
maxAbs = max(abs([vl, vr]));
if maxAbs > maxWheelSpeed
    scale = maxWheelSpeed / maxAbs;
    vx = vx * scale;
    wz = wz * scale;
    vl = vl * scale;
    vr = vr * scale;
end
wheelSpeeds = [vl, vr];
end
