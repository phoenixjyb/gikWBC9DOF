classdef purePursuitFollower < handle
    %PUREPURSUITFOLLOWER Chassis-aware path follower for differential drive.
    %   follower = gik9dof.control.purePursuitFollower(pathStates, options)
    %   builds a controller that converts SE(2) reference paths into chassis
    %   commands (Vx, Wz) honouring acceleration, curvature, and wheel-speed
    %   bounds defined by the loaded chassis profile.
    %
    %   Key options:
    %       ChassisParams     - struct from loadChassisProfile (required)
    %       ControllerMode    - "blended", "purePursuit", "stanley"
    %       SampleTime        - nominal dt used for lookahead adaptation
    %       LookaheadBase     - base lookahead distance (m)
    %       LookaheadVelGain  - scales with |vx|
    %       LookaheadTimeGain - scales with acceleration
    %       PathInfo          - preprocessed struct from preparePathForFollower
    %
    %   [vx, wz, status] = follower.step(pose, dt) uses the latest pose
    %   estimate [x y yaw] and timestep dt to produce commands alongside a
    %   status struct capturing curvature, acceleration, cross-track error,
    %   and wheel speeds for logging.

    properties
        SampleTime (1,1) double {mustBePositive} = 0.1
        ControllerMode (1,1) string {mustBeMember(ControllerMode, ["blended","purePursuit","stanley"])} = "blended"
        LookaheadBase (1,1) double {mustBePositive} = 0.6
        LookaheadVelGain (1,1) double {mustBeNonnegative} = 0.30
        LookaheadTimeGain (1,1) double {mustBeNonnegative} = 0.05
        GoalTolerance (1,1) double {mustBePositive} = 0.10
        ReverseEnabled (1,1) logical = false
        HeadingKp (1,1) double = 1.2
        HeadingKi (1,1) double = 0.0
        HeadingKd (1,1) double = 0.1
        FeedforwardGain (1,1) double = 0.9
    end

    properties (SetAccess = private)
        Chassis struct = struct('track', 0.574, 'wheel_speed_max', 3.3, ...
            'vx_max', 1.5, 'vx_min', -0.4, 'wz_max', 2.5, ...
            'accel_limit', 1.2, 'decel_limit', 1.8, 'jerk_limit', 5.0, ...
            'wheel_base', 0.36, 'curvature_slowdown', struct('kappa_threshold', 0.9, 'vx_reduction', 0.6), ...
            'reverse_enabled', false)
        PathInfo struct = struct('States', zeros(0,3), 'Curvature', [], 'ArcLength', [], ...
            'DistanceRemaining', [], 'SegmentIndex', [], 'Diagnostics', struct('warnings', strings(1,0)))
    end

    properties (Access = private)
        CurrentIndex (1,1) double {mustBeInteger, mustBeNonnegative} = 1
        LastVelocity double = 0
        LastAcceleration double = 0
        LastHeadingError double = 0
        HeadingIntegral double = 0
        StatusTemplate struct
    end

    methods
        function obj = purePursuitFollower(pathStates, varargin)
            parser = inputParser;
            addParameter(parser, 'SampleTime', obj.SampleTime, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(parser, 'ControllerMode', obj.ControllerMode, @(x) any(strcmpi(x, ["blended","purePursuit","stanley"])));
            addParameter(parser, 'ChassisParams', struct(), @(x) isstruct(x));
            addParameter(parser, 'LookaheadBase', obj.LookaheadBase);
            addParameter(parser, 'LookaheadVelGain', obj.LookaheadVelGain);
            addParameter(parser, 'LookaheadTimeGain', obj.LookaheadTimeGain);
            addParameter(parser, 'GoalTolerance', obj.GoalTolerance);
            addParameter(parser, 'ReverseEnabled', obj.ReverseEnabled);
            addParameter(parser, 'HeadingKp', obj.HeadingKp);
            addParameter(parser, 'HeadingKi', obj.HeadingKi);
            addParameter(parser, 'HeadingKd', obj.HeadingKd);
            addParameter(parser, 'FeedforwardGain', obj.FeedforwardGain);
            addParameter(parser, 'PathInfo', struct(), @(x) isstruct(x));
            parse(parser, varargin{:});

            obj.SampleTime = parser.Results.SampleTime;
            obj.ControllerMode = string(lower(parser.Results.ControllerMode));
            obj.LookaheadBase = parser.Results.LookaheadBase;
            obj.LookaheadVelGain = parser.Results.LookaheadVelGain;
            obj.LookaheadTimeGain = parser.Results.LookaheadTimeGain;
            obj.GoalTolerance = parser.Results.GoalTolerance;
            obj.ReverseEnabled = parser.Results.ReverseEnabled;
            obj.HeadingKp = parser.Results.HeadingKp;
            obj.HeadingKi = parser.Results.HeadingKi;
            obj.HeadingKd = parser.Results.HeadingKd;
            obj.FeedforwardGain = parser.Results.FeedforwardGain;

            if ~isempty(fieldnames(parser.Results.ChassisParams))
                obj.Chassis = mergeChassisStruct(obj.Chassis, parser.Results.ChassisParams);
            end
            obj.ReverseEnabled = obj.Chassis.reverse_enabled | obj.ReverseEnabled;

            obj.StatusTemplate = struct('isFinished', false, ...
                'nearestIndex', NaN, ...
                'targetIndex', NaN, ...
                'distanceToGoal', NaN, ...
                'lookaheadDistance', obj.LookaheadBase, ...
                'curvature', 0, ...
                'crossTrackError', 0, ...
                'headingError', 0, ...
                'vxCommand', 0, ...
                'wzCommand', 0, ...
                'acceleration', 0, ...
                'wheelSpeeds', [0 0], ...
                'controllerMode', obj.ControllerMode, ...
                'warnings', strings(1,0));

            if ~isempty(fieldnames(parser.Results.PathInfo))
                obj.PathInfo = parser.Results.PathInfo;
            else
                obj.setPath(pathStates);
            end
        end

        function setPath(obj, pathStates)
            if nargin < 2
                pathStates = zeros(0,3);
            end
            obj.PathInfo = gik9dof.control.preparePathForFollower(pathStates, obj.Chassis, 'Validate', true);
            obj.CurrentIndex = 1;
            obj.LastVelocity = 0;
            obj.LastAcceleration = 0;
            obj.LastHeadingError = 0;
            obj.HeadingIntegral = 0;
        end

        function reset(obj)
            obj.CurrentIndex = 1;
            obj.LastVelocity = 0;
            obj.LastAcceleration = 0;
            obj.LastHeadingError = 0;
            obj.HeadingIntegral = 0;
        end

        function [vx, wz, status] = step(obj, pose, dt)
            if nargin < 3 || isempty(dt)
                dt = obj.SampleTime;
            end
            dt = max(dt, 1e-3);

            if isempty(obj.PathInfo.States)
                vx = 0; wz = 0;
                status = obj.StatusTemplate;
                status.isFinished = true;
                return
            end

            states = obj.PathInfo.States;
            numPts = size(states,1);

            position = pose(1:2);
            theta = pose(3);

            diffs = states(:,1:2) - position;
            distSq = sum(diffs.^2, 2);
            [~, nearestIdx] = min(distSq);
            nearestIdx = max(nearestIdx, obj.CurrentIndex);
            obj.CurrentIndex = nearestIdx;

            lookahead = obj.computeLookahead(dt);
            arc = obj.PathInfo.ArcLength;
            targetS = min(arc(end), arc(nearestIdx) + lookahead);
            targetIdx = find(arc >= targetS, 1, 'first');
            if isempty(targetIdx)
                targetIdx = numPts;
            end

            targetPoint = states(targetIdx,1:2);
            delta = targetPoint - position;
            rot = [cos(theta) sin(theta); -sin(theta) cos(theta)];
            deltaBody = rot * delta(:);
            xLook = deltaBody(1);
            yLook = deltaBody(2);

            ld = max(lookahead, 1e-3);
            curvature = obj.PathInfo.Curvature(targetIdx);
            curvaturePP = 2 * yLook / (ld^2);
            crossTrackError = yLook;

            headingTarget = states(targetIdx,3);
            headingError = wrapToPi(headingTarget - theta);
            headingRate = (headingError - obj.LastHeadingError) / dt;

            direction = 1;
            if obj.ReverseEnabled && xLook < 0
                direction = -1;
            end

            vxDesired = obj.computeDesiredSpeed(curvature, obj.PathInfo.DistanceRemaining(nearestIdx));
            vxDesired = direction * abs(vxDesired);

            [vx, accel] = obj.applyAccelerationLimits(vxDesired, dt);
            obj.LastAcceleration = accel;
            obj.LastVelocity = vx;

            wz = obj.computeYawRate(vx, headingError, headingRate, curvature, curvaturePP);
            [vx, wz, wheelSpeeds] = obj.applyChassisLimits(vx, wz);

            obj.updateHeadingIntegral(headingError, dt, wz);
            obj.LastHeadingError = headingError;

            distanceToGoal = obj.PathInfo.DistanceRemaining(nearestIdx);
            isFinished = (distanceToGoal <= obj.GoalTolerance) || (nearestIdx >= numPts);

            status = obj.StatusTemplate;
            status.isFinished = isFinished;
            status.nearestIndex = nearestIdx;
            status.targetIndex = targetIdx;
            status.distanceToGoal = distanceToGoal;
            status.lookaheadDistance = lookahead;
            status.curvature = curvature;
            status.crossTrackError = crossTrackError;
            status.headingError = headingError;
            status.vxCommand = vx;
            status.wzCommand = wz;
            status.acceleration = accel;
            status.wheelSpeeds = wheelSpeeds;
            status.controllerMode = obj.ControllerMode;
            status.warnings = obj.PathInfo.Diagnostics.warnings;
        end
    end

    methods (Access = private)
        function lookahead = computeLookahead(obj, dt)
            lookahead = obj.LookaheadBase ...
                + obj.LookaheadVelGain * abs(obj.LastVelocity) ...
                + obj.LookaheadTimeGain * abs(obj.LastAcceleration) * dt;
            lookahead = max([lookahead, obj.GoalTolerance, 0.05]);
        end

        function vxDesired = computeDesiredSpeed(obj, curvature, distanceRemaining)
            vxCap = obj.Chassis.vx_max;
            slowdown = obj.Chassis.curvature_slowdown;
            if ~isempty(slowdown)
                kappaThr = slowdown.kappa_threshold;
                vxReduction = slowdown.vx_reduction;
                if abs(curvature) > kappaThr
                    scale = vxReduction;
                else
                    ratio = max(0, 1 - abs(curvature)/kappaThr);
                    scale = vxReduction + (1 - vxReduction) * ratio;
                end
                vxCap = max(obj.Chassis.vx_min, vxCap * scale);
            end

            if distanceRemaining < 3 * obj.GoalTolerance
                taper = max(distanceRemaining / (3 * obj.GoalTolerance), 0.2);
                vxCap = min(vxCap, obj.Chassis.vx_max * taper);
            end

            vxDesired = min(vxCap, obj.Chassis.vx_max);
        end

        function [vx, accel] = applyAccelerationLimits(obj, vxDesired, dt)
            deltaV = vxDesired - obj.LastVelocity;
            if deltaV >= 0
                maxDelta = obj.Chassis.accel_limit * dt;
                deltaV = min(deltaV, maxDelta);
            else
                maxDelta = obj.Chassis.decel_limit * dt;
                deltaV = max(deltaV, -maxDelta);
            end
            vx = obj.LastVelocity + deltaV;
            vx = min(obj.Chassis.vx_max, max(obj.Chassis.vx_min, vx));
            accel = (vx - obj.LastVelocity) / dt;
        end

        function wz = computeYawRate(obj, vx, headingError, headingRate, curvatureFF, curvaturePP)
            switch obj.ControllerMode
                case "purepursuit"
                    curvature = curvaturePP;
                    wzFF = curvature * vx;
                    wzFB = 0;
                case "stanley"
                    % Stanley method substitutes pp curvature for cross-track steering.
                    if vx ~= 0
                        crossTrackGain = obj.HeadingKp;
                        wzFF = obj.FeedforwardGain * curvatureFF * vx;
                        wzFB = crossTrackGain * atan2(obj.LastVelocity * headingError, obj.LookaheadBase);
                    else
                        wzFF = 0;
                        wzFB = obj.HeadingKp * headingError;
                    end
                otherwise % blended
                    wzFF = obj.FeedforwardGain * curvatureFF * vx;
                    wzFB = obj.HeadingKp * headingError + obj.HeadingKd * headingRate + obj.HeadingKi * obj.HeadingIntegral;
            end
            wz = wzFF + wzFB;
        end

        function [vx, wz, wheelSpeeds] = applyChassisLimits(obj, vx, wz)
            track = obj.Chassis.track;
            wheelSpeedMax = obj.Chassis.wheel_speed_max;
            yawCapWheel = 2 * max(0, (wheelSpeedMax - abs(vx))) / max(track, eps);
            yawCap = min(obj.Chassis.wz_max, yawCapWheel);
            wz = max(-yawCap, min(yawCap, wz));

            vx = max(obj.Chassis.vx_min, min(obj.Chassis.vx_max, vx));

            vl = vx - 0.5 * track * wz;
            vr = vx + 0.5 * track * wz;
            maxAbs = max(abs([vl, vr]));
            if maxAbs > wheelSpeedMax
                scale = wheelSpeedMax / maxAbs;
                vx = vx * scale;
                wz = wz * scale;
                vl = vl * scale;
                vr = vr * scale;
            end
            wheelSpeeds = [vl, vr];
        end

        function updateHeadingIntegral(obj, headingError, dt, wz)
            if obj.HeadingKi <= 0
                obj.HeadingIntegral = 0;
                return
            end
            obj.HeadingIntegral = obj.HeadingIntegral + headingError * dt;
            yawCap = obj.Chassis.wz_max;
            if abs(wz) >= yawCap * 0.98
                obj.HeadingIntegral = obj.HeadingIntegral * 0.9; % anti-windup bleed
            end
        end
    end
end

function merged = mergeChassisStruct(base, overrides)
merged = base;
fields = fieldnames(overrides);
for idx = 1:numel(fields)
    name = fields{idx};
    merged.(name) = overrides.(name);
end
if ~isfield(merged, 'curvature_slowdown')
    merged.curvature_slowdown = struct('kappa_threshold', 0.9, 'vx_reduction', 0.6);
end
if ~isfield(merged, 'reverse_enabled')
    merged.reverse_enabled = false;
end
end
