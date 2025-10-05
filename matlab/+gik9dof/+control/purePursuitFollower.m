classdef purePursuitFollower < handle
    %PUREPURSUITFOLLOWER Lightweight pure-pursuit controller for Stage B.
    %   follower = gik9dof.control.purePursuitFollower(pathStates, options)
    %   builds a differential-drive friendly pure pursuit tracker. pathStates
    %   is an Nx3 array [x y yaw]. Options (name-value):
    %       LookaheadDistance      - default 0.6 m
    %       DesiredLinearVelocity  - default 0.6 m/s
    %       MaxAngularVelocity     - default 2.5 rad/s
    %       GoalRadius             - default 0.15 m
    %       ReverseAllowed         - default false
    %       CloseLoopOnHeading     - default false (use heading from path)
    %
    %   Methods:
    %       reset()                        - reset internal indices
    %       setPath(pathStates)            - replace route
    %       [v, w, status] = step(pose)    - compute (v,w) given pose [x y yaw]
    %       status struct fields: .isFinished, .lookaheadIndex, .distanceToGoal
    %
    %   The follower is intentionally simple so it can hand values straight
    %   into gik9dof.control.unifiedChassisCtrl when running Stage B. It does
    %   not manage wheel limits or yaw feasibility—that is handled downstream.

    properties
        LookaheadDistance (1,1) double {mustBePositive} = 0.6
        DesiredLinearVelocity (1,1) double {mustBePositive} = 0.6
        MaxAngularVelocity (1,1) double {mustBePositive} = 2.5
        GoalRadius (1,1) double {mustBePositive} = 0.15
        ReverseAllowed (1,1) logical = false
        CloseLoopOnHeading (1,1) logical = false
    end

    properties (Access = private)
        Path double = zeros(0,3)
        CumulativeDistance double = double.empty(1,0)
        CurrentIndex (1,1) double {mustBeInteger, mustBeNonnegative} = 1
    end

    methods
        function obj = purePursuitFollower(pathStates, varargin)
            if nargin < 1, pathStates = zeros(0,3); end
            if ~isempty(varargin)
                set(obj, varargin{:});
            end
            obj.setPath(pathStates);
        end

        function setPath(obj, pathStates)
            arguments
                obj
                pathStates double
            end
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
                pathStates(:,3) = wrapToPi(atan2(diff([pathStates(:,2); pathStates(end,2)]), ...
                    diff([pathStates(:,1); pathStates(end,1)]))); %#ok<AGROW>
            end
            obj.Path = pathStates;
            diffs = diff(pathStates(:,1:2));
            segLen = hypot(diffs(:,1), diffs(:,2));
            obj.CumulativeDistance = [0; cumsum(segLen)];
            obj.CurrentIndex = 1;
        end

        function reset(obj)
            obj.CurrentIndex = 1;
        end

        function [v, w, status] = step(obj, pose)
            arguments
                obj
                pose (1,3) double
            end
            if isempty(obj.Path)
                v = 0; w = 0;
                status = struct('isFinished', true, 'lookaheadIndex', NaN, 'distanceToGoal', NaN);
                return
            end

            position = pose(1:2);
            theta = pose(3);
            % Update nearest index
            distances = vecnorm(obj.Path(:,1:2) - position, 2, 2);
            [~, nearestIdx] = min(distances);
            obj.CurrentIndex = nearestIdx;

            cumulativeFromCurrent = obj.CumulativeDistance - obj.CumulativeDistance(nearestIdx);
            targetIdx = nearestIdx;
            lookahead = obj.LookaheadDistance;
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

            % Optional heading closure near end of path
            if obj.CloseLoopOnHeading && targetIdx == size(obj.Path,1)
                desiredHeading = obj.Path(targetIdx,3);
                headingError = wrapToPi(desiredHeading - theta);
            else
                headingError = 0;
            end

            v = obj.DesiredLinearVelocity;
            if obj.ReverseAllowed && xLook < 0
                v = -v;
            end

            w = curvature * v + 0.5 * headingError;
            w = max(-obj.MaxAngularVelocity, min(obj.MaxAngularVelocity, w));

            distanceToGoal = distances(end);
            if distanceToGoal < obj.GoalRadius || targetIdx == size(obj.Path,1)
                if distanceToGoal < obj.GoalRadius
                    v = 0;
                    w = 0;
                end
                finished = true;
            else
                finished = false;
            end

            status = struct('isFinished', finished, ...
                'lookaheadIndex', targetIdx, ...
                'distanceToGoal', distanceToGoal, ...
                'headingError', headingError);
        end
    end
end
