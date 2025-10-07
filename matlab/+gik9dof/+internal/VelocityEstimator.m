classdef VelocityEstimator < handle
    %VELOCITYESTIMATOR Adaptive finite-difference estimator for planar base.
    %   Computes linear and angular velocities from buffered base poses using
    %   multi-order backward differentiation with angle unwrapping and
    %   differential-drive projection.

    properties (Constant)
        MaxHistory = 5;
        MinDt = 1e-6;
        MaxDt = 1.0;
        TimeToleranceFraction = 0.1;
    end

    properties (Access = private)
        configHistory double = zeros(0,3);
        timeHistory double = zeros(0,1);
        thetaRaw double = zeros(0,1);
        thetaUnwrapped double = zeros(0,1);
    end

    methods
        function obj = VelocityEstimator()
            obj.reset();
        end

        function reset(obj)
            obj.configHistory = zeros(0,3);
            obj.timeHistory = zeros(0,1);
            obj.thetaRaw = zeros(0,1);
            obj.thetaUnwrapped = zeros(0,1);
        end

        function result = update(obj, state, time)
            arguments
                obj
                state (1,3) double {mustBeFinite}
                time (1,1) double {mustBeFinite}
            end

            result = obj.defaultResult();

            if isempty(obj.timeHistory)
                obj.addSample(state, time);
                result.method = "startup";
                return
            end

            dt = time - obj.timeHistory(end);
            if ~(dt >= obj.MinDt && dt <= obj.MaxDt)
                obj.reset();
                obj.addSample(state, time);
                result.method = "invalid_dt";
                return
            end

            obj.addSample(state, time);
            nSamples = size(obj.configHistory, 1);
            if nSamples < 2
                result.method = "startup";
                return
            end

            [method, indices, h] = obj.selectStencil();
            if h < obj.MinDt
                result.method = "invalid_dt";
                return
            end

            xVals = obj.configHistory(indices, 1);
            yVals = obj.configHistory(indices, 2);
            thetaVals = obj.thetaUnwrapped(indices);

            switch method
                case "5pt"
                    coeff = [25, -48, 36, -16, 3];
                    denom = 12 * h;
                case "3pt"
                    coeff = [3, -4, 1];
                    denom = 2 * h;
                otherwise
                    coeff = [1, -1];
                    denom = h;
            end

            vx = dot(coeff, xVals) / denom;
            vy = dot(coeff, yVals) / denom;
            omega = dot(coeff, thetaVals) / denom;

            vWorldRaw = [vx; vy];

            theta = obj.configHistory(end, 3);
            c = cos(theta);
            s = sin(theta);
            rot = [c, -s; s, c];

            vRobot = rot.' * vWorldRaw;
            vRobot(2) = 0;
            vWorld = rot * vRobot;

            result.vWorldRaw = vWorldRaw.';
            result.vWorld = vWorld.';
            result.vRobot = vRobot.';
            result.omega = omega;
            result.method = method;
            result.valid = true;
        end
    end

    methods (Access = private)
        function addSample(obj, state, time)
            if isempty(obj.configHistory)
                thetaUnwrapped = state(3);
            else
                delta = wrapToPi(state(3) - obj.thetaRaw(end));
                thetaUnwrapped = obj.thetaUnwrapped(end) + delta;
            end

            obj.configHistory(end+1, :) = state;
            obj.timeHistory(end+1, 1) = time;
            obj.thetaRaw(end+1, 1) = state(3);
            obj.thetaUnwrapped(end+1, 1) = thetaUnwrapped;

            if size(obj.configHistory, 1) > obj.MaxHistory
                obj.configHistory(1, :) = [];
                obj.timeHistory(1, :) = [];
                obj.thetaRaw(1, :) = [];
                obj.thetaUnwrapped(1, :) = [];
            end
        end

        function [method, indices, h] = selectStencil(obj)
            n = size(obj.configHistory, 1);
            if n >= 5
                times = obj.timeHistory(end-4:end);
                if obj.isUniform(times)
                    method = "5pt";
                    indices = n:-1:n-4;
                    h = mean(diff(times));
                    return
                end
            end
            if n >= 3
                times = obj.timeHistory(end-2:end);
                if obj.isUniform(times)
                    method = "3pt";
                    indices = n:-1:n-2;
                    h = mean(diff(times));
                    return
                end
            end
            method = "2pt";
            indices = [n, n-1];
            h = obj.timeHistory(n) - obj.timeHistory(n-1);
        end

        function tf = isUniform(obj, times)
            dt = diff(times);
            if isempty(dt) || any(dt < obj.MinDt)
                tf = false;
                return
            end
            tolerance = max(obj.MinDt, obj.TimeToleranceFraction * mean(dt));
            tf = (max(dt) - min(dt)) <= tolerance;
        end

        function result = defaultResult(~)
            result = struct( ...
                'vWorld', [0 0], ...
                'vWorldRaw', [0 0], ...
                'vRobot', [0 0], ...
                'omega', 0, ...
                'method', "startup", ...
                'valid', false);
        end
    end
end
