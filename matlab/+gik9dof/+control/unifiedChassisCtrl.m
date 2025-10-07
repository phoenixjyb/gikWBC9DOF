function [cmd, state] = unifiedChassisCtrl(mode, ref, estPose, params, state)
%UNIFIEDCHASSISCTRL Convert heterogeneous references into unified base commands.
%   [cmd, state] = gik9dof.control.unifiedChassisCtrl(mode, ref, estPose, params, state)
%   accepts references from either the holistic GIK pipeline ("holistic" or
%   "staged-C") or a staged base follower ("staged-B") and produces a
%   UnifiedCmd struct with fields:
%       .mode (string)  - input mode echoed back
%       .time (double)  - ref timestamp [s]
%       .base.Vx (double) - forward velocity command (m/s)
%       .base.Vy (double) - always 0 for diff-drive
%       .base.Wz (double) - yaw rate command (rad/s)
%       .arm.qdot (double[]) - optional arm joint velocities (holistic/Stage-C)
%
%   Inputs
%   ------
%   mode    : string/char, one of "holistic", "staged-C", "staged-B".
%   ref     : struct containing reference data. For holistic/Stage-C the
%             struct must include fields x,y,theta,t and optionally arm_qdot.
%             For staged-B it must include v,w,t.
%   estPose : 1x3 [x y theta] estimated robot pose in world.
%   params  : struct with fields:
%                track        - wheel track (m)
%                Vwheel_max   - per-wheel speed limit (m/s)
%                Vx_max       - forward speed clamp (m/s)
%                W_max        - yaw rate clamp (rad/s)
%                yawKp        - heading proportional gain
%                yawKff       - yaw feed-forward gain
%   state   : struct carrying persistent fields used by the controller.
%
%   Outputs
%   -------
%   cmd     : struct matching UnifiedCmd schema (see summary doc).
%   state   : updated state struct (stores previous holistic sample).
%
%   Notes
%   -----
%   - Differentiation uses timestamps in the reference; ensure consistent
%     sample times.
%   - The yaw gate enforces both platform limits and wheel limits derived from
%     the diff-drive kinematics.
%
%   See also gik9dof.control.defaultUnifiedParams.

arguments
    mode {mustBeMember(mode, {'holistic','staged-C','staged-B'})}  % Cell array of char arrays for codegen
    ref (1,1) struct
    estPose (1,3) double
    params struct
    state struct = struct()
end

% Ensure required parameter fields exist
requiredParamFields = {'track','Vwheel_max','Vx_max','W_max','yawKp','yawKff'};  % Cell array for codegen
for k = 1:numel(requiredParamFields)
    name = requiredParamFields{k};
    if ~isfield(params, name)
        error("gik9dof:unifiedChassisCtrl:MissingParam", ...
            "Parameter '%s' missing from params struct.", name);
    end
end

switch mode
    case {'holistic','staged-C'}  % Use char arrays for codegen compatibility
        % Guard: ensure reference carries pose + timestamp
        needed = {'x','y','theta','t'};  % Cell array of char arrays
        for k = 1:numel(needed)
            if ~isfield(ref, needed{k})
                error("gik9dof:unifiedChassisCtrl:MissingField", ...
                    "Holistic/Stage-C reference missing field '%s'.", needed{k});
            end
        end

        if ~isfield(state, 'prev') || isempty(state.prev)
            state.prev = ref;
            % Initialise to zero command on first call.
            dt = 1e-3;
            vxWorld = 0; vyWorld = 0; wRef = 0;
        else
            dt = max(1e-3, ref.t - state.prev.t);
            dx = ref.x - state.prev.x;
            dy = ref.y - state.prev.y;
            dtheta = wrapToPi(ref.theta - state.prev.theta);
            vxWorld = dx / dt;
            vyWorld = dy / dt;
            wRef = dtheta / dt;
            state.prev = ref;
        end

        yaw = estPose(3);
        Rwb = [cos(yaw), sin(yaw); -sin(yaw), cos(yaw)];
        vBody = Rwb * [vxWorld; vyWorld];

        % Desired heading from world-frame velocity
        phiDesired = atan2(vyWorld, vxWorld);
        headingError = wrapToPi(phiDesired - yaw);

        Vx = sign(vBody(1)) * min(abs(vBody(1)), params.Vx_max) * cos(headingError);
        Wz = params.yawKp * headingError + params.yawKff * wRef;

    case 'staged-B'  % Use char array for codegen compatibility
        needed = {'v','w','t'};  % Cell array of char arrays
        for k = 1:numel(needed)
            if ~isfield(ref, needed{k})
                error("gik9dof:unifiedChassisCtrl:MissingField", ...
                    "Stage-B reference missing field '%s'.", needed{k});
            end
        end
        Vx = ref.v;
        Wz = ref.w;
end

% Apply yaw feasibility gate and forward speed clamp
[Wz, yawCaps] = gik9dof.control.clampYawByWheelLimit(Vx, Wz, params.track, ...
    params.Vwheel_max, params.W_max);
Vx = max(-params.Vx_max, min(params.Vx_max, Vx));

% Populate command struct
cmd = struct();
cmd.mode = mode;
cmd.time = ref.t;
cmd.base = struct('Vx', Vx, 'Vy', 0, 'Wz', Wz, ...
    'YawLimitWheel', yawCaps.wheel, 'YawLimitCmd', yawCaps.applied);
cmd.arm = struct('qdot', []);
if ~strcmp(mode, 'staged-B') && isfield(ref, 'arm_qdot') && ~isempty(ref.arm_qdot)  % Use strcmp for char array comparison
    cmd.arm.qdot = ref.arm_qdot;
end
cmd.flags = struct();
end
