%% PP-for-GIK full-cycle demo (Option 3)
%  Predict -> Constrain -> Solve (GIK), optional refine, simulate follower,
%  then final GIK with the base fixed to the executed trajectory.

% no need to integrate this file to the project for now. 


clear; clc;

%% ------------------------------------------------------------------------
%% User / robot inputs  (TODO: fill these in for your robot)
% -------------------------------------------------------------------------
robot = loadrobot("kinovaGen3", "DataFormat", "row");  % <-- TODO: your robot
eeName = "EndEffector_Link";                           % <-- TODO
baseBodyName = robot.BaseName;                         % if your chassis is a body, put its name

% If you have virtual joints for base motion, ensure these exist:
baseJointNames = ["base_x","base_y","base_yaw"];       % <-- TODO: verify names

% Waypoints for Pure-Pursuit (XY only)
waypts = [ 0.0  0.0;
           0.6  0.2;
           1.0  0.0;
           1.2 -0.3;
           1.2  0.3];                                 % <-- TODO

% End-effector desired trajectory function (world frame)  (TODO)
eeTrajFcn = @(t) trvec2tform([0.8 0.0 0.6]) * axang2tform([0 0 1 0]); % static pose example

% Limits & params (edit as needed)
params.dt           = 0.02;
params.T            = 6.0;               % total seconds
params.track        = 0.57;              % chassis track [m]
params.wheelRadius  = 0.08;              % wheel radius [m]   <-- TODO
params.wheelRateMax = 12.0;              % [rad/s]            <-- TODO

params.vmax         = 0.6;               % [m/s]  cap early
params.wmax         = 2.5;               % [rad/s] cap early
params.lookahead    = 0.6;               % PP lookahead [m]
params.goalRadius   = 0.05;              % finish criterion for PP

% Corridor and yaw tolerances
params.eps_lat      = 0.01;              % 5–20 mm recommended
params.eps_long_min = 0.01;              % min forward/back allowance (m)
params.yawTolDeg    = 0.8;

% Reverse-capable? (if you later use Reeds–Shepp, keep this true)
params.allowReverse = true;

% Optional refinement
params.doRefine     = false;             % set true if you implement Stage B

%% ------------------------------------------------------------------------
%% Build GIK solvers (first pass and final pass)
% -------------------------------------------------------------------------
gikRef = generalizedInverseKinematics( ...
    "RigidBodyTree", robot, ...
    "ConstraintInputs", {'pose','orientation','cartesian','joint'});

gikFinal = generalizedInverseKinematics( ...
    "RigidBodyTree", robot, ...
    "ConstraintInputs", {'pose','joint'});

% Pre-allocate common constraints (we'll update their fields each tick)
eePose   = constraintPoseTarget(eeName);
baseYaw  = constraintOrientationTarget(baseBodyName);
baseBox  = constraintCartesianBounds(baseBodyName);
jointLim = constraintJointBounds(robot);

% Tolerances for EE (edit for your use-case)
eePose.PositionTolerance     = 1e-3 * [1 1 1];
eePose.OrientationTolerance  = deg2rad(1.0);

% Prepare PP controller
pp = controllerPurePursuit( ...
        "Waypoints", waypts, ...
        "LookaheadDistance", params.lookahead, ...
        "DesiredLinearVelocity", params.vmax, ...
        "MaxAngularVelocity", params.wmax);

%% ------------------------------------------------------------------------
%% Helper: indices and starting configuration
% -------------------------------------------------------------------------
cfg = homeConfiguration(robot);
jnames = string({cfg.JointName});
ix = find(jnames==baseJointNames(1));    % base_x
iy = find(jnames==baseJointNames(2));    % base_y
ith= find(jnames==baseJointNames(3));    % base_yaw
iarm = setdiff(1:numel(cfg), [ix iy ith]);

% Initial base pose (TODO if different)
q = [cfg.JointPosition]; % row
q(ix) = waypts(1,1);
q(iy) = waypts(1,2);
q(ith)= 0;

% Diagnostics
fprintf("Base joints: x->%s  y->%s  yaw->%s\n", baseJointNames(1), baseJointNames(2), baseJointNames(3));

%% ------------------------------------------------------------------------
%% STAGE A: PP-for-GIK pass -> build reference base path (Predict/Constrain/Solve)
% -------------------------------------------------------------------------
N = round(params.T/params.dt) + 1;
logRef.t      = zeros(N,1);
logRef.x      = zeros(N,1);
logRef.y      = zeros(N,1);
logRef.th     = zeros(N,1);
logRef.v_pp   = zeros(N,1);
logRef.w_pp   = zeros(N,1);
logRef.v_lat  = zeros(N,1);
logRef.info   = repmat(struct('ExitFlag',[],'Iterations',[]), N,1);

[x, y, th] = deal(q(ix), q(iy), q(ith));
t = 0;

for k = 1:N
    % --- 1) Desired EE pose at this time
    Tdes = eeTrajFcn(t);
    eePose.ReferenceBody = '';
    eePose.TargetTransform = Tdes;

    % --- 2) Pure-pursuit predict (reverse-aware call)
    [v_pp, w_pp] = ppStepReverseAware(pp, [x y th], params);

    % Clip (v_pp, w_pp) by wheel limits early
    [v_pp, w_pp] = saturateByWheelLimits(v_pp, w_pp, params.track, params.wheelRadius, params.wheelRateMax);

    % --- 3) Integrate unicycle for prediction
    [x_pp, y_pp, th_pp] = unicycleStep(x, y, th, v_pp, w_pp, params.dt);

    % Corridor sizes
    eps_long = max(params.eps_long_min, abs(v_pp)*params.dt + 0.01);
    eps_lat  = params.eps_lat;

    % --- 4) Build base constraints (yaw + yaw-aligned corridor or joint windows)
    baseYaw.ReferenceBody       = '';
    baseYaw.TargetOrientation   = axang2quat([0 0 1 th_pp]);
    baseYaw.OrientationTolerance= deg2rad(params.yawTolDeg);

    % Try yaw-aligned corridor using Cartesian bounds around a target transform.
    canUseCartesian = true;
    try
        Tpp = trvec2tform([x_pp y_pp 0]) * axang2tform([0 0 1 th_pp]);
        baseBox.ReferenceBody    = '';
        baseBox.TargetTransform  = Tpp;  % works in recent MATLAB versions
        baseBox.Bounds           = [ -eps_long, +eps_long; ...
                                     -eps_lat,  +eps_lat;  ...
                                      -Inf,      +Inf    ];
    catch
        canUseCartesian = false;
    end

    % Always keep tight joint windows as a fallback / extra pin
    bnds = jointLim.Bounds;
    bnds(ix,:)  = [x_pp-eps_long,  x_pp+eps_long];
    bnds(iy,:)  = [y_pp-eps_lat,   y_pp+eps_lat];
    bnds(ith,:) = [th_pp-deg2rad(params.yawTolDeg), th_pp+deg2rad(params.yawTolDeg)];
    jointLim.Bounds = bnds;

    % --- 5) Solve GIK
    if canUseCartesian
        [qNext,info] = gikRef(q, eePose, baseYaw, baseBox, jointLim);
    else
        [qNext,info] = gikRef(q, eePose, baseYaw,           jointLim);
    end

    % --- 6) Update state with solved base pose (and warm start for next tick)
    x1 = qNext(ix);  y1 = qNext(iy);  th1 = wrapToPi(qNext(ith));

    % Lateral residual (should be small)
    [v_gik, w_gik, v_lat] = vw_from_gik_step(x, y, th, x1, y1, th1, params.dt);

    q = qNext;  x = x1; y = y1; th = th1;

    % --- 7) Log
    logRef.t(k)    = t;
    logRef.x(k)    = x;
    logRef.y(k)    = y;
    logRef.th(k)   = th;
    logRef.v_pp(k) = v_pp;
    logRef.w_pp(k) = w_pp;
    logRef.v_lat(k)= v_lat;
    logRef.info(k).ExitFlag   = info.ExitFlag;
    logRef.info(k).Iterations = info.Iterations;

    % Terminate when we’re close enough to the final waypoint
    if norm([x y] - waypts(end,:)) < params.goalRadius
        logRef = truncateLog(logRef, k);
        break;
    end

    t = t + params.dt;
end

fprintf("Stage A done. Mean |v_lat| = %.4f m/s\n", mean(abs(logRef.v_lat)));

%% ------------------------------------------------------------------------
%% STAGE B: Optional base-path refinement (hooks only)
% -------------------------------------------------------------------------
if params.doRefine
    baseRef = [logRef.x logRef.y logRef.th];
    refined = refineBasePathRSClothoid(baseRef, params);  %#ok<NASGU>
    % TODO: implement using stateSpaceReedsShepp + clothoid/autoware style smoothing.
else
    refined = struct('x', logRef.x, 'y', logRef.y, 'th', logRef.th, ...
                     'v', logRef.v_pp, 'w', logRef.w_pp, 't', logRef.t);
end

%% ------------------------------------------------------------------------
%% STAGE C: Simulate chassis follower to get executed trajectory + commands
% -------------------------------------------------------------------------
sim = simulateFollowerPP(refined, params, pp);

fprintf("Stage C done. Executed %d steps. Mean |v_cmd|=%.3f m/s, Max |ω_cmd|=%.3f rad/s\n", ...
        numel(sim.t), mean(abs(sim.v_cmd)), max(abs(sim.w_cmd)));

%% ------------------------------------------------------------------------
%% STAGE D: Final GIK with base fixed to the executed trajectory
% -------------------------------------------------------------------------
Nf = numel(sim.t);
armQ = zeros(Nf, numel(cfg));    % full config for logging
q = armWarmStart(robot, eeName, q);  % start from last q of Stage A

for k = 1:Nf
    % Fix the three base joints to the executed pose at this time
    bnds = jointLim.Bounds;
    bnds(ix,:)  = [sim.x(k) sim.x(k)];
    bnds(iy,:)  = [sim.y(k) sim.y(k)];
    bnds(ith,:) = [sim.th(k) sim.th(k)];
    jointLim.Bounds = bnds;

    % EE target for this time (could be time-parametrized; here static)
    Tdes = eeTrajFcn(sim.t(k));
    eePose.TargetTransform = Tdes;

    % Solve GIK for arm with base fixed
    [q,~] = gikFinal(q, eePose, jointLim);

    armQ(k,:) = q;
end

fprintf("Stage D done. Arm trajectory computed (%d samples).\n", Nf);

%% ------------------------------------------------------------------------
%% Simple plots
% -------------------------------------------------------------------------
figure('Name','Base trajectories'); hold on; axis equal; grid on;
plot(waypts(:,1), waypts(:,2), '--k', 'DisplayName','Waypoints');
plot(logRef.x, logRef.y, 'b', 'DisplayName','Stage A (Ref)');
plot(sim.x,    sim.y,    'r', 'DisplayName','Stage C (Exec)');
legend('Location','best'); xlabel('x [m]'); ylabel('y [m]');

figure('Name','Commands'); 
subplot(2,1,1); plot(sim.t, sim.v_cmd); ylabel('v [m/s]'); grid on;
subplot(2,1,2); plot(sim.t, sim.w_cmd); ylabel('\omega [rad/s]'); xlabel('t [s]'); grid on;

%% ========================= Helper functions ==============================

function [v, w] = ppStepReverseAware(pp, pose, params)
% Reverse-aware pure pursuit:
% If you later annotate the path with reverse segments, flip heading by pi.
% Here we keep it forward unless allowReverse + lookahead behind rule triggers.
    x=pose(1); y=pose(2); th=pose(3);

    % Default: forward
    [v, w] = pp([x y th]);

    % Clip by global linear/angular caps first (will be clipped again by wheels)
    v = max(-params.vmax, min(params.vmax, v));
    w = max(-params.wmax, min(params.wmax, w));

    if params.allowReverse
        % Heuristic: if the lookahead lies behind the robot significantly,
        % use the "carrot-behind" trick.
        wp = pp.Waypoints( min(size(pp.Waypoints,1), 2), : ); % crude: 2nd point as lookahead hint
        rel = [cos(th) -sin(th); sin(th) cos(th)]' * (wp(:) - [x;y]);
        if rel(1) < -0.05     % lookahead mostly behind in body-x
            [v2, w2] = pp([x y wrapToPi(th+pi)]);
            v = -abs(v2);     % reverse
            w =  w2;
        end
    end
end

function [x1,y1,th1] = unicycleStep(x,y,th, v,w, dt)
    x1  = x  + v*cos(th)*dt;
    y1  = y  + v*sin(th)*dt;
    th1 = wrapToPi(th + w*dt);
end

function [v, w] = saturateByWheelLimits(v, w, track, r, phidotMax)
% Enforce |v_L|, |v_R| <= r*phidotMax by scaling (v,w) together
    vL = v - w*(track/2);
    vR = v + w*(track/2);
    lim = r*phidotMax;
    s = max(1, max(abs([vL vR]))/lim);
    v = v / s;  w = w / s;
end

function [v, w, v_lat] = vw_from_gik_step(x0,y0,th0, x1,y1,th1, dt)
    dth = wrapToPi(th1 - th0);
    dx  = x1 - x0;  dy = y1 - y0;
    thm = wrapToPi(th0 + 0.5*dth);
    v     = (cos(thm)*dx + sin(thm)*dy) / dt;   % forward
    v_lat = (-sin(thm)*dx + cos(thm)*dy) / dt;  % sideways residual
    w     = dth / dt;
end

function log2 = truncateLog(log, k)
    f = fieldnames(log);
    for i=1:numel(f)
        log2.(f{i}) = log.(f{i})(1:k);
    end
end

function out = simulateFollowerPP(refined, params, pp)
% Follow the refined path with PP (or reuse PP) including wheel saturations,
% return executed trajectory and commands.
    x = refined.x(1);  y = refined.y(1);  th = refined.th(1);
    T = refined.t(end); dt = params.dt;
    N = round(T/dt)+1;

    out.t     = zeros(N,1);
    out.x     = zeros(N,1);
    out.y     = zeros(N,1);
    out.th    = zeros(N,1);
    out.v_cmd = zeros(N,1);
    out.w_cmd = zeros(N,1);

    % ensure PP has the waypoints
    pp.Waypoints = [refined.x refined.y];

    t=0;
    for k=1:N
        [v,w] = ppStepReverseAware(pp, [x y th], params);
        [v,w] = saturateByWheelLimits(v,w, params.track, params.wheelRadius, params.wheelRateMax);

        [x,y,th] = unicycleStep(x,y,th, v,w, dt);

        out.t(k)=t; out.x(k)=x; out.y(k)=y; out.th(k)=th;
        out.v_cmd(k)=v; out.w_cmd(k)=w;

        if norm([x y]-pp.Waypoints(end,:)) < params.goalRadius
            out = truncateLog(out, k);
            break;
        end
        t = t + dt;
    end
end

function q = armWarmStart(robot, eeName, q0)
% Keep a good warm-start for final pass (no-op here, but place-holder)
    q = q0;
    % You could compute IK for the very first EE pose and set arm joints close.
end

function refined = refineBasePathRSClothoid(baseRef, params)
% Placeholder for Stage B. Sketch:
%  1) Use stateSpaceReedsShepp + planner/connection to connect successive nodes.
%  2) Extract segment directions and cusps.
%  3) Clothoid smoothing for curvature continuity + build speed profile.
    refined.x  = baseRef(:,1);
    refined.y  = baseRef(:,2);
    refined.th = baseRef(:,3);
    refined.v  = params.vmax * ones(size(baseRef,1),1);
    refined.w  = zeros(size(baseRef,1),1);
    refined.t  = (0:numel(refined.x)-1)' * params.dt;
    warning("refineBasePathRSClothoid: placeholder returned unrefined path.");
end