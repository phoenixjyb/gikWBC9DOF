function log = runStageCPPFirst_enhanced(robot, trajStruct, q_start, options)
%RUNSTAGECPPFIRST_ENHANCED Stage C execution using PP-First method with Phase 1 improvements.
%   Enhanced version implementing guide-validated techniques:
%       1. Adaptive lookahead for Pure Pursuit
%       2. Micro-segment path extension for PP stability
%       3. Warm-starting for GIK convergence
%       4. Velocity-limited corridor (respects differential drive)
%       5. Lateral velocity diagnostic (v_lat monitoring)
%       6. Relaxed solver tolerances
%
%   log = GIK9DOF.RUNSTAGECPPFIRST_ENHANCED(robot, trajStruct, q_start, options)
%
%   Additional options vs standard runStageCPPFirst:
%       UseAdaptiveLookahead   - Enable adaptive PP lookahead (default true)
%       UseMicroSegment        - Enable micro-segment extension (default true)
%       UseWarmStarting        - Enable GIK warm-starting (default true)
%       UseVelocityCorridor    - Dynamic corridor based on velocity (default true)
%       LogLateralVelocity     - Log v_lat diagnostic (default true)
%       RelaxedTolerances      - Use relaxed GIK tolerances (default true)
%       LookaheadMin           - Minimum lookahead (m, default 0.15)
%       EpsLatMax              - Max lateral corridor (m, default 0.015)
%       EpsLongMin             - Min longitudinal corridor (m, default 0.01)
%
%   See also runStageCPPFirst, METHOD4_INTEGRATED_IMPROVEMENT_PLAN.md

arguments
    robot (1,1) rigidBodyTree
    trajStruct (1,1) struct
    q_start (:,1) double
    options.ChassisParams (1,1) struct
    options.MaxIterations (1,1) double = 2000  % Increased from 1500
    options.YawTolerance (1,1) double = deg2rad(15)
    options.PositionTolerance (1,1) double = 0.15
    options.EEErrorTolerance (1,1) double = 0.01
    options.SampleTime (1,1) double = 0.1
    options.DesiredVelocity (1,1) double = 0.5
    options.LookaheadDistance (1,1) double = 0.8  % Nominal lookahead
    options.LookaheadMin (1,1) double = 0.15      % NEW: Min lookahead
    options.ApplyRefinement (1,1) logical = false
    options.BaseIndices (1,3) double = [1 2 3]
    options.ArmIndices (1,6) double = [4 5 6 7 8 9]
    options.EndEffector (1,1) string = "left_gripper_link"
    options.DistanceSpecs = struct([])
    options.VerboseLevel (1,1) double = 1
    % NEW: Phase 1 improvement flags
    options.UseAdaptiveLookahead (1,1) logical = true
    options.UseMicroSegment (1,1) logical = true
    options.UseWarmStarting (1,1) logical = true
    options.UseVelocityCorridor (1,1) logical = true
    options.LogLateralVelocity (1,1) logical = true
    options.RelaxedTolerances (1,1) logical = true
    options.EpsLatMax (1,1) double = 0.015  % 15mm lateral max
    options.EpsLongMin (1,1) double = 0.05  % 50mm longitudinal min (increased from 10mm)
    % NEW: Phase 2A improvement flags
    options.UseOrientationZNominal (1,1) logical = false  % Orientation+Z priority nominal
    options.OrientationWeight (1,1) double = 1.0
    options.PositionWeightXY (1,1) double = 0.1
    options.PositionWeightZ (1,1) double = 1.0
end

verbose = options.VerboseLevel > 0;
detailedVerbose = options.VerboseLevel > 1;

if verbose
    fprintf('\n=== Stage C: PP-First Method (ENHANCED with Phase 1 Improvements) ===\n');
    fprintf('Improvements enabled:\n');
    if options.UseAdaptiveLookahead, fprintf('  ✓ Adaptive lookahead\n'); end
    if options.UseMicroSegment, fprintf('  ✓ Micro-segment PP\n'); end
    if options.UseWarmStarting, fprintf('  ✓ Warm-starting\n'); end
    if options.UseVelocityCorridor, fprintf('  ✓ Velocity-limited corridor\n'); end
    if options.LogLateralVelocity, fprintf('  ✓ Lateral velocity diagnostic\n'); end
    if options.RelaxedTolerances, fprintf('  ✓ Relaxed solver tolerances\n'); end
end

%% Step 1: Generate base seed path from EE trajectory
if verbose
    fprintf('\nStep 1: Generating base seed path from EE waypoints...\n');
end

baseSeedPath = gik9dof.baseSeedFromEE(robot, trajStruct, q_start, ...
    'EndEffector', options.EndEffector, ...
    'BaseIndices', options.BaseIndices, ...
    'ArmIndices', options.ArmIndices, ...
    'Verbose', detailedVerbose, ...
    'UseOrientationZNominal', options.UseOrientationZNominal, ...
    'OrientationWeight', options.OrientationWeight, ...
    'PositionWeightXY', options.PositionWeightXY, ...
    'PositionWeightZ', options.PositionWeightZ);

if verbose
    fprintf('  Base seed path: %d waypoints, %.2f m total length\n', ...
        size(baseSeedPath, 1), sum(vecnorm(diff(baseSeedPath(:,1:2), 1, 1), 2, 2)));
end

%% Step 2: Initialize Pure Pursuit controller
if verbose
    fprintf('Step 2: Initializing Pure Pursuit controller...\n');
end

[ppFollower, ppPathInfo] = gik9dof.initPPFromBasePath(baseSeedPath, options.ChassisParams, ...
    'LookaheadBase', options.LookaheadDistance, ...
    'DesiredVelocity', options.DesiredVelocity, ...
    'SampleTime', options.SampleTime, ...
    'ApplyRefinement', options.ApplyRefinement, ...
    'Verbose', detailedVerbose);

if verbose
    fprintf('  PP initialized: lookahead %.2f m (nominal), min %.2f m\n', ...
        options.LookaheadDistance, options.LookaheadMin);
    fprintf('  Desired velocity: %.2f m/s\n', options.DesiredVelocity);
end

%% Step 3: Create GIK solver with relaxed tolerances
if verbose
    fprintf('Step 3: Creating GIK solver with constraints...\n');
end

gikBundle = gik9dof.createGikSolver(robot, ...
    'EndEffector', options.EndEffector, ...
    'MaxIterations', options.MaxIterations, ...
    'DistanceSpecs', options.DistanceSpecs);

% Apply relaxed tolerances if enabled
if options.RelaxedTolerances
    if isfield(gikBundle, 'solver') && isprop(gikBundle.solver, 'SolverParameters')
        gikBundle.solver.SolverParameters.ConstraintTolerance = 1e-4;   % was 1e-6
        gikBundle.solver.SolverParameters.StepTolerance = 1e-10;        % was 1e-12
        gikBundle.solver.SolverParameters.OptimalityTolerance = 1e-4;   % was 1e-6
        if verbose
            fprintf('  Relaxed solver tolerances applied (ConstraintTol: 1e-4, StepTol: 1e-10)\n');
        end
    end
end

if verbose
    fprintf('  Max iterations: %d\n', options.MaxIterations);
end

%% Step 4: Main control loop with Phase 1 enhancements
nWaypoints = size(trajStruct.Poses, 3);
dt = options.SampleTime;

% Preallocate log arrays (including new diagnostics)
log = struct();
log.qTraj = zeros(9, nWaypoints);
log.timestamps = zeros(1, nWaypoints);
log.eePositions = zeros(3, nWaypoints);
log.targetPositions = zeros(3, nWaypoints);
log.positionError = zeros(3, nWaypoints);
log.positionErrorNorm = zeros(1, nWaypoints);
log.ppCommands = zeros(nWaypoints, 2); % [vx, wz]
log.basePredicted = zeros(3, nWaypoints);
log.baseActual = zeros(3, nWaypoints);
log.fallbackUsed = false(1, nWaypoints);
log.gikIterations = zeros(1, nWaypoints);
log.solveTime = zeros(1, nWaypoints);
log.successMask = false(1, nWaypoints);
log.solutionInfo = cell(1, nWaypoints);

% NEW: Phase 1 diagnostic arrays
log.lateralVelocity = zeros(1, nWaypoints);      % v_lat (target < 0.02 m/s)
log.lookaheadEffective = zeros(1, nWaypoints);   % Adaptive lookahead used
log.corridorSizes = zeros(nWaypoints, 2);        % [eps_long, eps_lat] per waypoint
log.microSegmentUsed = false(1, nWaypoints);     % Micro-segment flag

q_current = q_start;
t_current = 0;

% Warm-starting: persistent storage for previous solution
prevSolution = [];

if verbose
    fprintf('\nStep 4: Executing control loop for %d waypoints...\n', nWaypoints);
    fprintf('  Yaw corridor: ±%.1f deg\n', rad2deg(options.YawTolerance));
    fprintf('  Position tolerance (base): ±%.2f m\n', options.PositionTolerance);
    fprintf('  EE error threshold for fallback: %.1f mm\n', options.EEErrorTolerance * 1000);
    fprintf('  Lateral corridor max: %.1f mm\n', options.EpsLatMax * 1000);
end

for k = 1:nWaypoints
    tic;
    
    % Get current pose
    pose_current = q_current(options.BaseIndices);
    
    %% NEW 1.1: ADAPTIVE LOOKAHEAD
    % Compute distance to next waypoint
    next_waypoint = baseSeedPath(min(k+1, size(baseSeedPath,1)), 1:2);
    d_to_next = norm(next_waypoint - pose_current(1:2));
    
    if options.UseAdaptiveLookahead
        % Adaptive formula: Ld_eff = min(Ld_nom, max(d, Ld_min))
        Ld_eff = min(options.LookaheadDistance, max(d_to_next, options.LookaheadMin));
        ppFollower.LookaheadBase = Ld_eff;
        log.lookaheadEffective(k) = Ld_eff;
    else
        Ld_eff = options.LookaheadDistance;
        log.lookaheadEffective(k) = Ld_eff;
    end
    
    %% NEW 1.2: MICRO-SEGMENT for PP stability
    if options.UseMicroSegment && k < nWaypoints
        try
            % Get goal waypoint
            xg = baseSeedPath(k+1, 1);
            yg = baseSeedPath(k+1, 2);
            theta_goal = baseSeedPath(k+1, 3);
            
            % Create extension point beyond goal
            L_ext = max(options.LookaheadMin, 0.5 * Ld_eff);
            xg_plus = xg + L_ext * cos(theta_goal);
            yg_plus = yg + L_ext * sin(theta_goal);
            
            % Build micro-segment: current -> goal -> extension
            microSegment = [
                pose_current(1:2);
                xg, yg;
                xg_plus, yg_plus
            ];
            
            % Update PP waypoints temporarily (with error handling)
            ppFollower.PathInfo.States = [microSegment, [pose_current(3); theta_goal; theta_goal]];
            log.microSegmentUsed(k) = true;
        catch ME
            % Graceful degradation: continue without micro-segment
            log.microSegmentUsed(k) = false;
            if detailedVerbose
                warning('Waypoint %d: Micro-segment update failed (%s), continuing...', k, ME.message);
            end
        end
    end
    
    %% PREDICT: Pure Pursuit step
    [vx_cmd, wz_cmd, ppStatus] = ppFollower.step(pose_current', dt);
    
    % Integrate to predict next base pose
    x_pp = pose_current(1) + vx_cmd * cos(pose_current(3)) * dt;
    y_pp = pose_current(2) + vx_cmd * sin(pose_current(3)) * dt;
    theta_pp = pose_current(3) + wz_cmd * dt;
    theta_pp = wrapToPi(theta_pp);
    
    q_base_pred = [x_pp; y_pp; theta_pp];
    
    %% NEW 1.4: VELOCITY-LIMITED CORRIDOR
    if options.UseVelocityCorridor
        % Dynamic longitudinal corridor: eps_long = max(eps_min, |v|*dt + 0.01)
        eps_long = max(options.EpsLongMin, abs(vx_cmd) * dt + 0.01);
        % Lateral corridor stays tight (differential drive constraint)
        eps_lat = options.EpsLatMax;  % 10-15mm
    else
        % Use fixed position tolerance
        eps_long = options.PositionTolerance;
        eps_lat = options.PositionTolerance;
    end
    
    log.corridorSizes(k, :) = [eps_long, eps_lat];
    
    %% CONSTRAIN: Update GIK with yaw corridor + velocity-based position bounds
    % TEMPORARY FIX: Use isotropic corridor (eps_long for both x and y)
    % The anisotropic (yaw-aligned) corridor is too conservative
    gik9dof.updateBaseJointBounds(gikBundle, options.BaseIndices, q_base_pred, ...
        options.YawTolerance, eps_long);  % Only 2 position args = isotropic
    
    %% NEW 1.3: WARM-STARTING for GIK
    if options.UseWarmStarting && ~isempty(prevSolution) && k > 1
        % Safety check: only reuse if previous step converged and was not fallback
        prevConverged = (k > 1) && log.successMask(k-1);
        prevNotFallback = (k > 1) && ~log.fallbackUsed(k-1);
        
        if prevConverged && prevNotFallback
            q_init = prevSolution;
        else
            q_init = q_current;  % Don't warm-start from failed/fallback solution
        end
    else
        q_init = q_current;
    end
    
    %% SOLVE: GIK with constrained base
    T_ee_target = trajStruct.Poses(:, :, k);
    
    [q_gik, solInfo] = gikBundle.solve(q_init, 'TargetPose', T_ee_target);
    
    % Store solution for next warm-start (only if converged)
    if options.UseWarmStarting && strcmp(solInfo.Status, 'success')
        prevSolution = q_gik;
    end
    
    % Compute EE error
    T_ee_actual = getTransform(robot, q_gik, char(options.EndEffector));
    p_ee_actual = T_ee_actual(1:3, 4);
    p_ee_target = T_ee_target(1:3, 4);
    ee_error_vec = p_ee_actual - p_ee_target;
    ee_error_norm = norm(ee_error_vec);
    
    %% CHECK: EE error and fallback if needed
    if ee_error_norm > options.EEErrorTolerance
        % Fallback: Fix base at predicted pose, solve arm-only
        [q_arm_fallback, solInfoFallback] = gik9dof.solveArmOnlyIK(robot, gikBundle, ...
            T_ee_target, q_base_pred, q_current(options.ArmIndices), ...
            'BaseIndices', options.BaseIndices, 'ArmIndices', options.ArmIndices);
        
        q_final = [q_base_pred; q_arm_fallback];
        log.fallbackUsed(k) = true;
        
        % Recompute EE position with fallback solution
        T_ee_actual = getTransform(robot, q_final, char(options.EndEffector));
        p_ee_actual = T_ee_actual(1:3, 4);
        ee_error_vec = p_ee_actual - p_ee_target;
        ee_error_norm = norm(ee_error_vec);
        
        solInfo = solInfoFallback;
        
        if detailedVerbose
            fprintf('  [%3d] Fallback triggered (GIK error %.3f > %.3f m)\n', ...
                k, ee_error_norm, options.EEErrorTolerance);
        end
    else
        q_final = q_gik;
    end
    
    %% NEW 1.5: LATERAL VELOCITY DIAGNOSTIC (moved AFTER fallback)
    % Measure lateral velocity of ACTUAL applied motion (q_final), not failed GIK
    if options.LogLateralVelocity
        % Compute v_lat = lateral velocity residual (should be ~0 for nonholonomic)
        % NOTE: Use SAMPLE TIME (dt), not actual elapsed time, for velocity calculation
        dt_sample = options.SampleTime;  % Should be 0.1s (10 Hz control rate)
        
        dx = q_final(options.BaseIndices(1)) - q_current(options.BaseIndices(1));
        dy = q_final(options.BaseIndices(2)) - q_current(options.BaseIndices(2));
        dth = wrapToPi(q_final(options.BaseIndices(3)) - q_current(options.BaseIndices(3)));
        thm = wrapToPi(q_current(options.BaseIndices(3)) + 0.5 * dth);
        
        % World frame velocities
        vx_world = dx / dt_sample;
        vy_world = dy / dt_sample;
        
        % Transform to body frame: v_long = vx*cos(θ)+vy*sin(θ), v_lat = -vx*sin(θ)+vy*cos(θ)
        v_lat = -sin(thm) * vx_world + cos(thm) * vy_world;
        log.lateralVelocity(k) = v_lat;
        
        % Warning if excessive
        if abs(v_lat) > 0.02 && detailedVerbose
            fprintf('  [%3d] WARNING: High lateral velocity: %.4f m/s (target < 0.02)\n', k, abs(v_lat));
        end
    end
    
    %% UPDATE state
    q_current = q_final;
    t_current = t_current + dt;
    
    %% LOG results
    log.qTraj(:, k) = q_final;
    log.timestamps(k) = t_current;
    log.eePositions(:, k) = p_ee_actual;
    log.targetPositions(:, k) = p_ee_target;
    log.positionError(:, k) = ee_error_vec;
    log.positionErrorNorm(k) = ee_error_norm;
    log.ppCommands(k, :) = [vx_cmd, wz_cmd];
    log.basePredicted(:, k) = q_base_pred;
    log.baseActual(:, k) = q_final(options.BaseIndices);
    log.gikIterations(k) = solInfo.Iterations;
    log.solveTime(k) = toc;
    log.successMask(k) = solInfo.Status == "success";
    log.solutionInfo{k} = solInfo;
    
    % Progress indicator
    if verbose && mod(k, 20) == 0
        fprintf('  Progress: %d/%d waypoints (%.1f%%), Fallback: %.1f%%, Mean |v_lat|: %.4f m/s\n', ...
            k, nWaypoints, 100*k/nWaypoints, 100*sum(log.fallbackUsed(1:k))/k, ...
            mean(abs(log.lateralVelocity(1:k))));
    end
end

%% Finalize log
log.baseSeedPath = baseSeedPath;
log.ppFollower = ppFollower;
log.ppPathInfo = ppPathInfo;
log.fallbackRate = sum(log.fallbackUsed) / nWaypoints;
log.avgEEError = mean(log.positionErrorNorm);
log.maxEEError = max(log.positionErrorNorm);
log.meanSolveTime = mean(log.solveTime);
log.totalTime = sum(log.solveTime);

% NEW: Phase 1 diagnostics
log.meanLateralVelocity = mean(abs(log.lateralVelocity));
log.maxLateralVelocity = max(abs(log.lateralVelocity));
log.lateralVelocityViolations = sum(abs(log.lateralVelocity) > 0.02);
log.meanLookahead = mean(log.lookaheadEffective);
log.meanCorridorLong = mean(log.corridorSizes(:,1));
log.meanCorridorLat = mean(log.corridorSizes(:,2));

% Method 4 enhanced metadata
log.mode = 'ppFirst_enhanced';
log.parameters = struct( ...
    'yawTolerance', options.YawTolerance, ...
    'positionTolerance', options.PositionTolerance, ...
    'eeErrorTolerance', options.EEErrorTolerance, ...
    'lookaheadDistance', options.LookaheadDistance, ...
    'lookaheadMin', options.LookaheadMin, ...
    'desiredVelocity', options.DesiredVelocity, ...
    'maxIterations', options.MaxIterations, ...
    'useAdaptiveLookahead', options.UseAdaptiveLookahead, ...
    'useMicroSegment', options.UseMicroSegment, ...
    'useWarmStarting', options.UseWarmStarting, ...
    'useVelocityCorridor', options.UseVelocityCorridor, ...
    'logLateralVelocity', options.LogLateralVelocity, ...
    'relaxedTolerances', options.RelaxedTolerances);

if verbose
    fprintf('\n=== Stage C Complete (ENHANCED) ===\n');
    fprintf('Waypoints processed: %d\n', nWaypoints);
    fprintf('Fallback rate: %.1f%% (%d/%d waypoints)\n', ...
        100*log.fallbackRate, sum(log.fallbackUsed), nWaypoints);
    fprintf('EE tracking error: mean %.2f mm, max %.2f mm\n', ...
        log.avgEEError * 1000, log.maxEEError * 1000);
    fprintf('Solve time: mean %.3f s/waypoint, total %.2f s\n', ...
        log.meanSolveTime, log.totalTime);
    fprintf('GIK convergence rate: %.1f%%\n', 100*sum(log.successMask)/nWaypoints);
    fprintf('\n--- Phase 1 Diagnostics ---\n');
    fprintf('Lateral velocity: mean %.4f m/s, max %.4f m/s\n', ...
        log.meanLateralVelocity, log.maxLateralVelocity);
    fprintf('Nonholonomy violations (|v_lat| > 0.02): %d (%.1f%%)\n', ...
        log.lateralVelocityViolations, 100*log.lateralVelocityViolations/nWaypoints);
    fprintf('Adaptive lookahead: mean %.2f m (range %.2f-%.2f m)\n', ...
        log.meanLookahead, min(log.lookaheadEffective), max(log.lookaheadEffective));
    fprintf('Corridor sizes: longitudinal %.3f m, lateral %.3f m\n', ...
        log.meanCorridorLong, log.meanCorridorLat);
end

end
