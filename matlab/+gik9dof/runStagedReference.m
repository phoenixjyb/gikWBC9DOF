function result = runStagedReference(options)
%RUNSTAGEDREFERENCE Execute the staged pipeline with configurable defaults.
%   result = gik9dof.runStagedReference() runs the staged controller using
%   the shared environment configuration, writes the log to a timestamped
%   results folder, and returns metadata about the run. Name-value options:
%       PipelineConfig   - Unified configuration struct loaded via
%                          gik9dof.loadPipelineProfile('profile_name'). 
%                          Recommended over individual parameters below.
%       RunLabel         - Text appended to results folder name.
%       ExecutionMode    - 'ppForIk' (default) or 'pureIk'.
%       RateHz           - Control loop frequency (default 10 Hz).
%       MaxIterations    - Solver iteration cap (default 150).
%       UseStageBHybridAStar - Toggle Stage B hybrid A* (default true).
%       StageBMode       - 'pureHyb' (default) or 'gikInLoop'.
%       DistanceMargin   - Override environment DistanceMargin if provided.
%       DistanceWeight   - Override environment DistanceWeight if provided.
%       StageBDockingPositionTolerance - Override tolerance if provided.
%       StageBDockingYawTolerance      - Override tolerance if provided.
%       StageBLookaheadDistance        - Override Stage B lookahead (default 0.6 m).
%       StageBDesiredLinearVelocity    - Stage B nominal linear velocity (default 0.5 m/s).
%       StageBMaxAngularVelocity       - Stage B max yaw rate (default 2.0 rad/s).
%       StageBReedsSheppParams         - Struct of RS shortcut params (see
%                                        gik9dof.control.defaultReedsSheppParams).
%       StageBHybridResolution / MotionPrimitiveLength default to 0.05 m /
%       0.20 m for denser Hybrid A* primitives.
%       StageCUseBaseRefinement - Run RS/clothoid smoothing on Stage C base
%                                 ribbon prior to pure pursuit (default true).
%       StageBChassisControllerMode - 0 legacy diff, 1 heading, 2 pure pursuit,
%                                     -1 = take from chassis profile.
%       StageCChassisControllerMode - 0 legacy diff, 1 heading, 2 pure pursuit,
%                                     -1 = take from chassis profile.
%       SaveLog          - Save MAT file (default true).
%
%   Example (Recommended - Unified Config):
%       cfg = gik9dof.loadPipelineProfile('aggressive');
%       result = gik9dof.runStagedReference('PipelineConfig', cfg);
%
%   Example (Legacy - Individual Parameters):
%       result = gik9dof.runStagedReference('StageBDesiredLinearVelocity', 0.8);
%
arguments
    options.PipelineConfig struct = struct()  % NEW: Unified configuration from loadPipelineProfile
    options.RunLabel (1,1) string = "staged_reference"
    options.ExecutionMode (1,1) string {mustBeMember(options.ExecutionMode, ["ppForIk","pureIk","ppFirst"])} = "ppForIk"
    options.RateHz (1,1) double {mustBePositive} = 10
    options.MaxIterations (1,1) double {mustBePositive} = 1500
    options.UseStageBHybridAStar (1,1) logical = true
    options.StageBMode (1,1) string {mustBeMember(options.StageBMode, ["gikInLoop","pureHyb"])} = "pureHyb"
    options.DistanceMargin double = NaN
    options.DistanceWeight double = NaN
    options.StageBDockingPositionTolerance double = NaN
    options.StageBDockingYawTolerance double = NaN
    options.StageBLookaheadDistance (1,1) double {mustBePositive} = 0.6
    options.StageBDesiredLinearVelocity (1,1) double = 0.5
    options.StageBMaxAngularVelocity (1,1) double {mustBePositive} = 2.0
    options.StageBHybridResolution (1,1) double = 0.05
    options.StageBHybridSafetyMargin (1,1) double = 0.15
    options.StageBHybridMinTurningRadius (1,1) double = 0.5
    options.StageBHybridMotionPrimitiveLength (1,1) double = 0.2
    options.StageBUseReedsShepp (1,1) logical = false
    options.StageBReedsSheppParams struct = gik9dof.control.defaultReedsSheppParams()
    options.StageBUseClothoid (1,1) logical = false
    options.StageBClothoidParams struct = struct()
    options.StageCUseBaseRefinement (1,1) logical = true
    options.StageBChassisControllerMode (1,1) double {mustBeMember(options.StageBChassisControllerMode, [-1 0 1 2])} = -1
    options.StageCChassisControllerMode (1,1) double {mustBeMember(options.StageCChassisControllerMode, [-1 0 1 2])} = -1
    options.ChassisProfile (1,1) string = "wide_track"
    options.ChassisOverrides struct = struct()
    options.SaveLog (1,1) logical = true
end

% =========================================================================
% Apply unified PipelineConfig if provided  
% =========================================================================
if ~isempty(fieldnames(options.PipelineConfig))
    cfg = options.PipelineConfig;
    % Extract Stage B parameters from config if not explicitly overridden
    if isfield(cfg, 'stage_b')
        sb = cfg.stage_b;
        if isfield(sb, 'mode') && ismember('StageBMode', who('-file')), options.StageBMode = string(sb.mode); end
        if isfield(sb, 'lookahead_distance'), options.StageBLookaheadDistance = sb.lookahead_distance; end
        if isfield(sb, 'desired_linear_velocity'), options.StageBDesiredLinearVelocity = sb.desired_linear_velocity; end
        if isfield(sb, 'max_angular_velocity'), options.StageBMaxAngularVelocity = sb.max_angular_velocity; end
        if isfield(sb, 'hybrid_resolution'), options.StageBHybridResolution = sb.hybrid_resolution; end
        if isfield(sb, 'hybrid_safety_margin'), options.StageBHybridSafetyMargin = sb.hybrid_safety_margin; end
        if isfield(sb, 'hybrid_min_turning_radius'), options.StageBHybridMinTurningRadius = sb.hybrid_min_turning_radius; end
        if isfield(sb, 'hybrid_motion_primitive_length'), options.StageBHybridMotionPrimitiveLength = sb.hybrid_motion_primitive_length; end
        if isfield(sb, 'use_hybrid_astar'), options.UseStageBHybridAStar = sb.use_hybrid_astar; end
        if isfield(sb, 'use_reeds_shepp'), options.StageBUseReedsShepp = sb.use_reeds_shepp; end
        if isfield(sb, 'reeds_shepp_params'), options.StageBReedsSheppParams = sb.reeds_shepp_params; end
        if isfield(sb, 'use_clothoid'), options.StageBUseClothoid = sb.use_clothoid; end
        if isfield(sb, 'clothoid_params'), options.StageBClothoidParams = sb.clothoid_params; end
        if isfield(sb, 'chassis_controller_mode'), options.StageBChassisControllerMode = sb.chassis_controller_mode; end
    end
    if isfield(cfg, 'stage_c')
        sc = cfg.stage_c;
        if isfield(sc, 'use_base_refinement'), options.StageCUseBaseRefinement = sc.use_base_refinement; end
        if isfield(sc, 'chassis_controller_mode'), options.StageCChassisControllerMode = sc.chassis_controller_mode; end
    end
    if isfield(cfg, 'gik') && isfield(cfg.gik, 'max_iterations')
        options.MaxIterations = cfg.gik.max_iterations;
    end
    if isfield(cfg, 'chassis')
        options.ChassisOverrides = mergeStructs(options.ChassisOverrides, cfg.chassis);
    end
end

env = gik9dof.environmentConfig();

if ~isnan(options.DistanceMargin)
    env.DistanceMargin = options.DistanceMargin;
end
if ~isnan(options.DistanceWeight)
    env.DistanceWeight = options.DistanceWeight;
end
env.StageBMode = options.StageBMode;

if ~isnan(options.StageBDockingPositionTolerance)
    env.StageBDockingPositionTolerance = options.StageBDockingPositionTolerance;
end
if ~isnan(options.StageBDockingYawTolerance)
    env.StageBDockingYawTolerance = options.StageBDockingYawTolerance;
end

log = gik9dof.trackReferenceTrajectory( ...
    'PipelineConfig', options.PipelineConfig, ...  % Pass through unified config
    'Mode', 'staged', ...
    'RateHz', options.RateHz, ...
    'Verbose', false, ...
    'EnvironmentConfig', env, ...
    'UseStageBHybridAStar', options.UseStageBHybridAStar, ...
    'StageBMode', options.StageBMode, ...
    'StageBLookaheadDistance', options.StageBLookaheadDistance, ...
    'StageBDesiredLinearVelocity', options.StageBDesiredLinearVelocity, ...
    'StageBMaxAngularVelocity', options.StageBMaxAngularVelocity, ...
    'StageBHybridResolution', options.StageBHybridResolution, ...
    'StageBHybridSafetyMargin', options.StageBHybridSafetyMargin, ...
    'StageBHybridMinTurningRadius', options.StageBHybridMinTurningRadius, ...
    'StageBHybridMotionPrimitiveLength', options.StageBHybridMotionPrimitiveLength, ...
    'StageBUseReedsShepp', options.StageBUseReedsShepp, ...
    'StageBReedsSheppParams', options.StageBReedsSheppParams, ...
    'StageBUseClothoid', options.StageBUseClothoid, ...
    'StageBClothoidParams', options.StageBClothoidParams, ...
    'StageCUseBaseRefinement', options.StageCUseBaseRefinement, ...
    'StageBChassisControllerMode', options.StageBChassisControllerMode, ...
    'StageCChassisControllerMode', options.StageCChassisControllerMode, ...
    'StageBDockingPositionTolerance', env.StageBDockingPositionTolerance, ...
    'StageBDockingYawTolerance', env.StageBDockingYawTolerance, ...
    'DistanceMargin', env.DistanceMargin, ...
    'DistanceWeight', env.DistanceWeight, ...
    'FloorDiscs', env.FloorDiscs, ...
    'ChassisProfile', options.ChassisProfile, ...
    'ChassisOverrides', options.ChassisOverrides, ...
    'ExecutionMode', options.ExecutionMode, ...
    'MaxIterations', options.MaxIterations);

runDir = gik9dof.internal.createResultsFolder(options.RunLabel);
resultsDir = char(runDir);
logFilename = sprintf("log_staged_%s.mat", char(options.ExecutionMode));
logPath = fullfile(resultsDir, logFilename);

if options.SaveLog
    save(logPath, 'log', '-v7.3');
end

fprintf('Staged run (%s) complete. Log saved to %s\n', ...
    options.ExecutionMode, logPath);

result = struct();
result.log = log;
result.logPath = logPath;
result.resultsDir = resultsDir;
result.environment = env;
result.executionMode = options.ExecutionMode;
result.options = options;

end

%% Helper Functions
function merged = mergeStructs(base, override)
    %MERGESTRUCTS Deep merge two structures, with override taking precedence
    %   merged = mergeStructs(base, override)
    %   Fields in override will replace corresponding fields in base.
    %   For nested structs, recursively merges.
    
    merged = base;
    if isempty(override)
        return;
    end
    
    fields = fieldnames(override);
    for i = 1:length(fields)
        fname = fields{i};
        if isfield(merged, fname) && isstruct(merged.(fname)) && isstruct(override.(fname))
            % Recursively merge nested structs
            merged.(fname) = mergeStructs(merged.(fname), override.(fname));
        else
            % Direct override
            merged.(fname) = override.(fname);
        end
    end
end
