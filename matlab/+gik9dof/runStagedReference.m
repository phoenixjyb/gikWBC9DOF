function result = runStagedReference(options)
%RUNSTAGEDREFERENCE Execute the staged pipeline with configurable defaults.
%   result = gik9dof.runStagedReference() runs the staged controller using
%   the shared environment configuration, writes the log to a timestamped
%   results folder, and returns metadata about the run. Name-value options:
%       RunLabel         - Text appended to results folder name.
%       ExecutionMode    - 'ppForIk' (default) or 'pureIk'.
%       RateHz           - Control loop frequency (default 30 Hz).
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
%       SaveLog          - Save MAT file (default true).
%
arguments
    options.RunLabel (1,1) string = "staged_reference"
    options.ExecutionMode (1,1) string {mustBeMember(options.ExecutionMode, ["ppForIk","pureIk"])} = "ppForIk"
    options.RateHz (1,1) double {mustBePositive} = 30
    options.MaxIterations (1,1) double {mustBePositive} = 150
    options.UseStageBHybridAStar (1,1) logical = true
    options.StageBMode (1,1) string {mustBeMember(options.StageBMode, ["gikInLoop","pureHyb"])} = "pureHyb"
    options.DistanceMargin double = NaN
    options.DistanceWeight double = NaN
    options.StageBDockingPositionTolerance double = NaN
    options.StageBDockingYawTolerance double = NaN
    options.StageBLookaheadDistance (1,1) double {mustBePositive} = 0.6
    options.StageBDesiredLinearVelocity (1,1) double = 0.5
    options.StageBMaxAngularVelocity (1,1) double {mustBePositive} = 2.0
    options.StageBHybridResolution (1,1) double = 0.1
    options.StageBHybridSafetyMargin (1,1) double = 0.15
    options.StageBHybridMinTurningRadius (1,1) double = 0.5
    options.StageBHybridMotionPrimitiveLength (1,1) double = 0.5
    options.StageBUseReedsShepp (1,1) logical = false
    options.StageBReedsSheppNumSamples (1,1) double {mustBePositive} = 50
    options.ChassisProfile (1,1) string = "wide_track"
    options.ChassisOverrides struct = struct()
    options.SaveLog (1,1) logical = true
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
    'StageBReedsSheppNumSamples', options.StageBReedsSheppNumSamples, ...
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
