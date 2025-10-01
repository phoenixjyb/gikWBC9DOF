function artifacts = demoFloorDiscs(options)
%DEMOFLOORDISCS Showcase floor-disc avoidance using the reference trajectory.
%   artifacts = gik9dof.demoFloorDiscs() runs the reference trajectory while
%   enforcing distance constraints to two floor discs near the base. The
%   function saves the resulting log/plots to the specified output folder and
%   returns a struct describing generated artifacts.
%
%   Name-value options:
%       Discs          - Struct array defining discs (default demo set).
%       DistanceMargin - Additional clearance added to each disc (default 0.05 m).
%       DistanceWeight - Distance constraint weight (default 0.8).
%       RunLabel       - Optional label used in the results folder name
%                        (default 'floor_demo').
%       GeneratePlot   - Logical flag to render tracking summary PNG (default true).
%       GenerateVideo  - Logical flag to export MP4 animation (default false).
%       Verbose        - Logical flag for on-screen reporting (default true).
%
%   Example:
%       artifacts = gik9dof.demoFloorDiscs('GenerateVideo', true);
%
%   See also gik9dof.trackReferenceTrajectory, gik9dof.evaluateLog,
%   gik9dof.plotTrajectoryLog, gik9dof.animateTrajectory.

arguments
    options.Discs (1,:) struct = defaultDiscs()
    options.DistanceMargin (1,1) double {mustBeNonnegative} = 0.05
    options.DistanceWeight (1,1) double {mustBeNonnegative} = 0.8
    options.RunLabel (1,1) string = "floor_demo"
    options.GeneratePlot (1,1) logical = true
    options.GenerateVideo (1,1) logical = false
    options.Verbose (1,1) logical = true
    options.ControlMode (1,1) string {mustBeMember(options.ControlMode, ["holistic","staged"])} = "staged"
end

log = gik9dof.trackReferenceTrajectory( ...
    "Verbose", false, ...
    "FloorDiscs", options.Discs, ...
    "DistanceMargin", options.DistanceMargin, ...
    "DistanceWeight", options.DistanceWeight, ...
    "Mode", options.ControlMode);

artifacts = gik9dof.saveRunArtifacts(log, ...
    'RunLabel', options.RunLabel, ...
    'GeneratePlot', options.GeneratePlot, ...
    'GenerateVideo', options.GenerateVideo, ...
    'SampleStep', 4, ...
    'EvaluateVerbose', options.Verbose, ...
    'UseExternalPlots', true);

artifacts.floorDiscs = log.floorDiscs;
artifacts.distanceSpecs = log.distanceSpecs;
end

function discs = defaultDiscs()
discs = struct( ...
    'Center', {[-0.2, 0.3], [0.8, -0.4]}, ...
    'Radius', {0.35, 0.25}, ...
    'SafetyMargin', {0.08, 0.05});
end
