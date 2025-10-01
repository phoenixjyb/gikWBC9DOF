function artifacts = saveRunArtifacts(logInput, options)
%SAVERUNARTIFACTS Persist log, report, plots, and animation in results/.
%   artifacts = gik9dof.saveRunArtifacts(logInput) creates a timestamped
%   folder under results/ and stores the provided trajectory log, generated
%   tracking plot, solver report, and optional animation there. The function
%   returns a struct with paths to the generated artifacts.
%
%   Name-value options:
%       RunLabel          - Optional label appended to folder name.
%       GeneratePlot      - Whether to export tracking summary PNG (default true).
%       GenerateVideo     - Whether to export MP4 animation (default true).
%       SampleStep        - Frame subsampling step for animation (default 4).
%       FrameRate         - Animation frame rate in Hz (default 30).
%       PositionUnit      - Position unit label for evaluation (default 'm').
%       OrientationUnit   - Orientation unit ('deg' or 'rad', default 'deg').
%       EvaluateVerbose   - If true, evaluation prints to console (default false).
%
%   Example:
%       artifacts = gik9dof.saveRunArtifacts('trajectory_log.mat', ...
%           'RunLabel', 'baseline');
%
%   See also gik9dof.evaluateLog, gik9dof.plotTrajectoryLog,
%   gik9dof.animateTrajectory.

arguments
    logInput
    options.RunLabel (1,1) string = ""
    options.GeneratePlot (1,1) logical = true
    options.GenerateVideo (1,1) logical = true
    options.SampleStep (1,1) double {mustBeInteger, mustBePositive} = 4
    options.FrameRate (1,1) double {mustBePositive} = 30
    options.PositionUnit (1,1) string = "m"
    options.OrientationUnit (1,1) string = "deg"
    options.EvaluateVerbose (1,1) logical = false
    options.UseExternalPlots (1,1) logical = false
    options.ExternalPackagePath (1,1) string = "~/Projects/mobile_manip_3stage_package"
    options.ExternalSafetyMargin (1,1) double {mustBeNonnegative} = 0.0
end

log = resolveLog(logInput);
runDir = gik9dof.internal.createResultsFolder(options.RunLabel);

logPath = fullfile(runDir, 'run_log.mat');
save(logPath, 'log');

report = gik9dof.evaluateLog(log, ...
    'PositionUnit', options.PositionUnit, ...
    'OrientationUnit', options.OrientationUnit, ...
    'Verbose', options.EvaluateVerbose);

reportPath = fullfile(runDir, 'run_report.json');
reportJson = jsonencode(report);
fid = fopen(reportPath, 'w');
if fid == -1
    error("gik9dof:saveRunArtifacts:ReportWriteFailed", ...
        "Unable to write report to %s", reportPath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', reportJson);
clear cleanup

plotPath = "";
if options.GeneratePlot
    plotPath = fullfile(runDir, 'run_plot.png');
    gik9dof.plotTrajectoryLog(log, 'ExportPath', plotPath);
end

videoPath = "";
if options.GenerateVideo
    videoPath = fullfile(runDir, 'run_animation.mp4');
    gik9dof.animateTrajectory(log, ...
        'OutputVideo', videoPath, ...
        'SampleStep', options.SampleStep, ...
        'FrameRate', options.FrameRate);
end

externalFiles = struct('topView', "", 'threeD', "", 'errors', "");
if options.UseExternalPlots
    world = {};
    if isfield(log, 'floorDiscs') && ~isempty(log.floorDiscs) && isfield(log.floorDiscs, 'Center')
        worldRaw = arrayfun(@(d) createDiscCollision(d), log.floorDiscs, 'UniformOutput', false);
        world = worldRaw(~cellfun(@isempty, worldRaw));
    end
    extOutputs = gik9dof.generateExternalPlots(log, ...
        'ExternalPackagePath', options.ExternalPackagePath, ...
        'OutputDir', runDir, ...
        'World', world, ...
        'SafetyMargin', options.ExternalSafetyMargin);
    externalFiles.topView = extOutputs.files.topView;
    externalFiles.threeD = extOutputs.files.threeD;
    externalFiles.errors = extOutputs.files.errors;
end

artifacts = struct('runDir', runDir, ...
    'logPath', logPath, ...
    'reportPath', reportPath, ...
    'plotPath', plotPath, ...
    'videoPath', videoPath, ...
    'externalPlots', externalFiles);
end

function out = resolveLog(input)
if isstruct(input)
    out = input;
    return
end

if ~(ischar(input) || (isstring(input) && isscalar(input)))
    error("gik9dof:saveRunArtifacts:InvalidInput", ...
        "Input must be a log struct or path to a MAT file.");
end

path = gik9dof.internal.resolvePath(string(input));
if ~isfile(path)
    error("gik9dof:saveRunArtifacts:MissingFile", ...
        "Could not find log file: %s", path);
end

loaded = load(path, 'log');
if ~isfield(loaded, 'log')
    error("gik9dof:saveRunArtifacts:MissingVariable", ...
        "MAT file %s does not contain a variable named 'log'.", path);
end

out = loaded.log;
end

function cyl = createDiscCollision(disc)
if ~isfield(disc, 'Center') || numel(disc.Center) < 2
    cyl = [];
    return
end

if isfield(disc, 'SafetyMargin') && ~isempty(disc.SafetyMargin)
    margin = disc.SafetyMargin;
else
    margin = 0;
end
radius = disc.Radius + margin;
cyl = collisionCylinder(radius, 0.05);
pose = eye(4);
pose(1:2,4) = disc.Center(:);
cyl.Pose = pose;
end

function mustBeNonnegative(x)
if any(x < 0)
    error("gik9dof:saveRunArtifacts:NegativeValue", ...
        "Value must be non-negative.");
end
end
