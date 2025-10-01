function figs = plotTrajectoryLog(logInput, options)
%PLOTTRAJECTORYLOG Visualise joint trajectories and tracking errors.
%   gik9dof.plotTrajectoryLog(logInput) accepts either the log struct
%   returned by runTrajectoryControl/trackReferenceTrajectory or a path to a
%   MAT file containing a variable named 'log'. The function generates a
%   figure with joint trajectories and another with Cartesian tracking data.
%
%   gik9dof.plotTrajectoryLog(logInput, options) supports name-value pairs:
%       JointIndices  - Indices of joints to plot (default all entries).
%       ExportPath    - File path for saving the composite figure. When
%                       provided, the figure is saved and the handle is still
%                       returned.
%       PositionUnits - Label for positional plots (default 'm').
%
%   The function returns a struct with figure handles for further
%   customisation. Figures are left open unless an export path is supplied,
%   in which case the caller may close them manually if desired.
%
%   Example:
%       log = gik9dof.trackReferenceTrajectory('Verbose', false);
%       gik9dof.plotTrajectoryLog(log, 'ExportPath', 'tracking_summary.png');
%
%   See also gik9dof.runTrajectoryControl, gik9dof.trackReferenceTrajectory.

arguments
    logInput
    options.JointIndices (1,:) double = []
    options.ExportPath (1,1) string = ""
    options.PositionUnits (1,1) string = "m"
end

log = resolveLog(logInput);

if ~isfield(log, "qTraj") || ~isfield(log, "timestamps")
    error("gik9dof:plotTrajectoryLog:MissingFields", ...
        "Log does not contain required fields 'qTraj' and 'timestamps'.");
end

numJoints = size(log.qTraj, 1);
if isempty(options.JointIndices)
    jointIdx = 1:numJoints;
else
    jointIdx = unique(options.JointIndices);
    if any(jointIdx < 1) || any(jointIdx > numJoints)
        error("gik9dof:plotTrajectoryLog:JointIndexOutOfBounds", ...
            "Joint indices must be within [1, %d].", numJoints);
    end
end

% Time vector aligns initial configuration at t=0.
if isfield(log, "time") && numel(log.time) == size(log.qTraj, 2)
    timeVec = log.time;
else
    timeVec = [0, reshape(log.timestamps, 1, [])];
end

if numel(timeVec) ~= size(log.qTraj, 2)
    error("gik9dof:plotTrajectoryLog:TimeMismatch", ...
        "Time vector length (%d) does not match qTraj columns (%d).", ...
        numel(timeVec), size(log.qTraj, 2));
end

% Main figure using tiled layout.
fig = figure('Name', 'GIK Trajectory Summary', 'NumberTitle', 'off');
numTiles = 2 + double(isfield(log, "orientationErrorAngle") && ~isempty(log.orientationErrorAngle));
tiledlayout(fig, numTiles, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

% Joint trajectories.
nexttile;
plot(timeVec, log.qTraj(jointIdx, :)');
xlabel('Time [s]');
ylabel('Joint Position [rad]');
title('Joint Trajectories');
legend(arrayfun(@(i) sprintf('q_{%d}', i), jointIdx, 'UniformOutput', false), ...
    'Location', 'eastoutside');

% Cartesian tracking.
if ~isfield(log, "eePositions") || ~isfield(log, "targetPositions")
    warning("gik9dof:plotTrajectoryLog:MissingCartesianData", ...
        "Log lacks end-effector or target positions; skipping Cartesian plot.");
    figs.primary = fig;
    return
end

positions = log.eePositions;
targets = log.targetPositions;
errors = positions - targets;
errorNorm = vecnorm(errors, 2, 1);

nexttile;
plot(timeVec(2:end), targets', '--', 'LineWidth', 1.2);
hold on;
plot(timeVec(2:end), positions', 'LineWidth', 1.5);
plot(timeVec(2:end), errorNorm', 'k', 'LineWidth', 1);
hold off;
xlabel('Time [s]');
ylabel(sprintf('Position [%s]', options.PositionUnits));
title('End-Effector Position Tracking');
legend({'Target X', 'Target Y', 'Target Z', ...
        'Actual X', 'Actual Y', 'Actual Z', ...
        'Error Norm'}, 'Location', 'eastoutside');

% Orientation error tile when available.
if isfield(log, "orientationErrorAngle") && ~isempty(log.orientationErrorAngle)
    nexttile;
    plot(timeVec(2:end), rad2deg(log.orientationErrorAngle), 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Orientation Error [deg]');
    title('End-Effector Orientation Error');
    grid on;
end

% Optional export.
if strlength(options.ExportPath) > 0
    exportPath = gik9dof.internal.resolvePath(options.ExportPath);
    [exportDir, ~, ~] = fileparts(exportPath);
    if exportDir ~= "" && ~isfolder(exportDir)
        mkdir(exportDir);
    end
    exportgraphics(fig, exportPath, 'Resolution', 150);
end

figs.primary = fig;
end

function log = resolveLog(input)
if isstruct(input)
    log = input;
    return
end

if ~(ischar(input) || (isstring(input) && isscalar(input)))
    error("gik9dof:plotTrajectoryLog:InvalidInput", ...
        "Input must be a log struct or path to a MAT file.");
end

path = gik9dof.internal.resolvePath(string(input));
if ~isfile(path)
    error("gik9dof:plotTrajectoryLog:MissingFile", ...
        "Could not find log file: %s", path);
end

loaded = load(path, 'log');
if ~isfield(loaded, 'log')
    error("gik9dof:plotTrajectoryLog:MissingVariable", ...
        "MAT file %s does not contain a variable named 'log'.", path);
end

log = loaded.log;
end
