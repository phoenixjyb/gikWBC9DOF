function report = evaluateLog(logInput, options)
%EVALUATELOG Compute summary metrics from a trajectory log.
%   report = gik9dof.evaluateLog(logInput) loads the provided log struct or
%   MAT-file and returns a struct containing position/orientation tracking
%   statistics, solver diagnostics, and constraint violation summaries.
%
%   gik9dof.evaluateLog(logInput, Name, Value, ...) supports name-value
%   options:
%       PositionUnit      - String label for distance units (default 'm').
%       OrientationUnit   - String label ('deg' or 'rad', default 'deg').
%       Verbose           - Logical flag; when true prints the report to the
%                           command window (default true).
%
%   Example:
%       report = gik9dof.evaluateLog('trajectory_log.mat');
%
%   See also gik9dof.runTrajectoryControl, gik9dof.trackReferenceTrajectory.

arguments
    logInput
    options.PositionUnit (1,1) string = "m"
    options.OrientationUnit (1,1) string {mustBeMember(options.OrientationUnit, ["deg","rad"])} = "deg"
    options.Verbose (1,1) logical = true
end

log = loadLog(logInput);

positionErr = getFieldOrEmpty(log, 'positionErrorNorm');
orientErr = getFieldOrEmpty(log, 'orientationErrorAngle');
iterations = getFieldOrEmpty(log, 'iterations');
exitFlags = getFieldOrEmpty(log, 'exitFlags');
violations = getFieldOrEmpty(log, 'constraintViolationMax');

report = struct();
report.numWaypoints = numel(log.successMask);
report.completedWaypoints = log.completedWaypoints;
report.successRate = safeDiv(nnz(log.successMask), report.numWaypoints);

if ~isempty(positionErr)
    report.positionMean = mean(positionErr, 'omitnan');
    report.positionMax = max(positionErr, [], 'omitnan');
else
    report.positionMean = NaN;
    report.positionMax = NaN;
end

if ~isempty(orientErr)
    if options.OrientationUnit == "deg"
        orientErr = rad2deg(orientErr);
    end
    report.orientationMean = mean(orientErr, 'omitnan');
    report.orientationMax = max(orientErr, [], 'omitnan');
    report.orientationUnit = options.OrientationUnit;
else
    report.orientationMean = NaN;
    report.orientationMax = NaN;
    report.orientationUnit = options.OrientationUnit;
end

if ~isempty(iterations)
    report.solverIterationsMean = mean(iterations, 'omitnan');
    report.solverIterationsMax = max(iterations, [], 'omitnan');
else
    report.solverIterationsMean = NaN;
    report.solverIterationsMax = NaN;
end

if ~isempty(exitFlags)
    report.exitFlagHistogram = groupcounts(exitFlags(:));
else
    report.exitFlagHistogram = [];
end

if ~isempty(violations)
    report.constraintViolationMax = max(violations, [], 'omitnan');
else
    report.constraintViolationMax = NaN;
end

report.positionUnit = options.PositionUnit;

if options.Verbose
    printReport(report);
end
end

function log = loadLog(input)
if isstruct(input)
    log = input;
    return
end

if ~(ischar(input) || (isstring(input) && isscalar(input)))
    error("gik9dof:evaluateLog:InvalidInput", ...
        "Input must be a log struct or path to a MAT file.");
end

path = gik9dof.internal.resolvePath(string(input));
if ~isfile(path)
    error("gik9dof:evaluateLog:MissingFile", ...
        "Could not find log file: %s", path);
end

loaded = load(path, 'log');
if ~isfield(loaded, 'log')
    error("gik9dof:evaluateLog:MissingVariable", ...
        "MAT file %s does not contain a variable named 'log'.", path);
end

log = loaded.log;
end

function value = getFieldOrEmpty(log, field)
if isfield(log, field)
    value = log.(field);
else
    value = [];
end
end

function ratio = safeDiv(a, b)
if b == 0
    ratio = NaN;
else
    ratio = a / b;
end
end

function printReport(report)
fprintf("Waypoints completed: %d/%d (%.1f%%)\n", ...
    report.completedWaypoints, report.numWaypoints, 100 * report.successRate);
if ~isnan(report.positionMean)
    fprintf("Position error mean/max: %.4f / %.4f %s\n", ...
        report.positionMean, report.positionMax, report.positionUnit);
end
if ~isnan(report.orientationMean)
    fprintf("Orientation error mean/max: %.4f / %.4f %s\n", ...
        report.orientationMean, report.orientationMax, report.orientationUnit);
end
if ~isnan(report.solverIterationsMean)
    fprintf("Solver iterations mean/max: %.1f / %.1f\n", ...
        report.solverIterationsMean, report.solverIterationsMax);
end
if ~isnan(report.constraintViolationMax)
    fprintf("Max constraint violation: %.4g\n", report.constraintViolationMax);
end
end
