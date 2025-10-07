function summary = run_stageb_mode_compare(options)
%RUN_STAGEB_MODE_COMPARE Compare Stage B submodes (gikInLoop vs pureHyb).
%   summary = run_stageb_mode_compare() executes the staged controller twice,
%   once with Stage B running through the legacy GIK loop and once using the
%   pure hybrid (Hybrid A* + pure pursuit) submode. Logs, path samples, and
%   command tables are saved under results/<timestamp>_stageB_compare/ for
%   offline analysis.
%
%   Optional name-value:
%       RateHz  - Control rate (default 50 Hz)
%       UseStageBHybridAStar - Whether to enable the planner (default true)
%       Label   - Optional suffix appended to the results folder name.
%
%   Example:
%       run_stageb_mode_compare('RateHz', 40);
%
arguments
    options.RateHz (1,1) double {mustBePositive} = 50
    options.UseStageBHybridAStar (1,1) logical = true
    options.Label (1,1) string = ""
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(projectRoot, 'matlab')));

env = gik9dof.environmentConfig();

timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
folderName = timestamp + "_stageB_compare";
if strlength(options.Label) > 0
    folderName = folderName + "_" + options.Label;
end
resultsDir = fullfile(projectRoot, 'results', folderName);
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

modes = ["gikInLoop", "pureHyb"];
modeResults = struct([]);

for idx = 1:numel(modes)
    modeName = modes(idx);
    fprintf("\n[StageB-Compare] Running mode %s...\n", modeName);
    log = gik9dof.trackReferenceTrajectory( ...
        'Mode', 'staged', ...
        'Verbose', false, ...
        'RateHz', options.RateHz, ...
        'EnvironmentConfig', env, ...
        'UseStageBHybridAStar', options.UseStageBHybridAStar, ...
        'StageBMode', modeName);

    logPath = fullfile(resultsDir, "log_stageB_" + lower(modeName) + ".mat");
    save(logPath, 'log');

    stageB = log.stageLogs.stageB;
    pathStates = stageB.pathStates;
    pathFile = fullfile(resultsDir, "stageB_path_" + lower(modeName) + ".csv");
    writematrix(pathStates, pathFile);

    execFile = fullfile(resultsDir, "stageB_exec_" + lower(modeName) + ".csv");
    if isfield(stageB, 'execBaseStates') && ~isempty(stageB.execBaseStates)
        writematrix(stageB.execBaseStates, execFile);
        execStates = stageB.execBaseStates;
    else
        execStates = pathStates;
        writematrix(execStates, execFile);
    end

    cmdFile = fullfile(resultsDir, "stageB_cmd_" + lower(modeName) + ".csv");
    if isfield(stageB, 'cmdLog') && ~isempty(stageB.cmdLog) && height(stageB.cmdLog) > 0
        cmdLog = stageB.cmdLog;
    else
        cmdLog = table('Size', [0 3], 'VariableTypes', {'double','double','double'}, ...
            'VariableNames', {'time','Vx','Wz'});
    end

    if height(cmdLog) == 0 && isfield(stageB, 'baseVelocityEstimate') && ...
            ~isempty(stageB.baseVelocityEstimate)
        est = stageB.baseVelocityEstimate;
        estTime = est.time(:);
        vxRobot = est.vxRobot(:);
        omegaRobot = est.omega(:);
        methodCol = string(est.method(:));
        cmdLog = table(estTime, vxRobot, omegaRobot, methodCol, ...
            'VariableNames', {'time','Vx','Wz','Method'});
        stageB.cmdLog = cmdLog;
        log.stageLogs.stageB = stageB;
    end

    writetable(cmdLog, cmdFile);

    modeResults(idx).mode = modeName;
    modeResults(idx).logPath = logPath;
    modeResults(idx).pathFile = pathFile;
    modeResults(idx).execFile = execFile;
    modeResults(idx).cmdFile = cmdFile;
    modeResults(idx).goalBase = getFieldIfPresent(stageB, 'goalBase');
    modeResults(idx).achievedBase = getFieldIfPresent(stageB, 'achievedBase');
    modeResults(idx).pathStates = pathStates;
    modeResults(idx).execStates = execStates;
    [tUniform, tActual] = getStageBTimes(stageB, size(pathStates,1), options.RateHz);
    modeResults(idx).timestamps = tUniform;
    modeResults(idx).timestampsActual = tActual;
    modeResults(idx).cmdLog = cmdLog;
    save(logPath, 'log');
    modeResults(idx).solveTelemetry = extractSolveTelemetry(log.stageLogs);
    modeResults(idx).stageBSolveStats = getStageSolveStats(modeResults(idx).solveTelemetry, "stageB");
end

summary = struct();
summary.resultsDir = resultsDir;
summary.rateHz = options.RateHz;
summary.modes = modeResults;
summary.environment = env;

summaryPath = fullfile(resultsDir, 'stageB_compare_summary.mat');
save(summaryPath, 'summary');

plotComparison(modeResults, resultsDir);
plotExecutedStates(modeResults, resultsDir);
plotSolveTelemetry(modeResults, resultsDir);

fprintf("\n[StageB-Compare] Complete. Results stored in %s\n", resultsDir);
if nargout == 0
    clear summary
end
end

function value = getFieldIfPresent(structure, fieldName)
if isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = [];
end
end

function [tUniform, tActual] = getStageBTimes(stageB, numStates, rateHz)
sampleTime = 1 / rateHz;
tUniform = (0:numStates-1)' * sampleTime;

if isfield(stageB, 'timestamps') && ~isempty(stageB.timestamps)
    ts = stageB.timestamps(:);
else
    ts = [];
end

if numel(ts) == numStates
    tActual = ts;
elseif numel(ts) + 1 == numStates
    tActual = [0; ts];
elseif isfield(stageB, 'time') && numel(stageB.time) == numStates
    tActual = stageB.time(:);
elseif isfield(stageB, 'time') && numel(stageB.time) == numStates + 1
    tActual = stageB.time(1:end-1).';
else
    if isfield(stageB, 'time') && ~isempty(stageB.time)
        duration = stageB.time(end);
    else
        duration = (numStates-1) * sampleTime;
    end
    tActual = linspace(0, duration, numStates).';
end
end

function plotComparison(modeResults, resultsDir)
figure('Name', 'Stage B Comparison', 'Position', [100 100 1200 800], 'Color', 'w');

subplot(2,2,1);
hold on; grid on; axis equal;
for idx = 1:numel(modeResults)
    path = modeResults(idx).pathStates;
    plot(path(:,1), path(:,2), 'DisplayName', modeResults(idx).mode);
end
xlabel('x (m)'); ylabel('y (m)'); title('Stage B Base Path'); legend('Location', 'best');

subplot(2,2,2);
hold on; grid on;
for idx = 1:numel(modeResults)
    path = modeResults(idx).pathStates;
    theta = wrapToPi(path(:,3));
    t = modeResults(idx).timestamps;
    plot(t, theta, 'DisplayName', modeResults(idx).mode);
end
xlabel('time (s)'); ylabel('\theta (rad)'); title('Heading vs Time'); legend('Location', 'best');

subplot(2,2,3);
hold on; grid on;
for idx = 1:numel(modeResults)
    cmdLog = modeResults(idx).cmdLog;
    if ~isempty(cmdLog)
        plot(cmdLog.time, cmdLog.Vx, 'DisplayName', modeResults(idx).mode);
    end
end
xlabel('time (s)'); ylabel('V_x (m/s)'); title('Linear Velocity Commands'); legend('Location', 'best');

subplot(2,2,4);
hold on; grid on;
for idx = 1:numel(modeResults)
    cmdLog = modeResults(idx).cmdLog;
    if ~isempty(cmdLog)
        plot(cmdLog.time, cmdLog.Wz, 'DisplayName', modeResults(idx).mode);
    end
end
xlabel('time (s)'); ylabel('\omega_z (rad/s)'); title('Yaw Rate Commands'); legend('Location', 'best');

plotPath = fullfile(resultsDir, 'stageB_comparison.png');
saveas(gcf, plotPath);
close(gcf);
end

function plotExecutedStates(modeResults, resultsDir)
figure('Name', 'Stage B Executed States', 'Position', [100 100 1200 600], 'Color', 'w');

labels = {'x (m)', 'y (m)', '\theta (rad)'};
for dim = 1:3
    subplot(3,1,dim);
    hold on; grid on;
    for idx = 1:numel(modeResults)
        states = modeResults(idx).execStates;
        t = modeResults(idx).timestamps;
        if size(states,1) ~= numel(t)
            t = linspace(0, max(t,[],'all'), size(states,1));
        end
        data = states(:, dim);
        if dim == 3
            data = wrapToPi(data);
        end
        plot(t, data, 'DisplayName', modeResults(idx).mode);
    end
    xlabel('time (s)');
    ylabel(labels{dim});
    title(['Executed Stage B ', labels{dim}]);
    legend('Location', 'best');
end

plotPath = fullfile(resultsDir, 'stageB_executed_states.png');
saveas(gcf, plotPath);
close(gcf);
end

function plotSolveTelemetry(modeResults, resultsDir)
stageNames = collectStageNames(modeResults);
if isempty(stageNames)
    return
end

nStages = numel(stageNames);
nCols = min(2, nStages);
nRows = ceil(nStages / max(nCols, 1));
figure('Name', 'GIK Solve Time Telemetry', 'Position', [100 100 1200 400 * nRows], 'Color', 'w');

for sIdx = 1:nStages
    subplot(nRows, nCols, sIdx);
    hold on; grid on;
    stageKey = stageNames{sIdx};
    plotted = false;
    statLines = {};
    for idx = 1:numel(modeResults)
        if ~isfield(modeResults(idx), 'solveTelemetry')
            continue
        end
        telemetry = modeResults(idx).solveTelemetry;
        if ~isfield(telemetry, stageKey)
            continue
        end
        entry = telemetry.(stageKey);
        solveTime = entry.solveTime;
        if isempty(solveTime)
            continue
        end
        stepIdx = 1:numel(solveTime);
        plot(stepIdx, solveTime, 'DisplayName', modeResults(idx).mode);
        plotted = true;
        stats = entry.solveTimeStats;
        failures = countFailures(entry.solverStatus);
        iterStats = entry.iterationStats;
        violStats = entry.constraintViolationStats;
        summary = entry.solverSummary;
        line = sprintf('%s: N=%d mean=%.3fs max=%.3fs total=%.3fs fail=%d', ...
            modeResults(idx).mode, numel(solveTime), stats.Mean, stats.Max, stats.Total, failures);
        if isfield(iterStats, 'Count') && iterStats.Count > 0
            line = sprintf('%s | iter(mean=%.2f max=%.0f)', line, iterStats.Mean, iterStats.Max);
        else
            line = sprintf('%s | iter(n/a)', line);
        end
        if isfield(violStats, 'Count') && violStats.Count > 0
            line = sprintf('%s | viol(max=%.3g)', line, violStats.Max);
        else
            line = sprintf('%s | viol(n/a)', line);
        end
        if isstruct(summary) && ~isempty(summary)
            if isfield(summary, 'Successes') && isfield(summary, 'Failures')
                line = sprintf('%s | success=%d fail=%d', line, summary.Successes, summary.Failures);
            end
        end
        statLines{end+1} = line; %#ok<AGROW>
    end
    title(stageDisplayLabel(stageKey));
    if plotted
        xlabel('Waypoint Index');
        ylabel('Solve Time (s)');
        legend('Location', 'best');
        if ~isempty(statLines)
            text(0.02, 0.95, strjoin(statLines, '\n'), 'Units', 'normalized', ...
                'VerticalAlignment', 'top', 'FontSize', 9, 'Interpreter', 'none');
        end
    else
        text(0.5, 0.5, 'No data', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        set(gca, 'XTick', [], 'YTick', []);
    end
end

plotPath = fullfile(resultsDir, 'gik_solve_time.png');
saveas(gcf, plotPath);
close(gcf);
end

function stageNames = collectStageNames(modeResults)
stageList = strings(0, 1);
for idx = 1:numel(modeResults)
    if ~isfield(modeResults(idx), 'solveTelemetry')
        continue
    end
    fn = fieldnames(modeResults(idx).solveTelemetry);
    for k = 1:numel(fn)
        key = string(fn{k});
        if ~any(stageList == key)
            stageList(end+1) = key; %#ok<AGROW>
        end
    end
end

if isempty(stageList)
    stageNames = {};
    return
end

preferred = string({'stageA','stageB','stageC'});
ordered = strings(0,1);
for k = 1:numel(preferred)
    key = preferred(k);
    if any(stageList == key)
        ordered(end+1) = key; %#ok<AGROW>
    end
end
for k = 1:numel(stageList)
    key = stageList(k);
    if ~any(ordered == key)
        ordered(end+1) = key; %#ok<AGROW>
    end
end

stageNames = cellstr(ordered);
end

function telemetry = extractSolveTelemetry(stageLogs)
telemetry = struct();
if isempty(stageLogs)
    return
end

stageFields = fieldnames(stageLogs);
for idx = 1:numel(stageFields)
    key = stageFields{idx};
    stageLog = stageLogs.(key);
    entry = struct();
    if isfield(stageLog, 'solveTime') && ~isempty(stageLog.solveTime)
        entry.solveTime = stageLog.solveTime(:);
    else
        entry.solveTime = [];
    end
    if isfield(stageLog, 'solveTimeStats') && ~isempty(stageLog.solveTimeStats)
        entry.solveTimeStats = stageLog.solveTimeStats;
    else
        entry.solveTimeStats = defaultStats();
    end
    if isfield(stageLog, 'solverStatus') && ~isempty(stageLog.solverStatus)
        entry.solverStatus = string(stageLog.solverStatus);
    else
        entry.solverStatus = strings(0,1);
    end
    if isfield(stageLog, 'iterations') && ~isempty(stageLog.iterations)
        entry.iterations = stageLog.iterations(:);
    else
        entry.iterations = [];
    end
    if isfield(stageLog, 'iterationStats') && ~isempty(stageLog.iterationStats)
        entry.iterationStats = stageLog.iterationStats;
    else
        entry.iterationStats = defaultStats();
    end
    if isfield(stageLog, 'constraintViolationMax') && ~isempty(stageLog.constraintViolationMax)
        entry.constraintViolationMax = stageLog.constraintViolationMax(:);
    else
        entry.constraintViolationMax = [];
    end
    if isfield(stageLog, 'constraintViolationStats') && ~isempty(stageLog.constraintViolationStats)
        entry.constraintViolationStats = stageLog.constraintViolationStats;
    else
        entry.constraintViolationStats = defaultStats();
    end
    if isfield(stageLog, 'solverSummary') && ~isempty(stageLog.solverSummary)
        entry.solverSummary = stageLog.solverSummary;
    else
        entry.solverSummary = struct();
    end
    telemetry.(key) = entry;
end
end

function stats = getStageSolveStats(telemetry, stageName)
stats = defaultStats();
if nargin < 2 || isempty(stageName)
    return
end
stageKey = char(stageName);
if isfield(telemetry, stageKey)
    entry = telemetry.(stageKey);
    if isfield(entry, 'solveTimeStats') && ~isempty(entry.solveTimeStats)
        stats = entry.solveTimeStats;
    end
end
end

function label = stageDisplayLabel(stageName)
nameLower = lower(stageName);
if startsWith(nameLower, 'stage') && numel(stageName) >= 6
    label = sprintf('Stage %s Solve Time', upper(stageName(6)));
else
    label = [strrep(stageName, '_', ' '), ' Solve Time'];
end
end

function failures = countFailures(statusArray)
if isempty(statusArray)
    failures = 0;
    return
end
statuses = string(statusArray);
failures = sum(~strcmpi(statuses, 'success'));
end

function stats = defaultStats()
stats = struct('Count', 0, 'Mean', NaN, 'Std', NaN, 'Min', NaN, 'Max', NaN, 'Total', NaN);
end
