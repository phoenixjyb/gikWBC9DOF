function summary = run_gik_iteration_study(options)
%RUN_GIK_ITERATION_STUDY Compare solver iteration caps for holistic & staged runs.
%
%   summary = run_gik_iteration_study() executes the reference trajectory in
%   holistic and staged modes for two iteration caps (1500 and 150) while
%   collecting end-effector and solver metrics, animations, and diagnostic
%   plots. Results are stored under results/<timestamp>_gik_iter_study/.
%
%   Name-value options:
%       IterationCaps          - Vector of positive integers (default [1500 150]).
%       RateHz                 - Control loop rate (default 30).
%       UseStageBHybridAStar   - Logical (default false).
%       StageBMode             - "gikInLoop" or "pureHyb" (default "gikInLoop").
%       Label                  - Optional suffix for results folder name.
%       Verbose                - Enable controller verbose printouts (default false).
%       SaveAnimations         - Save MP4 animations for each run (default true).
%
%   The returned summary struct contains per-mode metrics and file references.
%
%   Example:
%       run_gik_iteration_study('IterationCaps', [1500 200]);

arguments
    options.IterationCaps (1,:) double {mustBePositive} = [1500 150]
    options.RateHz (1,1) double {mustBePositive} = 30
    options.UseStageBHybridAStar (1,1) logical = false
    options.StageBMode (1,1) string {mustBeMember(options.StageBMode, ["gikInLoop","pureHyb"])} = "gikInLoop"
    options.Label (1,1) string = ""
    options.Verbose (1,1) logical = false
    options.SaveAnimations (1,1) logical = true
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(projectRoot, 'matlab')));

iterCaps = unique(options.IterationCaps(:).');

stamp = string(datetime('now','Format','yyyyMMdd_HHmmss'));
folderName = stamp + "_gik_iter_study";
if strlength(options.Label) > 0
    folderName = folderName + "_" + options.Label;
end
resultsDir = fullfile(projectRoot, 'results', folderName);
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

modes = ["holistic", "staged"];
runs = struct('Mode', {}, 'IterationCap', {}, 'LogPath', {}, ...
    'Metrics', struct(), 'PlotPath', "", 'VideoPath', "");

for cap = iterCaps
    for mode = modes
        fprintf('\n[GIK-IterStudy] Running mode %s with MaxIterations=%d...\n', mode, cap);
        log = gik9dof.trackReferenceTrajectory( ...
            'Mode', mode, ...
            'RateHz', options.RateHz, ...
            'Verbose', options.Verbose, ...
            'UseStageBHybridAStar', options.UseStageBHybridAStar, ...
            'StageBMode', options.StageBMode, ...
            'MaxIterations', cap);

        logFile = fullfile(resultsDir, sprintf('log_%s_iter%04d.mat', mode, cap));
        save(logFile, 'log');

        metrics = computeMetrics(log);
        plotFile = fullfile(resultsDir, sprintf('metrics_%s_iter%04d.png', mode, cap));
        try
            plotMetricsFigure(log, metrics, plotFile);
        catch plotErr
            warning('run_gik_iteration_study:PlotFailed', ...
                'Failed to generate plot for %s (iter=%d): %s', mode, cap, plotErr.message);
            plotFile = "";
        end

        videoFile = "";
        if options.SaveAnimations
            try
                videoFile = fullfile(resultsDir, sprintf('anim_%s_iter%04d.mp4', mode, cap));
                gik9dof.animateTrajectory(log, 'OutputVideo', videoFile, 'SampleStep', 5);
            catch animErr
                warning('run_gik_iteration_study:AnimationFailed', ...
                    'Failed to render animation for %s (iter=%d): %s', mode, cap, animErr.message);
                videoFile = "";
            end
        end

        runEntry.Mode = mode;
        runEntry.IterationCap = cap;
        runEntry.LogPath = logFile;
        runEntry.Metrics = metrics;
        runEntry.PlotPath = plotFile;
        runEntry.VideoPath = videoFile;
        runs(end+1) = runEntry; %#ok<AGROW>
    end
end

summary = struct();
summary.resultsDir = resultsDir;
summary.runs = runs;
summary.options = options;
summary.iterationCaps = iterCaps;

metricsTable = buildMetricsTable(runs);
metricsCsv = fullfile(resultsDir, 'iteration_metrics.csv');
writetable(metricsTable, metricsCsv);
summary.metricsTablePath = metricsCsv;

summaryMat = fullfile(resultsDir, 'iteration_study_summary.mat');
save(summaryMat, 'summary');
summary.summaryMatPath = summaryMat;

fprintf('\n[GIK-IterStudy] Results stored in %s\n', resultsDir);
if nargout == 0
    clear summary
end
end

function metrics = computeMetrics(log)
metrics = struct();

if isfield(log, 'time') && ~isempty(log.time)
    sampleTime = median(diff(log.time(~isnan(log.time) & log.time > 0)));
else
    sampleTime = NaN;
end
metrics.SampleTime = sampleTime;
metrics.RateHz = getfieldwithdefault(log, 'rateHz', NaN); %#ok<GFLD>

if isfield(log, 'positionError') && ~isempty(log.positionError)
    eeError = vecnorm(log.positionError, 2, 1);
    metrics.EEErrorMean = mean(eeError(~isnan(eeError)));
    metrics.EEErrorRMS = rms(eeError(~isnan(eeError)));
    metrics.EEErrorMax = max(eeError(~isnan(eeError)));
    metrics.EEError95 = prctile(eeError(~isnan(eeError)), 95);
    metrics.EEErrorFinal = eeError(find(~isnan(eeError), 1, 'last'));
else
    metrics.EEErrorMean = NaN;
    metrics.EEErrorRMS = NaN;
    metrics.EEErrorMax = NaN;
    metrics.EEError95 = NaN;
    metrics.EEErrorFinal = NaN;
end

iterations = getfieldwithdefault(log, 'iterations', []);
if ~isempty(iterations)
    iterFinite = iterations(isfinite(iterations));
    metrics.IterationsMean = mean(iterFinite);
    metrics.IterationsMax = max(iterFinite);
    metrics.IterationsMedian = median(iterFinite);
else
    metrics.IterationsMean = NaN;
    metrics.IterationsMax = NaN;
    metrics.IterationsMedian = NaN;
end

if isfield(log, 'iterationStats') && ~isempty(log.iterationStats)
    stats = log.iterationStats;
    metrics.IterStatsCount = getfieldwithdefault(stats, 'Count', NaN);
else
    metrics.IterStatsCount = NaN;
end

if isfield(log, 'solverSummary') && ~isempty(log.solverSummary)
    summary = log.solverSummary;
    total = getfieldwithdefault(summary, 'TotalSteps', NaN);
    success = getfieldwithdefault(summary, 'Successes', NaN);
    metrics.SolverSuccessRate = success / total;
else
    metrics.SolverSuccessRate = NaN;
end

solveTime = getfieldwithdefault(log, 'solveTime', []);
if ~isempty(solveTime)
    solveFinite = solveTime(isfinite(solveTime));
    metrics.SolveTimeMean = mean(solveFinite);
    metrics.SolveTimeMax = max(solveFinite);
    metrics.SolveTimeMedian = median(solveFinite);
else
    metrics.SolveTimeMean = NaN;
    metrics.SolveTimeMax = NaN;
    metrics.SolveTimeMedian = NaN;
end

if isfield(log, 'baseVelocityEstimate') && ~isempty(log.baseVelocityEstimate)
    est = log.baseVelocityEstimate;
    metrics.BaseVelocityMean = mean(est.vxRobot(~isnan(est.vxRobot)));
else
    metrics.BaseVelocityMean = NaN;
end
end

function plotMetricsFigure(log, metrics, outputPath)
time = getfieldwithdefault(log, 'time', []);
if isempty(time)
    time = (0:size(log.positionError,2)) * metrics.SampleTime;
end
if numel(time) > numel(log.positionError) + 1
    time = time(1:numel(log.positionError)+1);
end

if ~isempty(log.positionError)
    eeError = vecnorm(log.positionError, 2, 1);
else
    eeError = zeros(1, numel(time)-1);
end

iterations = getfieldwithdefault(log, 'iterations', []);
solveTime = getfieldwithdefault(log, 'solveTime', []);

fig = figure('Name', 'GIK Iteration Study Metrics', 'Position', [100 100 1000 720], 'Color', 'w');

subplot(3,1,1);
plot(time(2:end), eeError, 'LineWidth', 1.2);
grid on;
xlabel('time (s)'); ylabel('EE error (m)');
title(sprintf('EE Error (mean=%.3f, max=%.3f)', metrics.EEErrorMean, metrics.EEErrorMax));

subplot(3,1,2);
if ~isempty(iterations)
    stem(time(2:numel(iterations)+1), iterations, '.-');
else
    plot(0,0);
end
grid on;
xlabel('time (s)'); ylabel('Iterations');
title(sprintf('Iterations per Step (mean=%.1f, max=%d)', metrics.IterationsMean, metrics.IterationsMax));

subplot(3,1,3);
if ~isempty(solveTime)
    plot(time(2:numel(solveTime)+1), solveTime, 'LineWidth', 1.2);
else
    plot(0,0);
end
grid on;
xlabel('time (s)'); ylabel('Solve time (s)');
title(sprintf('Solve Time (mean=%.3f s, max=%.3f s)', metrics.SolveTimeMean, metrics.SolveTimeMax));

exportgraphics(fig, outputPath, 'Resolution', 200);
close(fig);
end

function tableOut = buildMetricsTable(runs)
numRuns = numel(runs);
fields = fieldnames(runs(1).Metrics);
data = cell(numRuns, numel(fields)+2);
for idx = 1:numRuns
    data{idx,1} = runs(idx).Mode;
    data{idx,2} = runs(idx).IterationCap;
    metrics = runs(idx).Metrics;
    for fIdx = 1:numel(fields)
        fieldName = fields{fIdx};
        data{idx,2+fIdx} = metrics.(fieldName);
    end
end

varNames = ['Mode', 'IterationCap', fields'];
tableOut = cell2table(data, 'VariableNames', varNames);
end

function val = getfieldwithdefault(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    val = structure.(fieldName);
else
    val = defaultValue;
end
end
