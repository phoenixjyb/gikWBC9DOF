function summary = run_environment_compare(options)
%RUN_ENVIRONMENT_COMPARE Execute holistic vs staged runs with shared config.
%   summary = run_environment_compare() runs both controllers using
%   gik9dof.environmentConfig(), saves logs/videos to a timestamped results
%   directory, and prints a comparison summary. Name-value options:
%       RunLabel        - Text appended to result folder (default 'compare').
%       SampleStep      - Frame subsampling for animations (default 4).
%       FrameRate       - Animation frame rate in Hz (default 20).
%       SaveAnimations  - Logical flag to write MP4 outputs (default true).
%       FigureScale     - Scaling passed to staged helper (default 0.5).
%
%   Example:
%       summary = run_environment_compare('RunLabel','nightly');
%
arguments
    options.RunLabel (1,1) string = "compare"
    options.SampleStep (1,1) double {mustBePositive, mustBeInteger} = 4
    options.FrameRate (1,1) double {mustBePositive} = 20
    options.SaveAnimations (1,1) logical = true
    options.FigureScale (1,1) double {mustBePositive} = 0.5
end

currentDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(currentDir);
addpath(genpath(fullfile(projectRoot, 'matlab')));

runDir = gik9dof.internal.createResultsFolder(options.RunLabel);
resultsDir = char(runDir);

env = gik9dof.environmentConfig();

logHolistic = gik9dof.trackReferenceTrajectory( ...
    'Verbose', false, ...
    'Mode', 'holistic', ...
    'EnvironmentConfig', env);

logStaged = gik9dof.trackReferenceTrajectory( ...
    'Verbose', false, ...
    'Mode', 'staged', ...
    'UseStageBHybridAStar', true, ...
    'EnvironmentConfig', env);

holisticPath = fullfile(resultsDir, "log_holistic.mat");
stagedPath = fullfile(resultsDir, "log_staged.mat");
save(holisticPath, 'logHolistic');
save(stagedPath, 'logStaged');

summary = struct();
summary.timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
summary.runLabel = char(options.RunLabel);
summary.resultsDir = resultsDir;
summary.holisticLog = holisticPath;
summary.stagedLog = stagedPath;
summary.discsIdentical = isequal(logHolistic.floorDiscs, logStaged.floorDiscs);
summary.baseHomeHolistic = logHolistic.environment.BaseHome;
summary.baseHomeStaged = logStaged.environment.BaseHome;
summary.maxTargetDifference = max(abs(logStaged.stageLogs.stageC.targetPositions(:) - logHolistic.targetPositions(:)));
summary.finalErrorHolistic = norm(logHolistic.positionError(:, end));
summary.finalErrorStaged = norm(logStaged.positionError(:, end));
summary.environment = env;

fprintf('\n=== Environment Comparison (%s) ===\n', summary.timestamp);
fprintf('Results dir: %s\n', resultsDir);
fprintf('Holistic log: %s\n', holisticPath);
fprintf('Staged log:   %s\n', stagedPath);
fprintf('Disc geometry identical: %s\n', logicalToString(summary.discsIdentical));
fprintf('Base home (holistic): [%g %g %g]\n', summary.baseHomeHolistic);
fprintf('Base home (staged):   [%g %g %g]\n', summary.baseHomeStaged);
fprintf('Max StageC vs holistic waypoint diff: %.6f m\n', summary.maxTargetDifference);
fprintf('Final EE error (holistic): %.6f\n', summary.finalErrorHolistic);
fprintf('Final EE error (staged):   %.6f\n', summary.finalErrorStaged);

if options.SaveAnimations
    holVideo = fullfile(resultsDir, "holistic.mp4");
    gik9dof.animateTrajectory(logHolistic, ...
        'OutputVideo', holVideo, ...
        'FrameRate', options.FrameRate, ...
        'SampleStep', options.SampleStep);
    summary.holisticVideo = holVideo;
    fprintf('Holistic animation saved to %s\n', holVideo);

    stagedVideo = fullfile(resultsDir, "staged.mp4");
    helperOpts = struct('FigureScale', options.FigureScale);
    gik9dof.animateStagedWithHelper(logStaged, ...
        'SampleStep', options.SampleStep, ...
        'FrameRate', options.FrameRate, ...
        'ExportVideo', stagedVideo, ...
        'HelperOptions', helperOpts);
    summary.stagedVideo = stagedVideo;
    fprintf('Staged animation saved to %s\n', stagedVideo);
end

% Diagnostic plots
summary.holisticPlots = gik9dof.generateLogPlots(logHolistic, fullfile(resultsDir, 'holistic'));
summary.stagedPlots = gik9dof.generateLogPlots(logStaged, fullfile(resultsDir, 'staged'));

summary.runOptions = options;

if nargout == 0
    assignin('base', 'runEnvSummary', summary);
end
end

function out = logicalToString(value)
if value
    out = 'true';
else
    out = 'false';
end
end
