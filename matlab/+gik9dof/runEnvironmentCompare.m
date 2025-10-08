function summary = runEnvironmentCompare(options)
%RUNENVIRONMENTCOMPARE Execute holistic vs staged runs with shared config.
%   summary = gik9dof.runEnvironmentCompare() runs both controllers using
%   gik9dof.environmentConfig(), saves logs/videos to a timestamped results
%   directory, and prints a comparison summary. Name-value options:
%       RunLabel            - Text appended to result folder (default 'compare').
%       SampleStep          - Frame subsampling for animations (default 4).
%       FrameRate           - Animation frame rate in Hz (default 20).
%       SaveAnimations      - Logical flag to write MP4 outputs (default true).
%       FigureScale         - Scaling passed to visualization helpers (default 0.5).
%       RateHz              - Control loop rate shared by both runs (default 50).
%       HolisticExecutionMode - 'ppForIk' (default) or 'pureIk'.
%       StagedExecutionMode   - 'ppForIk' (default) or 'pureIk'.
%       UseStageBHybridAStar  - Enable Stage B hybrid A* for staged run (default true).
%       StageBMode            - Stage B alignment mode ('gikInLoop' or 'pureHyb', default 'pureHyb').
%
%   Example:
%       summary = gik9dof.runEnvironmentCompare('RunLabel','nightly');
%
arguments
    options.RunLabel (1,1) string = "compare"
    options.SampleStep (1,1) double {mustBePositive, mustBeInteger} = 4
    options.FrameRate (1,1) double {mustBePositive} = 20
    options.SaveAnimations (1,1) logical = true
    options.FigureScale (1,1) double {mustBePositive} = 0.5
    options.RateHz (1,1) double {mustBePositive} = 50
    options.HolisticExecutionMode (1,1) string {mustBeMember(options.HolisticExecutionMode, ["ppForIk","pureIk"])} = "ppForIk"
    options.StagedExecutionMode (1,1) string {mustBeMember(options.StagedExecutionMode, ["ppForIk","pureIk"])} = "ppForIk"
    options.UseStageBHybridAStar (1,1) logical = true
    options.StageBMode (1,1) string {mustBeMember(options.StageBMode, ["gikInLoop","pureHyb"])} = "pureHyb"
    options.MaxIterations (1,1) double {mustBePositive} = 1500
end

runDir = gik9dof.internal.createResultsFolder(options.RunLabel);
resultsDir = char(runDir);

env = gik9dof.environmentConfig();

logHolistic = gik9dof.trackReferenceTrajectory( ...
    'Verbose', false, ...
    'Mode', 'holistic', ...
    'RateHz', options.RateHz, ...
    'MaxIterations', options.MaxIterations, ...
    'ExecutionMode', options.HolisticExecutionMode, ...
    'EnvironmentConfig', env);

logStaged = gik9dof.trackReferenceTrajectory( ...
    'Verbose', false, ...
    'Mode', 'staged', ...
    'UseStageBHybridAStar', options.UseStageBHybridAStar, ...
    'StageBMode', options.StageBMode, ...
    'RateHz', options.RateHz, ...
    'MaxIterations', options.MaxIterations, ...
    'ExecutionMode', options.StagedExecutionMode, ...
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
summary.holisticExecutionMode = options.HolisticExecutionMode;
summary.stagedExecutionMode = options.StagedExecutionMode;
summary.useStageBHybridAStar = options.UseStageBHybridAStar;
summary.stageBMode = options.StageBMode;

fprintf('\n=== Environment Comparison (%s) ===\n', summary.timestamp);
fprintf('Results dir: %s\n', resultsDir);
fprintf('Holistic log: %s\n', holisticPath);
fprintf('Staged log:   %s\n', stagedPath);
fprintf('Holistic execution mode: %s\n', summary.holisticExecutionMode);
fprintf('Staged execution mode:   %s\n', summary.stagedExecutionMode);
fprintf('Disc geometry identical: %s\n', localLogicalToString(summary.discsIdentical));
fprintf('Base home (holistic): [%g %g %g]\n', summary.baseHomeHolistic);
fprintf('Base home (staged):   [%g %g %g]\n', summary.baseHomeStaged);
fprintf('Max StageC vs holistic waypoint diff: %.6f m\n', summary.maxTargetDifference);
fprintf('Final EE error (holistic): %.6f\n', summary.finalErrorHolistic);
fprintf('Final EE error (staged):   %.6f\n', summary.finalErrorStaged);

if options.SaveAnimations
    holVideo = fullfile(resultsDir, "holistic.mp4");
    gik9dof.animateHolisticWithHelper(logHolistic, ...
        'ExportVideo', holVideo, ...
        'FrameRate', options.FrameRate, ...
        'SampleStep', options.SampleStep, ...
        'HelperOptions', struct('FigureScale', options.FigureScale));
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

function out = localLogicalToString(val)
if val
    out = 'true';
else
    out = 'false';
end
end
