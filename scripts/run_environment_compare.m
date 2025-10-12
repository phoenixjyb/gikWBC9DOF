%RUN_ENVIRONMENT_COMPARE Entry point for holistic vs staged regression runs.
%   Adjust the configuration block below as needed. The script adds the
%   MATLAB utilities to the path, invokes gik9dof.runEnvironmentCompare, and
%   stores the returned summary alongside the generated artifacts.
%
%   TODO: Migrate to unified config system (see UNIFIED_CONFIG_MIGRATION_COMPLETE.md)
%     New recommended approach:
%       cfg = gik9dof.loadPipelineProfile('default');  % or 'aggressive'
%       summary = gik9dof.runEnvironmentCompare('PipelineConfig', cfg, ...);
%     Benefits: Consistent parameters for both holistic and staged runs

runLabel = "compare";            % appended to the timestamped results folder
sampleStep = 4;                  % subsampling for animations
frameRate = 20;                  % animation frame rate (Hz)
figureScale = 0.5;               % visualization scale factor
rateHz = 50;                     % control loop rate used for both runs
holisticMode = "ppForIk";        % holistic execution mode: "ppForIk" or "pureIk"
stagedMode = "ppForIk";          % staged execution mode: "ppForIk" or "pureIk"
useStageBHybridAStar = true;     % toggle Stage B hybrid A* for staged run
stageBMode = "pureHyb";          % staged pipeline base-alignment mode
maxIterations = 1500;            % solver iteration cap for both runs
chassisProfile = "wide_track";  % chassis preset defined in config/chassis_profiles.yaml

scriptDir = fileparts(mfilename("fullpath"));
addpath(genpath(fullfile(scriptDir, "matlab")));

summary = gik9dof.runEnvironmentCompare( ...
    "RunLabel", runLabel, ...
    "SampleStep", sampleStep, ...
    "FrameRate", frameRate, ...
    "FigureScale", figureScale, ...
    "RateHz", rateHz, ...
    "MaxIterations", maxIterations, ...
    "HolisticExecutionMode", holisticMode, ...
    "StagedExecutionMode", stagedMode, ...
    "UseStageBHybridAStar", useStageBHybridAStar, ...
    "StageBMode", stageBMode, ...
    "ChassisProfile", chassisProfile);

summaryPath = fullfile(summary.resultsDir, "summary.mat");
save(summaryPath, "summary");

fprintf("\nAll artifacts saved in %s\n", summary.resultsDir);
fprintf("Summary stored at %s\n", summaryPath);
