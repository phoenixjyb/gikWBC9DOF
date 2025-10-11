%RUN_STAGED_REFERENCE Entry point for the staged controller regression.
%   Adjust the configuration block below if you want to exercise different
%   execution modes or base-planning parameters.

% TODO: Migrate to unified config system (see UNIFIED_CONFIG_MIGRATION_COMPLETE.md)
%   New recommended approach:
%     cfg = gik9dof.loadPipelineProfile('default');  % or 'aggressive', 'conservative'
%     result = gik9dof.runStagedReference('PipelineConfig', cfg, 'RunLabel', runLabel);
%   Benefits: Single source of truth, consistent parameters across all functions

runLabel = "staged_purehyb_iter150_margin30";
executionMode = "ppForIk";            % "ppForIk" or "pureIk"
rateHz = 30;
maxIterations = 150;
useStageBHybridAStar = true;
stageBMode = "pureHyb";
distanceMargin = 0.30;
stageBLookaheadDistance = 0.6;
stageBDesiredLinearVelocity = 0.5;
stageBMaxAngularVelocity = 2.0;

scriptDir = fileparts(mfilename("fullpath"));
addpath(genpath(fullfile(scriptDir, "matlab")));

result = gik9dof.runStagedReference( ...
    "RunLabel", runLabel, ...
    "ExecutionMode", executionMode, ...
    "RateHz", rateHz, ...
    "MaxIterations", maxIterations, ...
    "UseStageBHybridAStar", useStageBHybridAStar, ...
    "StageBMode", stageBMode, ...
    "DistanceMargin", distanceMargin, ...
    "StageBLookaheadDistance", stageBLookaheadDistance, ...
    "StageBDesiredLinearVelocity", stageBDesiredLinearVelocity, ...
    "StageBMaxAngularVelocity", stageBMaxAngularVelocity);

fprintf('Artifacts available under %s\n', result.resultsDir);
