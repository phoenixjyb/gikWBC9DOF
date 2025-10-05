%RUN_ENVIRONMENT_COMPARE_LATEST Convenience wrapper around run_environment_compare.
%   Edit the runLabel variable below if you want a custom suffix; the helper
%   will create a timestamped subdirectory, run holistic and staged pipelines,
%   generate animations/plots, and save the summary struct alongside the logs.

runLabel = "compare";  % change if you need a different label

scriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(scriptDir, 'matlab')));

summary = run_environment_compare('RunLabel', runLabel);
summaryPath = fullfile(summary.resultsDir, 'summary.mat');
save(summaryPath, 'summary');

fprintf('\nAll artifacts saved in %s\n', summary.resultsDir);
fprintf('Summary stored at %s\n', summaryPath);
