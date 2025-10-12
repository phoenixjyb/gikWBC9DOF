%GENERATE_SWEEP_ANIMATIONS Create animations for parameter sweep results.
%   Extracts logs from sweep_results.mat and generates animations for each.

clear; clc;
fprintf('=== Generating Animations for Sweep Results ===\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

projRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projRoot, 'matlab')));

% Find the most recent sweep results
sweepDir = 'results/20251010_211447_SWEEP_STAGEB_quick_test';
if ~exist(fullfile(sweepDir, 'sweep_results.mat'), 'file')
    allDirs = dir('results/2025*SWEEP*');
    if isempty(allDirs)
        error('No sweep results found.');
    end
    [~, idx] = max([allDirs.datenum]);
    sweepDir = fullfile('results', allDirs(idx).name);
end

fprintf('Loading sweep results from:\n  %s\n\n', sweepDir);

% Load results
data = load(fullfile(sweepDir, 'sweep_results.mat'));
results = data.results;

fprintf('Found %d configurations.\n', length(results));
fprintf('Generating animations...\n\n');

% Create animations subdirectory
animDir = fullfile(sweepDir, 'animations');
if ~exist(animDir, 'dir')
    mkdir(animDir);
end

% Generate animation for each result
for i = 1:length(results)
    result = results(i);
    
    fprintf('[%d/%d] Config: SafetyMargin=%.2f, LambdaCusp=%.1f, MaxIters=%d\n', ...
        i, length(results), ...
        result.config.safetyMargin, ...
        result.config.lambdaCusp, ...
        result.config.maxIters);
    
    % Check if log exists
    if ~isfield(result, 'log') || isempty(result.log)
        fprintf('  ⚠ No log data, skipping\n\n');
        continue;
    end
    
    % Generate animation
    animFile = fullfile(animDir, sprintf('config%d_margin%.2f_cusp%.1f_iter%d.mp4', ...
        i, result.config.safetyMargin, result.config.lambdaCusp, result.config.maxIters));
    
    fprintf('  Generating: %s\n', animFile);
    
    try
        tic;
        gik9dof.animateStagedWithHelper(result.log, ...
            'ExportVideo', animFile, ...
            'FrameRate', 20, ...
            'SampleStep', 2, ...
            'HelperOptions', struct('FigureScale', 0.5));
        animTime = toc;
        close all;
        
        fprintf('  ✓ Complete in %.1f seconds\n', animTime);
        fprintf('  Metrics: EE mean=%.4fm, max=%.4fm, score=%.4f\n\n', ...
            result.metrics.eeMean, result.metrics.eeMax, result.score);
    catch ME
        fprintf('  ⚠ Error: %s\n\n', ME.message);
        close all;
        continue;
    end
end

fprintf('=== Complete ===\n');
fprintf('Animations saved to:\n  %s\n', animDir);
fprintf('View with: open %s\n', animDir);
