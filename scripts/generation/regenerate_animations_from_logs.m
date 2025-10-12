%REGENERATE_ANIMATIONS_FROM_LOGS Reload saved logs and export animations/plots.
%   Edit the runFolder variable to point at the timestamped directory inside
%   results/. The script regenerates holistic and staged animations using the
%   staged-style visualization, and reproduces the arm/chassis plots.

if ~exist('runFolder', 'var') || isempty(runFolder)
    runFolder = fullfile('results', '20251005_195649_compare_fixed');
end

projRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projRoot, 'matlab')));

holisticLogPath = fullfile(runFolder, 'log_holistic.mat');
stagedLogPath = fullfile(runFolder, 'log_staged.mat');

if ~isfile(holisticLogPath) || ~isfile(stagedLogPath)
    error('Missing log files in %s', runFolder);
end

dataH = load(holisticLogPath);
dataS = load(stagedLogPath);
logH = dataH.logHolistic;
logS = dataS.logStaged;

% Regenerate animations
holVideo = fullfile(runFolder, 'holistic_regen.mp4');
gik9dof.animateHolisticWithHelper(logH, ...
    'ExportVideo', holVideo, ...
    'FrameRate', 20, ...
    'SampleStep', 2, ...
    'HelperOptions', struct('FigureScale', 0.5));

stagedVideo = fullfile(runFolder, 'staged_regen.mp4');
gik9dof.animateStagedWithHelper(logS, ...
    'SampleStep', 2, ...
    'FrameRate', 20, ...
    'ExportVideo', stagedVideo, ...
    'HelperOptions', struct('FigureScale', 0.5));

% Regenerate diagnostic plots
gik9dof.generateLogPlots(logH, fullfile(runFolder, 'holistic_regen'));
gik9dof.generateLogPlots(logS, fullfile(runFolder, 'staged_regen'));

fprintf('Animations and plots regenerated under %s\n', runFolder);
