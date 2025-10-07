function regenerate_stageb_comparison_animations(resultsDir, options)
%REGENERATE_STAGEB_COMPARISON_ANIMATIONS Rebuild staged animations from logs.
%   regenerate_stageb_comparison_animations(resultsDir) loads the
%   `log_stageB_gikinloop.mat` and `log_stageB_purehyb.mat` files located in
%   the specified results directory (produced by run_stageb_mode_compare)
%   and generates staged animations using gik9dof.animateStagedWithHelper.
%
%   Optional name-value in the `options` struct:
%       SampleStep  - Frame subsampling factor (default 2)
%       FrameRate   - Output video frame rate (default 20 Hz)
%       FigureScale - Visualization scaling passed to helper (default 0.5)
%       ExportVideo - Logical flag to save MP4 files (default true)
%
%   Example:
%       regenerate_stageb_comparison_animations('results/20251006_233440_stageB_compare');
%
arguments
    resultsDir (1,1) string
    options.SampleStep (1,1) double {mustBePositive} = 2
    options.FrameRate (1,1) double {mustBePositive} = 20
    options.FigureScale (1,1) double {mustBePositive} = 0.5
    options.ExportVideo (1,1) logical = true
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(projectRoot, 'matlab')));

modes = ["gikinloop", "purehyb"];
for idx = 1:numel(modes)
    modeName = modes(idx);
    logFile = fullfile(resultsDir, "log_stageB_" + modeName + ".mat");
    if ~isfile(logFile)
        warning('Log file %s not found. Skipping %s.', logFile, modeName);
        continue
    end
    data = load(logFile);
    if ~isfield(data, 'log')
        warning('File %s does not contain variable ''log''. Skipping.', logFile);
        continue
    end
    log = data.log;

    helperOpts = struct('FigureScale', options.FigureScale);
    args = {'SampleStep', options.SampleStep, ...
            'FrameRate', options.FrameRate, ...
            'HelperOptions', helperOpts};
    if options.ExportVideo
        videoPath = fullfile(resultsDir, "staged_" + modeName + ".mp4");
        args = [args, {'ExportVideo', videoPath}]; %#ok<AGROW>
        fprintf('[StageB-Compare] Writing animation %s\n', videoPath);
    else
        fprintf('[StageB-Compare] Rendering %s without video export\n', modeName);
    end

    gik9dof.animateStagedWithHelper(log, args{:});
end
end
