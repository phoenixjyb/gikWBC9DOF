%% Generate Animations from Comprehensive Study Results
%  This script generates animations for selected configurations from the
%  comprehensive parametric study. By default, it generates animations for
%  the top 10 configurations by overall score.

clear; clc;
addpath(genpath('matlab'));

%% Configuration
% Point to the comprehensive study results directory
STUDY_DIR = input('Enter study directory (e.g., results/comprehensive_study_20251010_220000): ', 's');

if ~isfolder(STUDY_DIR)
    error('Study directory not found: %s', STUDY_DIR);
end

RESULTS_FILE = fullfile(STUDY_DIR, 'comprehensive_results.mat');
TABLE_FILE = fullfile(STUDY_DIR, 'results_table.csv');
ANIMATION_DIR = fullfile(STUDY_DIR, 'animations');

% Animation settings
TOP_N = 10;  % Generate animations for top N configs
FRAME_RATE = 20;
SAMPLE_STEP = 2;  % Sample every N timesteps for speed

mkdir(ANIMATION_DIR);

%% Load Results
fprintf('Loading results from: %s\n', STUDY_DIR);

if ~isfile(RESULTS_FILE)
    error('Results file not found: %s', RESULTS_FILE);
end

load(RESULTS_FILE, 'results', 'WEIGHTS');

if isfile(TABLE_FILE)
    T = readtable(TABLE_FILE);
    fprintf('  %d configurations loaded\n', height(T));
else
    error('Results table not found: %s', TABLE_FILE);
end

%% Select Configurations to Animate
fprintf('\nSelect configurations to animate:\n');
fprintf('  1. Top %d by overall score (default)\n', TOP_N);
fprintf('  2. Specific config IDs (comma-separated)\n');
fprintf('  3. Best per criterion (6 configs)\n');
fprintf('  4. All configs with 0 violations\n');

choice = input('Choice [1]: ', 's');
if isempty(choice)
    choice = '1';
end

switch choice
    case '1'
        % Top N by overall score
        configIDs = T.ConfigID(1:min(TOP_N, height(T)));
        fprintf('Selected top %d configs\n', numel(configIDs));
        
    case '2'
        % Specific IDs
        idStr = input('Enter config IDs (comma-separated): ', 's');
        configIDs = str2double(split(idStr, ','));
        configIDs = configIDs(~isnan(configIDs));
        fprintf('Selected %d configs\n', numel(configIDs));
        
    case '3'
        % Best per criterion
        [~, bestC1] = max(T.Score_EERefSmooth);
        [~, bestC2] = max(T.Score_BaseRefSmooth);
        [~, bestC3] = max(T.Score_EETracking);
        [~, bestC4] = max(T.Score_Collisions);
        [~, bestC5] = max(T.Score_Cusps);
        [~, bestC6] = max(T.Score_Sideways);
        
        configIDs = unique([T.ConfigID(bestC1), T.ConfigID(bestC2), ...
                           T.ConfigID(bestC3), T.ConfigID(bestC4), ...
                           T.ConfigID(bestC5), T.ConfigID(bestC6)]);
        fprintf('Selected %d unique best-per-criterion configs\n', numel(configIDs));
        
    case '4'
        % All with 0 violations
        configIDs = T.ConfigID(T.ViolationCount == 0);
        fprintf('Selected %d configs with 0 violations\n', numel(configIDs));
        
    otherwise
        error('Invalid choice');
end

%% Generate Animations
fprintf('\nGenerating animations...\n');
fprintf('Output directory: %s\n', ANIMATION_DIR);

for i = 1:numel(configIDs)
    configID = configIDs(i);
    
    % Find result
    resultIdx = find([results.config.id] == configID, 1);
    if isempty(resultIdx)
        fprintf('[%d/%d] Config %04d: NOT FOUND\n', i, numel(configIDs), configID);
        continue;
    end
    
    result = results(resultIdx);
    
    % Load log
    if isempty(result.logFile) || ~isfile(result.logFile)
        fprintf('[%d/%d] Config %04d: Log file not found\n', i, numel(configIDs), configID);
        continue;
    end
    
    fprintf('[%d/%d] Config %04d (Score: %.3f, Violations: %d)\n', ...
            i, numel(configIDs), configID, result.overallScore, ...
            result.evaluation.violations.count);
    
    logData = load(result.logFile);
    log = logData.log;
    
    % Generate descriptive filename
    cfg = result.config;
    filename = sprintf('config_%04d_score_%.3f_SM%.2f_LC%.1f_MI%d_AR%d_CD%.2f_LH%.1f_AL%.1f_HK%.1f.mp4', ...
                      configID, result.overallScore, ...
                      cfg.stageB.safetyMargin, cfg.stageB.lambdaCusp, ...
                      cfg.stageB.maxIters, cfg.stageB.allowReverse, ...
                      cfg.stageB.clothoidDisc, cfg.stageC.lookahead, ...
                      cfg.stageC.accelLimit, cfg.stageC.headingKp);
    
    outputPath = fullfile(ANIMATION_DIR, filename);
    
    try
        % Generate animation
        gik9dof.animateStagedWithHelper(log, ...
                                        'OutputPath', outputPath, ...
                                        'FrameRate', FRAME_RATE, ...
                                        'SampleStep', SAMPLE_STEP, ...
                                        'Verbose', false);
        
        fprintf('  ✓ Animation saved: %s\n', filename);
        
    catch ME
        fprintf('  ✗ Animation failed: %s\n', ME.message);
    end
end

fprintf('\n=== Animation Generation Complete ===\n');
fprintf('Animations saved to: %s\n', ANIMATION_DIR);
fprintf('Total animations: %d\n', numel(dir(fullfile(ANIMATION_DIR, '*.mp4'))));
