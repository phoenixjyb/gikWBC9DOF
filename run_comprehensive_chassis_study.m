%% Comprehensive Parametric Study for Chassis Controller
%  This script performs an extensive parameter sweep across both Stage B
%  (path planning) and Stage C (tracking) parameters, evaluating each
%  configuration against 6 rigorous criteria:
%
%  1. EE reference path smoothness (minimize kinky/jumpy)
%  2. Base reference path smoothness
%  3. EE tracking accuracy to JSON waypoints
%  4. Collision avoidance (penalize disc intrusion, not margin)
%  5. Cusp count and severity minimization
%  6. No sideways movement (differential drive constraint)
%
%  All logs are saved for post-hoc analysis and animation generation.

clear; clc;
addpath(genpath('matlab'));

%% Configuration
SAVE_DIR = 'results/comprehensive_study_' + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
LOG_DIR = fullfile(SAVE_DIR, 'logs');
mkdir(LOG_DIR);

% Trajectory to evaluate
JSON_FILE = '1_pull_world_scaled.json';
URDF_FILE = 'mobile_manipulator_PPR_base_corrected_sltRdcd.urdf';

% Evaluation weights (must sum to 1.0)
WEIGHTS = [0.15, 0.15, 0.25, 0.20, 0.15, 0.10];  % [c1, c2, c3, c4, c5, c6]

% Progress logging
CHECKPOINT_INTERVAL = 10;  % Save intermediate results every N configs

%% Parameter Grid Definition
fprintf('=== Comprehensive Chassis Parameter Study ===\n');
fprintf('Defining parameter space...\n');

% Stage B (Path Planning) Parameters
stageBGrid = struct();
stageBGrid.SafetyMargins = [0.05, 0.08, 0.10, 0.12, 0.15];  % [m]
stageBGrid.LambdaCusps = [0.5, 1.0, 1.5, 2.0, 3.0];
stageBGrid.MaxIters = [100, 200, 400, 600];
stageBGrid.AllowReverse = [true, false];
stageBGrid.ClothoidDiscretization = [0.03, 0.05, 0.08, 0.10];  % [m]

% Stage C (Tracking) Parameters  
stageCGrid = struct();
stageCGrid.Lookaheads = [0.4, 0.6, 0.8, 1.0, 1.2];  % [m]
stageCGrid.AccelLimits = [0.6, 0.8, 1.0, 1.2];  % [m/s²]
stageCGrid.HeadingKps = [0.8, 1.0, 1.2, 1.5];

% Compute grid size
nStageB = numel(stageBGrid.SafetyMargins) * numel(stageBGrid.LambdaCusps) * ...
          numel(stageBGrid.MaxIters) * numel(stageBGrid.AllowReverse) * ...
          numel(stageBGrid.ClothoidDiscretization);
nStageC = numel(stageCGrid.Lookaheads) * numel(stageCGrid.AccelLimits) * ...
          numel(stageCGrid.HeadingKps);
nTotal = nStageB * nStageC;

fprintf('Parameter space:\n');
fprintf('  Stage B: %d configs\n', nStageB);
fprintf('  Stage C: %d configs\n', nStageC);
fprintf('  Total:   %d configs\n', nTotal);
fprintf('  Estimated time: %.1f - %.1f hours (@ 30-60s/config)\n', ...
        nTotal*30/3600, nTotal*60/3600);

% User confirmation
response = input('Proceed with study? [y/N]: ', 's');
if ~strcmpi(response, 'y')
    fprintf('Study cancelled.\n');
    return;
end

%% Load Trajectory and Setup
fprintf('\nLoading trajectory and robot model...\n');
trajStruct = gik9dof.loadTrajectory(JSON_FILE);
robot = importrobot(URDF_FILE);

% Parse obstacle discs from trajectory
obstacleDiscs = [];
if isfield(trajStruct, 'ObstacleDiscs') && ~isempty(trajStruct.ObstacleDiscs)
    obstacleDiscs = trajStruct.ObstacleDiscs;  % Mx3 [x, y, radius]
    fprintf('  %d obstacle discs loaded\n', size(obstacleDiscs, 1));
end

%% Generate Full Parameter Combinations
fprintf('Generating parameter combinations...\n');

configID = 0;
allConfigs = [];

for sm = stageBGrid.SafetyMargins
    for lc = stageBGrid.LambdaCusps
        for mi = stageBGrid.MaxIters
            for ar = stageBGrid.AllowReverse
                for cd = stageBGrid.ClothoidDiscretization
                    for lh = stageCGrid.Lookaheads
                        for al = stageCGrid.AccelLimits
                            for hk = stageCGrid.HeadingKps
                                configID = configID + 1;
                                
                                cfg = struct();
                                cfg.id = configID;
                                
                                % Stage B params
                                cfg.stageB.safetyMargin = sm;
                                cfg.stageB.lambdaCusp = lc;
                                cfg.stageB.maxIters = mi;
                                cfg.stageB.allowReverse = ar;
                                cfg.stageB.clothoidDisc = cd;
                                
                                % Stage C params
                                cfg.stageC.lookahead = lh;
                                cfg.stageC.accelLimit = al;
                                cfg.stageC.headingKp = hk;
                                
                                allConfigs = [allConfigs; cfg]; %#ok<AGROW>
                            end
                        end
                    end
                end
            end
        end
    end
end

fprintf('  Generated %d configurations\n', numel(allConfigs));

%% Execute Parameter Sweep
fprintf('\nStarting parameter sweep...\n');
fprintf('Logs will be saved to: %s\n', LOG_DIR);

results = struct('config', {}, 'params', {}, 'evaluation', {}, ...
                 'overallScore', {}, 'logFile', {});

tic;
for i = 1:numel(allConfigs)
    cfg = allConfigs(i);
    
    fprintf('\n[%d/%d] Config %04d: ', i, numel(allConfigs), cfg.id);
    fprintf('SM=%.2f, LC=%.1f, MI=%d, AR=%d, CD=%.2f, LH=%.1f, AL=%.1f, HK=%.1f\n', ...
            cfg.stageB.safetyMargin, cfg.stageB.lambdaCusp, cfg.stageB.maxIters, ...
            cfg.stageB.allowReverse, cfg.stageB.clothoidDisc, ...
            cfg.stageC.lookahead, cfg.stageC.accelLimit, cfg.stageC.headingKp);
    
    % Override parameters
    overrides = struct();
    overrides.SafetyMargin = cfg.stageB.safetyMargin;
    overrides.LambdaCusp = cfg.stageB.lambdaCusp;
    overrides.MaxRSIterations = cfg.stageB.maxIters;
    overrides.AllowReverse = cfg.stageB.allowReverse;
    overrides.ClothoidDiscretization = cfg.stageB.clothoidDisc;
    overrides.Lookahead = cfg.stageC.lookahead;
    overrides.AccelLimit = cfg.stageC.accelLimit;
    overrides.HeadingKp = cfg.stageC.headingKp;
    
    try
        % Run staged trajectory
        log = gik9dof.runStagedReference(trajStruct, robot, ...
                                         'ExecutionMode', 'pureIk', ...
                                         'ParameterOverrides', overrides, ...
                                         'Verbose', false);
        
        % Comprehensive evaluation
        evalReport = gik9dof.comprehensiveEvaluation(log, trajStruct, obstacleDiscs, ...
                                                     'Weights', WEIGHTS, ...
                                                     'SafteyMargin', cfg.stageB.safetyMargin, ...
                                                     'Verbose', false);
        
        % Save log
        logFile = fullfile(LOG_DIR, sprintf('config_%04d.mat', cfg.id));
        save(logFile, 'log', 'evalReport', 'cfg', '-v7.3');
        
        % Store result
        result = struct();
        result.config = cfg;
        result.params = overrides;
        result.evaluation = evalReport;
        result.overallScore = evalReport.overallScore;
        result.logFile = logFile;
        
        results(i) = result;
        
        % Display scores
        fprintf('  → Overall: %.3f | Scores: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] | Violations: %d\n', ...
                evalReport.overallScore, ...
                evalReport.individualScores(1), evalReport.individualScores(2), ...
                evalReport.individualScores(3), evalReport.individualScores(4), ...
                evalReport.individualScores(5), evalReport.individualScores(6), ...
                evalReport.violations.count);
        
    catch ME
        fprintf('  ✗ ERROR: %s\n', ME.message);
        
        % Store failed result
        result = struct();
        result.config = cfg;
        result.params = overrides;
        result.evaluation = struct('overallScore', 0, 'error', ME.message);
        result.overallScore = 0;
        result.logFile = '';
        
        results(i) = result;
    end
    
    % Checkpoint save
    if mod(i, CHECKPOINT_INTERVAL) == 0
        checkpointFile = fullfile(SAVE_DIR, sprintf('checkpoint_%04d.mat', i));
        save(checkpointFile, 'results', 'allConfigs', '-v7.3');
        fprintf('  ✓ Checkpoint saved: %s\n', checkpointFile);
    end
end

elapsedTime = toc;
fprintf('\n=== Sweep Complete ===\n');
fprintf('Total time: %.1f hours\n', elapsedTime / 3600);
fprintf('Average time per config: %.1f seconds\n', elapsedTime / numel(allConfigs));

%% Save Final Results
resultsFile = fullfile(SAVE_DIR, 'comprehensive_results.mat');
save(resultsFile, 'results', 'allConfigs', 'WEIGHTS', 'stageBGrid', 'stageCGrid', '-v7.3');
fprintf('Results saved: %s\n', resultsFile);

%% Generate Results Table (CSV)
fprintf('\nGenerating results table...\n');

% Extract data
nResults = numel(results);
configIDs = zeros(nResults, 1);
safetyMargins = zeros(nResults, 1);
lambdaCusps = zeros(nResults, 1);
maxIters = zeros(nResults, 1);
allowReverse = zeros(nResults, 1);
clothoidDiscs = zeros(nResults, 1);
lookaheads = zeros(nResults, 1);
accelLimits = zeros(nResults, 1);
headingKps = zeros(nResults, 1);

score1 = zeros(nResults, 1);  % EE ref smoothness
score2 = zeros(nResults, 1);  % Base ref smoothness
score3 = zeros(nResults, 1);  % EE tracking
score4 = zeros(nResults, 1);  % Collisions
score5 = zeros(nResults, 1);  % Cusps
score6 = zeros(nResults, 1);  % Sideways
overallScores = zeros(nResults, 1);
violationCounts = zeros(nResults, 1);

logFiles = cell(nResults, 1);

for i = 1:nResults
    r = results(i);
    
    configIDs(i) = r.config.id;
    safetyMargins(i) = r.config.stageB.safetyMargin;
    lambdaCusps(i) = r.config.stageB.lambdaCusp;
    maxIters(i) = r.config.stageB.maxIters;
    allowReverse(i) = double(r.config.stageB.allowReverse);
    clothoidDiscs(i) = r.config.stageB.clothoidDisc;
    lookaheads(i) = r.config.stageC.lookahead;
    accelLimits(i) = r.config.stageC.accelLimit;
    headingKps(i) = r.config.stageC.headingKp;
    
    if isfield(r.evaluation, 'individualScores')
        score1(i) = r.evaluation.individualScores(1);
        score2(i) = r.evaluation.individualScores(2);
        score3(i) = r.evaluation.individualScores(3);
        score4(i) = r.evaluation.individualScores(4);
        score5(i) = r.evaluation.individualScores(5);
        score6(i) = r.evaluation.individualScores(6);
        overallScores(i) = r.evaluation.overallScore;
        violationCounts(i) = r.evaluation.violations.count;
    else
        % Failed config
        overallScores(i) = 0;
        violationCounts(i) = 6;
    end
    
    logFiles{i} = r.logFile;
end

% Create table
T = table(configIDs, safetyMargins, lambdaCusps, maxIters, allowReverse, ...
          clothoidDiscs, lookaheads, accelLimits, headingKps, ...
          score1, score2, score3, score4, score5, score6, ...
          overallScores, violationCounts, logFiles, ...
          'VariableNames', {'ConfigID', 'SafetyMargin', 'LambdaCusp', 'MaxIters', ...
                            'AllowReverse', 'ClothoidDisc', 'Lookahead', 'AccelLimit', ...
                            'HeadingKp', 'Score_EERefSmooth', 'Score_BaseRefSmooth', ...
                            'Score_EETracking', 'Score_Collisions', 'Score_Cusps', ...
                            'Score_Sideways', 'OverallScore', 'ViolationCount', 'LogFile'});

% Sort by overall score (descending)
T = sortrows(T, 'OverallScore', 'descend');

% Save CSV
csvFile = fullfile(SAVE_DIR, 'results_table.csv');
writetable(T, csvFile);
fprintf('Results table saved: %s\n', csvFile);

%% Display Top 20 Configurations
fprintf('\n=== Top 20 Configurations ===\n');
topN = min(20, height(T));
for i = 1:topN
    fprintf('[Rank %2d] Config %04d | Score: %.3f | Violations: %d\n', ...
            i, T.ConfigID(i), T.OverallScore(i), T.ViolationCount(i));
    fprintf('           SM=%.2f, LC=%.1f, MI=%d, AR=%d, CD=%.2f, LH=%.1f, AL=%.1f, HK=%.1f\n', ...
            T.SafetyMargin(i), T.LambdaCusp(i), T.MaxIters(i), T.AllowReverse(i), ...
            T.ClothoidDisc(i), T.Lookahead(i), T.AccelLimit(i), T.HeadingKp(i));
    fprintf('           Scores: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
            T.Score_EERefSmooth(i), T.Score_BaseRefSmooth(i), T.Score_EETracking(i), ...
            T.Score_Collisions(i), T.Score_Cusps(i), T.Score_Sideways(i));
end

%% Summary Statistics
fprintf('\n=== Summary Statistics ===\n');
fprintf('Best overall score:  %.3f (Config %04d)\n', T.OverallScore(1), T.ConfigID(1));
fprintf('Worst overall score: %.3f (Config %04d)\n', T.OverallScore(end), T.ConfigID(end));
fprintf('Mean overall score:  %.3f\n', mean(T.OverallScore));
fprintf('Std overall score:   %.3f\n', std(T.OverallScore));

fprintf('\nConfigs with 0 violations: %d (%.1f%%)\n', ...
        sum(T.ViolationCount == 0), 100 * sum(T.ViolationCount == 0) / height(T));
fprintf('Configs with 1-2 violations: %d (%.1f%%)\n', ...
        sum(T.ViolationCount <= 2 & T.ViolationCount > 0), ...
        100 * sum(T.ViolationCount <= 2 & T.ViolationCount > 0) / height(T));
fprintf('Configs with 3+ violations: %d (%.1f%%)\n', ...
        sum(T.ViolationCount >= 3), 100 * sum(T.ViolationCount >= 3) / height(T));

%% Criterion-Specific Best Configs
fprintf('\n=== Best Config Per Criterion ===\n');
[~, bestC1] = max(T.Score_EERefSmooth);
[~, bestC2] = max(T.Score_BaseRefSmooth);
[~, bestC3] = max(T.Score_EETracking);
[~, bestC4] = max(T.Score_Collisions);
[~, bestC5] = max(T.Score_Cusps);
[~, bestC6] = max(T.Score_Sideways);

fprintf('[C1] EE Ref Smoothness:   Config %04d (score %.3f)\n', T.ConfigID(bestC1), T.Score_EERefSmooth(bestC1));
fprintf('[C2] Base Ref Smoothness: Config %04d (score %.3f)\n', T.ConfigID(bestC2), T.Score_BaseRefSmooth(bestC2));
fprintf('[C3] EE Tracking:         Config %04d (score %.3f)\n', T.ConfigID(bestC3), T.Score_EETracking(bestC3));
fprintf('[C4] Collisions:          Config %04d (score %.3f)\n', T.ConfigID(bestC4), T.Score_Collisions(bestC4));
fprintf('[C5] Cusps:               Config %04d (score %.3f)\n', T.ConfigID(bestC5), T.Score_Cusps(bestC5));
fprintf('[C6] Sideways:            Config %04d (score %.3f)\n', T.ConfigID(bestC6), T.Score_Sideways(bestC6));

fprintf('\n=== Study Complete ===\n');
fprintf('Next steps:\n');
fprintf('  1. Review top configs in results_table.csv\n');
fprintf('  2. Generate animations for top 10 using generate_comprehensive_animations.m\n');
fprintf('  3. Analyze parameter sensitivity and trade-offs\n');
