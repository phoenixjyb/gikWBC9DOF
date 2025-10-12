%% Quick Parametric Study (5 configs) for Animation Review
%  Streamlined version with 5 key configurations

clear; clc;
addpath(genpath('matlab'));

fprintf('=== Quick Parametric Study (5 Configs) ===\n');

%% Configuration
STUDY_DIR = 'results/parametric_study_' + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
LOG_DIR = fullfile(STUDY_DIR, 'logs');
ANIM_DIR = fullfile(STUDY_DIR, 'animations');
mkdir(LOG_DIR);
mkdir(ANIM_DIR);

fprintf('Results: %s\n\n', STUDY_DIR);

%% Define 5 Key Configurations
configs = {};

% 1. BASELINE
configs{1} = struct('name', 'Baseline_Tuned', ...
    'SafetyMargin', 0.10, 'LambdaCusp', 1.0, 'MaxRSIterations', 200, ...
    'AllowReverse', true, 'ClothoidDiscretization', 0.08, ...
    'Lookahead', 0.8, 'AccelLimit', 0.8, 'HeadingKp', 1.0);

% 2. AGGRESSIVE (Low safety, allow cusps)
configs{2} = struct('name', 'Aggressive_LowSafety', ...
    'SafetyMargin', 0.05, 'LambdaCusp', 0.5, 'MaxRSIterations', 200, ...
    'AllowReverse', true, 'ClothoidDiscretization', 0.08, ...
    'Lookahead', 1.2, 'AccelLimit', 1.2, 'HeadingKp', 1.5);

% 3. CONSERVATIVE (High safety, penalize cusps)
configs{3} = struct('name', 'Conservative_HighSafety', ...
    'SafetyMargin', 0.20, 'LambdaCusp', 3.0, 'MaxRSIterations', 400, ...
    'AllowReverse', false, 'ClothoidDiscretization', 0.05, ...
    'Lookahead', 0.6, 'AccelLimit', 0.6, 'HeadingKp', 0.8);

% 4. EXTREME ANTI-CUSP
configs{4} = struct('name', 'AntiCusp_Lambda10', ...
    'SafetyMargin', 0.10, 'LambdaCusp', 10.0, 'MaxRSIterations', 600, ...
    'AllowReverse', true, 'ClothoidDiscretization', 0.08, ...
    'Lookahead', 0.8, 'AccelLimit', 0.8, 'HeadingKp', 1.0);

% 5. HIGH RESOLUTION SMOOTHING
configs{5} = struct('name', 'HighRes_MaxSmooth', ...
    'SafetyMargin', 0.10, 'LambdaCusp', 1.5, 'MaxRSIterations', 1000, ...
    'AllowReverse', true, 'ClothoidDiscretization', 0.03, ...
    'Lookahead', 0.8, 'AccelLimit', 0.8, 'HeadingKp', 1.0);

nConfigs = length(configs);
fprintf('Running %d configurations...\n\n', nConfigs);

%% Run Each Configuration
results = [];

for i = 1:nConfigs
    cfg = configs{i};
    
    fprintf('[%d/%d] %s\n', i, nConfigs, cfg.name);
    fprintf('  SM=%.2f LC=%.1f Iters=%d Rev=%d\n', ...
            cfg.SafetyMargin, cfg.LambdaCusp, cfg.MaxRSIterations, cfg.AllowReverse);
    
    try
        tic;
        runResult = gik9dof.runStagedReference( ...
            'ExecutionMode', 'pureIk', 'SaveLog', false, ...
            'StageBHybridSafetyMargin', cfg.SafetyMargin, ...
            'StageBReedsSheppParams', struct('lambdaCusp', cfg.LambdaCusp, ...
                'maxIterations', cfg.MaxRSIterations, 'allowReverse', cfg.AllowReverse), ...
            'StageBUseReedsShepp', true, 'StageBUseClothoid', true, ...
            'StageBClothoidParams', struct('discretization', cfg.ClothoidDiscretization), ...
            'StageBLookaheadDistance', cfg.Lookahead, ...
            'ChassisOverrides', struct('accel_limit', cfg.AccelLimit, 'heading_kp', cfg.HeadingKp));
        
        log = runResult.log;
        elapsedTime = toc;
        
        % Extract metrics
        stageC = log.stageLogs.stageC;
        stageB = log.stageLogs.stageB;
        
        metrics = struct();
        metrics.eeErrorMean = mean(stageC.positionErrorNorm);
        metrics.eeErrorMax = max(stageC.positionErrorNorm);
        metrics.cuspCount = stageB.diagnostics.cuspCount;
        metrics.pathSmoothness = stageB.diagnostics.pathSmoothness;
        metrics.rsImprovements = stageB.diagnostics.rsImprovements;
        metrics.rsAcceptanceRate = stageB.diagnostics.rsAcceptanceRate;
        metrics.runTime = elapsedTime;
        
        % Compute jerk
        if isfield(stageC, 'referenceBaseStates')
            refBase = stageC.referenceBaseStates;
            dt = 0.1;
            dx = diff(refBase(:, 1)) / dt;
            dy = diff(refBase(:, 2)) / dt;
            velocity = sqrt(dx.^2 + dy.^2);
            accel = diff(velocity) / dt;
            jerk = diff(accel) / dt;
            metrics.refBaseJerkRMS = rms(jerk);
            metrics.refBaseJerkMax = max(abs(jerk));
        else
            metrics.refBaseJerkRMS = NaN;
            metrics.refBaseJerkMax = NaN;
        end
        
        fprintf('  ✓ %.1fs | EE=%.4f/%.4f | Cusps=%d | Jerk=%.1f\n', ...
                elapsedTime, metrics.eeErrorMean, metrics.eeErrorMax, ...
                metrics.cuspCount, metrics.refBaseJerkRMS);
        
        % Save log
        logFile = fullfile(LOG_DIR, sprintf('%s.mat', cfg.name));
        save(logFile, 'log', 'cfg', 'metrics', '-v7.3');
        
        result = struct('config', cfg, 'log', log, 'metrics', metrics, ...
                       'logFile', logFile, 'success', true);
        results = [results; result]; %#ok<AGROW>
        
    catch ME
        fprintf('  ✗ ERROR: %s\n', ME.message);
        result = struct('config', cfg, 'log', [], 'metrics', struct(), ...
                       'logFile', '', 'success', false, 'error', ME.message);
        results = [results; result]; %#ok<AGROW>
    end
end

%% Save Results
resultsFile = fullfile(STUDY_DIR, 'parametric_study_results.mat');
save(resultsFile, 'results', 'configs', '-v7.3');

fprintf('\n=== Study Complete ===\n');
fprintf('Successful: %d/%d\n', sum([results.success]), nConfigs);
fprintf('Results: %s\n', resultsFile);

%% Summary Table
fprintf('\n%-25s | EE Mean | EE Max  | Cusps | Jerk RMS | RS Acc\n', 'Config');
fprintf('%s\n', repmat('-', 1, 80));
for i = 1:length(results)
    if results(i).success
        m = results(i).metrics;
        fprintf('%-25s | %.4f  | %.4f  | %5d | %8.1f | %6.1f%%\n', ...
                results(i).config.name, m.eeErrorMean, m.eeErrorMax, ...
                m.cuspCount, m.refBaseJerkRMS, m.rsAcceptanceRate * 100);
    end
end

fprintf('\nNext: generate_parametric_animations\n');
