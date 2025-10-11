%% Compare Test Configurations
% Compare test_comprehensive_evaluation.mat vs parametric_study results

clear; clc;

fprintf('=== Configuration Comparison ===\n\n');

%% Load test_comprehensive_evaluation.mat
fprintf('Loading: results/test_comprehensive_evaluation.mat\n');
data1 = load('results/test_comprehensive_evaluation.mat');
results1 = data1.results;

fprintf('  Found %d configurations\n\n', length(results1));

fprintf('FILE 1: test_comprehensive_evaluation.mat\n');
fprintf('%s\n', repmat('=', 1, 100));
for i = 1:length(results1)
    fprintf('\n[%d] %s\n', i, results1(i).name);
    fprintf('  Parameters:\n');
    ov = results1(i).overrides;
    fprintf('    SafetyMargin:     %.2f\n', ov.SafetyMargin);
    fprintf('    LambdaCusp:       %.1f\n', ov.LambdaCusp);
    fprintf('    MaxRSIterations:  %d\n', ov.MaxRSIterations);
    fprintf('    AllowReverse:     %d\n', ov.AllowReverse);
    fprintf('    ClothoidDisc:     %.2f\n', ov.ClothoidDiscretization);
    fprintf('    Lookahead:        %.2f\n', ov.Lookahead);
    fprintf('    AccelLimit:       %.1f\n', ov.AccelLimit);
    fprintf('    HeadingKp:        %.1f\n', ov.HeadingKp);
    fprintf('  Metrics:\n');
    r = results1(i).evalReport;
    fprintf('    EE Error:  mean=%.4f  max=%.4f\n', r.c3_meanError, r.c3_maxError);
    fprintf('    Cusps:     %d\n', r.c5_cuspCount);
    fprintf('    Smoothness: %.3f\n', r.c5_pathSmoothness);
    fprintf('    Score:     %.3f\n', r.overallScore);
end

%% Load parametric_study results
fprintf('\n\n%s\n', repmat('=', 1, 100));
fprintf('Loading: results/parametric_study_20251011_085252/parametric_study_results.mat\n');
data2 = load('results/parametric_study_20251011_085252/parametric_study_results.mat');
results2 = data2.results;

fprintf('  Found %d configurations\n\n', length(results2));

fprintf('FILE 2: parametric_study_20251011_085252/parametric_study_results.mat\n');
fprintf('%s\n', repmat('=', 1, 100));
for i = 1:length(results2)
    if ~results2(i).success
        fprintf('\n[%d] %s - FAILED\n', i, results2(i).config.name);
        continue;
    end
    
    fprintf('\n[%d] %s\n', i, results2(i).config.name);
    fprintf('  Parameters:\n');
    cfg = results2(i).config;
    fprintf('    SafetyMargin:     %.2f\n', cfg.SafetyMargin);
    fprintf('    LambdaCusp:       %.1f\n', cfg.LambdaCusp);
    fprintf('    MaxRSIterations:  %d\n', cfg.MaxRSIterations);
    fprintf('    AllowReverse:     %d\n', cfg.AllowReverse);
    fprintf('    ClothoidDisc:     %.2f\n', cfg.ClothoidDiscretization);
    fprintf('    Lookahead:        %.2f\n', cfg.Lookahead);
    fprintf('    AccelLimit:       %.1f\n', cfg.AccelLimit);
    fprintf('    HeadingKp:        %.1f\n', cfg.HeadingKp);
    fprintf('  Metrics:\n');
    m = results2(i).metrics;
    fprintf('    EE Error:  mean=%.4f  max=%.4f\n', m.eeErrorMean, m.eeErrorMax);
    fprintf('    Cusps:     %d\n', m.cuspCount);
    fprintf('    Smoothness: %.3f\n', m.pathSmoothness);
    fprintf('    RS Accept: %.1f%%\n', m.rsAcceptanceRate * 100);
    fprintf('    Jerk RMS:  %.1f m/s³\n', m.refBaseJerkRMS);
end

%% Compare overlapping configurations
fprintf('\n\n%s\n', repmat('=', 1, 100));
fprintf('COMPARISON: Overlapping Configurations\n');
fprintf('%s\n\n', repmat('=', 1, 100));

% Find Tuned Baseline in both
idx1_baseline = find(strcmp({results1.name}, 'Tuned Baseline'));
configNames2 = cell(1, length(results2));
for i = 1:length(results2)
    configNames2{i} = results2(i).config.name;
end
idx2_baseline = find(strcmp(configNames2, 'Baseline_Tuned'));

if ~isempty(idx1_baseline) && ~isempty(idx2_baseline)
    fprintf('BASELINE Configuration:\n');
    fprintf('  File 1 (Tuned Baseline):\n');
    ov = results1(idx1_baseline).overrides;
    fprintf('    SM=%.2f LC=%.1f Iters=%d Rev=%d CD=%.2f LH=%.2f AL=%.1f HK=%.1f\n', ...
        ov.SafetyMargin, ov.LambdaCusp, ov.MaxRSIterations, ov.AllowReverse, ...
        ov.ClothoidDiscretization, ov.Lookahead, ov.AccelLimit, ov.HeadingKp);
    
    fprintf('  File 2 (Baseline_Tuned):\n');
    cfg = results2(idx2_baseline).config;
    fprintf('    SM=%.2f LC=%.1f Iters=%d Rev=%d CD=%.2f LH=%.2f AL=%.1f HK=%.1f\n', ...
        cfg.SafetyMargin, cfg.LambdaCusp, cfg.MaxRSIterations, cfg.AllowReverse, ...
        cfg.ClothoidDiscretization, cfg.Lookahead, cfg.AccelLimit, cfg.HeadingKp);
    
    % Check if identical
    paramsMatch = (ov.SafetyMargin == cfg.SafetyMargin) && ...
                  (ov.LambdaCusp == cfg.LambdaCusp) && ...
                  (ov.MaxRSIterations == cfg.MaxRSIterations) && ...
                  (ov.AllowReverse == cfg.AllowReverse) && ...
                  (abs(ov.ClothoidDiscretization - cfg.ClothoidDiscretization) < 0.001) && ...
                  (abs(ov.Lookahead - cfg.Lookahead) < 0.001) && ...
                  (abs(ov.AccelLimit - cfg.AccelLimit) < 0.1) && ...
                  (abs(ov.HeadingKp - cfg.HeadingKp) < 0.1);
    
    if paramsMatch
        fprintf('  ✅ IDENTICAL parameters\n\n');
    else
        fprintf('  ❌ DIFFERENT parameters\n\n');
    end
end

% Check Conservative
idx1_cons = find(strcmp({results1.name}, 'Conservative'));
idx2_cons = find(strcmp(configNames2, 'Conservative_HighSafety'));

if ~isempty(idx1_cons) && ~isempty(idx2_cons)
    fprintf('CONSERVATIVE Configuration:\n');
    fprintf('  File 1 (Conservative):\n');
    ov = results1(idx1_cons).overrides;
    fprintf('    SM=%.2f LC=%.1f Iters=%d Rev=%d CD=%.2f LH=%.2f AL=%.1f HK=%.1f\n', ...
        ov.SafetyMargin, ov.LambdaCusp, ov.MaxRSIterations, ov.AllowReverse, ...
        ov.ClothoidDiscretization, ov.Lookahead, ov.AccelLimit, ov.HeadingKp);
    
    fprintf('  File 2 (Conservative_HighSafety):\n');
    cfg = results2(idx2_cons).config;
    fprintf('    SM=%.2f LC=%.1f Iters=%d Rev=%d CD=%.2f LH=%.2f AL=%.1f HK=%.1f\n', ...
        cfg.SafetyMargin, cfg.LambdaCusp, cfg.MaxRSIterations, cfg.AllowReverse, ...
        cfg.ClothoidDiscretization, cfg.Lookahead, cfg.AccelLimit, cfg.HeadingKp);
    
    % Check if identical
    paramsMatch = (ov.SafetyMargin == cfg.SafetyMargin) && ...
                  (ov.LambdaCusp == cfg.LambdaCusp) && ...
                  (ov.MaxRSIterations == cfg.MaxRSIterations) && ...
                  (ov.AllowReverse == cfg.AllowReverse) && ...
                  (abs(ov.ClothoidDiscretization - cfg.ClothoidDiscretization) < 0.001) && ...
                  (abs(ov.Lookahead - cfg.Lookahead) < 0.001) && ...
                  (abs(ov.AccelLimit - cfg.AccelLimit) < 0.1) && ...
                  (abs(ov.HeadingKp - cfg.HeadingKp) < 0.1);
    
    if paramsMatch
        fprintf('  ✅ IDENTICAL parameters\n\n');
    else
        fprintf('  ❌ DIFFERENT parameters\n\n');
    end
end

% Check Aggressive
idx1_agg = find(strcmp({results1.name}, 'Aggressive'));
idx2_agg = find(strcmp(configNames2, 'Aggressive_LowSafety'));

if ~isempty(idx1_agg) && ~isempty(idx2_agg)
    fprintf('AGGRESSIVE Configuration:\n');
    fprintf('  File 1 (Aggressive):\n');
    ov = results1(idx1_agg).overrides;
    fprintf('    SM=%.2f LC=%.1f Iters=%d Rev=%d CD=%.2f LH=%.2f AL=%.1f HK=%.1f\n', ...
        ov.SafetyMargin, ov.LambdaCusp, ov.MaxRSIterations, ov.AllowReverse, ...
        ov.ClothoidDiscretization, ov.Lookahead, ov.AccelLimit, ov.HeadingKp);
    
    fprintf('  File 2 (Aggressive_LowSafety):\n');
    cfg = results2(idx2_agg).config;
    fprintf('    SM=%.2f LC=%.1f Iters=%d Rev=%d CD=%.2f LH=%.2f AL=%.1f HK=%.1f\n', ...
        cfg.SafetyMargin, cfg.LambdaCusp, cfg.MaxRSIterations, cfg.AllowReverse, ...
        cfg.ClothoidDiscretization, cfg.Lookahead, cfg.AccelLimit, cfg.HeadingKp);
    
    % Check if identical
    paramsMatch = (ov.SafetyMargin == cfg.SafetyMargin) && ...
                  (ov.LambdaCusp == cfg.LambdaCusp) && ...
                  (ov.MaxRSIterations == cfg.MaxRSIterations) && ...
                  (ov.AllowReverse == cfg.AllowReverse) && ...
                  (abs(ov.ClothoidDiscretization - cfg.ClothoidDiscretization) < 0.001) && ...
                  (abs(ov.Lookahead - cfg.Lookahead) < 0.001) && ...
                  (abs(ov.AccelLimit - cfg.AccelLimit) < 0.1) && ...
                  (abs(ov.HeadingKp - cfg.HeadingKp) < 0.1);
    
    if paramsMatch
        fprintf('  ✅ IDENTICAL parameters\n\n');
    else
        fprintf('  ❌ DIFFERENT parameters\n\n');
    end
end

fprintf('\n%s\n', repmat('=', 1, 100));
fprintf('SUMMARY\n');
fprintf('%s\n\n', repmat('=', 1, 100));
fprintf('File 1: %d configurations (Tuned Baseline, Conservative, Aggressive, High Resolution)\n', length(results1));
fprintf('File 2: %d configurations (Baseline_Tuned, Aggressive, Conservative, AntiCusp, HighRes)\n', length(results2));
fprintf('\nOverlapping configs: 3 (Baseline, Conservative, Aggressive)\n');
fprintf('File 2 extras: 2 (AntiCusp_Lambda10, HighRes_MaxSmooth)\n');
fprintf('File 1 extras: 1 (High Resolution - different from HighRes_MaxSmooth)\n');
