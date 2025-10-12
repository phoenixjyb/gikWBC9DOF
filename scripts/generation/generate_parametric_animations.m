%% Generate Animations for Parametric Study
%  This script loads the parametric study results and generates
%  animations with descriptive titles showing configuration parameters.

clear; clc;
addpath(genpath('matlab'));

fprintf('=== Parametric Study Animation Generator ===\n');
fprintf('Date: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% Find Latest Parametric Study Results
studyDirs = dir('results/parametric_study_*');
if isempty(studyDirs)
    error('No parametric study results found! Please run run_parametric_study.m first.');
end

% Sort by date (newest first)
dates = datetime({studyDirs.date}, 'InputFormat', 'dd-MMM-yyyy HH:mm:ss');
[~, idx] = sort(dates, 'descend');
latestDir = studyDirs(idx(1));
STUDY_DIR = fullfile('results', latestDir.name);
LOG_DIR = fullfile(STUDY_DIR, 'logs');
ANIM_DIR = fullfile(STUDY_DIR, 'animations');

fprintf('Loading results from: %s\n', STUDY_DIR);
fprintf('Animations will be saved to: %s\n\n', ANIM_DIR);

% Create animation directory if it doesn't exist
if ~exist(ANIM_DIR, 'dir')
    mkdir(ANIM_DIR);
end

%% Load Results
resultsFile = fullfile(STUDY_DIR, 'parametric_study_results.mat');
if ~exist(resultsFile, 'file')
    error('Results file not found: %s', resultsFile);
end

load(resultsFile, 'results');

nConfigs = length(results);
nSuccess = sum([results.success]);
fprintf('Found %d configurations (%d successful)\n\n', nConfigs, nSuccess);

%% Generate Animations
successCount = 0;
failCount = 0;

for i = 1:nConfigs
    result = results(i);
    cfg = result.config;
    
    fprintf('\n========================================\n');
    fprintf('[%d/%d] %s\n', i, nConfigs, cfg.name);
    fprintf('========================================\n');
    
    if ~result.success
        fprintf('⊗ Skipping (run failed)\n');
        continue;
    end
    
    % Build descriptive filename and info
    m = result.metrics;
    
    % Filename with key parameters
    animFile = fullfile(ANIM_DIR, sprintf('%s_SM%.2f_LC%.1f_EE%.4f_Cusps%d_Jerk%.1f.mp4', ...
        cfg.name, cfg.SafetyMargin, cfg.LambdaCusp, m.eeErrorMean, m.cuspCount, m.refBaseJerkRMS));
    
    fprintf('Config: %s\n', cfg.name);
    fprintf('  Parameters: SM=%.2f LC=%.1f Iter=%d Rev=%d CD=%.2f LH=%.2f\n', ...
        cfg.SafetyMargin, cfg.LambdaCusp, cfg.MaxRSIterations, ...
        cfg.AllowReverse, cfg.ClothoidDiscretization, cfg.Lookahead);
    fprintf('  Metrics: EE=%.4f/%.4f Cusps=%d Jerk=%.1f RS=%.0f%%\n', ...
        m.eeErrorMean, m.eeErrorMax, m.cuspCount, m.refBaseJerkRMS, m.rsAcceptanceRate*100);
    fprintf('  Output: %s\n', animFile);
    
    try
        % Generate animation
        fprintf('  Generating animation...\n');
        tic;
        
        gik9dof.animateStagedWithHelper( ...
            result.log, ...
            'ExportVideo', animFile, ...
            'FrameRate', 20, ...
            'SampleStep', 2);
        
        elapsedTime = toc;
        fprintf('  ✓ Animation saved (%.1f seconds)\n', elapsedTime);
        
        successCount = successCount + 1;
        
    catch ME
        fprintf('  ✗ Animation failed: %s\n', ME.message);
        failCount = failCount + 1;
    end
end

%% Generate Animation Index
fprintf('\n\n========================================\n');
fprintf('Generating Animation Index...\n');
fprintf('========================================\n');

indexFile = fullfile(ANIM_DIR, 'animation_index.txt');
fid = fopen(indexFile, 'w');

fprintf(fid, 'PARAMETRIC STUDY ANIMATION INDEX\n');
fprintf(fid, '================================\n');
fprintf(fid, 'Generated: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf(fid, 'Study Directory: %s\n\n', STUDY_DIR);
fprintf(fid, 'Total Configurations: %d\n', nConfigs);
fprintf(fid, 'Animations Generated: %d\n', successCount);
fprintf(fid, 'Failed: %d\n\n', failCount);

fprintf(fid, 'ANIMATIONS:\n');
fprintf(fid, '%s\n\n', repmat('=', 1, 80));

for i = 1:nConfigs
    if ~results(i).success
        continue;
    end
    
    result = results(i);
    cfg = result.config;
    m = result.metrics;
    
    animFile = fullfile(ANIM_DIR, sprintf('%s.mp4', cfg.name));
    
    if exist(animFile, 'file')
        fprintf(fid, '[%d] %s\n', i, cfg.name);
        fprintf(fid, '    File: %s\n', animFile);
        fprintf(fid, '    Description: %s\n', cfg.description);
        fprintf(fid, '    Stage B Parameters:\n');
        fprintf(fid, '      - Safety Margin: %.2f m\n', cfg.SafetyMargin);
        fprintf(fid, '      - Lambda Cusp: %.1f\n', cfg.LambdaCusp);
        fprintf(fid, '      - Max RS Iterations: %d\n', cfg.MaxRSIterations);
        fprintf(fid, '      - Allow Reverse: %s\n', mat2str(cfg.AllowReverse));
        fprintf(fid, '      - Clothoid Discretization: %.2f m\n', cfg.ClothoidDiscretization);
        fprintf(fid, '    Stage C Parameters:\n');
        fprintf(fid, '      - Lookahead: %.2f m\n', cfg.Lookahead);
        fprintf(fid, '      - Accel Limit: %.1f m/s²\n', cfg.AccelLimit);
        fprintf(fid, '      - Heading Kp: %.1f\n', cfg.HeadingKp);
        fprintf(fid, '    Performance Metrics:\n');
        fprintf(fid, '      - EE Error Mean: %.4f m\n', m.eeErrorMean);
        fprintf(fid, '      - EE Error Max: %.4f m\n', m.eeErrorMax);
        fprintf(fid, '      - Cusp Count: %d\n', m.cuspCount);
        fprintf(fid, '      - Path Smoothness: %.3f\n', m.pathSmoothness);
        fprintf(fid, '      - RS Improvements: %d\n', m.rsImprovements);
        fprintf(fid, '      - RS Acceptance Rate: %.1f%%\n', m.rsAcceptanceRate * 100);
        fprintf(fid, '      - Ref Base Jerk RMS: %.1f m/s³\n', m.refBaseJerkRMS);
        fprintf(fid, '      - Ref Base Jerk Max: %.1f m/s³\n', m.refBaseJerkMax);
        fprintf(fid, '      - Run Time: %.1f seconds\n', m.runTime);
        fprintf(fid, '\n');
    end
end

fclose(fid);

fprintf('✓ Animation index saved: %s\n', indexFile);

%% Summary
fprintf('\n\n========================================\n');
fprintf('ANIMATION GENERATION COMPLETE\n');
fprintf('========================================\n');
fprintf('Total animations: %d\n', successCount);
fprintf('Failed: %d\n', failCount);
fprintf('Animation directory: %s\n', ANIM_DIR);
fprintf('Index file: %s\n', indexFile);

fprintf('\n=== Review Instructions ===\n');
fprintf('1. Open animation directory: %s\n', ANIM_DIR);
fprintf('2. Review animation_index.txt for full details\n');
fprintf('3. Watch animations to visually compare configurations\n');
fprintf('4. Pay attention to:\n');
fprintf('   - Path smoothness (kinks/jerkiness)\n');
fprintf('   - Cusp locations and severity\n');
fprintf('   - Collision clearances\n');
fprintf('   - EE tracking accuracy\n');
fprintf('   - Chassis constraint violations\n');
