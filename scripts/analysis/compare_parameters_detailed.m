% Deep parameter comparison between isolated and staged runs
% Focus on finding what parameters/settings are different

clearvars; clc;

fprintf('========================================\n');
fprintf('  PARAMETER DEEP DIVE\n');
fprintf('========================================\n\n');

%% Check the test scripts to see how they call the functions

fprintf('PART 1: How Isolated Test Calls runStageCPPFirst_enhanced\n');
fprintf('=========================================================\n\n');

% Read test_stagec_only_home_start.m
fid = fopen('test_stagec_only_home_start.m', 'r');
testCode = fread(fid, '*char')';
fclose(fid);

% Find the runStageCPPFirst_enhanced call
pattern = 'runStageCPPFirst_enhanced\(([^)]+(?:\([^)]*\)[^)]*)*)';
matches = regexp(testCode, pattern, 'match');

if ~isempty(matches)
    fprintf('Isolated test calls runStageCPPFirst_enhanced with:\n');
    % Extract the full function call including parameters
    callStart = strfind(testCode, 'gik9dof.runStageCPPFirst_enhanced');
    if ~isempty(callStart)
        callEnd = find(testCode(callStart:end) == ';', 1) + callStart - 1;
        fullCall = testCode(callStart:callEnd);
        fprintf('%s\n\n', fullCall);
    end
else
    fprintf('Could not find runStageCPPFirst_enhanced call\n\n');
end

fprintf('PART 2: How Staged Run Calls runStageCPPFirst_enhanced\n');
fprintf('=========================================================\n\n');

% Read runStagedTrajectory.m executeStageCPPFirst function
fprintf('Checking executeStageCPPFirst in runStagedTrajectory.m...\n\n');

% We know it's around line 938 based on earlier investigation
% Let's read that section

fprintf('PART 3: Comparing Actual Parameters from Log Files\n');
fprintf('=========================================================\n\n');

% Load isolated test
isoData = load('results/stage_c_only_home_start/log_20251014_162314.mat');
isoLog = isoData.log;

% Load staged run
stgData = load('results/20251014_145719_fresh_no_discs/log_staged_ppForIk.mat');
logC_staged = stgData.log.stageLogs.stageC;

fprintf('Isolated test parameters (from log.parameters field):\n');
if isfield(isoLog, 'parameters')
    paramFields = fieldnames(isoLog.parameters);
    for i = 1:length(paramFields)
        field = paramFields{i};
        value = isoLog.parameters.(field);
        if isnumeric(value) && isscalar(value)
            fprintf('  %s: %.6f\n', field, value);
        elseif islogical(value) && isscalar(value)
            fprintf('  %s: %s\n', field, mat2str(value));
        elseif isnumeric(value) && isvector(value) && length(value) <= 10
            fprintf('  %s: %s\n', field, mat2str(value));
        else
            fprintf('  %s: [%s]\n', field, class(value));
        end
    end
else
    fprintf('  No parameters field found in isolated log\n');
end

fprintf('\nStaged run chassis parameters:\n');
if isfield(logC_staged, 'chassisParams')
    paramFields = fieldnames(logC_staged.chassisParams);
    for i = 1:length(paramFields)
        field = paramFields{i};
        value = logC_staged.chassisParams.(field);
        if isnumeric(value) && isscalar(value)
            fprintf('  %s: %.6f\n', field, value);
        elseif islogical(value) && isscalar(value)
            fprintf('  %s: %s\n', field, mat2str(value));
        elseif isnumeric(value) && isvector(value) && length(value) <= 10
            fprintf('  %s: %s\n', field, mat2str(value));
        else
            fprintf('  %s: [%s]\n', field, class(value));
        end
    end
else
    fprintf('  No chassisParams field found in staged log\n');
end

fprintf('\n\nPART 4: Key Diagnostic Fields Comparison\n');
fprintf('=========================================================\n\n');

% Check if Phase 1/2A diagnostic fields are present
fprintf('Phase 1 diagnostic fields:\n');
fprintf('  Isolated has lateralVelocity:      %s\n', mat2str(isfield(isoLog, 'lateralVelocity')));
fprintf('  Staged has lateralVelocity:        %s\n', mat2str(isfield(logC_staged, 'lateralVelocity')));
fprintf('  Isolated has lookaheadEffective:   %s\n', mat2str(isfield(isoLog, 'lookaheadEffective')));
fprintf('  Staged has lookaheadEffective:     %s\n', mat2str(isfield(logC_staged, 'lookaheadEffective')));
fprintf('  Isolated has microSegmentUsed:     %s\n', mat2str(isfield(isoLog, 'microSegmentUsed')));
fprintf('  Staged has microSegmentUsed:       %s\n', mat2str(isfield(logC_staged, 'microSegmentUsed')));
fprintf('  Isolated has corridorSizes:        %s\n', mat2str(isfield(isoLog, 'corridorSizes')));
fprintf('  Staged has corridorSizes:          %s\n', mat2str(isfield(logC_staged, 'corridorSizes')));

fprintf('\nThis tells us if Phase 1 improvements are enabled!\n');

if isfield(isoLog, 'lateralVelocity') && ~isfield(logC_staged, 'lateralVelocity')
    fprintf('\nWARNING: Phase 1 diagnostics present in isolated but NOT in staged!\n');
    fprintf('   This suggests Phase 1 improvements may not be enabled in staged run.\n');
end

fprintf('\n\nPART 5: Examining Stage C Starting Configuration\n');
fprintf('=========================================================\n\n');

q0_iso = isoLog.qTraj(:, 1);
q0_stg = logC_staged.qTraj(:, 1);

fprintf('Starting configurations:\n');
fprintf('  Isolated: [');
for i = 1:9
    fprintf('%.4f', q0_iso(i));
    if i < 9, fprintf(', '); end
end
fprintf(']\n');

fprintf('  Staged:   [');
for i = 1:9
    fprintf('%.4f', q0_stg(i));
    if i < 9, fprintf(', '); end
end
fprintf(']\n');

fprintf('\nKey observation:\n');
fprintf('  Base position diff: %.4f m\n', norm(q0_iso(1:2) - q0_stg(1:2)));
fprintf('  Base heading diff:  %.2f deg\n', rad2deg(abs(q0_iso(3) - q0_stg(3))));
fprintf('  Arm config diff:    %.4f rad\n', norm(q0_iso(4:9) - q0_stg(4:9)));

if norm(q0_iso(4:9) - q0_stg(4:9)) > 1.0
    fprintf('\nMAJOR DIFFERENCE: Arm configurations are VERY different!\n');
    fprintf('   Isolated starts from home arm config\n');
    fprintf('   Staged starts from Stage B ending arm config\n');
    fprintf('   This could affect GIK solutions!\n');
end

fprintf('\n\nPART 6: Checking if runStageCPPFirst_enhanced is called correctly\n');
fprintf('==================================================================\n\n');

% Check if the staged run is calling the enhanced version
if isfield(logC_staged, 'diagnostics')
    fprintf('Staged run has diagnostics field - checking content...\n');
    diag = logC_staged.diagnostics;
    if isstruct(diag)
        diagFields = fieldnames(diag);
        fprintf('Diagnostics fields: %s\n', strjoin(diagFields, ', '));
    end
else
    fprintf('WARNING: Staged run does NOT have diagnostics field\n');
    fprintf('   This might indicate it is not using runStageCPPFirst_enhanced\n');
end

fprintf('\n========================================\n');
fprintf('  ANALYSIS COMPLETE\n');
fprintf('========================================\n');
