%% Smoke Test: Method 5 (pureMPC) through the staged pipeline
% Ensures that trackReferenceTrajectory can execute Stage C with pureMPC
% without throwing integration errors. The test trims the bundled JSON to
% a short segment so the run completes quickly.

clear; clc;

fprintf('=== Method 5 (pureMPC) Pipeline Smoke Test ===\n');

% Load the pureMPC profile so Stage C uses the whole-body MPC settings.
pipelineConfig = gik9dof.loadPipelineProfile('pureMPC');

% Read and trim the reference trajectory to limit runtime.
projectRoot = fileparts(fileparts(mfilename('fullpath')));
jsonPath = fullfile(projectRoot, '1_pull_world_scaled.json');
rawJson = jsondecode(fileread(jsonPath));
maxWaypoints = min(25, numel(rawJson.poses));
rawJson.poses = rawJson.poses(1:maxWaypoints);

% Write the trimmed trajectory to a temporary JSON file.
tempJson = tempname;
tempJson = [tempJson, '.json']; %#ok<AGROW>
fid = fopen(tempJson, 'w');
if fid == -1
    error('test_method5_pipeline_smoke:TempFileError', ...
        'Unable to open temporary trajectory file for writing: %s', tempJson);
end
cleanup = onCleanup(@() delete(tempJson));
fprintf(fid, '%s', jsonencode(rawJson));
fclose(fid);

fprintf('Running staged pipeline (pureMPC) on %d waypoints...\n', maxWaypoints);

try
    log = gik9dof.trackReferenceTrajectory( ...
        'Mode', 'staged', ...
        'ExecutionMode', 'pureMPC', ...
        'PipelineConfig', pipelineConfig, ...
        'JsonPath', tempJson, ...
        'Verbose', false);
catch ME
    disp(getReport(ME, 'extended'));
    error('test_method5_pipeline_smoke:ExecutionFailed', ...
        'PureMPC staged pipeline failed: %s', ME.message);
end

assert(isfield(log, 'stageLogs') && isfield(log.stageLogs, 'stageC'), ...
    'Stage C log missing from trackReferenceTrajectory output.');
diagnostics = log.stageLogs.stageC.stageCDiagnostics;
assert(isfield(diagnostics, 'mpcConvergenceRate'), ...
    'Stage C diagnostics missing MPC convergence rate.');

fprintf('PureMPC pipeline executed successfully. MPC convergence: %.1f%%\n', ...
    diagnostics.mpcConvergenceRate * 100);
