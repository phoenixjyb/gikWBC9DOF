# GIK Validation Asset Export

This note captures the workflow for turning an existing MATLAB log into a lightweight
validation bundle that can be consumed by the C++ GIK solver tests (e.g., on WSL).

## 1. Load a log

```matlab
% Replace the log path with the specific run you want to export.
data = load('results/20251007_122040_gik_iter_study_iterCompare/log_holistic_iter1500.mat');
log  = data.log;

robot = gik9dof.createRobotModel();
cfg   = gik9dof.configurationTools(robot);
names = string(cfg.templateJointNames());

qInit   = log.qTraj(:,1);      % 9-DOF initial configuration
poses4x4 = log.targetPoses;    % 4x4xN target pose stack
solution = log.qTraj(:,2:end); % optional reference joint solution
timestamps = log.time(2:end);  % time stamps aligned with poses
```

Any log produced by `trackReferenceTrajectory`, `run_stageb_mode_compare`, or
`run_gik_iteration_study` exposes the same fields.

## 2. Export helper

Add the following helper to your MATLAB toolbox (e.g., save under
`matlab/export_gik_validation_assets.m`). It loads a log, subsamples waypoints,
and writes CSV/JSON files for direct consumption in C++.

```matlab
function assets = export_gik_validation_assets(logInput, outDir, varargin)
arguments
    logInput
    outDir (1,1) string
    varargin.SampleStride (1,1) double {mustBeInteger, mustBePositive} = 1
end

if ~isstruct(logInput)
    s = load(logInput);
    if ~isfield(s, 'log')
        error('export_gik_validation_assets:MissingLog', 'MAT file must contain a variable named ''log''.');
    end
    log = s.log;
else
    log = logInput;
end

robotColumn = gik9dof.createRobotModel();
configTools = gik9dof.configurationTools(robotColumn);
jointNames  = string(configTools.templateJointNames());

if size(log.qTraj,1) ~= numel(jointNames)
    error('export_gik_validation_assets:Mismatch', 'Log joint dimension (%d) differs from robot DOFs (%d).', size(log.qTraj,1), numel(jointNames));
end

sampleIdx = 1:varargin.SampleStride:(size(log.qTraj,2)-1);

initialState = struct('Names', jointNames, 'Values', log.qTraj(:,1));
qSolutions   = log.qTraj(:, sampleIdx + 1);
TTargets     = log.targetPoses(:,:, sampleIdx);
timeVec      = deriveTime(log, sampleIdx + 1);

poseTable = buildPoseTable(TTargets, timeVec);
solutionTable = array2table(qSolutions', 'VariableNames', jointNames);
solutionTable.time = timeVec(:);

if ~isfolder(outDir)
    mkdir(outDir);
end

% initial_state.csv
initTable = cell2table([cellstr(initialState.Names)', num2cell(initialState.Values')], ...
    'VariableNames', {'joint','value'});
writetable(initTable, fullfile(outDir, 'initial_state.csv'));

% target_poses.csv
writetable(poseTable, fullfile(outDir, 'target_poses.csv'));

% reference_solution.csv (optional ground-truth joints)
writetable(solutionTable, fullfile(outDir, 'reference_solution.csv'));

meta = struct('rateHz', getfieldwithdefault(log, 'rateHz', NaN), ...
              'sampleStride', varargin.SampleStride, ...
              'numTargets', numel(sampleIdx), ...
              'iterationStats', getfieldwithdefault(log, 'iterationStats', struct()), ...
              'solveTimeStats', getfieldwithdefault(log, 'solveTimeStats', struct()));

jsonencode(meta, 'PrettyPrint', true, 'Filename', fullfile(outDir, 'metadata.json'));

assets = struct('InitialState', initialState, 'PoseTable', poseTable, ...
    'SolutionTable', solutionTable, 'Metadata', meta, 'OutputDir', outDir);
end

function poseTable = buildPoseTable(Tstack, timestamps)
numPose = size(Tstack,3);
pos = zeros(numPose, 3);
quat = zeros(numPose, 4);
for k = 1:numPose
    T = Tstack(:,:,k);
    pos(k,:) = T(1:3,4)';
    quat(k,:) = rotm2quat(T(1:3,1:3));
end
poseTable = table(timestamps(:), pos(:,1), pos(:,2), pos(:,3), ...
    quat(:,1), quat(:,2), quat(:,3), quat(:,4), ...
    'VariableNames', {'time','px','py','pz','qw','qx','qy','qz'});
end

function t = deriveTime(log, idx)
if isfield(log, 'time') && numel(log.time) >= max(idx)
    t = log.time(idx);
else
    sampleTime = 1 / getfieldwithdefault(log, 'rateHz', 30);
    t = (idx - 1) * sampleTime;
end
end

function val = getfieldwithdefault(structure, fieldName, defaultVal)
if isstruct(structure) && isfield(structure, fieldName)
    val = structure.(fieldName);
else
    val = defaultVal;
end
end
```

## 3. Sample usage

```matlab
assets = export_gik_validation_assets( ...
    'results/20251007_122040_gik_iter_study_iterCompare/log_holistic_iter1500.mat', ...
    'validation_assets/holistic_iter1500', ...
    'SampleStride', 5);
```

You will obtain:

- `initial_state.csv` — joint names and the starting 9-DOF configuration.
- `target_poses.csv` — timestamped EE positions + quaternions.
- `reference_solution.csv` — optional solver output joints (aligned with the EE poses).
- `metadata.json` — rate, stride, iteration/solve-time statistics for context.

The `SampleStride` option lets you downsample to a manageable size for quick unit tests.

## 4. Consuming the assets in C++

Typical validation loop:

```cpp
auto init = readCsv("initial_state.csv");      // map joint -> value
auto poses = readPoseCsv("target_poses.csv");  // time, px, py, pz, qw, qx, qy, qz

solver.setInitialConfiguration(init);
for (const Pose& target : poses) {
    solver.setTargetPose(target);
    auto q = solver.solve();
    // Optional: compare q against reference_solution.csv or check convergence statistics
}
```

Load `metadata.json` if you want to assert solve-time/iteration budgets.

## 5. Extensions

- To export Stage-specific trajectories, pass `log.stageLogs.stageC` (or other stage logs) to the helper.
- Randomize or reduce the target set by adjusting `sampleIdx` before writing.
- Save the same assets as `.mat` for MATLAB-driven unit tests or for round-trip comparisons.

This workflow keeps the MATLAB validation assets in a consistent, lightweight format, allowing rapid testing of the generated GIK solver in external environments.
