function export_all_commands()
% EXPORT_ALL_COMMANDS
% Scans ./results/*_compare and exports:
%   - commands_arm_<tag>.csv
%   - commands_chassis_<tag>.csv
%
% Robust to minor log format differences (time fields, qTraj orientation).

resultsRoot = fullfile(pwd, "results");
folders = dir(fullfile(resultsRoot, '*_compare'));
folders = folders([folders.isdir]);

for k = 1:numel(folders)
    runDir = fullfile(folders(k).folder, folders(k).name);
    export_commands_for_log(fullfile(runDir, "log_holistic.mat"), runDir, "holistic");
    export_commands_for_log(fullfile(runDir, "log_staged.mat"),   runDir, "staged");
end
end

function export_commands_for_log(matPath, runDir, tag)
% EXPORT_COMMANDS_FOR_LOG  Export arm joint commands and chassis velocity commands.

if ~isfile(matPath), return; end

% --- Load .mat and pick the first variable (or 'log' if present) safely
data = load(matPath);
f = fieldnames(data);
if isfield(data, "log")
    log = data.log;
elseif ~isempty(f)
    log = data.(f{1});
else
    warning('No variables found in %s. Skipping.', matPath);
    return
end

% --- Derive joint names and indices (requires your gik9dof utilities)
tools = gik9dof.configurationTools(gik9dof.createRobotModel());
names = string(tools.templateJointNames());  % ensure string type

% Look up base joints
idxBase = [find(names=="joint_x",1), find(names=="joint_y",1), find(names=="joint_theta",1)];
if numel(idxBase) ~= 3
    error('Base joints [joint_x, joint_y, joint_theta] not all found. Names: %s', strjoin(names, ', '));
end
idxArm  = setdiff(1:numel(names), idxBase);

% --- Get trajectory; auto-fix orientation if needed
if ~isfield(log, "qTraj")
    error('Field "qTraj" not found in %s', matPath);
end
qTraj = log.qTraj;
% Expecting size = [nJoints x T]. If not, try transpose.
if size(qTraj,1) ~= numel(names) && size(qTraj,2) == numel(names)
    qTraj = qTraj.'; % transpose to [nJoints x T]
end
if size(qTraj,1) ~= numel(names)
    error('qTraj has incompatible shape: [%d x %d] vs numel(names)=%d', size(qTraj,1), size(qTraj,2), numel(names));
end
T = size(qTraj,2);

% --- Build time vector
% Prefer 'timestamps' then 'time'. If length equals T-1, prepend 0 to get T samples.
t = [];
if isfield(log, "timestamps"), t = log.timestamps; end
if isempty(t) && isfield(log, "time"), t = log.time; end
t = t(:)'; % row

if isempty(t)
    % Fallback: synthetic sample times (0,1,2,...)
    time = 0:(T-1);
elseif numel(t) == T
    time = t;
elseif numel(t) == T-1
    time = [0, t];
else
    % Best-effort: resample/trim or pad to T
    if numel(t) > T
        time = t(1:T);
    else
        % pad with last known step or zeros
        dt_tail = 0;
        if numel(t) >= 2
            dt_tail = t(end) - t(end-1);
        end
        time = [t, t(end) + (1:(T-numel(t)))*max(dt_tail, eps)];
        if isempty(t), time = 0:(T-1); end
    end
end
time = time(:)'; % row vector length T

% --- Split base vs arm
qBase = qTraj(idxBase, :);   % [3 x T]
qArm  = qTraj(idxArm , :);   % [nArm x T]

% --- Write arm joint commands table
varNames = ["time", string(names(idxArm))];
% Ensure valid & unique column names
varNames = string(matlab.lang.makeValidName(varNames));
varNames = string(matlab.lang.makeUniqueStrings(varNames));

table_arm = array2table([time(:), qArm'], 'VariableNames', varNames);
writetable(table_arm, fullfile(runDir, "commands_arm_" + tag + ".csv"));

% --- Chassis commands (prefer logged cmdLog if present & meaningful)
useCmdLog = isfield(log, "cmdLog") && ~isempty(log.cmdLog) && ...
            isfield(log.cmdLog, "time") && isfield(log.cmdLog, "Vx") && isfield(log.cmdLog, "Wz") && ...
            ~(all(isnan(log.cmdLog.Vx)) || all(isnan(log.cmdLog.Wz)));

if useCmdLog
    % Reshape to columns, guard sizes
    tCmd = log.cmdLog.time(:);
    vx   = log.cmdLog.Vx(:);
    wz   = log.cmdLog.Wz(:);
    cmdTable = table(tCmd, vx, wz, 'VariableNames', ["time","Vx","Wz"]);
else
    % Derive from base pose deltas
    dt = diff(time);                     % length T-1
    if isempty(dt)
        dt = 1;                          % single-sample fallback
    end
    dt = max(dt, eps);                   % avoid divide-by-zero
    vx = [diff(qBase(1,:))./dt, 0];      % length T
    wz = [diff(qBase(3,:))./dt, 0];      % length T
    cmdTable = table(time(:), vx(:), wz(:), 'VariableNames', ["time","Vx","Wz"]);
end

writetable(cmdTable, fullfile(runDir, "commands_chassis_" + tag + ".csv"));
end
