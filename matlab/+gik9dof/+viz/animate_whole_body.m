function animate_whole_body(robot, armJointNames, armTrajectory, armTimes, basePose, baseTimes, eePoses, options)
%ANIMATE_WHOLE_BODY Visualize arm + chassis motion over time.
%   animate_whole_body(robot, armJointNames, armTrajectory, armTimes,
%                      basePose, baseTimes, eePoses) animates the robot arm
%   and chassis trajectory using Robotics System Toolbox visualization.
%
%   Inputs:
%     robot          - rigidBodyTree of the arm mounted on the chassis.
%     armJointNames  - cell array of arm joint names (same order as columns
%                      in armTrajectory).
%     armTrajectory  - N-by-M matrix of arm joint positions.
%     armTimes       - 1-by-N vector (or N-by-1) of timestamps for armTrajectory.
%     basePose       - K-by-3 matrix [x y yaw] for chassis pose history.
%     baseTimes      - 1-by-K vector timestamps matching basePose.
%     eePoses        - Optional N-by-3 or N-by-6 end-effector poses (for path).
%     options        - struct with optional fields:
%                       .ArrowLength (default 0.25 m)
%                       .PlaybackSpeed (default 1.0, 1 = real-time)
%
%   The function interpolates the chassis pose onto the arm timestamps so the
%   animation stays synchronized. End-effector waypoints (if provided) are
%   displayed as a reference path.
%
%   Requires MATLAB R2022a+ with Robotics System Toolbox.
%
%   This function is derived from the original helper used in the
%   WBC_camera project (helpers/animate_whole_body.m, commit d6cc2f5),
%   copied here to avoid cross-repository dependencies.

arguments
    robot (1,1) rigidBodyTree
    armJointNames (1,:) cell
    armTrajectory double
    armTimes double
    basePose double
    baseTimes double
    eePoses double = []
    options.ArrowLength double = 0.25
    options.PlaybackSpeed double = 1.0
    options.VideoFile string = ""
    options.VideoFrameRate double = 30
    options.EndEffectorName string = "left_gripper_link"
    options.VisualAlpha double = 0.6
    options.HideEndEffectorVisual logical = false
    options.ChassisMesh string = ""
    options.ChassisColor (1,3) double = [0.4 0.4 0.4]
    options.ChassisAlpha double = 0.35
    options.ChassisScale double = 1e-3
    options.StageBreakIndex double = 1
    options.StageLabels (1,:) string = ["Ramp-up", "Tracking"]
    options.StageBoundaries double = []
    options.StageSelection string = "all"
    options.Obstacles = []
    options.TargetPath double = []
    options.ArmMeshDirectory string = ""
    options.ArmMeshColor (1,3) double = [0.78 0.82 0.93]
    options.FigureScale double = 1.0
    options.ReferenceBasePath double = []
    options.ReferenceBaseLabel (1,1) string = "Reference Base (GIK)"
    options.ExecutedBaseLabel (1,1) string = "Executed Base (Log)"
    options.StageBPath double = []
    options.StageBLabel (1,1) string = "Stage B Executed Base"
end

if options.VisualAlpha < 0 || options.VisualAlpha > 1
    error('options.VisualAlpha must be within [0, 1].');
end
if options.ChassisScale <= 0
    error('options.ChassisScale must be positive.');
end
stageLabels = string(options.StageLabels);
stageSelection = lower(string(options.StageSelection));
stageBreak = max(1, round(options.StageBreakIndex));

% Work on a copy so visual tweaks don't leak back to caller
robot = copy(robot);
robot = ensureArmVisuals(robot, armJointNames, options.ArmMeshDirectory, options.ArmMeshColor);
eeName = char(options.EndEffectorName);
if options.HideEndEffectorVisual
    try
        eeBody = copy(getBody(robot, eeName));
        clearVisual(eeBody);
        replaceBody(robot, eeName, eeBody);
    catch
        warning('animate_whole_body:FailedToHideEE', ...
            'Unable to hide end-effector visual for body %s.', eeName);
    end
end

% Validate sizes

numSteps = size(armTrajectory, 1);
if numSteps ~= numel(armTimes)
    error('armTrajectory rows (%d) must match numel(armTimes) (%d).', numSteps, numel(armTimes));
end
armTimes = armTimes(:)';
baseTimes = baseTimes(:)';
stageBreak = min(stageBreak, numSteps + 1);

if isempty(options.StageBoundaries)
    defaultBoundaries = [stageBreak - 1, numSteps];
else
    defaultBoundaries = options.StageBoundaries(:)';
end
if isempty(defaultBoundaries)
    defaultBoundaries = numSteps;
end
stageBoundaries = round(defaultBoundaries);
stageBoundaries(stageBoundaries < 1) = 1;
stageBoundaries(stageBoundaries > numSteps) = numSteps;
stageBoundaries = unique(stageBoundaries);
if stageBoundaries(end) ~= numSteps
    stageBoundaries = [stageBoundaries, numSteps];
end
numStages = numel(stageBoundaries);
if numel(stageLabels) < numStages
    stageLabels(end+1:numStages) = stageLabels(end);
elseif numel(stageLabels) > numStages
    stageLabels = stageLabels(1:numStages);
end
stageStarts = [1, stageBoundaries(1:end-1) + 1];
stageRanges = arrayfun(@(s,e) s:e, stageStarts, stageBoundaries, 'UniformOutput', false);

switch stageSelection
    case {'all','*'}
        frameIdx = 1:numSteps;
        stageBoundariesActive = stageBoundaries;
        stageLabelsActive = stageLabels;
    case {'arm','arm_ramp','arm-ramp'}
        frameIdx = stageRanges{1};
        stageBoundariesActive = stageBoundaries(1);
        stageLabelsActive = stageLabels(1);
    case {'chassis','chassis_ramp','base_ramp'}
        if numStages < 2
            error('animate_whole_body:StageSelection', 'Chassis ramp stage not available.');
        end
        stageIdx = min(2, numStages);
        frameIdx = stageRanges{stageIdx};
        stageBoundariesActive = stageBoundaries(stageIdx) - stageBoundaries(stageIdx-1);
        stageLabelsActive = stageLabels(stageIdx);
    case {'tracking','track'}
        frameIdx = stageRanges{end};
        if numStages >= 2
            prevEnd = stageBoundaries(end-1);
        else
            prevEnd = 0;
        end
        stageBoundariesActive = stageBoundaries(end) - prevEnd;
        stageLabelsActive = stageLabels(end);
    otherwise
        error('animate_whole_body:StageSelection', 'Unknown StageSelection option: %s.', stageSelection);
end

frameIdx = frameIdx(:)';
armTrajectory = armTrajectory(frameIdx, :);
armTimes = armTimes(frameIdx);
armTimes = armTimes - armTimes(1);
numSteps = numel(frameIdx);

stageBoundariesActive = round(stageBoundariesActive);
if isempty(stageBoundariesActive)
    stageBoundariesActive = numSteps;
end
stageBoundariesActive(stageBoundariesActive < 1) = 1;
stageBoundariesActive(stageBoundariesActive > numSteps) = numSteps;
if stageBoundariesActive(end) ~= numSteps
    stageBoundariesActive(end+1) = numSteps;
end
stageLabelsActive = string(stageLabelsActive);
if numel(stageLabelsActive) < numel(stageBoundariesActive)
    stageLabelsActive(end+1:numel(stageBoundariesActive)) = stageLabelsActive(end);
elseif numel(stageLabelsActive) > numel(stageBoundariesActive)
    stageLabelsActive = stageLabelsActive(1:numel(stageBoundariesActive));
end

if size(basePose,2) ~= 3
    error('basePose must be K-by-3 with columns [x y yaw].');
end
if numel(baseTimes) ~= size(basePose,1)
    error('baseTimes length must equal number of basePose rows.');
end

% Interpolate base pose onto arm timeline
baseX = interp1(baseTimes, basePose(:,1), armTimes, 'linear', 'extrap');
baseY = interp1(baseTimes, basePose(:,2), armTimes, 'linear', 'extrap');
baseYawUnwrapped = interp1(baseTimes, unwrap(basePose(:,3)), armTimes, 'linear', 'extrap');
baseYaw = wrapToPi(baseYawUnwrapped);

% Prepare figure: left = 3D perspective, right = top view
scale = max(options.FigureScale, 0.1);
baseSize = [1120 840];
canvasSize = [baseSize(1) * 2, baseSize(2)];
figSize = round(canvasSize * scale);
fig = figure('Name', 'Whole-Body Animation', 'Color', [0.12 0.12 0.12], ...
    'Units', 'pixels', 'Position', [100 100 figSize], 'Resize', 'off');

axPersp = subplot(1,2,1, 'Parent', fig);
axTop   = subplot(1,2,2, 'Parent', fig);
set(axPersp, 'Position', [0.05 0.1 0.42 0.82]);
set(axTop,   'Position', [0.55 0.1 0.42 0.82]);

setupAxes(axPersp, [0.05 0.05 0.05], false);
setupAxes(axTop,   [0.05 0.05 0.05], true);

view(axPersp, 45, 25);
view(axTop,   0, 90);
title(axPersp, '3D Perspective', 'Color', [0.95 0.95 0.95]);
title(axTop,   'Top View', 'Color', [0.95 0.95 0.95]);

% Plot base paths
executedColor = [0.10 0.45 0.90];
referenceColor = [0.85 0.70 0.10];
stageBColor = [0.60 0.60 0.60];

plot(axPersp, basePose(:,1), basePose(:,2), '-', 'Color', executedColor, 'LineWidth', 1.5, 'DisplayName', char(options.ExecutedBaseLabel));
plot(axTop,   basePose(:,1), basePose(:,2), '-', 'Color', executedColor, 'LineWidth', 1.5, 'DisplayName', char(options.ExecutedBaseLabel));

if ~isempty(options.ReferenceBasePath)
    refPath = options.ReferenceBasePath;
    plot(axPersp, refPath(:,1), refPath(:,2), '--', 'Color', referenceColor, 'LineWidth', 1.2, 'DisplayName', char(options.ReferenceBaseLabel));
    plot(axTop,   refPath(:,1), refPath(:,2), '--', 'Color', referenceColor, 'LineWidth', 1.2, 'DisplayName', char(options.ReferenceBaseLabel));
end

if ~isempty(options.StageBPath)
    sbPath = options.StageBPath;
    plot(axPersp, sbPath(:,1), sbPath(:,2), ':', 'Color', stageBColor, 'LineWidth', 1.2, 'DisplayName', char(options.StageBLabel));
    plot(axTop,   sbPath(:,1), sbPath(:,2), ':', 'Color', stageBColor, 'LineWidth', 1.2, 'DisplayName', char(options.StageBLabel));
end

if ~isempty(eePoses) && size(eePoses,2) >= 3
    plot3(axPersp, eePoses(:,1), eePoses(:,2), eePoses(:,3), 'r-.', 'LineWidth', 1.0, 'DisplayName', 'Desired EE path');
    plot(axTop,   eePoses(:,1), eePoses(:,2), 'r-.', 'LineWidth', 1.0, 'DisplayName', 'Desired EE path');
end
if ~isempty(options.TargetPath)
    tp = options.TargetPath;
    plot(axPersp, tp(:,1), tp(:,2), 'm:', 'LineWidth', 1.4, 'DisplayName', 'Planned chassis path');
    plot(axTop,   tp(:,1), tp(:,2), 'm:', 'LineWidth', 1.4, 'DisplayName', 'Planned chassis path');
end
actualEE = nan(numSteps,3);
actualLinePersp = plot3(axPersp, nan, nan, nan, 'Color', [0 0.7 0.2], 'LineWidth', 1.5, 'DisplayName', 'Actual EE path');
actualLineTop   = plot(axTop, nan, nan, 'Color', [0 0.7 0.2], 'LineWidth', 1.5, 'DisplayName', 'Actual EE path');
leg = legend(axPersp, 'Location', 'bestoutside');
set(leg, 'TextColor', [0.95 0.95 0.95], 'Color', [0.2 0.2 0.2]);

% Markers for current pose
baseMarkerPersp = plot3(axPersp, baseX(1), baseY(1), 0, 'o', 'Color', executedColor, 'MarkerFaceColor', executedColor);
baseMarkerTop   = plot(axTop,   baseX(1), baseY(1), 'o', 'Color', executedColor, 'MarkerFaceColor', executedColor);
headingLen = options.ArrowLength;
headingLinePersp = plot3(axPersp, [baseX(1), baseX(1)+headingLen*cos(baseYaw(1))], ...
                      [baseY(1), baseY(1)+headingLen*sin(baseYaw(1))], ...
                      [0 0], '-', 'Color', executedColor, 'LineWidth', 2);
headingLineTop = plot(axTop, [baseX(1), baseX(1)+headingLen*cos(baseYaw(1))], ...
                         [baseY(1), baseY(1)+headingLen*sin(baseYaw(1))], ...
                         '-', 'Color', executedColor, 'LineWidth', 2);
if ~isempty(eePoses) && size(eePoses,2) >= 3
    eeMarkerPersp = plot3(axPersp, eePoses(1,1), eePoses(1,2), eePoses(1,3), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'Desired EE waypoint');
    eeMarkerTop   = plot(axTop,   eePoses(1,1), eePoses(1,2), 'ro', 'MarkerFaceColor', 'r');
else
    eeMarkerPersp = [];
    eeMarkerTop   = [];
end
actualMarkerPersp = plot3(axPersp, baseX(1), baseY(1), 0, 's', 'Color', [0.0 0.6 0.2], 'MarkerFaceColor', [0.0 0.6 0.2], 'DisplayName', 'Actual EE');
actualMarkerTop   = plot(axTop,   baseX(1), baseY(1), 's', 'Color', [0.0 0.6 0.2], 'MarkerFaceColor', [0.0 0.6 0.2]);
stageText = text(axPersp, 'Units', 'normalized', 'Position', [0.02 0.95 0], ...
    'String', '', 'Color', [0.95 0.95 0.95], 'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.1 0.1 0.1], 'Margin', 4, 'HorizontalAlignment', 'left');
eeErrorText = text(axPersp, 'Units', 'normalized', 'Position', [0.72 0.05 0], ...
    'String', 'EE pos err: n/a', 'Color', [0.95 0.95 0.95], 'FontSize', 10, ...
    'BackgroundColor', [0.1 0.1 0.1], 'Margin', 4, 'HorizontalAlignment', 'right');

% Map joint names
configTemplate = homeConfiguration(robot);
config = configTemplate;
vectorNames = {config.JointName};
armIdx = zeros(1, numel(armJointNames));
for j = 1:numel(armJointNames)
    idx = find(strcmp(vectorNames, armJointNames{j}), 1);
    if isempty(idx)
        error('Joint %s not present in robot model.', armJointNames{j});
    end
    armIdx(j) = idx;
end

% Animation loop
hgPersp = hgtransform('Parent', axPersp);
hgTop = hgtransform('Parent', axTop);
chassisPatchPersp = [];
chassisPatchTop = [];
robotHandlesPrevPersp = [];
robotHandlesPrevTop = [];

helpersDir = fileparts(mfilename('fullpath'));
matlabDir = fileparts(helpersDir);
meshRoot = fullfile(matlabDir, '..', '..', 'meshes');
meshOutputs = fullfile(meshRoot, 'outputs');

% Attempt to load chassis/support mesh once so it follows the base hgtransform
meshPath = string(options.ChassisMesh);
if strlength(meshPath) == 0
    chassisCandidates = {
        fullfile(meshOutputs, 'base_link_reduced.STL');
        fullfile(meshOutputs, 'base_link_reduced.stl');
        fullfile(meshRoot, 'base_link.STL')
        };
    meshPath = "";
    for idxCand = 1:numel(chassisCandidates)
        cand = chassisCandidates{idxCand};
        if exist(cand, 'file')
            meshPath = string(cand);
            break
        end
    end
    if strlength(meshPath) == 0
        meshPath = string(chassisCandidates{end});
    end
end
if exist(meshPath, 'file') == 2
    try
        triData = stlread(meshPath);
        chassisVertices = triData.Points * options.ChassisScale;
        chassisPatchPersp = patch('Parent', hgPersp, 'Faces', triData.ConnectivityList, ...
            'Vertices', chassisVertices, 'FaceColor', options.ChassisColor, ...
            'FaceAlpha', options.ChassisAlpha, 'EdgeColor', 'none', ...
            'DisplayName', 'Chassis model');
        chassisPatchTop = patch('Parent', hgTop, 'Faces', triData.ConnectivityList, ...
            'Vertices', chassisVertices, 'FaceColor', options.ChassisColor, ...
            'FaceAlpha', options.ChassisAlpha, 'EdgeColor', 'none', ...
            'HandleVisibility', 'off');
    catch ME
        warning('animate_whole_body:ChassisMeshFailed', ...
            'Failed to load chassis mesh from %s (%s).', meshPath, ME.message);
        chassisPatchPersp = [];
        chassisPatchTop = [];
    end
else
    warning('animate_whole_body:ChassisMeshMissing', ...
        'Chassis mesh file %s not found. Skipping static chassis visual.', meshPath);
end

videoWriter = [];
if strlength(options.VideoFile) > 0
    try
        videoWriter = VideoWriter(char(options.VideoFile), 'MPEG-4');
        videoWriter.FrameRate = options.VideoFrameRate;
        open(videoWriter);
    catch ME
        warning('Failed to create video writer (%s). Continuing without video.', ME.message);
        videoWriter = [];
    end
end

% Draw obstacles as translucent primitives
obstacles = normalizeObstacles(options.Obstacles);
drawObstacles(axPersp, obstacles, false);
drawObstacles(axTop, obstacles, true);

for k = 1:numSteps
    if ~isempty(robotHandlesPrevPersp)
        delete(robotHandlesPrevPersp(ishghandle(robotHandlesPrevPersp)));
        robotHandlesPrevPersp = [];
    end
    if ~isempty(robotHandlesPrevTop)
        delete(robotHandlesPrevTop(ishghandle(robotHandlesPrevTop)));
        robotHandlesPrevTop = [];
    end

    for j = 1:numel(armIdx)
        config(armIdx(j)).JointPosition = armTrajectory(k, j);
    end
    % Draw robot (positions shown relative to chassis frame with base pose)
    Tbase = trvec2tform([baseX(k), baseY(k), 0]) * axang2tform([0 0 1 baseYaw(k)]);
    show(robot, config, 'Parent', axPersp, 'PreservePlot', false, ...
        'Frames', 'off', 'Visuals', 'on', 'FastUpdate', false);

    robotHandlesPersp = findall(axPersp, '-regexp', 'Tag', '^DO_NOT_EDIT');
    for h = reshape(robotHandlesPersp,1,[])
        if ~ishghandle(h)
            continue;
        end
        if get(h, 'Parent') ~= hgPersp
            set(h, 'Parent', hgPersp);
        end
        if isprop(h, 'FaceAlpha')
            set(h, 'FaceAlpha', options.VisualAlpha);
        end
        if isprop(h, 'EdgeAlpha')
            set(h, 'EdgeAlpha', options.VisualAlpha);
        end
    end
    robotHandlesPrevPersp = robotHandlesPersp;

    show(robot, config, 'Parent', axTop, 'PreservePlot', false, ...
        'Frames', 'off', 'Visuals', 'on', 'FastUpdate', false);

    robotHandlesTop = findall(axTop, '-regexp', 'Tag', '^DO_NOT_EDIT');
    for h = reshape(robotHandlesTop,1,[])
        if ~ishghandle(h)
            continue;
        end
        if get(h, 'Parent') ~= hgTop
            set(h, 'Parent', hgTop);
        end
        if isprop(h, 'FaceAlpha')
            set(h, 'FaceAlpha', options.VisualAlpha);
        end
        if isprop(h, 'EdgeAlpha')
            set(h, 'EdgeAlpha', options.VisualAlpha);
        end
        if isgraphics(h)
            set(h, 'HandleVisibility', 'off');
        end
    end
    robotHandlesPrevTop = robotHandlesTop;

    set(hgPersp, 'Matrix', Tbase);
    set(hgTop, 'Matrix', Tbase);

    % Update markers
    updateMarkers(k);

    if ~isempty(eeMarkerPersp)
        idx = min(k, size(eePoses,1));
        set(eeMarkerPersp, 'XData', eePoses(idx,1), 'YData', eePoses(idx,2), 'ZData', eePoses(idx,3));
        set(eeMarkerTop,   'XData', eePoses(idx,1), 'YData', eePoses(idx,2));
    end

    try
        Tee = getTransform(robot, config, eeName);
    catch
        Tee = eye(4);
    end
    TeeWorld = Tbase * Tee;
    actualEE(k, :) = TeeWorld(1:3,4)';
    set(actualMarkerPersp, 'XData', actualEE(k,1), 'YData', actualEE(k,2), 'ZData', actualEE(k,3));
    set(actualMarkerTop,   'XData', actualEE(k,1), 'YData', actualEE(k,2));
    set(actualLinePersp, 'XData', actualEE(1:k,1), 'YData', actualEE(1:k,2), 'ZData', actualEE(1:k,3));
    set(actualLineTop,   'XData', actualEE(1:k,1), 'YData', actualEE(1:k,2));

    stageIdx = find(k <= stageBoundariesActive, 1, 'first');
    if isempty(stageIdx)
        stageIdx = numel(stageBoundariesActive);
    end
    if ~isempty(eePoses) && size(eePoses,2) >= 3
        idx = min(k, size(eePoses,1));
        desiredEE = eePoses(idx,1:3);
        errPos = norm(actualEE(k,:) - desiredEE);
    else
        errPos = NaN;
    end
    if isgraphics(eeErrorText)
        if isnan(errPos)
            eeErrorText.String = 'EE pos err: n/a';
        else
            eeErrorText.String = sprintf('EE pos err: %.3f m', errPos);
        end
    end

    stageText.String = sprintf('%s (frame %d/%d)', stageLabelsActive(stageIdx), k, numSteps);

    drawnow limitrate;
    if ~isempty(videoWriter)
        frame = getframe(fig);
        writeVideo(videoWriter, frame);
    end
    if k < numSteps
        dt = max(armTimes(k+1) - armTimes(k), 0);
        pause(dt / max(options.PlaybackSpeed, eps));
    end
end

if ~isempty(videoWriter)
    close(videoWriter);
end

    function updateMarkers(idx)
        set(baseMarkerPersp, 'XData', baseX(idx), 'YData', baseY(idx), 'ZData', 0);
        set(baseMarkerTop,   'XData', baseX(idx), 'YData', baseY(idx));
        set(headingLinePersp, 'XData', [baseX(idx), baseX(idx)+headingLen*cos(baseYaw(idx))], ...
                               'YData', [baseY(idx), baseY(idx)+headingLen*sin(baseYaw(idx))], ...
                               'ZData', [0 0]);
        set(headingLineTop, 'XData', [baseX(idx), baseX(idx)+headingLen*cos(baseYaw(idx))], ...
                             'YData', [baseY(idx), baseY(idx)+headingLen*sin(baseYaw(idx))]);
    end

    function drawObstacles(ax, obstaclesLocal, isTop)
        if nargin < 3
            isTop = false;
        end
        if isempty(obstaclesLocal)
            return
        end
        hold(ax, 'on');
        haveLegendActual = false;
        haveLegendInflated = false;
        for obsIdx = 1:numel(obstaclesLocal)
            obs = obstaclesLocal(obsIdx);
            color = obs.color;
            if numel(color) < 3
                color = [1.0 0.2 0.2];
            else
                color = color(1:3);
            end
            switch lower(obs.type)
                case {'circle','disc','disk'}
                    actualR = obs.radius;
                    inflatedR = obs.radiusInflated;
                    safetyR = obs.radiusSafety;
                    if inflatedR > actualR + 1e-6
                        [Xi, Yi, Zi] = cylinder(inflatedR, 80);
                        Zi = Zi * obs.height;
                        safetyColor = lightenColor(color, 0.4);
                        surfInflated = surf(ax, Xi + obs.center(1), Yi + obs.center(2), Zi, ...
                            'FaceAlpha', 0.1, 'FaceColor', safetyColor, 'EdgeColor', safetyColor, ...
                            'LineStyle', '--');
                        if ~haveLegendInflated
                            set(surfInflated, 'DisplayName', 'Safety zone');
                            haveLegendInflated = true;
                        else
                            set(surfInflated, 'HandleVisibility', 'off');
                        end
                    end
                    if safetyR > actualR + 1e-6 && safetyR < inflatedR - 1e-6
                        [Xs, Ys, Zs] = cylinder(safetyR, 80);
                        Zs = Zs * obs.height;
                        surf(ax, Xs + obs.center(1), Ys + obs.center(2), Zs, ...
                            'FaceAlpha', 0.15, 'FaceColor', lightenColor(color, 0.2), 'EdgeColor', 'none', ...
                            'HandleVisibility', 'off');
                    end
                    [Xa, Ya, Za] = cylinder(actualR, 60);
                    Za = Za * obs.height;
                    surfActual = surf(ax, Xa + obs.center(1), Ya + obs.center(2), Za, ...
                        'FaceAlpha', 0.35, 'FaceColor', color, 'EdgeColor', 'none');
                    if strlength(obs.label) > 0 && ~haveLegendActual
                        set(surfActual, 'DisplayName', char(obs.label));
                        haveLegendActual = true;
                    elseif ~haveLegendActual
                        set(surfActual, 'DisplayName', 'Obstacle');
                        haveLegendActual = true;
                    else
                        set(surfActual, 'HandleVisibility', 'off');
                    end
                    if isTop
                        theta = linspace(0, 2*pi, 240);
                        circleX = obs.center(1) + actualR * cos(theta);
                        circleY = obs.center(2) + actualR * sin(theta);
                        circleLine = plot3(ax, circleX, circleY, zeros(size(circleX)), '-', ...
                            'Color', color, 'LineWidth', 1.4);
                        set(circleLine, 'HandleVisibility', 'off');
                    end
                case {'rectangle','box','aabb'}
                    bounds = obs.bounds;
                    xRect = [bounds(1) bounds(2) bounds(2) bounds(1) bounds(1)];
                    yRect = [bounds(3) bounds(3) bounds(4) bounds(4) bounds(3)];
                    surfHandle = fill3(ax, xRect, yRect, obs.height * ones(size(xRect)), color, ...
                        'FaceAlpha', 0.25, 'EdgeColor', 'none');
                    if strlength(obs.label) > 0 && ~haveLegendActual
                        set(surfHandle, 'DisplayName', char(obs.label));
                        haveLegendActual = true;
                    elseif ~haveLegendActual
                        set(surfHandle, 'DisplayName', 'Obstacle');
                        haveLegendActual = true;
                    else
                        set(surfHandle, 'HandleVisibility', 'off');
                    end
                otherwise
                    % ignore unknown types
            end
        end
    end

    function setupAxes(ax, color, isTop)
        if nargin < 3
            isTop = false;
        end
        set(ax, 'Color', color, 'XColor', [0.9 0.9 0.9], 'YColor', [0.9 0.9 0.9], ...
            'ZColor', [0.9 0.9 0.9], 'GridColor', [0.35 0.35 0.35]);
        hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
        xlabel(ax, 'X (m)', 'Color', [0.9 0.9 0.9]);
        ylabel(ax, 'Y (m)', 'Color', [0.9 0.9 0.9]);
        if ~isTop
            zlabel(ax, 'Z (m)', 'Color', [0.9 0.9 0.9]);
        else
            zlabel(ax, '');
            set(ax, 'Projection', 'orthographic');
        end
    end
end

function robot = ensureArmVisuals(robot, armJointNames, meshDirOption, meshColor)
% Ensure the rigidBodyTree carries mesh visuals for arm links so show()
% renders geometry even when the imported URDF omits them.

if nargin < 3
    meshDirOption = "";
end
if nargin < 4
    meshColor = [0.78 0.82 0.93];
end

meshDir = string(meshDirOption);
if strlength(meshDir) == 0
    helpersDir = fileparts(mfilename('fullpath'));
    matlabDir = fileparts(helpersDir);
    meshDir = string(fullfile(matlabDir, '..', '..', 'meshes'));
end

if strlength(meshDir) == 0 || ~isfolder(meshDir)
    return
end

meshDirChar = char(meshDir);
meshColor = double(reshape(meshColor, 1, []));

% Build list of body names associated with animated joints.
bodyNames = cell(size(armJointNames));
for k = 1:numel(armJointNames)
    bodyNames{k} = findBodyForJoint(robot, armJointNames{k});
end
bodyNames = unique([bodyNames(~cellfun(@isempty, bodyNames)), {'left_arm_base_link','left_gripper_link'}]);

for idx = 1:numel(bodyNames)
    bodyName = bodyNames{idx};
    if isempty(bodyName)
        continue
    end
    try
        body = copy(getBody(robot, bodyName));
    catch
        continue
    end
    if ~isempty(body.Visuals)
        continue
    end

outputsDir = fullfile(meshDirChar, 'stl_output');
    candidates = {};
    if exist(outputsDir, 'dir')
        candidates = [candidates, ...
            {fullfile(outputsDir, [bodyName, '_reduced.STL'])}, ...
            {fullfile(outputsDir, [bodyName, '_reduced.stl'])}, ...
            {fullfile(outputsDir, [bodyName, '.STL'])}, ...
            {fullfile(outputsDir, [bodyName, '.stl'])}]; %#ok<AGROW>
    end
    candidates = [candidates, ...
        {fullfile(meshDirChar, [bodyName, '.STL'])}, ...
        {fullfile(meshDirChar, [bodyName, '.stl'])}]; %#ok<AGROW>
    meshPath = '';
    for cIdx = 1:numel(candidates)
        if exist(candidates{cIdx}, 'file') == 2
            meshPath = candidates{cIdx};
            break
        end
    end
    if isempty(meshPath)
        continue
    end

    addVisual(body, 'Mesh', meshPath);
    try
        % Apply requested color when possible.
        visuals = body.Visuals;
        for vIdx = 1:numel(visuals)
            visualEntry = visuals{vIdx};
            if isstruct(visualEntry) && isfield(visualEntry, 'Color')
                visualEntry.Color = meshColor;
                visuals{vIdx} = visualEntry;
            end
        end
        body.Visuals = visuals;
    catch
        % Swallow coloring issues; geometry is more important.
    end

    replaceBody(robot, bodyName, body);
end
end

function bodyName = findBodyForJoint(robot, jointName)
bodyName = '';
if ~iscell(jointName)
    jointName = {jointName};
end
for idx = 1:numel(robot.Bodies)
    joint = robot.Bodies{idx}.Joint;
    if any(strcmp(jointName, joint.Name))
        bodyName = robot.Bodies{idx}.Name;
        return
    end
end
end

function obstacles = normalizeObstacles(obstacleInput)
%NORMALIZEOBSTACLES Convert mixed obstacle formats into a common struct.

base = struct('type', "", 'center', [0 0], 'radius', 0, 'radiusSafety', 0, ...
    'radiusInflated', 0, 'radiusActual', 0, 'safetyMargin', 0, 'distanceMargin', 0, ...
    'height', 0.05, 'bounds', [0 0 0 0], 'color', [1.0 0.2 0.2], 'label', "");
cells = flattenObstacles(obstacleInput);
obstacles = repmat(base, 0, 1);

for idx = 1:numel(cells)
    obs = cells{idx};
    if ~isstruct(obs)
        continue
    end

    % Disc-style definitions (Center/Radius or center/radius)
    if hasFields(obs, {'Center','Radius'}) || hasFields(obs, {'center','radius'})
        if isfield(obs, 'Center')
            center = obs.Center;
        else
            center = obs.center;
        end
        if isfield(obs, 'Radius')
            radius = obs.Radius;
        else
            radius = obs.radius;
        end
        margin = 0;
        if isfield(obs, 'SafetyMargin') && ~isempty(obs.SafetyMargin)
            margin = obs.SafetyMargin;
        elseif isfield(obs, 'safetyMargin') && ~isempty(obs.safetyMargin)
            margin = obs.safetyMargin;
        end
        distMargin = pickFirstField(obs, {'DistanceMargin','distanceMargin'}, 0);
        height = pickFirstField(obs, {'Height','height','Thickness','thickness'}, 0.05);
        color = pickFirstField(obs, {'Color','color'}, base.color);
        label = pickFirstField(obs, {'Label','label','Name','name'}, "");

        entry = base;
        entry.type = "circle";
        center = double(center(:)');
        if numel(center) < 2
            center = [center, 0];
        end
        entry.center = center(1:2);
        rVal = double(radius);
        if numel(rVal) < 1
            continue
        end
        mVal = double(margin);
        if isempty(mVal)
            mVal = 0;
        end
        actualRadius = rVal(1);
        safetyRadius = actualRadius + mVal(1);
        distVal = double(distMargin);
        if isempty(distVal)
            distVal = 0;
        end
        inflatedRadius = safetyRadius + distVal(1);
        entry.radius = actualRadius;
        entry.radiusActual = actualRadius;
        entry.radiusSafety = safetyRadius;
        entry.radiusInflated = inflatedRadius;
        entry.safetyMargin = mVal(1);
        entry.distanceMargin = distVal(1);
        hVal = double(height);
        if numel(hVal) < 1
            hVal = 0.05;
        end
        entry.height = max(hVal(1), 0.01);
        entry.color = sanitizeColor(color, base.color);
        if isstring(label) || ischar(label)
            entry.label = string(label);
        else
            entry.label = "";
        end
        obstacles(end+1) = entry; %#ok<AGROW>
        continue
    end

    % Axis-aligned rectangles (bounds field)
    if hasFields(obs, {'bounds'})
        bounds = obs.bounds;
        if numel(bounds) == 4
            entry = base;
            entry.type = "rectangle";
            entry.bounds = double(bounds(:)).';
            hVal = pickFirstField(obs, {'Height','height'}, 0.05);
            hVal = double(hVal);
            if numel(hVal) < 1
                hVal = 0.05;
            end
            entry.height = max(hVal(1), 0.01);
            color = pickFirstField(obs, {'Color','color'}, base.color);
            entry.color = sanitizeColor(color, base.color);
            label = pickFirstField(obs, {'Label','label','Name','name'}, "");
            if isstring(label) || ischar(label)
                entry.label = string(label);
            end
            obstacles(end+1) = entry; %#ok<AGROW>
        end
    end
end
end

function cells = flattenObstacles(input)
cells = {};
if isempty(input)
    return
end

if iscell(input)
    for idx = 1:numel(input)
        cells = [cells, flattenObstacles(input{idx})]; %#ok<AGROW>
    end
    return
end

if isstruct(input)
    for idx = 1:numel(input)
        entry = input(idx);
        if isfield(entry, 'discs')
            cells = [cells, flattenObstacles(entry.discs)]; %#ok<AGROW>
        else
            cells{end+1} = entry; %#ok<AGROW>
        end
    end
end
end

function tf = hasFields(s, names)
tf = true;
for idx = 1:numel(names)
    name = names{idx};
    if ~(isfield(s, name) && ~isempty(s.(name)))
        tf = false;
        return
    end
end
end

function value = pickFirstField(s, candidates, defaultValue)
value = defaultValue;
for idx = 1:numel(candidates)
    name = candidates{idx};
    if isfield(s, name) && ~isempty(s.(name))
        value = s.(name);
        return
    end
end
end

function color = sanitizeColor(candidate, fallback)
if isnumeric(candidate) && ~isempty(candidate)
    color = double(candidate(:).');
else
    color = double(fallback(:).');
end
if numel(color) >= 3
    color = color(1:3);
else
    color = double(fallback(:).');
    color = color(1:3);
end
end

function out = lightenColor(color, amount)
base = sanitizeColor(color, [1 0 0]);
amount = max(min(amount, 1), 0);
out = base + (1 - base) * amount;
out = max(min(out, 1), 0);
end
