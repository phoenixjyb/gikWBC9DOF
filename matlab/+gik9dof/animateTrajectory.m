function frames = animateTrajectory(logInput, options)
%ANIMATETRAJECTORY Visualize a joint-space trajectory for the 9-DOF robot.
%   gik9dof.animateTrajectory(logInput) replays the joint configurations
%   contained in the provided log (struct or MAT-file) using the project
%   robot model and displays the animation in an off-screen figure.
%
%   frames = gik9dof.animateTrajectory(___) returns the captured movie
%   frames. Supplying an OutputVideo option writes the animation to disk.
%
%   Name-value options:
%       Robot          - Preconstructed rigidBodyTree (defaults to
%                        gik9dof.createRobotModel()).
%       OutputVideo    - Path to create an .mp4 file (optional).
%       FrameRate      - Playback frame rate in Hz (default 30).
%       SampleStep     - Positive integer; sample every N configurations
%                        (default 2).
%       Workspace      - 1x6 axis limits [xmin xmax ymin ymax zmin zmax]. If
%                        empty, limits are auto-computed.
%       ShowFrames     - Logical flag (default false). When true, the figure
%                        is shown instead of using off-screen rendering.
%
%   Example:
%       gik9dof.animateTrajectory('trajectory_log.mat', 'OutputVideo', 'outputs/demo.mp4');
%
%   See also gik9dof.runTrajectoryControl, VideoWriter.

arguments
    logInput
    options.Robot (1,1) rigidBodyTree = gik9dof.createRobotModel()
    options.OutputVideo (1,1) string = ""
    options.FrameRate (1,1) double {mustBePositive} = 30
    options.SampleStep (1,1) double {mustBeInteger, mustBePositive} = 2
    options.Workspace = []
    options.ShowFrames (1,1) logical = false
end

log = resolveLog(logInput);
qTraj = log.qTraj;
if isempty(qTraj)
    error("gik9dof:animateTrajectory:EmptyTrajectory", ...
        "Log does not contain qTraj data.");
end

robot = options.Robot;
configTools = gik9dof.configurationTools(robot);

sampleIdx = 1:options.SampleStep:size(qTraj, 2);
numFrames = numel(sampleIdx);

figVisible = ternary(options.ShowFrames, 'on', 'off');
fig = figure('Name', 'GIK Trajectory Animation', 'NumberTitle', 'off', ...
    'Visible', figVisible, 'Position', [100 100 640 480]);
ax = axes('Parent', fig);
view(ax, 135, 20);
lighting(ax, 'gouraud');
camlight(ax, 'headlight');
hold(ax, 'on');

if ~isempty(options.Workspace)
    validateattributes(options.Workspace, {'double'}, {'vector','numel',6,'finite','real'});
    axis(ax, options.Workspace);
else
    axis(ax, 'equal');
end

drawnow;

frames(1, numFrames) = struct('cdata',[],'colormap',[]); %#ok<AGROW>

for k = 1:numFrames
    idx = sampleIdx(k);
    q = configTools.column(qTraj(:, idx));
    show(robot, q, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
    title(ax, sprintf('Frame %d / %d', k, numFrames));
    drawnow;
    frames(k) = getframe(fig);
end

if strlength(options.OutputVideo) > 0
    outputPath = gik9dof.internal.resolvePath(options.OutputVideo);
    outDir = fileparts(outputPath);
    if outDir ~= "" && ~isfolder(outDir)
        mkdir(outDir);
    end
    writer = VideoWriter(outputPath, 'MPEG-4');
    writer.FrameRate = options.FrameRate;
    open(writer);
    writeVideo(writer, frames);
    close(writer);
end

if ~options.ShowFrames
    close(fig);
end
end

function out = resolveLog(input)
if isstruct(input)
    out = input;
    return
end

if ~(ischar(input) || (isstring(input) && isscalar(input)))
    error("gik9dof:animateTrajectory:InvalidInput", ...
        "Input must be a log struct or path to a MAT file.");
end

path = gik9dof.internal.resolvePath(string(input));
if ~isfile(path)
    error("gik9dof:animateTrajectory:MissingFile", ...
        "Could not find log file: %s", path);
end

loaded = load(path, 'log');
if ~isfield(loaded, 'log')
    error("gik9dof:animateTrajectory:MissingVariable", ...
        "MAT file %s does not contain a variable named 'log'.", path);
end

out = loaded.log;
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end
