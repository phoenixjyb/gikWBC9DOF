function info = plotJsonPath(jsonPath)
%PLOTJSONPATH Visualise end-effector path defined in a JSON trajectory file.
%   plotJsonPath(jsonPath) loads the trajectory stored in JSONPATH (the
%   default is 1_pull_world_scaled.json in the project root), extracts the
%   pose sequence, and renders both a 3-D perspective view and an X-Y plane
%   projection.  The function also prints the number of samples and returns
%   a struct containing the decoded positions and orientations.
%
%   info = plotJsonPath(...) returns a struct with fields:
%       .positions      - N-by-3 matrix of [x y z] waypoints.
%       .orientations   - N-by-4 matrix of quaternion [w x y z] values (if present).
%       .file           - Path to the JSON file that was parsed.
%
%   Example
%   -------
%   >> plotJsonPath()                          % default bundled trajectory
%   >> plotJsonPath("results/my_path.json")    % custom trajectory
%
%   See also jsondecode, plot3.

if nargin < 1 || isempty(jsonPath)
    jsonPath = fullfile(gik9dof.internal.projectRoot(), "1_pull_world_scaled.json");
end

jsonPath = string(jsonPath);
if ~isfile(jsonPath)
    error("plotJsonPath:FileNotFound", "JSON trajectory file not found: %s", jsonPath);
end

raw = jsondecode(fileread(jsonPath));
if ~isfield(raw, "poses") || isempty(raw.poses)
    error("plotJsonPath:InvalidJSON", "JSON file %s does not contain a non-empty 'poses' array.", jsonPath);
end

numSamples = numel(raw.poses);
positions = zeros(numSamples, 3);
orientations = nan(numSamples, 4);

for k = 1:numSamples
    poseK = raw.poses(k);
    if ~isfield(poseK, "position")
        error("plotJsonPath:MissingPosition", "Pose %d has no 'position' field.", k);
    end
    pos = poseK.position(:);
    if numel(pos) ~= 3
        error("plotJsonPath:InvalidPosition", "Pose %d position must have 3 elements.", k);
    end
    positions(k, :) = pos.';

    if isfield(poseK, "orientation")
        quat = poseK.orientation(:);
        if numel(quat) == 4
            % Stored as [x y z w] or [w x y z]?  Try to detect order.
            % If the last element has the largest magnitude assume [x y z w].
            if abs(quat(4)) >= max(abs(quat(1:3)))
                orientations(k, :) = quat([4 1 2 3]).';
            else
                orientations(k, :) = quat.';
            end
        end
    end
end

fprintf("Loaded %d pose samples from %s\n", numSamples, jsonPath);
if all(isnan(orientations(:,1)))
    fprintf("No orientation data detected.\n");
else
    fprintf("Orientation quaternions available for %d samples.\n", ...
        sum(all(~isnan(orientations), 2)));
end

% 3-D view
fig3d = figure('Name', 'EE Path (3-D view)');
plot3(positions(:,1), positions(:,2), positions(:,3), 'k-o', ...
    'LineWidth', 1.2, 'MarkerFaceColor', 'k', 'MarkerSize', 3);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('End-Effector Path (3-D)');
view(45, 30);

% X-Y plane projection
figXY = figure('Name', 'EE Path (X-Y projection)');
plot(positions(:,1), positions(:,2), 'k-o', ...
    'LineWidth', 1.2, 'MarkerFaceColor', 'k', 'MarkerSize', 3);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('End-Effector Path (X-Y)');

info = struct('positions', positions, ...
    'orientations', orientations, ...
    'file', jsonPath);

if nargout == 0
    clear info
end
end
