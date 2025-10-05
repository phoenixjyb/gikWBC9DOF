function tools = collisionTools(robot, options)
%COLLISIONTOOLS Provide utilities for collision mesh management.
%   tools = GIK9DOF.COLLISIONTOOLS(robot) returns helper functions for
%   attaching the project STL meshes to the rigidBodyTree bodies so that
%   generalized inverse kinematics and motion planning can reason about
%   self-collisions.
%
%   The returned struct exposes:
%       apply()    - Attach meshes defined by the default catalog or the
%                    caller supplied mapping.
%       clear()    - Remove any collision geometry that was previously
%                    attached.
%       catalog    - Table of body-to-mesh associations being used.
%
%   Example:
%       robot = gik9dof.createRobotModel();
%       colTools = gik9dof.collisionTools(robot);
%       colTools.apply();
%       isSelfColliding(robot, colTools.catalog.body{1}, colTools.catalog.body{2});
%
%   See also addCollision, removeCollision.

arguments
    robot (1,1) rigidBodyTree
    options.MeshDirectory (1,1) string = "meshes"
    options.Catalog table = defaultCatalog()
end

meshDir = gik9dof.internal.resolvePath(options.MeshDirectory, true);
userCatalog = options.Catalog;

validateCatalog(robot, userCatalog);

poseColumn = "";
if ismember("Pose", userCatalog.Properties.VariableNames)
    poseColumn = "Pose";
elseif ismember("pose", userCatalog.Properties.VariableNames)
    poseColumn = "pose";
end

    function apply()
        clearedBodies = strings(0,1);
        for k = 1:height(userCatalog)
            body = string(userCatalog.body{k});
            meshFile = fullfile(meshDir, userCatalog.mesh{k});
            if ~isfile(meshFile)
                warning("gik9dof:collisionTools:MissingMesh", ...
                    "Mesh '%s' for body '%s' not found.", meshFile, body);
                continue
            end
            try
                rb = robot.getBody(body);
            catch
                warning("gik9dof:collisionTools:MissingBody", ...
                    "Body '%s' not present in robot; skipping collision mesh.", body);
                continue
            end

            if ~any(clearedBodies == body)
                clearCollision(rb);
                clearedBodies(end+1) = body; %#ok<AGROW>
            end

            if poseColumn ~= ""
                poseVal = userCatalog.(poseColumn){k};
            else
                poseVal = [];
            end

            try
                if ~isempty(poseVal)
                    addCollision(rb, "mesh", meshFile, poseVal);
                else
                    addCollision(rb, "mesh", meshFile);
                end
            catch collisionErr
                warning("gik9dof:collisionTools:AttachFailed", ...
                    "Failed to add mesh '%s' to body '%s': %s", meshFile, body, collisionErr.message);
            end
        end
    end

    function clear()
        bodies = unique(string(userCatalog.body));
        for idx = 1:numel(bodies)
            try
                rb = robot.getBody(bodies(idx));
            catch
                continue
            end
            clearCollision(rb);
        end
    end

tools.apply = @apply;
tools.clear = @clear;
tools.catalog = userCatalog;
end

function catalog = defaultCatalog()
%DEFAULTCATALOG Default meshes aligned with the project URDF.
body = [
    "abstract_chassis_link";
    "left_arm_base_link";
    "left_arm_link1";
    "left_arm_link2";
    "left_arm_link3";
    "left_arm_link4";
    "left_arm_link5";
    "left_arm_link6";
    "left_gripper_link"
];

mesh = [
    "base_link.STL";
    "left_arm_base_link.STL";
    "left_arm_link1.STL";
    "left_arm_link2.STL";
    "left_arm_link3.STL";
    "left_arm_link4.STL";
    "left_arm_link5.STL";
    "left_arm_link6.STL";
    "end_effector.STL"
];

catalog = table(body, mesh);
end

function validateCatalog(robot, catalog)
%VALIDATECATALOG Ensure catalog schema is consistent.
if ~istable(catalog)
    error("gik9dof:collisionTools:InvalidCatalog", ...
        "Catalog must be provided as a table.");
end

vars = string(catalog.Properties.VariableNames);
required = ["body","mesh"];
if ~all(ismember(required, vars))
    error("gik9dof:collisionTools:InvalidCatalog", ...
        "Catalog must contain variables 'body' and 'mesh'.");
end

allowed = [required, "Pose", "pose"];
if ~all(ismember(vars, allowed))
    error("gik9dof:collisionTools:UnexpectedColumn", ...
        "Catalog contains unsupported variables: %s", strjoin(setdiff(vars, allowed), ", "));
end

if height(catalog) == 0
    warning("gik9dof:collisionTools:EmptyCatalog", ...
        "Collision catalog is empty; no meshes will be attached.");
end

poseCol = "";
if ismember("Pose", vars)
    poseCol = "Pose";
elseif ismember("pose", vars)
    poseCol = "pose";
end

if poseCol ~= ""
    for k = 1:height(catalog)
        poseVal = catalog.(poseCol){k};
        if isempty(poseVal)
            continue
        end
        if ~(isnumeric(poseVal) && isequal(size(poseVal), [4 4]))
            error("gik9dof:collisionTools:InvalidPose", ...
                "Pose entry for row %d must be a 4x4 numeric transform or empty.", k);
        end
    end
end

% Inform user about bodies that may be missing.
robotBodies = strings(1, numel(robot.Bodies));
for i = 1:numel(robot.Bodies)
    robotBodies(i) = string(robot.Bodies{i}.Name);
end
missing = setdiff(string(catalog.body), robotBodies);
if ~isempty(missing)
    warning("gik9dof:collisionTools:CatalogBodyMissing", ...
        "Catalog references bodies not present in robot: %s", strjoin(missing, ", "));
end
end
