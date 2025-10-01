function tools = configurationTools(robot)
%CONFIGURATIONTOOLS Generate helpers for manipulating configurations.
%   tools = GIK9DOF.CONFIGURATIONTOOLS(robot) returns a struct of function
%   handles that streamline configuration bookkeeping for the 9-DOF model.
%   Provide the rigidBodyTree to allow access to body/joint metadata.
%
%   Returned fields:
%       homeConfig()   - Joint configuration column vector of the robot's
%                        home configuration.
%       column(q)      - Normalise arbitrary configuration representations
%                        (struct/row/column) into a column vector.
%       struct(q)      - Convert a column vector back to the rigid
%                        body tree's struct representation.
%       random()       - Draw a random configuration that respects joint
%                        limits.
%
%   Example:
%       robot = gik9dof.createRobotModel();
%       cfg = configTools.homeConfig();
%       cfgRand = configTools.random();
%
%   See also homeConfiguration, randomConfiguration.

arguments
    robot (1,1) rigidBodyTree
end

dataFormat = robot.DataFormat;

if ~strcmp(dataFormat, "column")
    % The GIK solver expects column vectors; enforce that upstream.
    warning("gik9dof:configurationTools:DataFormat", ...
        "Robot data format is '%s'; consider using 'column' for GIK.", dataFormat);
end
% Precompute struct home configuration once for conversions. Temporarily
% switch the data format to access the struct representation.
originalFormat = robot.DataFormat;
robot.DataFormat = "struct";
structTemplate = homeConfiguration(robot);
robot.DataFormat = originalFormat;
templateNames = {structTemplate.JointName};

% Build conversion closures.
    function qcol = toColumn(q)
        if isstruct(q)
            qcol = reshape([q.JointPosition], [], 1);
        elseif iscolumn(q)
            qcol = q;
        elseif isrow(q)
            qcol = q.';
        else
            error("gik9dof:configurationTools:InvalidShape", ...
                "Expected configuration as struct, column, or row vector.");
        end
    end

    function qstruct = toStruct(q)
        qcol = toColumn(q);
        qstruct = structTemplate;
        for i = 1:numel(qstruct)
            qstruct(i).JointPosition = qcol(i);
        end
    end

% Pack outputs.
tools.homeConfig = @() toColumn(structTemplate);
tools.column = @toColumn;
tools.struct = @toStruct;
tools.random = @() toColumn(randomConfiguration(robot));
tools.templateJointNames = @() templateNames;
end
