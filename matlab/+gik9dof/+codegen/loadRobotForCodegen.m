function robot = loadRobotForCodegen()
%LOADROBOTFORCODEGEN Build rigidBodyTree for C++ code generation.
%#codegen
persistent robotPersist
if isempty(robotPersist)
    robotPersist = gik9dof.codegen.buildRobotForCodegen();
end
robot = robotPersist;
end
