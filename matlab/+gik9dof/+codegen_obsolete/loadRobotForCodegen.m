function robot = loadRobotForCodegen()
%LOADROBOTFORCODEGEN Load rigidBodyTree for C++ code generation.
%#codegen
persistent robotPersist
if isempty(robotPersist)
    data = coder.load('matlab/+gik9dof/+codegen/robotModel.mat');
    robotPersist = data.robot;
end
robot = robotPersist;
end
