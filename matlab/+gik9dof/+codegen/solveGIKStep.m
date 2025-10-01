function qNext = solveGIKStep(qCurrent, targetPose, distanceLower, distanceWeight)
%SOLVEGIKSTEP Single generalized IK step for code generation.
%#codegen
persistent solver poseConstraint jointConstraint distanceConstraint robot
if isempty(solver)
    robot = gik9dof.codegen.loadRobotForCodegen();
    constraintInputs = {'pose','joint','distance'};
    solver = generalizedInverseKinematics('RigidBodyTree', robot, ...
        'ConstraintInputs', constraintInputs, ...
        'SolverAlgorithm', 'LevenbergMarquardt');
    poseConstraint = constraintPoseTarget('left_gripper_link');
    jointConstraint = constraintJointBounds(robot);
    distanceConstraint = constraintDistanceBounds('left_gripper_link');
    distanceConstraint.ReferenceBody = robot.BaseName;
end
poseConstraint.TargetTransform = targetPose;
if distanceLower <= 0
    lowerBound = 0;
else
    lowerBound = distanceLower;
end
if distanceWeight < 0
    w = 0;
else
    w = distanceWeight;
end
distanceConstraint.Bounds = [lowerBound, Inf];
distanceConstraint.Weights = w;
[qNext, ~] = solver(qCurrent, poseConstraint, jointConstraint, distanceConstraint);
end
