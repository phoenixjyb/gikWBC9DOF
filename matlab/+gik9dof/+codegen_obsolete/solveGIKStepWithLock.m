function qNext = solveGIKStepWithLock(qCurrent, targetPose, distanceLower, distanceWeight, lockMask)
%SOLVEGIKSTEPWITHLOCK Generalized IK step with optional joint locks.
%#codegen

arguments
    qCurrent (9,1) double
    targetPose (4,4) double
    distanceLower (1,1) double
    distanceWeight (1,1) double
    lockMask (9,1) logical
end

persistent solver poseConstraint jointConstraint distanceConstraint robot jointBoundsDefault
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
    jointBoundsDefault = jointConstraint.Bounds;
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

% Restore default joint bounds then apply lock mask.
jointConstraint.Bounds = jointBoundsDefault;
for idx = 1:numel(lockMask)
    if lockMask(idx)
        jointConstraint.Bounds(idx, 1) = qCurrent(idx);
        jointConstraint.Bounds(idx, 2) = qCurrent(idx);
    end
end

[qNext, ~] = solver(qCurrent, poseConstraint, jointConstraint, distanceConstraint);
end
