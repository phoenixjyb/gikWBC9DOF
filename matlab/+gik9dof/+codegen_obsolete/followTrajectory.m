function qOut = followTrajectory(q0, poses, distanceLower, distanceWeight)
%FOLLOWTRAJECTORY Iterate solveGIKStep across a pose sequence.
%#codegen
coder.inline('never');
q = q0;
numWaypoints = size(poses, 3);
for k = 1:numWaypoints
    q = gik9dof.codegen.solveGIKStep(q, poses(:,:,k), distanceLower, distanceWeight);
end
qOut = q;
end
