function h_cost = computeEuclideanHeuristic(x_start, y_start, x_goal, y_goal)
%COMPUTEEUCLIDEANHEURISTIC Simple Euclidean distance heuristic (position only)
%   h_cost = computeEuclideanHeuristic(x_start, y_start, x_goal, y_goal)
%
%   Computes straight-line distance between two positions.
%   Ignores heading (theta) - useful for comparison with Dubins heuristic.
%
%   INPUTS:
%       x_start, y_start - Start position [m]
%       x_goal, y_goal   - Goal position [m]
%
%   OUTPUTS:
%       h_cost - Euclidean distance [m]
%
%   Properties:
%       - Admissible: Always underestimates (straight line is shortest)
%       - Consistent: Satisfies triangle inequality
%       - Fast: ~0.001 ms/call
%       - Ignores non-holonomic constraints (less informed than Dubins)
%
%   Note:
%       For non-holonomic robots, Euclidean is overly optimistic.
%       Dubins heuristic is more accurate but still admissible.
%
%   See also computeDubinsHeuristic, planHybridAStar

%#codegen

dx = x_goal - x_start;
dy = y_goal - y_start;

h_cost = sqrt(dx^2 + dy^2);

end
