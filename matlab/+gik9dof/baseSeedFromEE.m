function basePath = baseSeedFromEE(robot, trajStruct, q_nominal, options)
%BASESEDFROMEE Generate candidate base path from end-effector trajectory.
%   basePath = GIK9DOF.BASESEDFROMEE(robot, trajStruct, q_nominal) computes
%   a candidate base path by performing inverse FK for each EE waypoint.
%   For each desired EE pose, it finds a base position that keeps the arm
%   in a reasonable configuration close to q_nominal.
%
%   basePath = GIK9DOF.BASESEDFROMEE(..., options) supports:
%       EndEffector    - EE body name (default 'left_gripper_link')
%       BaseIndices    - Joint indices for base [x, y, theta] (default [1 2 3])
%       ArmIndices     - Joint indices for arm (default [4 5 6 7 8 9])
%       SearchRadius   - Max search distance for base position (m, default 2.0)
%       SearchAngles   - Candidate base orientations to try (default -pi:pi/6:pi)
%       MaxIterations  - Max IK iterations per waypoint (default 200)
%
%   Returns:
%       basePath       - Nx3 matrix [x, y, theta] of candidate base poses
%
%   Example:
%       robot = gik9dof.createRobotModel();
%       traj = gik9dof.loadJsonTrajectory('1_pull_world_scaled.json');
%       q_nom = homeConfiguration(robot);
%       basePath = gik9dof.baseSeedFromEE(robot, traj, q_nom);
%
%   See also createGikSolver, runStageCPPFirst.

arguments
    robot (1,1) rigidBodyTree
    trajStruct (1,1) struct
    q_nominal (:,1) double
    options.EndEffector (1,1) string = "left_gripper_link"
    options.BaseIndices (1,3) double = [1 2 3]
    options.ArmIndices (1,6) double = [4 5 6 7 8 9]
    options.SearchRadius (1,1) double = 2.0
    options.SearchAngles (1,:) double = -pi:(pi/6):pi
    options.MaxIterations (1,1) double = 200
    options.Verbose (1,1) logical = false
    % NEW: Phase 2A - Orientation+Z nominal pose generation
    options.UseOrientationZNominal (1,1) logical = false
    options.OrientationWeight (1,1) double = 1.0
    options.PositionWeightXY (1,1) double = 0.1
    options.PositionWeightZ (1,1) double = 1.0
end

nWaypoints = size(trajStruct.Poses, 3);
basePath = zeros(nWaypoints, 3);

% Create a simple IK solver for this purpose
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1]; % Position + orientation

if options.Verbose
    fprintf('Generating base seed path from %d EE waypoints...\n', nWaypoints);
    if options.UseOrientationZNominal
        fprintf('  Using Phase 2A: Orientation+Z priority nominal\n');
    else
        fprintf('  Using standard search-based nominal\n');
    end
end

% Track configuration for sequential IK
q_current = q_nominal;

for k = 1:nWaypoints
    T_ee_desired = trajStruct.Poses(:, :, k);
    
    %% PHASE 2A: Orientation+Z priority nominal (if enabled)
    if options.UseOrientationZNominal
        % Use weighted IK: match orientation+Z, relax X-Y
        [q_nominal_k, basePose_k, diagInfo] = gik9dof.computeNominalPoseOrientationZ(...
            robot, T_ee_desired, q_current, ...
            'EndEffector', options.EndEffector, ...
            'BaseIndices', options.BaseIndices, ...
            'ArmIndices', options.ArmIndices, ...
            'OrientationWeight', options.OrientationWeight, ...
            'PositionWeightXY', options.PositionWeightXY, ...
            'PositionWeightZ', options.PositionWeightZ, ...
            'MaxIterations', options.MaxIterations);
        
        basePath(k, :) = basePose_k';
        q_current = q_nominal_k;  % Update for next iteration
        
        if options.Verbose && (mod(k, 20) == 0 || ~diagInfo.success)
            if ~diagInfo.success
                fprintf('  [%3d] IK did not converge (xy_err=%.3f, z_err=%.3f, orient_err=%.3f)\n', ...
                    k, diagInfo.xy_error, diagInfo.z_error, diagInfo.orient_error);
            end
        end
        
        continue;  % Skip standard search method
    end
    
    %% STANDARD: Search-based nominal (Phase 1)
    
    % Extract desired EE position
    p_ee_desired = T_ee_desired(1:3, 4);
    
    % Strategy: Try multiple base orientations and distances
    % Pick the one that results in best arm configuration (closest to nominal)
    bestConfig = [];
    bestScore = inf;
    
    % Sample base positions in a circle around the EE
    searchDistances = linspace(0.5, options.SearchRadius, 5);
    
    for theta_base = options.SearchAngles
        for dist = searchDistances
            % Candidate base position: offset from EE in direction of theta_base
            x_base = p_ee_desired(1) - dist * cos(theta_base);
            y_base = p_ee_desired(2) - dist * sin(theta_base);
            
            % Build candidate configuration
            q_candidate = q_nominal;
            q_candidate(options.BaseIndices) = [x_base; y_base; theta_base];
            
            % Try IK to reach the EE pose
            [q_ik, solInfo] = ik(options.EndEffector, T_ee_desired, ...
                weights, q_candidate);
            
            % Check if IK succeeded
            if solInfo.ExitFlag > 0
                % Score based on:
                % 1. Arm configuration deviation from nominal
                % 2. Base distance from EE (prefer closer for stability)
                arm_deviation = norm(q_ik(options.ArmIndices) - q_nominal(options.ArmIndices));
                base_dist = norm([x_base; y_base] - p_ee_desired(1:2));
                
                score = arm_deviation + 0.1 * base_dist;
                
                if score < bestScore
                    bestScore = score;
                    bestConfig = q_ik;
                end
            end
        end
    end
    
    if isempty(bestConfig)
        % Fallback: place base directly behind EE
        if k == 1
            % First waypoint: use nominal base pose
            basePath(k, :) = q_nominal(options.BaseIndices)';
        else
            % Use previous base pose (assume slow motion)
            basePath(k, :) = basePath(k-1, :);
        end
        if options.Verbose
            warning('baseSeedFromEE: IK failed for waypoint %d, using fallback', k);
        end
    else
        basePath(k, :) = bestConfig(options.BaseIndices)';
    end
    
    if options.Verbose && mod(k, 20) == 0
        fprintf('  Processed %d/%d waypoints\n', k, nWaypoints);
    end
end

if options.Verbose
    fprintf('Base seed path generation complete.\n');
    fprintf('  Path length: %.2f m\n', sum(vecnorm(diff(basePath(:,1:2), 1, 1), 2, 2)));
    fprintf('  Yaw range: [%.1f, %.1f] deg\n', rad2deg(min(basePath(:,3))), rad2deg(max(basePath(:,3))));
end

end
