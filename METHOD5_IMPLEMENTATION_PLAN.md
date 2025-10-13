# Method 5 (pureMPC) Implementation Plan

**Created:** October 13, 2025  
**Status:** Planning → Execution  
**Estimated Duration:** 8-12 weeks  

## Executive Summary

Method 5 (`pureMPC`) implements true receding horizon Nonlinear Model Predictive Control (NMPC) for Stage C trajectory tracking. Unlike Methods 0-4 which are offline planning approaches, Method 5 performs online optimization with lookahead, fundamentally embedding nonholonomic constraints in the dynamics model.

**Key Characteristics:**
- **ExecutionMode:** `'pureMPC'`
- **Control Architecture:** Receding horizon NMPC at 10-20 Hz
- **Horizon:** N = 20 steps (2 seconds @ dt=0.1s)
- **Decision Variables:** [v, ω, q̇_arm, ṡ] per step
- **Constraint Handling:** Embedded in dynamics (unicycle model) + wheel speed limits
- **Online/Offline:** Online optimization loop

**Fundamental Difference from Methods 1-4:**
- **Methods 1-4:** Offline planning, can request sideways base motion then correct
- **Method 5:** Online, cannot request sideways motion by construction (y only changes via v·sin(θ))

## Phase 1: Foundation (Weeks 1-2)

### 1.1 Documentation Updates
**Files to modify:**
- `docs/METHOD_NUMBERING_GUIDE.md` - Add Method 5 entry
- `projectDiagnosis.md` - Add Method 5 to Section 10
- `DOCUMENTATION_MAP.md` - Link Method 5 documentation

**Content:**
```yaml
Method 5 (pureMPC):
  ExecutionMode: 'pureMPC'
  Description: "True receding horizon NMPC"
  Status: "In Development (Oct 2025)"
  Key Features:
    - Online optimization at 10-20 Hz
    - Receding horizon N=20 steps
    - Unicycle dynamics model
    - Wheel speed constraints
    - Collision avoidance
  Suitable For: "Online control, dynamic environments"
```

### 1.2 Dependency Setup
**Requirements:**
- ✅ MATLAB MPC Toolbox: Native nonlinear MPC (`nlmpc` class) - **Already installed (R2024b)**
- ✅ Robotics System Toolbox: IK solver for arm joints - **Already available**
- ✅ Optimization Toolbox: Required by MPC Toolbox - **Already available**

**Verification:**
```matlab
% Verify MPC Toolbox
assert(license('test', 'MPC_Toolbox'), 'MPC Toolbox required');
ver('mpc')

% Test nlmpc creation
nlobj = nlmpc(3, 3, 2);  % 3 states, 3 outputs, 2 inputs
disp('✅ MATLAB MPC Toolbox ready');
```

**Advantages over CasADi/IPOPT:**
- No external dependencies
- Native MATLAB integration with existing framework
- Built-in constraint handling and code generation
- Better documentation and support

### 1.3 Configuration Profile
**File:** `config/pipeline_profiles.yaml`

```yaml
pureMPC:
  description: "Method 5 - True receding horizon NMPC"
  executionMode: 'pureMPC'
  
  # NMPC parameters
  nmpc:
    horizon: 20              # N steps
    dt: 0.1                  # Time step (s)
    control_rate: 10         # Hz (1/dt)
    
  # Cost weights
  weights:
    tracking: 100.0          # Base path tracking
    input_v: 1.0             # Linear velocity smoothness
    input_omega: 10.0        # Angular velocity smoothness
    terminal: 500.0          # Terminal cost
    
  # Constraints
  constraints:
    v_max: 0.5               # m/s
    omega_max: 0.8           # rad/s
    wheel_max: 0.6           # m/s (|v ± ω·W/2| ≤ this)
    track_width: 0.574       # m
    
  # Collision avoidance
  collision:
    enabled: true
    safety_radius: 0.3       # m
    obstacle_zones: []       # To be populated
    
  # Arm IK
  arm_ik:
    max_iterations: 100
    tolerance: 1e-3          # m
```

## Phase 2: Core NMPC Implementation (Weeks 3-5)

### 2.1 Main Controller Structure
**File:** `matlab/+gik9dof/runStageCPureMPC.m`

**Function Signature:**
```matlab
function pipeline = runStageCPureMPC(robot, trajWaypoints, options)
% RUNSTAGECPUREMPC Method 5 (pureMPC) - True receding horizon NMPC
%
% Uses MATLAB MPC Toolbox (nlmpc) for online optimization
%
% Inputs:
%   robot          - Robot model (rigidBodyTree)
%   trajWaypoints  - [Nx8] waypoints [x y z qw qx qy qz s_param]
%   options        - struct with fields:
%                    .ConfigTools (required)
%                    .ChassisProfile (optional, default 'wide_track')
%                    .Verbose (optional, default false)
%                    .nmpcHorizon (optional, default 20)
%                    .nmpcDt (optional, default 0.1)
%
% Outputs:
%   pipeline       - struct with fields:
%                    .tFull, .qFull, .basePose, .armJoints
%                    .solveStats (timing, iterations, cost)
```

**Algorithm Structure (Using MATLAB MPC Toolbox):**
```
1. Initialize:
   - Load NMPC parameters from profile
   - Create nlmpc object with unicycle dynamics
   - Configure constraints (wheel speeds, collisions)
   - Extract reference base path from trajWaypoints
   - Initialize state: x₀ = [x, y, θ]'

2. Build nlmpc Controller (once):
   nlobj = nlmpc(3, 3, 2);  % 3 states, 3 outputs, 2 MVs
   nlobj.Model.StateFcn = @unicycleStateFcn;
   nlobj.Model.OutputFcn = @(x,u) x;  % Full state feedback
   nlobj.Ts = 0.1;
   nlobj.PredictionHorizon = 20;
   nlobj.ControlHorizon = 10;
   
   % Set constraints
   nlobj.MV(1).Min = -0.5;  % v_min
   nlobj.MV(1).Max = 0.5;   % v_max
   nlobj.MV(2).Min = -0.8;  % ω_min
   nlobj.MV(2).Max = 0.8;   % ω_max
   
   % Custom constraints for wheel speeds
   nlobj.Optimization.CustomIneqConFcn = @wheelSpeedConstraints;

3. Main Loop (for each control step k):
   a. Get reference trajectory: yref(k:k+p) from base path
   
   b. Solve NMPC using nlmpcmove:
      [u_opt, info] = nlmpcmove(nlobj, x_current, u_last, yref);
      % Returns optimal [v*, ω*] for current step
   
   c. Simulate one step:
      x_next = unicycleStateFcn(x_current, u_opt, nlobj.Ts);
   
   d. Solve arm IK:
      - Get EE target at corresponding s parameter
      - Solve IK with base fixed at x_next
      - Update q_arm(k+1)
   
   e. Store results: q_full(k+1) = [x, y, θ, q_arm](k+1)
   
   f. Update: x_current = x_next, u_last = u_opt, k = k + 1

4. Post-process:
   - Convert q_full to pipeline structure
   - Compute statistics (solve time, cost, EE errors)
```

### 2.2 Unicycle Dynamics Module
**File:** `matlab/+gik9dof/+mpc/unicycleStateFcn.m`

```matlab
function x_next = unicycleStateFcn(x, u, Ts)
% UNICYCLESTATEFCN State transition for MATLAB nlmpc controller
%
% State: x = [x_pos; y_pos; θ]
% Control: u = [v; ω]
% Dynamics: dx/dt = v·cos(θ)
%           dy/dt = v·sin(θ)
%           dθ/dt = ω
%
% Discretization: Euler integration (Ts = sample time)

% Extract state
x_pos = x(1);
y_pos = x(2);
theta = x(3);

% Extract control
v = u(1);
omega = u(2);

% Euler integration
x_next = zeros(3, 1);
x_next(1) = x_pos + Ts * v * cos(theta);
x_next(2) = y_pos + Ts * v * sin(theta);
x_next(3) = theta + Ts * omega;

% Normalize theta to [-π, π]
x_next(3) = atan2(sin(x_next(3)), cos(x_next(3)));

end
```

**Alternative: State Jacobian (for faster optimization)**
```matlab
function [A, B] = unicycleStateJacobian(x, u, Ts)
% Jacobian for linearization (improves nlmpc solve speed)
theta = x(3);
v = u(1);

A = [1,  0,  -Ts*v*sin(theta);
     0,  1,   Ts*v*cos(theta);
     0,  0,   1];

B = [Ts*cos(theta),  0;
     Ts*sin(theta),  0;
     0,              Ts];
end
```

### 2.3 NMPC Cost Function
**File:** `matlab/+gik9dof/+mpc/buildNMPCWeights.m`

```matlab
function nlobj = buildNMPCWeights(nlobj, weights)
% BUILDNMPCWEIGHTS Configure cost weights for nlmpc controller
%
% Inputs:
%   nlobj   - nlmpc controller object
%   weights - struct with fields: tracking, input_v, input_omega, terminal
%
% Output:
%   nlobj   - Updated controller with cost weights

% Output tracking weights (position tracking)
% States are [x, y, θ], we track [x, y] heavily, θ less so
nlobj.Weights.OutputVariables = [weights.tracking, ...  % x tracking
                                  weights.tracking, ...  % y tracking
                                  weights.tracking*0.1]; % θ (less important)

% Manipulated variable (input) weights
% Penalize control effort (v, ω)
nlobj.Weights.ManipulatedVariables = [weights.input_v, ...
                                       weights.input_omega];

% Rate of change weights (smoothness)
% Penalize sudden changes in v and ω
nlobj.Weights.ManipulatedVariablesRate = [weights.input_v*2, ...
                                          weights.input_omega*2];

% Terminal weight (via custom cost function if needed)
if isfield(weights, 'terminal') && weights.terminal > 0
    nlobj.Optimization.CustomCostFcn = @(X,U,e,data) ...
        terminalCostFcn(X,U,e,data,weights.terminal);
end

end

function J = terminalCostFcn(X, U, e, data, terminal_weight)
% Custom cost: add terminal penalty
% X(:,end) is terminal state, data.References(:,end) is terminal ref
terminal_error = X(1:2, end) - data.References(1:2, end);
J = terminal_weight * (terminal_error' * terminal_error);
end
```

**Note:** MATLAB MPC Toolbox uses **quadratic costs** by default:
```
J = Σ [||y - yref||²_Q + ||u||²_R + ||Δu||²_S] + ||y_N - yref_N||²_P
```
Weights are set via `nlobj.Weights` properties.

### 2.4 Constraint Functions
**File:** `matlab/+gik9dof/+mpc/wheelSpeedConstraints.m`

```matlab
function cineq = wheelSpeedConstraints(X, U, data, params)
% WHEELSPEEDCONSTRAINTS Custom inequality constraints for nlmpc
%
% Implements: |v ± ω·W/2| ≤ v_wheel_max
%
% Inputs:
%   X      - [nx x (p+1)] predicted states (not used here)
%   U      - [nu x p] predicted control moves
%   data   - Additional data from nlmpcmove
%   params - struct with track_width, wheel_max
%
% Output:
%   cineq  - [nc x 1] constraint vector (cineq ≤ 0)

% Extract parameters
W = params.track_width;
v_wheel_max = params.wheel_max;

% Number of prediction steps
p = size(U, 2);

% Initialize constraint vector (2 constraints per step)
cineq = zeros(2*p, 1);

% Apply constraints for each step
for k = 1:p
    v = U(1, k);
    omega = U(2, k);
    
    % Left wheel: v - ω·W/2 ∈ [-v_max, v_max]
    v_left = v - omega * W/2;
    cineq(2*k-1) = abs(v_left) - v_wheel_max;  % |v_left| ≤ v_max
    
    % Right wheel: v + ω·W/2 ∈ [-v_max, v_max]
    v_right = v + omega * W/2;
    cineq(2*k) = abs(v_right) - v_wheel_max;   % |v_right| ≤ v_max
end

end
```

**Collision Avoidance (Optional):**
```matlab
function cineq = collisionConstraints(X, U, data, obstacles, r_safe)
% Avoid circular obstacles: dist(x,y, obs) ≥ r_safe + r_obs
p = size(X, 2) - 1;
n_obs = size(obstacles, 1);

cineq = zeros(n_obs * (p+1), 1);
idx = 1;

for k = 1:(p+1)
    x_pos = X(1, k);
    y_pos = X(2, k);
    
    for i = 1:n_obs
        obs = obstacles(i, :);  % [x_obs, y_obs, r_obs]
        dist = sqrt((x_pos - obs(1))^2 + (y_pos - obs(2))^2);
        min_dist = r_safe + obs(3);
        cineq(idx) = min_dist - dist;  % dist ≥ min_dist
        idx = idx + 1;
    end
end
end
```

### 2.5 Arm IK Integration
**File:** `matlab/+gik9dof/+mpc/solveArmIKForBase.m`

```matlab
function [q_arm, success] = solveArmIKForBase(robot, eeTarget, basePose, q_arm_init)
% SOLVEARMIKFORBASE Solve arm IK given fixed base pose
%
% Inputs:
%   robot      - rigidBodyTree
%   eeTarget   - [x y z qw qx qy qz] EE target
%   basePose   - [x y θ] base pose
%   q_arm_init - [6x1] initial guess for arm joints
%
% Outputs:
%   q_arm      - [6x1] arm joint angles
%   success    - boolean

% Build full joint configuration guess
q_guess = [basePose(1); basePose(2); basePose(3); q_arm_init];

% Create IK solver with base joints locked
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1];  % Position + orientation
endEffector = 'gripper_link';  % Replace with actual EE name

% Convert target to tform
tform = trvec2tform([eeTarget(1:3)']) * quat2tform([eeTarget(4:7)]);

% Lock base joints via bounds
ik.SolverParameters.MaxIterations = 100;
ik.SolverParameters.AllowRandomRestart = false;

% Solve with constrained base
[q_full, solInfo] = ik(endEffector, tform, weights, q_guess);

q_arm = q_full(4:9);  % Extract arm joints (indices 4-9)
success = (solInfo.ExitFlag == 1) && (solInfo.PoseErrorNorm < 1e-3);

end
```

## Phase 3: Integration & Testing (Weeks 6-8)

### 3.1 Update runStagedTrajectory Router
**File:** `matlab/+gik9dof/runStagedTrajectory.m`

Add routing case:
```matlab
case 'pureMPC'
    fprintf('Running Stage C with Method 5 (pureMPC)\n');
    pipeline = gik9dof.runStageCPureMPC(robot, trajWaypoints, ...
        'ConfigTools', options.ConfigTools, ...
        'ChassisProfile', options.ChassisProfile, ...
        'Verbose', options.Verbose);
```

### 3.2 Create Integration Test
**File:** `tests/test_method5_integration.m`

```matlab
%% Test Method 5 (pureMPC) Integration
% Smoke test: 5-10 waypoints, verify NMPC loop executes

%% Setup
clearvars; close all; clc;
addpath(genpath('matlab'));

% Load robot
robot = loadrobot('mobileDualUR3e', 'DataFormat', 'column');

% Load short trajectory
trajData = jsondecode(fileread('refEETrajs/1_pull_world_scaled.json'));
waypoints = extractWaypoints(trajData, 1:10);  % First 10 waypoints

% Create config
configTools = gik9dof.ConfigTools('config/pipeline_profiles.yaml', ...
                                   'config/chassis_profiles.yaml');

%% Run Method 5
tic;
pipeline = gik9dof.runStagedTrajectory(robot, waypoints, ...
    'ExecutionMode', 'pureMPC', ...
    'ConfigTools', configTools, ...
    'ChassisProfile', 'wide_track', ...
    'Verbose', true);
elapsed = toc;

%% Analyze Results
fprintf('\n=== Method 5 Integration Test ===\n');
fprintf('Waypoints: %d\n', size(waypoints, 1));
fprintf('Total time: %.2f s\n', elapsed);
fprintf('Solve time: %.2f s\n', pipeline.solveStats.totalTime);
fprintf('Mean iteration: %.1f\n', mean(pipeline.solveStats.iterations));
fprintf('NMPC cost: %.2f\n', pipeline.solveStats.finalCost);

% EE tracking error
ee_errors = computeEEErrors(robot, pipeline.qFull, waypoints);
fprintf('Mean EE error: %.2f mm\n', mean(ee_errors) * 1000);
fprintf('Max EE error: %.2f mm\n', max(ee_errors) * 1000);

%% Visualize (optional)
if exist('renderWholeBodyAnimation', 'file')
    renderWholeBodyAnimation(robot, pipeline, 'method5_test');
end
```

### 3.3 Full Trajectory Test
**File:** `matlab/run_method5_full.m`

```matlab
%% Run Method 5 on Full 148-Waypoint Trajectory
clearvars; close all; clc;

% Load full trajectory
trajData = jsondecode(fileread('refEETrajs/1_pull_world_scaled.json'));
waypoints = extractWaypoints(trajData, 1:148);

% Run Method 5
pipeline = gik9dof.runStagedTrajectory(robot, waypoints, ...
    'ExecutionMode', 'pureMPC', ...
    'ConfigTools', configTools, ...
    'Verbose', true);

% Save results
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
save(sprintf('results/%s_method5_full.mat', timestamp), 'pipeline');
```

## Phase 4: Optimization & Tuning (Weeks 9-12)

### 4.1 Parameter Tuning
**Tunable Parameters:**
- Horizon length N (10-30 steps)
- Time step dt (0.05-0.2 s)
- Cost weights (tracking, input, terminal)
- Constraint margins (wheel speed buffer)
- IPOPT solver options (tolerance, max_iter)

**Tuning Script:** `matlab/tune_method5_params.m`

### 4.2 Performance Benchmarking
**Metrics:**
- Solve time per step (target < 100 ms for 10 Hz control)
- EE tracking error (compare vs Method 1 & 4)
- Base trajectory smoothness (jerk, curvature)
- Fallback/failure rate

**Comparison Script:** `matlab/compare_all_methods.m`

### 4.3 Advanced Features (Optional)
- Adaptive horizon based on trajectory curvature
- Warm-start from previous solution
- RK4 integration for dynamics
- Terminal constraint for exact goal reaching
- Dynamic obstacle avoidance

## Implementation Checklist

- [ ] **Phase 1: Foundation**
  - [ ] Update METHOD_NUMBERING_GUIDE.md
  - [ ] Update projectDiagnosis.md Section 10
  - [ ] Update DOCUMENTATION_MAP.md
  - [ ] Install CasADi + IPOPT
  - [ ] Add pureMPC profile to pipeline_profiles.yaml

- [ ] **Phase 2: Core Implementation**
  - [ ] Create runStageCPureMPC.m skeleton
  - [ ] Implement unicycleDynamics.m
  - [ ] Implement buildNMPCCost.m
  - [ ] Implement buildConstraints.m
  - [ ] Implement solveArmIKForBase.m
  - [ ] Complete runStageCPureMPC.m main loop

- [ ] **Phase 3: Integration**
  - [ ] Update runStagedTrajectory.m router
  - [ ] Create test_method5_integration.m
  - [ ] Run integration test (5-10 waypoints)
  - [ ] Debug and fix issues
  - [ ] Run full trajectory test (148 waypoints)

- [ ] **Phase 4: Optimization**
  - [ ] Parameter tuning
  - [ ] Benchmark vs Method 1 & 4
  - [ ] Document results
  - [ ] Optional advanced features

## Risk Mitigation

**Risk 1: CasADi/IPOPT installation issues**
- Mitigation: Provide detailed installation guide, fallback to fmincon
- Contingency: Use MATLAB Optimization Toolbox instead (slower)

**Risk 2: NMPC solve time > 100 ms**
- Mitigation: Reduce horizon N, warm-start, sparse constraints
- Contingency: Lower control rate to 5 Hz

**Risk 3: Arm IK fails for NMPC-generated base trajectory**
- Mitigation: Add arm reachability constraints in NMPC cost
- Contingency: Fallback to Method 4 for problematic segments

**Risk 4: Nonholonomic constraints too restrictive**
- Mitigation: Tuning test on diverse trajectories
- Contingency: Hybrid approach - Method 5 for feasible segments, Method 4 for tight maneuvers

## Success Criteria

**Minimum Viable Product (MVP):**
- [ ] NMPC loop executes without errors
- [ ] Base trajectory respects wheel speed limits
- [ ] EE tracking error < 50mm mean (comparable to Method 4)
- [ ] Solve time < 200 ms per step (5 Hz feasible)

**Production Ready:**
- [ ] EE tracking error < 15mm mean (better than Method 4)
- [ ] Solve time < 100 ms per step (10 Hz control)
- [ ] Zero constraint violations
- [ ] Smoother base trajectory than Method 1 & 4 (lower jerk)
- [ ] Full documentation and test coverage

## References

- **Primary Design:** `g5wbcMpcDesign.md`
- **Method Comparison:** `STAGE_C_METHODS_COMPLETE_ANALYSIS.md`
- **MPC Analysis:** `MPC_IMPLEMENTATION_ANALYSIS.md`
- **CasADi Documentation:** https://web.casadi.org/
- **IPOPT Documentation:** https://coin-or.github.io/Ipopt/

---
**Document Status:** Living document, update as implementation progresses  
**Next Review:** After Phase 1 completion
