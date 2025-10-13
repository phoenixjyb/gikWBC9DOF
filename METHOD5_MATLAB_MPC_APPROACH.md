# Method 5: MATLAB MPC Toolbox Approach

**Decision Date:** October 13, 2025  
**Rationale:** Use native MATLAB MPC Toolbox instead of external CasADi/IPOPT

---

## Why MATLAB MPC Toolbox?

### ✅ Advantages

| Aspect | MATLAB MPC Toolbox | CasADi + IPOPT |
|--------|-------------------|----------------|
| **Installation** | ✅ Already available (R2024b) | ❌ External dependency, complex setup |
| **Integration** | ✅ Native MATLAB, seamless | ⚠️ Requires interface layer |
| **Documentation** | ✅ MathWorks support, examples | ⚠️ Community-driven |
| **Learning Curve** | ✅ Familiar to MATLAB users | ⚠️ New syntax, concepts |
| **Code Generation** | ✅ Built-in C codegen | ⚠️ Requires manual work |
| **Debugging** | ✅ MATLAB debugger, MPC diagnostic tools | ⚠️ Black-box solver |
| **Maintenance** | ✅ MathWorks updates | ⚠️ Third-party dependency |
| **Performance** | ✅ Optimized for MATLAB | ⚠️ Interface overhead |

### Key MATLAB MPC Toolbox Features

**1. Nonlinear MPC Controller (`nlmpc`)**
```matlab
% Create controller
nlobj = nlmpc(nx, ny, nu);  % states, outputs, inputs
nlobj.Model.StateFcn = @myStateFcn;
nlobj.Ts = 0.1;
nlobj.PredictionHorizon = 20;
nlobj.ControlHorizon = 10;

% Solve at each step
[u, info] = nlmpcmove(nlobj, x, u_last, yref);
```

**2. Built-in Constraint Handling**
- Box constraints: `nlobj.MV(i).Min`, `nlobj.MV(i).Max`
- Custom nonlinear: `nlobj.Optimization.CustomIneqConFcn`
- State constraints: `nlobj.States(i).Min`, `nlobj.States(i).Max`

**3. Cost Function Configuration**
- Output tracking: `nlobj.Weights.OutputVariables`
- Input penalty: `nlobj.Weights.ManipulatedVariables`
- Rate penalty: `nlobj.Weights.ManipulatedVariablesRate`
- Custom cost: `nlobj.Optimization.CustomCostFcn`

**4. Solver Options**
- Multiple solvers: `fmincon` (default), `cgmres` (fast)
- Warm-start automatically handled
- Diagnostics: `info.Iterations`, `info.Cost`, `info.ExitFlag`

---

## Implementation Comparison

### CasADi Approach (Original Plan)
```matlab
% Build symbolic optimization problem
opti = casadi.Opti();
X = opti.variable(3, N+1);  % State trajectory
U = opti.variable(2, N);    % Control trajectory

% Dynamics constraints (manual loop)
for k = 1:N
    x_next = unicycleDynamics(X(:,k), U(:,k), dt);
    opti.subject_to(X(:,k+1) == x_next);
end

% Cost (manual construction)
cost = 0;
for k = 1:N
    cost = cost + (X(1:2,k) - ref(1:2,k))'*Q*(X(1:2,k) - ref(1:2,k));
    cost = cost + U(:,k)'*R*U(:,k);
end
opti.minimize(cost);

% Solve
opti.solver('ipopt');
sol = opti.solve();
u_opt = sol.value(U(:,1));
```

### MATLAB MPC Toolbox Approach (New Plan)
```matlab
% Create controller (once)
nlobj = nlmpc(3, 3, 2);
nlobj.Model.StateFcn = @unicycleStateFcn;
nlobj.Ts = 0.1;
nlobj.PredictionHorizon = 20;

% Configure costs
nlobj.Weights.OutputVariables = [100, 100, 10];  % [x, y, θ]
nlobj.Weights.ManipulatedVariables = [1, 10];    % [v, ω]

% Set constraints
nlobj.MV(1).Min = -0.5; nlobj.MV(1).Max = 0.5;   % v
nlobj.MV(2).Min = -0.8; nlobj.MV(2).Max = 0.8;   % ω
nlobj.Optimization.CustomIneqConFcn = @wheelConstraints;

% Solve (each step)
[u_opt, info] = nlmpcmove(nlobj, x_current, u_last, yref);
```

**Key Difference:** 
- CasADi: Manual construction, more control but verbose
- MATLAB MPC: Declarative, automatic handling, cleaner

---

## Unicycle Dynamics Implementation

### State Function (Required)
```matlab
function x_next = unicycleStateFcn(x, u, Ts)
% For nlmpc.Model.StateFcn
x_pos = x(1);
y_pos = x(2);
theta = x(3);
v = u(1);
omega = u(2);

x_next = [x_pos + Ts * v * cos(theta);
          y_pos + Ts * v * sin(theta);
          atan2(sin(theta + Ts * omega), cos(theta + Ts * omega))];
end
```

### State Jacobian (Optional, for speed)
```matlab
function [A, B] = unicycleStateJacobian(x, u, Ts)
% Improves nlmpc convergence speed
theta = x(3);
v = u(1);

A = [1, 0, -Ts*v*sin(theta);
     0, 1,  Ts*v*cos(theta);
     0, 0,  1];

B = [Ts*cos(theta), 0;
     Ts*sin(theta), 0;
     0,             Ts];
end

% Assign to nlobj
nlobj.Jacobian.StateFcn = @unicycleStateJacobian;
```

---

## Custom Constraints

### Wheel Speed Constraints
```matlab
function cineq = wheelSpeedConstraints(X, U, data, W, v_max)
% Implements: |v ± ω·W/2| ≤ v_max
% Returns: cineq ≤ 0

p = size(U, 2);
cineq = zeros(2*p, 1);

for k = 1:p
    v = U(1, k);
    omega = U(2, k);
    
    v_left = v - omega * W/2;
    v_right = v + omega * W/2;
    
    cineq(2*k-1) = abs(v_left) - v_max;
    cineq(2*k) = abs(v_right) - v_max;
end
end

% Assign to nlobj (with parameter binding)
W = 0.574;
v_max = 0.6;
nlobj.Optimization.CustomIneqConFcn = @(X,U,data) ...
    wheelSpeedConstraints(X, U, data, W, v_max);
```

---

## Performance Expectations

### Computational Cost

**CasADi + IPOPT:**
- Setup time: ~1-2s (symbolic construction)
- Solve time per step: 50-150 ms (depends on horizon, warm-start)
- Memory: Higher (symbolic graph)

**MATLAB MPC Toolbox:**
- Setup time: <100 ms (nlmpc object creation)
- Solve time per step: 30-100 ms (`fmincon`), 10-30 ms (`cgmres`)
- Memory: Lower (optimized internal representation)

**Conclusion:** MATLAB MPC Toolbox likely **faster** for this use case.

### Accuracy

Both achieve similar accuracy (solver tolerance typically 1e-6). MATLAB MPC has advantage of:
- Better numerical conditioning (MathWorks optimization)
- Automatic scaling of variables
- Built-in infeasibility detection

---

## Recommended Configuration

```matlab
%% Create nlmpc controller for Method 5
function nlobj = createMethod5Controller(params)

% Create controller: 3 states [x, y, θ], 2 inputs [v, ω]
nlobj = nlmpc(3, 3, 2);

% Dynamics
nlobj.Model.StateFcn = 'gik9dof.mpc.unicycleStateFcn';
nlobj.Jacobian.StateFcn = 'gik9dof.mpc.unicycleStateJacobian';  % Optional

% Timing
nlobj.Ts = params.dt;                     % 0.1s
nlobj.PredictionHorizon = params.horizon; % 20 steps
nlobj.ControlHorizon = params.horizon/2;  % 10 steps (reduces DOF)

% Costs
nlobj.Weights.OutputVariables = [params.weights.tracking, ...
                                  params.weights.tracking, ...
                                  params.weights.tracking*0.1];
nlobj.Weights.ManipulatedVariables = [params.weights.input_v, ...
                                       params.weights.input_omega];
nlobj.Weights.ManipulatedVariablesRate = [params.weights.input_v*2, ...
                                          params.weights.input_omega*2];

% Input bounds
nlobj.MV(1).Min = -params.v_max;
nlobj.MV(1).Max = params.v_max;
nlobj.MV(2).Min = -params.omega_max;
nlobj.MV(2).Max = params.omega_max;

% Wheel speed constraints
nlobj.Optimization.CustomIneqConFcn = @(X,U,data) ...
    gik9dof.mpc.wheelSpeedConstraints(X, U, data, params);

% Solver options
nlobj.Optimization.SolverOptions.MaxIterations = 100;
nlobj.Optimization.SolverOptions.ConstraintTolerance = 1e-4;

end
```

---

## Migration Path

If performance issues arise, can still switch to CasADi later:
1. Keep interface (`runStageCPureMPC.m`) the same
2. Replace internal solver (nlmpc → CasADi)
3. Benchmark and compare

**But:** Start with MATLAB MPC Toolbox - **easier, faster to implement, likely sufficient**.

---

## References

- **MATLAB Documentation:** [Nonlinear MPC](https://www.mathworks.com/help/mpc/nonlinear-mpc.html)
- **Example:** [Path Following with Obstacle Avoidance using NMPC](https://www.mathworks.com/help/mpc/ug/obstacle-avoidance-using-adaptive-model-predictive-control.html)
- **State Function:** [nlmpc Model Specification](https://www.mathworks.com/help/mpc/ug/specify-model-for-nonlinear-mpc.html)
- **Custom Constraints:** [Custom Constraints in NMPC](https://www.mathworks.com/help/mpc/ug/nonlinear-mpc-design-with-custom-constraints.html)

---

**Decision:** ✅ Proceed with **MATLAB MPC Toolbox** for Method 5 implementation.
