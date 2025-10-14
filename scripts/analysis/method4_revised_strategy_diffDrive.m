% Revised Method 4 Improvement Strategy
% Respecting Differential Drive Constraints

%% CRITICAL INSIGHT FROM USER:
% Wide corridors (40°, 0.40m) allow large lateral deviations
% Differential drive CANNOT execute lateral motion
% → Kinematically infeasible trajectories!
%
% SOLUTION: Don't widen corridors; improve PP predictions instead

fprintf('════════════════════════════════════════════════════════════\n');
fprintf('  REVISED METHOD 4 IMPROVEMENT STRATEGY\n');
fprintf('  Constraint: Differential Drive (No Lateral Motion)\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

%% APPROACH 1: ARM-REACHABILITY-AWARE PURE PURSUIT
fprintf('APPROACH 1: Reachability-Aware Pure Pursuit\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

fprintf('Current PP Problem:\n');
fprintf('  - PP follows path centerline\n');
fprintf('  - Optimizes for path following, NOT arm reachability\n');
fprintf('  - Base may end up in poor configuration for EE task\n\n');

fprintf('Solution: Arm-Aware PP\n');
fprintf('  1. For each candidate base pose on path:\n');
fprintf('     - Check if arm can reach target EE pose\n');
fprintf('     - Compute reachability score (manipulability, distance)\n');
fprintf('  2. Choose base pose that maximizes reachability\n');
fprintf('  3. Still follows path, but optimizes within ±small tolerance\n\n');

fprintf('Implementation Sketch:\n');
fprintf('  function basePose = armAwarePurePursuit(path, eePose, lookahead)\n');
fprintf('      %% Sample candidate poses along path\n');
fprintf('      candidates = samplePathWindow(path, lookahead, ±0.1m);\n');
fprintf('      \n');
fprintf('      %% Evaluate reachability for each\n');
fprintf('      for i = 1:length(candidates)\n');
fprintf('          q_arm = solveArmIK(candidates(i), eePose);\n');
fprintf('          if isReachable(q_arm)\n');
fprintf('              score(i) = manipulability(q_arm);\n');
fprintf('          else\n');
fprintf('              score(i) = -inf;\n');
fprintf('          end\n');
fprintf('      end\n');
fprintf('      \n');
fprintf('      %% Pick best reachable pose\n');
fprintf('      [~, best] = max(score);\n');
fprintf('      basePose = candidates(best);\n');
fprintf('  end\n\n');

fprintf('Expected Impact:\n');
fprintf('  - PP predictions already near-optimal for arm\n');
fprintf('  - Tight corridors (15°, 0.15m) become sufficient\n');
fprintf('  - GIK converges faster within constraints\n');
fprintf('  - Respect differential drive kinematics\n\n');

fprintf('Effort: 12-16 hours\n');
fprintf('Expected error reduction: -100 to -150 mm\n\n\n');

%% APPROACH 2: WARM-STARTING (Still Valid!)
fprintf('APPROACH 2: Warm-Starting (STILL CRITICAL)\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

fprintf('This approach is STILL valid and important:\n');
fprintf('  - Use previous solution as initial guess\n');
fprintf('  - Helps convergence within TIGHT corridors\n');
fprintf('  - No kinematic feasibility issues\n\n');

fprintf('Implementation: (As before)\n');
fprintf('  if iWaypoint > 1\n');
fprintf('      gik.setInitialGuess(prevQ);\n');
fprintf('  end\n\n');

fprintf('Expected Impact:\n');
fprintf('  - +20%% convergence rate\n');
fprintf('  - -30 mm error\n');
fprintf('  - Faster solves\n\n');

fprintf('Effort: 4 hours\n\n\n');

%% APPROACH 3: DIFFERENTIAL DRIVE CONSTRAINTS IN GIK
fprintf('APPROACH 3: Explicit Differential Drive Constraints\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

fprintf('Add nonholonomic constraints to GIK formulation:\n\n');

fprintf('Current GIK:\n');
fprintf('  minimize: EE error + collision penalties\n');
fprintf('  subject to:\n');
fprintf('    - Joint limits\n');
fprintf('    - θ ∈ [θ_pp ± corridor]\n');
fprintf('    - (x,y) ∈ [pp ± tolerance]\n\n');

fprintf('Proposed GIK:\n');
fprintf('  minimize: EE error + collision penalties\n');
fprintf('  subject to:\n');
fprintf('    - Joint limits\n');
fprintf('    - θ ∈ [θ_pp ± corridor]\n');
fprintf('    - (x,y) ∈ [pp ± tolerance]\n');
fprintf('    - Δy/Δt = v * sin(θ)  ← enforce differential drive\n');
fprintf('    - Δx/Δt = v * cos(θ)\n');
fprintf('    - |Δθ/Δt| < ω_max\n\n');

fprintf('This ensures:\n');
fprintf('  - Solutions respect chassis kinematics\n');
fprintf('  - No lateral drift\n');
fprintf('  - Feasible base trajectories\n\n');

fprintf('Effort: 10-15 hours (requires GIK formulation changes)\n');
fprintf('Expected: Kinematically feasible solutions\n\n\n');

%% APPROACH 4: VELOCITY-LIMITED CORRIDOR
fprintf('APPROACH 4: Velocity-Constrained Corridor\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

fprintf('Key insight: Lateral deviation depends on velocity and time\n\n');

fprintf('At velocity v, time Δt:\n');
fprintf('  Max feasible lateral deviation = v * sin(θ_max) * Δt\n');
fprintf('  \n');
fprintf('  For v=0.5 m/s, Δt=0.1s (10Hz):\n');
fprintf('    θ=15°: Δy = 0.5 * sin(15°) * 0.1 = 0.013m ✓\n');
fprintf('    θ=40°: Δy = 0.5 * sin(40°) * 0.1 = 0.032m\n\n');

fprintf('Adaptive corridor based on velocity:\n');
fprintf('  function corridor = velocityLimitedCorridor(v, dt, lat_tol)\n');
fprintf('      %% Max yaw that keeps lateral motion within tolerance\n');
fprintf('      max_theta = asin(min(1, lat_tol / (v * dt)));\n');
fprintf('      corridor = rad2deg(max_theta);\n');
fprintf('  end\n\n');

fprintf('Example:\n');
fprintf('  v=0.5 m/s, dt=0.1s, lat_tol=0.05m:\n');
fprintf('  → corridor = asin(0.05/(0.5*0.1)) = asin(1) = 90° (unconstrained)\n');
fprintf('  \n');
fprintf('  v=0.5 m/s, dt=0.1s, lat_tol=0.02m:\n');
fprintf('  → corridor = asin(0.02/(0.5*0.1)) = 23.6°\n\n');

fprintf('Effort: 6-8 hours\n');
fprintf('Expected: Corridors sized to ensure kinematic feasibility\n\n\n');

%% APPROACH 5: TWO-PASS METHOD
fprintf('APPROACH 5: Two-Pass Optimization\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

fprintf('Pass 1: Unconstrained whole-body IK (like Method 1)\n');
fprintf('  - Find globally optimal configuration\n');
fprintf('  - Ignore PP corridor\n');
fprintf('  - May violate differential drive constraints\n\n');

fprintf('Pass 2: Project to feasible manifold\n');
fprintf('  - Take Pass 1 solution\n');
fprintf('  - Find nearest kinematically feasible base trajectory\n');
fprintf('  - Solve: min ||q_base - q_pass1|| s.t. diff-drive constraints\n');
fprintf('  - Re-solve arm IK with corrected base\n\n');

fprintf('This gives:\n');
fprintf('  - Best possible EE tracking (Pass 1)\n');
fprintf('  - Feasible execution (Pass 2)\n\n');

fprintf('Effort: 15-20 hours\n');
fprintf('Expected: Near-Method-1 accuracy with feasibility\n\n\n');

%% RECOMMENDED IMPLEMENTATION ORDER
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('  RECOMMENDED IMPLEMENTATION ORDER\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

fprintf('PHASE 1 (Week 1): Foundation - 10-12 hours\n');
fprintf('──────────────────────────────────────────────────\n');
fprintf('1. Implement warm-starting (4 hrs)\n');
fprintf('   → +20%% convergence, -30mm error\n\n');

fprintf('2. Relax solver tolerances (1 hr)\n');
fprintf('   → +10%% convergence, -20mm error\n\n');

fprintf('3. Velocity-limited corridor (6 hrs)\n');
fprintf('   → Ensure kinematic feasibility\n\n');

fprintf('Expected after Phase 1:\n');
fprintf('  - Error: ~270mm (from 319mm)\n');
fprintf('  - Convergence: ~77%%\n');
fprintf('  - All solutions kinematically feasible ✓\n\n\n');

fprintf('PHASE 2 (Week 2): Reachability - 12-16 hours\n');
fprintf('──────────────────────────────────────────────────\n');
fprintf('1. Arm-aware Pure Pursuit (12-16 hrs)\n');
fprintf('   → Better initial guesses\n');
fprintf('   → -100 to -150mm error reduction\n\n');

fprintf('Expected after Phase 2:\n');
fprintf('  - Error: ~120-170mm (GOAL ACHIEVED!)\n');
fprintf('  - Convergence: >85%%\n');
fprintf('  - Fallback: <15%%\n\n\n');

fprintf('PHASE 3 (Week 3+): Advanced - Optional\n');
fprintf('──────────────────────────────────────────────────\n');
fprintf('1. Explicit differential drive constraints in GIK (10-15 hrs)\n');
fprintf('2. Two-pass optimization (15-20 hrs)\n\n');

fprintf('Expected after Phase 3:\n');
fprintf('  - Error: <100mm (better than Method 1!)\n');
fprintf('  - Convergence: >90%%\n');
fprintf('  - Fallback: <5%%\n\n\n');

%% KEY INSIGHT SUMMARY
fprintf('════════════════════════════════════════════════════════════\n');
fprintf('  KEY INSIGHTS\n');
fprintf('════════════════════════════════════════════════════════════\n\n');

fprintf('❌ WRONG APPROACH (My initial recommendation):\n');
fprintf('   "Just widen corridor to 40°"\n');
fprintf('   → Creates kinematically infeasible trajectories\n');
fprintf('   → Differential drive cannot execute lateral motion\n\n');

fprintf('✓ CORRECT APPROACH:\n');
fprintf('   1. Keep corridors TIGHT (respect chassis limits)\n');
fprintf('   2. Make PP predictions BETTER (arm-aware)\n');
fprintf('   3. Help convergence (warm-starting, tolerances)\n');
fprintf('   4. Explicitly enforce differential drive constraints\n\n');

fprintf('DIFFERENTIAL DRIVE CONSTRAINT:\n');
fprintf('   Lateral motion must satisfy: ẏ = v*sin(θ)\n');
fprintf('   Cannot deviate arbitrarily from path!\n\n');

fprintf('TARGET:\n');
fprintf('   - Tight corridors: 15-20° (kinematically feasible)\n');
fprintf('   - Better PP: Arm-reachability-aware\n');
fprintf('   - Result: <150mm error with feasible trajectories\n\n');

fprintf('════════════════════════════════════════════════════════════\n');
fprintf('Analysis complete! User feedback was CRITICAL.\n');
fprintf('════════════════════════════════════════════════════════════\n');
