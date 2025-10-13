# MPC-Based Whole‑Body Control for a 6‑DoF Arm on a Nonholonomic Chassis

**Robot specifics**  
- Chassis: two-axle, wheelbase = **0.36 m**, track = **0.57 m**.  
- Front two wheels: **differential** (unicycle-like), no lateral/sideways motion.  
- Base control inputs: linear velocity \(v \equiv v_x\) and yaw rate \(\omega \equiv \dot\theta\).  
- Arm: 6‑DoF.  
- Goal: make the **end‑effector (EE)** track arbitrary 6‑DoF trajectories (positions & orientations) by **coordinating base + arm**.  
- Constraints: nonholonomic base (no \(v_y\)), wheel limits, joint limits, and **collision avoidance**.

---

## 0) High‑level picture

We solve, at 20–50 Hz, a **nonlinear MPC (NMPC)** (or a sequentially convex/LMPC approximation) whose **decision variables** over a horizon are:

- Base velocities: \(v_k \equiv v_x[k]\) and \(\omega_k \equiv \dot\theta[k]\)  
- Arm joint velocities: \(\dot{\mathbf q}_k \in \mathbb R^6\) (or accelerations if you prefer a double integrator)
- Optional **progress variable** \(s_k\) to time‑warp the EE reference (lets you slow down when the base can’t keep up, preventing infeasibility)

The **state** contains base pose \(\mathbf b_k=[x_k, y_k, \theta_k]\) and arm joints \(\mathbf q_k\).

The NMPC predicts EE pose from base+arm forward kinematics and **minimizes EE tracking error**, subject to:

- **Nonholonomic base dynamics** (unicycle): no lateral motion
- Joint/velocity/accel limits
- **Actuation‑consistent bounds on \((v,\omega)\)** from your track width
- **Collision constraints** (environment and self)

The controller outputs \((v_0,\omega_0,\dot{\mathbf q}_0)\) at each cycle. Your existing **pure‑pursuit/velocity follower** can sit below this: either track the MPC’s \((v,\omega)\) directly or (if you must keep pure pursuit) let MPC emit a base trajectory \((x,y,\theta)\) whose discrete derivatives satisfy the unicycle constraints.

---

## 1) Models and notation

### Base (nonholonomic unicycle)
Discrete time, step \(h\):
\[
\begin{aligned}
x_{k+1} &= x_k + h\, v_k \cos\theta_k \\
y_{k+1} &= y_k + h\, v_k \sin\theta_k \\
\theta_{k+1} &= \theta_k + h\, \omega_k
\end{aligned}
\]

This inherently enforces “no sideways velocity”. If you linearize, you’ll use the Jacobian around \((x_k,y_k,\theta_k,v_k,\omega_k)\).

### Manipulator (6‑DoF)
\[
\mathbf q_{k+1} = \mathbf q_k + h\, \dot{\mathbf q}_k
\]
(Use accelerations if you prefer smoother profiles: \(\dot{\mathbf q}_{k+1}=\dot{\mathbf q}_k + h\, \ddot{\mathbf q}_k\) and add rate costs.)

### Whole‑body forward kinematics
Let \({}^W\!T_{E}(\mathbf b, \mathbf q)\) be the EE pose in world. The instantaneous EE twist depends on both:
\[
\dot{\mathbf x}^{ee} =
\underbrace{J_{arm}(\mathbf q)}_{\in \mathbb R^{6\times 6}}\, \dot{\mathbf q}
\;+\;
\underbrace{J_{base}(\mathbf b)}_{\in \mathbb R^{6\times 2}}
\begin{bmatrix} v \\ \omega \end{bmatrix}
\]
For the base Jacobian, use the planar body twist \([v,0,\omega]\) in the base frame, then transform to the world/EE frame. This is the clean way to keep the “virtual prismatic x/y + revolute z” abstraction **while** respecting your nonholonomic constraint (set \(v_y\!=\!0\) in base frame).

---

## 2) Feasibility constraints for the chassis

Track is 0.57 m → half‑track \(w = 0.285\) m.

With a **front differential pair** (unicycle mapping), wheel linear speeds:
\[
v_R = v + \omega w,\qquad
v_L = v - \omega w
\]
Impose wheel speed/accel limits (from motor limits):
\[
|v_R| \le v^{max}_{wheel},\quad |v_L| \le v^{max}_{wheel}
\]
This yields **coupled bounds on \(v\) and \(\omega\)** that the MPC must honor, preventing unreachable yaw rates or sideways “wishes”.

Also add smoothness:
\[
|v_{k}-v_{k-1}| \le a_v^{max}h,\quad |\omega_{k}-\omega_{k-1}|\le a_\omega^{max}h
\]
and absolute limits \( |v_k|\le v^{max},\ |\omega_k|\le \omega^{max}\).

> **Why this matters:** It prevents the optimizer from proposing base motions that your velocity follower cannot realize, so you don’t get GIK solutions that *require* sideways base motion.

---

## 3) Task: end‑effector 6‑DoF tracking

Let the reference EE pose be \({}^W\!T_{E}^{ref}(t)\). For NMPC we need a discrete target per node \(k\). Two robust options:

1) **Fixed timing**: \(t_k = t_0 + kh\).  
2) **Time‑scaling** with progress \(s_k\):  
   \[
   s_{k+1} = s_k + h\,\dot s_k,\ \ \dot s_k \in [0,\dot s^{max}]
   \]
   and use \({}^W\!T_{E}^{ref}(s_k)\). This makes the controller **slow down instead of failing** if constraints bite (highly recommended for nonholonomic bases).

### Pose error metric
Position error: \(\mathbf e_p = \mathbf p^{ee} - \mathbf p^{ref}\).

Orientation error via quaternion log (or \(SO(3)\) log):  
\(\mathbf e_R = \text{Log}\!\left(R^{ref\,T} R^{ee}\right) \in \mathbb R^3\).

Stack into \(\mathbf e^{ee} = [\mathbf e_p;\mathbf e_R] \in \mathbb R^6\).

---

## 4) The NMPC objective

For horizon \(N\):
\[
\begin{aligned}
\min_{\{v,\omega,\dot{\mathbf q},\dot s\}}
J =
\sum_{k=0}^{N} \Big(
&\|\mathbf e^{ee}_k\|_{W_{ee}}^2
+ \lambda_u \|u_k\|_{W_u}^2
+ \lambda_{\Delta u} \|u_k - u_{k-1}\|_{W_{\Delta u}}^2 \\
&+ \lambda_q \|\mathbf q_k - \mathbf q^{nom}\|_{W_q}^2
+ \lambda_b \|\theta_k - \theta^{nom}_k\|^2
+ \lambda_s \|\dot s_k - \dot s^{des}\|^2
\Big)
\end{aligned}
\]
where \(u_k=[v_k,\omega_k,\dot{\mathbf q}_k]\).  
- Use \(\mathbf q^{nom}\) for posture regularization (center of joint ranges, away from singularities/self‑collision).  
- \(\theta^{nom}_k\) can bias the base to face the EE task direction if helpful.  
- The \(\dot s\) term tries to keep nominal timing but allows slowing.

Add **soft penalties** for near‑collisions via slacks (below).

---

## 5) Collision avoidance (environment + self)

Two layers that work in real time:

### A) Base footprint vs. environment (2‑D)
- Inflate obstacles by the base footprint (Minkowski sum) or inflate the base to a disc of radius \(r_{foot}\).  
- Either:
  - Precompute a **convex safe corridor** (sequence of convex polygons) along any global route. Then in the NMPC, enforce linear half‑space constraints that keep \((x_k,y_k)\) inside the current polygon; or
  - Use a 2‑D signed distance field \(\phi(x,y)\) and impose **linearized** constraints:
    \[
    \phi(x^{(i)},y^{(i)}) + \nabla\phi^\top \begin{bmatrix} x_k-x^{(i)} \\ y_k-y^{(i)} \end{bmatrix} \ge d_{base} - \sigma^{base}_k
    \]
    with slack \(\sigma^{base}_k \ge 0\) and a big penalty in \(J\).

### B) Manipulator links vs. environment & self
- Sample a handful of points on key links (or use spheres/capsules). For each point \(p(\mathbf b,\mathbf q)\) query a 3‑D SDF \(D(p)\).
- Linearize the distance at the last iterate:
  \[
  D(p(\mathbf b,\mathbf q)) \approx D_0 + \nabla D^\top \left(
  J_p^{base} \begin{bmatrix} v \\ \omega \end{bmatrix} h
  + J_p^{arm}\, \dot{\mathbf q} h \right)
  \ge d_{link} - \sigma^{link}_k
  \]
- Add \(\rho \sum (\sigma^{base}_k+\sigma^{link}_k)\) to the cost. This keeps feasibility while strongly discouraging collisions.

These linearizations are standard in **sequential convex programming (SCP)/LMPC** and run fast.

---

## 6) Putting it all together (constraints summary)

For all \(k=0..N-1\):
- Dynamics: base & arm as above
- Bounds: \( |v_k|\le v^{max},\ |\omega_k|\le \omega^{max},\ |\dot{\mathbf q}_k| \le \dot{\mathbf q}^{max}\)
- Smoothness: \(|v_k-v_{k-1}|\le a_v^{max}h\), etc.
- **Wheel‑speed‑consistent**:
  \[
  |v_k \pm \omega_k\,0.285| \le v^{max}_{wheel}
  \]
- Collision: base in corridor or \( \phi(x_k,y_k)\ge d_{base}-\sigma^{base}_k \); link SDFs \(\ge d_{link}-\sigma^{link}_k\)
- Progress: \(0 \le \dot s_k \le \dot s^{max}\) (if using time‑scaling)

---

## 7) How this avoids “sideways IK” failures

In classical whole‑body IK you might give the base virtual \(x,y,\theta\) and the solver happily “slides” \(y\). Here, **\(y\) only changes via \(v\sin\theta\)**. The only base decision variables are \(v,\omega\); the EE Jacobian includes the **base twist with \(v_y\!=\!0\)**. That guarantees every EE motion you generate is realizable by your chassis (and bounded by wheel speeds).

---

## 8) Coordination strategy: who moves when?

The optimization naturally allocates motion between base and arm:

- If the EE target is within the arm’s comfortable workspace, the cost on \(u\) and the base posture bias will tend to keep the base calm and use the arm.
- If the target would cause joint limits/singularity/near‑collision, the solver starts **turning/advancing the base** (within \((v,\omega)\) limits) to reposition the arm for a healthier configuration.
- The **time‑scaling variable \(s\)** is the safety valve: if neither can move fast enough without violating constraints, the controller simply slows the reference traversal instead of failing.

You can accentuate this with weights (e.g., higher \(\lambda_u\) on \(\dot{\mathbf q}\) than on \(v,\omega\) to encourage base motion—or vice versa).

---

## 9) Interface to your existing follower

Two clean options:

1) **Direct velocity mode (preferred):** NMPC outputs \((v_0,\omega_0)\). Your low‑level velocity follower uses those as feed‑forward commands, with closed‑loop wheel speed control.

2) **Pose path mode:** NMPC outputs \((x_k,y_k,\theta_k)\) over the short horizon (guaranteed unicycle‑feasible by construction). Your pure pursuit converts that to \((v,\omega)\). Keep the pure‑pursuit lookahead \(\ell\) \(\sim\) \(v\)-dependent to avoid chatter.

Either way, ensure your low‑level controller respects the same \((v,\omega)\) and wheel‑speed limits used in the MPC.

---

## 10) Practical tuning (numbers to start with)

- **Horizon:** \(N=20\), \(h=0.05\) s → 1 s lookahead (extend to 2 s if compute allows).
- **Velocity limits (example—adjust to your motors):**  
  \(v^{max}=1.0\)\,m/s, \(\omega^{max}=2.5\)\,rad/s.  
  Wheel speed: \(v^{max}_{wheel}=1.4\)\,m/s → \(|v \pm 0.285\,\omega| \le 1.4\).
- **Acceleration limits:** \(a_v^{max}=1.5\)\,m/s², \(a_\omega^{max}=4\)\,rad/s².
- **Clearances:** \(d_{base}=0.10\) m (inflated footprint), \(d_{link}=0.05\) m.
- **Weights (rough):**  
  \(W_{ee}=\text{diag}( [200,200,200,\,100,100,100] )\) (m vs rad);  
  \(W_u=\text{diag}([1, 1,\, 0.1\mathbf I_6])\);  
  \(W_{\Delta u}=10 W_u\);  
  \(\lambda_q=1\), \(\lambda_b=0.2\), \(\rho\) (collision slacks) \(=10^4\).  
- **Time‑scaling:** \(\dot s^{max}=1.2\), \(\dot s^{des}=1.0\).

---

## 11) Implementation skeleton (nonlinear or sequential convex)

You can implement as **NMPC** (e.g., direct multiple shooting with CasADi/IPOPT) or as **LMPC/SCP** (QP per cycle). Pseudocode for one cycle:

```text
Input: current (x,y,θ), q; previous warm start for {v, ω, qdot, s}; EE ref trajectory T_ref(s)

Repeat 1–3 Gauss-Newton/SCP iterations:
  1) Roll out dynamics with current control guess to get {b_k, q_k}
  2) Compute EE poses, errors e_ee,k, Jacobians J_arm, J_base at each node
  3) Build linearized constraints:
       - Dynamics (as equality in deviations)
       - Wheel-speed, bounds, smoothness
       - Collision (base corridor half-spaces OR SDF linearization)
       - Link SDF linearization for chosen link points
       - Progress [0 ≤ ṡ ≤ ṡmax]
  4) Form QP in control deviations Δu, Δs minimizing quadraticized cost
  5) Solve QP → update controls (line search if needed)

Apply first control (v0, ω0, qdot0). Advance one step. Recede horizon.
```

A pure NMPC (no linearization) is fine if you can afford the compute; SCP/LMPC is typically faster and very robust for this structure.

---

## 12) EE orientation handling and singularities

- Use quaternion log or exponential coordinates to avoid wrap issues.
- Add a **manipulability cost** (or damped least‑squares in the Gauss‑Newton metric) to stay away from singularities:
  \(J_{m} = \alpha / \text{det}(J_{arm}J_{arm}^\top + \epsilon I)\).

---

## 13) Edge cases & tips

- **In‑place turns:** If your base can spin in place, keep a small \(v^{min}\le 0\). If not (traction issues), add \(|\omega|\le \kappa_{max} |v| + \omega_0\) which discourages turning at near‑zero \(v\).
- **References that require lateral base motion:** The time‑scaling will slow down and the optimizer will reorient the base to *convert* sideways reach into feasible forward arcs plus arm motion.
- **Two‑axle footprint:** Use wheelbase \(0.36\) m and track \(0.57\) m to inflate the base footprint for collision; the **wheelbase doesn’t change unicycle kinematics**, but it does change swept volume/clearance.
- **Self‑collision:** Precompute critical link‑pair capsule distances and include those as SDF constraints as well (same linearization trick).
- **Warm starts:** Shift last solution forward each cycle—this makes the solver snap‑fast.

---

## 14) Minimal fallback if compute is tight

Run a **two‑stage controller**:

1) **Base NMPC** only: track a short‑horizon SE(2) waypoint sequence consistent with unicycle and wheel limits + base collision avoidance (fast QP).  
2) **Whole‑body differential IK** at 200–500 Hz using the augmented Jacobian with base twist \([v,0,\omega]\), with inequality tasks for joint limits and link distances.  

This still prevents sideways demands because the base twist has \(v_y=0\).

---

### What you get with this MPC design
- EE 6‑DoF trajectory tracking with **guaranteed nonholonomic feasibility**
- Automatic co‑allocation between base and arm
- **No “sideways base” IK artifacts**
- Real‑time collision avoidance (environment + self), with soft slacks for robustness
- A clean interface to your existing \((v_x,\omega_z)\) velocity follower
