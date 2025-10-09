# Unified Chassis Controller — Holistic (GIK) + Staged (B/C)
**Goal:** One control layer that accepts motion from either (1) **Holistic** (9‑DOF GIK with a P‑P‑R base) or (2) **Staged** (B: chassis‑only path; C: full 9‑DOF akin to holistic), and **emits a single, safe command** to the existing base firmware: **(Vx, Vy=0, Wz)**.

---

## 0) Baseline & Assumptions

- **Base:** differential‑drive. Firmware interface: `Drive_Motor(Vx, Vy, Vz)`; pass **`Vy = 0`**. The firmware performs kinematics → wheel targets, **10 ms velocity ramping**, global **speed clamps**, and **per‑wheel PI** for each side.
- **Feasibility (yaw gate):** derive a yaw‑rate limit from wheel speed & track:

  \[
  |Wz| \le \frac{2\,(V_{\text{wheel,max}} - |Vx|)}{\text{track}}
  \]
  (Clamp to zero if the RHS becomes ≤ 0.) Apply this **before** calling `Drive_Motor`.
- **Typical rates:** global planner 0.5–2 Hz; local follower / unified controller **50–100 Hz**; firmware wheel PI **100 Hz**.
- **Tracks you use:** 0.329 m (compact) or 0.573 m (wide). Choose one per run in the parameters.

> **Design rule:** The unified controller **only** outputs (Vx, Wz). All wheel‑level details (ramp, saturation, PI) remain inside firmware.

---

## 1) Unified Command Schema (one message)

Emit the same structure every tick, regardless of source.

```matlab
% Sent to downstream @ 50–100 Hz
UnifiedCmd.mode         % "staged-B" | "staged-C" | "holistic"
UnifiedCmd.time         % timestamp (s)
UnifiedCmd.base.Vx      % m/s
UnifiedCmd.base.Vy      % = 0 (diff-drive)
UnifiedCmd.base.Wz      % rad/s (yaw)
UnifiedCmd.arm.qdot     % 1x6 rad/s (optional; filled in holistic / staged-C)
UnifiedCmd.flags        % bitfield: reverseAllowed, hold, estPoseValid, etc.
```

Downstream call:
```matlab
Drive_Motor(UnifiedCmd.base.Vx, 0, UnifiedCmd.base.Wz);  % firmware ramp + PI
```

---

## 2) High‑Level Dataflow

```
Holistic (GIK, 9‑DOF)         Staged‑B (chassis‑only path)
    |                                  |
    v                                  v
[PPR base poses/vels]            [planner path + follower (PP/TEB)]
          \                            /
           \                          /
            ---> Common SE(2) reference [vxW, vyW, wRef]
                             |
                     world→body rotation
                             |
               Nonholonomic projection + heading law
                             |
               Yaw-feasibility gate + speed clamp
                             |
           UnifiedCmd.base = (Vx, Vy=0, Wz)  →  Drive_Motor
```

- **Staged‑C (9‑DOF)**: same as **Holistic** branch; fill `arm.qdot`.

---

## 3) Holistic / Stage‑C (GIK) → (Vx, Wz)

**Input:** time‑stamped base pose (or rates) from the GIK: \([x, y, \theta](t)\).

**Per‑tick steps:**
1) **Differentiate** with real `dt` (unwrap \(\theta\)):
   ```matlab
   dt   = max(1e-3, t(k) - t(k-1));
   dth  = wrapToPi(theta(k) - theta(k-1));
   vxW  = (x(k) - x(k-1)) / dt;
   vyW  = (y(k) - y(k-1)) / dt;
   wRef = dth / dt;
   ```
2) **World → body** using current yaw \(\theta\) (estimated from odom/MCL/SLAM):
   ```matlab
   Rwb = [ cos(theta)  sin(theta);
          -sin(theta)  cos(theta) ];
   vB = Rwb * [vxW; vyW];    % [vxBody; vyBody]
   ```
3) **Heading regulation (lightweight):**
   ```matlab
   phi_d = atan2(vyW, vxW);
   epsi  = wrapToPi(phi_d - theta);
   Vx = sign(vB(1)) * min(abs(vB(1)), Vx_max) * cos(epsi);
   Wz = yawKp * epsi + yawKff * wRef;
   ```
4) **Feasibility:** apply yaw gate and speed clamp; emit `(Vx, 0, Wz)` as `UnifiedCmd.base`.

> Use a small low‑pass or Savitzky–Golay filter on the differentiated velocities if needed.

---

## 4) Staged‑B (Chassis‑Only) → (Vx, Wz)

**Global planner:**  
- Default **A* Grid** on inflated `occupancyMap` (deterministic, fast).  
- **Hybrid A*** if you require curvature/heading along the route (e.g., docking, aesthetics).

**Local follower (choose one):**
- **Pure Pursuit (PP)** — simplest, outputs \((v,\omega)\) from path. Great for A* Grid; works for Hybrid A* if reverse is not required.
- **TEB** — local optimizer that follows the global path **and** avoids local obstacles; supports reverse, turning radius, safety margin.

**Then:** Take follower output `(v, w)` → **yaw gate + clamps** → `(Vx, 0, Wz)` → `Drive_Motor`.

**Notes**
- If Hybrid A* creates **reverse segments**, PP needs segmentation and negative \(v\) on those segments, or use TEB.
- For A* Grid, add a brief **goal‑heading alignment** at the end if docking orientation matters.

---

## 5) Reusable Core Function (drop‑in)

```matlab
function [cmd, st] = unifiedChassisCtrl(mode, inRef, estPose, st, P)
% mode: "holistic"|"staged-B"|"staged-C"
% inRef:
%   holistic/C: struct('x',x,'y',y,'theta',th,'t',t,'arm_qdot',qdot)   % poses/rates
%   staged-B  : struct('v',v,'w',w,'t',t)                              % follower output
% estPose: [x y theta]  (from odom/MCL/SLAM)
% st:      persistent state with .prev  (for holistic/C differentiation)
% P:       parameters: track, Vwheel_max, Vx_max, W_max, yawKp, yawKff

switch mode
  case {"holistic","staged-C"}
    if isempty(st) || ~isfield(st,'prev') || isempty(st.prev), st.prev = inRef; end
    dt   = max(1e-3, inRef.t - st.prev.t);
    dth  = wrapToPi(inRef.theta - st.prev.theta);
    vxW  = (inRef.x - st.prev.x)/dt;
    vyW  = (inRef.y - st.prev.y)/dt;
    wRef = dth/dt;
    st.prev = inRef;

    th  = estPose(3);
    Rwb = [cos(th) sin(th); -sin(th) cos(th)];
    vB  = Rwb * [vxW; vyW];
    phi_d = atan2(vyW, vxW);  epsi = wrapToPi(phi_d - th);

    Vx = sign(vB(1)) * min(abs(vB(1)), P.Vx_max) * cos(epsi);
    Wz = P.yawKp * epsi + P.yawKff * wRef;

  case "staged-B"
    Vx = inRef.v;    Wz = inRef.w;

  otherwise
    error("unifiedChassisCtrl: unknown mode");
end

% Yaw feasibility from wheel limit + cap
WcapWheels = 2 * max(0, (P.Vwheel_max - abs(Vx))) / P.track;
Wcap       = min(P.W_max, WcapWheels);
Wz = max(-Wcap, min(Wcap, Wz));
Vx = max(-P.Vx_max, min(P.Vx_max, Vx));

% Emit unified message
cmd.mode       = mode;
cmd.time       = inRef.t;
cmd.base.Vx    = Vx;
cmd.base.Vy    = 0;
cmd.base.Wz    = Wz;
cmd.arm.qdot   = [];
if mode=="staged-C" || mode=="holistic"
    if isfield(inRef,'arm_qdot'), cmd.arm.qdot = inRef.arm_qdot; end
end
end
```

---

## 6) Hooks for Staged‑B Followers

**Pure Pursuit**
```matlab
params = gik9dof.control.loadChassisProfile("wide_track");
pp = gik9dof.control.purePursuitFollower(pathStates, ...
    'ChassisParams', params, ...
    'ControllerMode', "blended", ...
    'SampleTime', 0.1, ...
    'LookaheadBase', 0.60, ...
    'LookaheadVelGain', 0.30, ...
    'GoalTolerance', 0.10, ...
    'ReverseEnabled', false);

dt = 0.1;
[v, w, status] = pp.step(currentPose, dt);
refB = struct('v', v, 'w', w, 't', now_t, 'pose', currentPose);
[cmd, st] = gik9dof.control.unifiedChassisCtrl("staged-B", refB, currentPose, st, Params);
Drive_Motor(cmd.base.Vx, 0, cmd.base.Wz);
if status.isFinished
    transitionToStageC();
end
```

**TEB**
```matlab
% teb: controllerTEB object with ref path + rolling local map
[velCmds, tStamps] = step(teb, currentPose, [vPrev wPrev]);
[v, w] = velocityCommand(velCmds, tStamps, now_t);
inputRef = struct('v', v, 'w', w, 't', now_t);
[cmd, st] = unifiedChassisCtrl("staged-B", inputRef, currentPose, st, Params);
Drive_Motor(cmd.base.Vx, 0, cmd.base.Wz);
```

The Stage‑B log now records `pathStates` populated from the Hybrid A* plan,
making it trivial to replay or drive the follower offline via
`gik9dof.control.purePursuitFollower` (with the same YAML-loaded profile) and the unified chassis interface.

---

## 7) Parameters (seed values)

Canonical presets now live in `config/chassis_profiles.yaml`. Load them in
MATLAB (and later in C++ codegen) with

```matlab
params = gik9dof.control.loadChassisProfile("wide_track");
```

| Field | `wide_track` | `compact_track` | Notes |
|---|---:|---:|---|
| `track` (m) | 0.573 | 0.329 | wheel separation |
| `vx_max` (m/s) | 1.5 | 1.0 | forward clamp (firmware cap) |
| `vx_min` (m/s) | –0.4 | –0.3 | reverse clamp (set 0 to disable) |
| `wz_max` (rad/s) | 2.5 | 2.8 | yaw clamp before wheel limit |
| `wheel_speed_max` (m/s) | 3.3 | 3.3 | keep <3.5 to match firmware |
| `accel_limit` / `decel_limit` (m/s²) | 1.2 / 1.8 | 1.0 / 1.4 | trapezoidal profile |
| `lookahead_base` (m) | 0.60 | 0.45 | tuned for 1.5 m/s / 1.0 m/s |
| `heading_kp, kd` | 1.2, 0.1 | 1.4, 0.12 | blended controller gains |
| `curvature_slowdown.vx_reduction` | 0.60 | 0.55 | scale when |κ| > threshold |

Override any field at runtime by supplying `ChassisOverrides` when calling
`trackReferenceTrajectory` / `runStagedTrajectory`.

---

## 8) Main Loop Skeleton

```matlab
while true
    estPose = getPose(); now_t = getTime();

    switch runStage
      case "B"   % chassis only (staged‑B)
        [v, w] = follower(estPose);                         % PP or TEB
        inputRef = struct('v', v, 'w', w, 't', now_t);
        [cmd, st] = unifiedChassisCtrl("staged-B", inputRef, estPose, st, Params);

      case "C"   % 9‑DOF whole‑body (staged‑C)  — same branch as holistic
        baseRef = nextGIKBaseSample(now_t);  % struct with x,y,theta,t,(arm qdot)
        [cmd, st] = unifiedChassisCtrl("staged-C", baseRef, estPose, st, Params);

      case "holistic"  % 9‑DOF (original holistic)
        baseRef = nextGIKBaseSample(now_t);
        [cmd, st] = unifiedChassisCtrl("holistic", baseRef, estPose, st, Params);
    end

    Drive_Motor(cmd.base.Vx, 0, cmd.base.Wz);              % firmware pipeline
    if ~isempty(cmd.arm.qdot), sendArm(cmd.arm.qdot); end  % optional
end
```

---

## 9) Testing & Guardrails

- **Real dt & unwrap:** never hard‑code dt; use timestamps and unwrap \(\theta\) before differencing.
- **Filter diff noise:** small LP or Savitzky–Golay on \([vx_W, vy_W]\).
- **Reverse handling:** PP assumes forward; for Hybrid A* reverse segments, segment the path with negative \(v\), or switch to TEB.
- **Docking heading:** if using A* Grid, add a short in‑place heading alignment at the goal when orientation matters.
- **Local avoidance:** blend VFH/TEB steering with the follower’s \(\omega\) **before** applying the yaw gate.
- **Logging:** record pre/post‑gate (Vx, Wz), the wheel‑limit \(Wz_{\max}(Vx)\), and PI saturation events to tune safely.

---

## 10) Minimal Helper: Yaw‑Gate

```matlab
function Wz = clampYawByWheelLimit(Vx, Wz, track, Vwheel_max, Wmax)
WcapWheels = 2 * max(0, (Vwheel_max - abs(Vx))) / track;
Wcap       = min(Wmax, WcapWheels);
Wz         = max(-Wcap, min(Wcap, Wz));
end
```

---

### TL;DR
- **One controller** emits **(Vx, 0, Wz)** no matter if the source is **Holistic (GIK)** or **Staged (B/C)**.
- **Holistic/C:** differentiate GIK base, rotate to body, heading law → yaw gate → `Drive_Motor`.
- **Staged‑B:** A* Grid (default) or Hybrid A* (if curvature/heading needed) + **PP (start)** or **TEB (when needed)** → yaw gate → `Drive_Motor`.
- Keep wheel‑level behaviors (ramp/clamps/PI) **inside firmware**; your unified layer focuses on safe, feasible chassis velocities.
- Implementation hooks now live under `gik9dof.control.*` (`unifiedChassisCtrl`, `purePursuitFollower`, `clampYawByWheelLimit`, `defaultUnifiedParams`).
- Use `run_environment_compare.m` to regenerate holistic vs staged traces with the shared environment, and `unified_chassis_replay` to preview the unified commands emitted from a saved log.
