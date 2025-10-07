# State Machine Visual Flow Diagrams

**Quick Reference**: [`STATE_MACHINE_QUICKREF.md`](STATE_MACHINE_QUICKREF.md)  
**Full Documentation**: [`STATE_MACHINE_ARCHITECTURE.md`](STATE_MACHINE_ARCHITECTURE.md)

---

## Complete System State Machine

```
                    ╔═══════════════════════════════════════╗
                    ║         SYSTEM INITIALIZATION         ║
                    ║   Parse YAML configuration file       ║
                    ╚═══════════════════════════════════════╝
                                      │
                                      ▼
                    ┌─────────────────────────────────────┐
                    │  control_mode: "holistic" | "staged"│
                    └─────────────┬───────────────────────┘
                                  │
                    ┌─────────────┴────────────────┐
                    │                              │
           ╔════════▼═════════╗         ╔═════════▼═════════╗
           ║  HOLISTIC MODE   ║         ║   STAGED MODE     ║
           ║  control_mode =  ║         ║   control_mode =  ║
           ║  "holistic"      ║         ║   "staged"        ║
           ╚════════╤═════════╝         ╚═════════╤═════════╝
                    │                             │
                    │                   ┌─────────▼──────────┐
                    │                   │     STAGE A        │
                    │                   │   Arm Ramp-Up      │
                    │                   │  (6-DOF, chassis   │
                    │                   │   frozen at init)  │
                    │                   └─────────┬──────────┘
                    │                             │
                    │                             │
                    │                    checkArmAtHome()
                    │                      returns TRUE
                    │                             │
                    │                             ▼
                    │                   ┌─────────────────────┐
                    │                   │     STAGE B         │
                    │                   │  Chassis Planning   │
                    │                   │  (Arm held at home) │
                    │                   └─────────┬───────────┘
                    │                             │
                    │                     Parse parameter:
                    │                  staged.stage_b.submode
                    │                             │
                    │            ┌────────────────┴─────────────────┐
                    │            │                                  │
                    │      ╔═════▼═══════╗              ╔═══════════▼═════╗
                    │      ║ SUBMODE B1  ║              ║   SUBMODE B2    ║
                    │      ║ "pure_      ║              ║ "gik_assisted"  ║
                    │      ║  hybrid_    ║              ║                 ║
                    │      ║  astar"     ║              ║  Hybrid A* +    ║
                    │      ║             ║              ║  GIK 3-DOF      ║
                    │      ║ Hybrid A* → ║              ║  refinement     ║
                    │      ║ Velocity    ║              ║                 ║
                    │      ╚═════╤═══════╝              ╚═══════════╤═════╝
                    │            │                                  │
                    │            └────────────────┬─────────────────┘
                    │                             │
                    │                stageBChassisReachedGoal()
                    │                      returns TRUE
                    │                             │
                    │                             ▼
                    │                   ┌─────────────────────┐
                    │                   │     STAGE C         │
                    │                   │  Whole-Body Tracking│
                    │                   │  (9-DOF GIK)        │
                    │                   │  = Holistic Control │
                    │                   └─────────┬───────────┘
                    │                             │
                    └─────────────────────────────┘
                                      │
                        ╔═════════════▼═════════════╗
                        ║   9-DOF GIK SOLVER        ║
                        ║   (MATLAB Codegen)        ║
                        ║   Solves for q (9-DOF)    ║
                        ╚═════════════╤═════════════╝
                                      │
                                      ▼
                        ┌──────────────────────────────┐
                        │  velocity_control_mode       │
                        │  0 | 1 | 2                   │
                        └─────┬────────────────────────┘
                              │
           ┌──────────────────┼──────────────────┐
           │                  │                  │
     ╔═════▼═════╗    ╔═══════▼══════╗   ╔══════▼══════╗
     ║  MODE 0   ║    ║   MODE 1     ║   ║   MODE 2    ║
     ║  Legacy   ║    ║   Heading    ║   ║ Pure Pursuit║
     ║  5-point  ║    ║  Controller  ║   ║ (RECOMMEND) ║
     ║   diff    ║    ║   P + FF     ║   ║  Lookahead  ║
     ╚═════╤═════╝    ╚═══════╤══════╝   ╚══════╤══════╝
           │                  │                  │
           └──────────────────┴──────────────────┘
                              │
                              ▼
                  ╔═══════════════════════╗
                  ║  WHEEL VELOCITIES     ║
                  ║  (vL, vR)             ║
                  ║  Published to robot   ║
                  ╚═══════════════════════╝
```

---

## Stage A Detailed Flow

```
╔══════════════════════════════════════════════════════╗
║              STAGE A: ARM RAMP-UP                    ║
╚══════════════════════════════════════════════════════╝
                        │
                        ▼
        ┌───────────────────────────────┐
        │  Initialize Stage A           │
        │  • Set target = arm home      │
        │  • Freeze chassis (vL=vR=0)   │
        │  • Start timeout timer        │
        └───────────────┬───────────────┘
                        │
                        ▼
        ╔═══════════════════════════════╗
        ║  Control Loop (10 Hz)         ║
        ╚═══════════════════════════════╝
                        │
                        ▼
        ┌───────────────────────────────┐
        │  Solve 6-DOF IK               │
        │  (Arm only, chassis fixed)    │
        └───────────────┬───────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │  Check arm position           │
        │  checkArmAtHome():            │
        │  • Compare joints 3-8         │
        │  • Tolerance: 0.1 rad         │
        │  • Target: [0,0,0,0,0,0]      │
        └───────────────┬───────────────┘
                        │
            ┌───────────┴────────────┐
            │                        │
    ┌───────▼───────┐        ┌───────▼────────┐
    │ NOT AT HOME   │        │   AT HOME      │
    │ (>0.1 rad)    │        │   (<0.1 rad)   │
    └───────┬───────┘        └───────┬────────┘
            │                        │
            │                        ▼
            │              ┌──────────────────┐
            │              │ Activate Stage B │
            │              │ Transition to B  │
            │              └──────────────────┘
            │                        │
            └────────────────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │  Check timeout                │
        │  (Default: 10 seconds)        │
        └───────────────┬───────────────┘
                        │
            ┌───────────┴────────────┐
            │                        │
    ┌───────▼────────┐      ┌────────▼────────┐
    │  NOT TIMEOUT   │      │   TIMEOUT       │
    │  Continue      │      │   Error/Abort   │
    └────────────────┘      └─────────────────┘
```

---

## Stage B Detailed Flow

```
╔══════════════════════════════════════════════════════╗
║           STAGE B: CHASSIS PLANNING                  ║
╚══════════════════════════════════════════════════════╝
                        │
                        ▼
        ┌───────────────────────────────┐
        │  stageBActivate()             │
        │  • Initialize planner         │
        │  • Set goal pose              │
        │  • Start timeout timer        │
        └───────────────┬───────────────┘
                        │
                        ▼
        ╔═══════════════════════════════╗
        ║  Control Loop (10 Hz)         ║
        ╚═══════════════════════════════╝
                        │
                        ▼
        ┌───────────────────────────────┐
        │  stageBExecuteStep()          │
        │  • Plan path (Hybrid A*)      │
        │  • Track path (velocity ctrl) │
        │  • Hold arm at home           │
        └───────────────┬───────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │  stageBChassisReachedGoal()   │
        │  • Check XY distance          │
        │  • Check heading error        │
        └───────────────┬───────────────┘
                        │
            ┌───────────┴────────────┐
            │                        │
    ┌───────▼────────┐       ┌───────▼─────────┐
    │  NOT AT GOAL   │       │   AT GOAL       │
    │  (>tolerance)  │       │   (<tolerance)  │
    └───────┬────────┘       └───────┬─────────┘
            │                        │
            │                        ▼
            │              ┌─────────────────────┐
            │              │ stageBDeactivate()  │
            │              │ Transition to       │
            │              │ STAGE C             │
            │              └─────────────────────┘
            │                        │
            └────────────────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │  Check timeout                │
        │  (Default: 30 seconds)        │
        └───────────────┬───────────────┘
                        │
            ┌───────────┴────────────┐
            │                        │
    ┌───────▼────────┐      ┌────────▼────────┐
    │  NOT TIMEOUT   │      │   TIMEOUT       │
    │  Continue      │      │   Replan/Abort  │
    └────────────────┘      └─────────────────┘
```

---

## Stage B Submode Decision

```
╔══════════════════════════════════════════════════════╗
║      STAGE B SUBMODE SELECTION                       ║
║      staged.stage_b.submode parameter                ║
╚══════════════════════════════════════════════════════╝
                        │
            ┌───────────┴────────────┐
            │                        │
    ┌───────▼─────────┐      ┌───────▼──────────┐
    │  SUBMODE B1     │      │  SUBMODE B2      │
    │ "pure_hybrid_   │      │ "gik_assisted"   │
    │  astar"         │      │                  │
    └───────┬─────────┘      └───────┬──────────┘
            │                        │
            ▼                        ▼
╔═══════════════════════╗  ╔═══════════════════════╗
║   B1 PIPELINE         ║  ║   B2 PIPELINE         ║
╠═══════════════════════╣  ╠═══════════════════════╣
║                       ║  ║                       ║
║  ┌─────────────────┐ ║  ║  ┌─────────────────┐ ║
║  │ Hybrid A*       │ ║  ║  │ Hybrid A*       │ ║
║  │ Path Planner    │ ║  ║  │ Coarse Planner  │ ║
║  │                 │ ║  ║  │                 │ ║
║  │ • Grid-based    │ ║  ║  │ • Grid-based    │ ║
║  │ • A* search     │ ║  ║  │ • Obstacles     │ ║
║  │ • Obstacle map  │ ║  ║  │                 │ ║
║  └────────┬────────┘ ║  ║  └────────┬────────┘ ║
║           │          ║  ║           │          ║
║           ▼          ║  ║           ▼          ║
║  Path Waypoints     ║  ║  Coarse Waypoints   ║
║           │          ║  ║           │          ║
║           │          ║  ║           ▼          ║
║           │          ║  ║  ┌─────────────────┐ ║
║           │          ║  ║  │ GIK 3-DOF       │ ║
║           │          ║  ║  │ Solver          │ ║
║           │          ║  ║  │                 │ ║
║           │          ║  ║  │ • Chassis-only  │ ║
║           │          ║  ║  │ • Constraints   │ ║
║           │          ║  ║  │ • Refinement    │ ║
║           │          ║  ║  └────────┬────────┘ ║
║           │          ║  ║           │          ║
║           │          ║  ║           ▼          ║
║           │          ║  ║  Refined Config     ║
║           │          ║  ║  q (x, y, θ)        ║
║           │          ║  ║           │          ║
║           ▼          ║  ║           ▼          ║
║  ┌─────────────────┐ ║  ║  ┌─────────────────┐ ║
║  │ Velocity        │ ║  ║  │ Velocity        │ ║
║  │ Controller      │ ║  ║  │ Controller      │ ║
║  │                 │ ║  ║  │                 │ ║
║  │ Mode: 0 | 1 | 2 │ ║  ║  │ Mode: 0 | 1 | 2 │ ║
║  └────────┬────────┘ ║  ║  └────────┬────────┘ ║
║           │          ║  ║           │          ║
╚═══════════╪══════════╝  ╚═══════════╪══════════╝
            │                        │
            └────────┬───────────────┘
                     │
                     ▼
          ╔════════════════════╗
          ║ Wheel Velocities   ║
          ║ (vL, vR)           ║
          ╚════════════════════╝
```

---

## Velocity Controller Selection

```
╔══════════════════════════════════════════════════════╗
║      VELOCITY CONTROLLER SELECTION                   ║
║      velocity_control_mode: 0 | 1 | 2                ║
║      (Shared by ALL control modes)                   ║
╚══════════════════════════════════════════════════════╝
                        │
        ┌───────────────┼───────────────┐
        │               │               │
┌───────▼──────┐ ┌──────▼──────┐ ┌──────▼────────┐
│   MODE 0     │ │   MODE 1    │ │   MODE 2      │
│   Legacy     │ │  Heading    │ │ Pure Pursuit  │
│   5-Point    │ │ Controller  │ │ (RECOMMENDED) │
│   Diff       │ │             │ │               │
└───────┬──────┘ └──────┬──────┘ └──────┬────────┘
        │               │               │
        ▼               ▼               ▼
╔═══════════════╗ ╔═════════════╗ ╔═══════════════╗
║ Numerical     ║ ║ Proportional║ ║ Geometric     ║
║ Diff:         ║ ║ Control:    ║ ║ Lookahead:    ║
║               ║ ║             ║ ║               ║
║ dq/dt ≈       ║ ║ vx = vx_cmd ║ ║ L = base +    ║
║ f(q[k-2:k+2]) ║ ║ wz = Kp*err ║ ║     K_v*|v|   ║
║               ║ ║    + Kff*wz ║ ║               ║
║               ║ ║             ║ ║ κ = 2sin(α)/L ║
║ ❌ Noisy      ║ ║ ✅ Simple   ║ ║ ω = v * κ     ║
║ ❌ Sensitive  ║ ║ ⚠️ No path  ║ ║               ║
║               ║ ║    tracking ║ ║ ✅ Smooth     ║
║               ║ ║             ║ ║ ✅ Adaptive   ║
║               ║ ║             ║ ║ ✅ Bidirect.  ║
╚═══════════════╝ ╚═════════════╝ ╚═══════════════╝
        │               │               │
        └───────────────┴───────────────┘
                        │
                        ▼
        ┌───────────────────────────────┐
        │  Differential Drive Kinematics│
        │  vL = (2*vx - w*track)/2      │
        │  vR = (2*vx + w*track)/2      │
        └───────────────┬───────────────┘
                        │
                        ▼
              ╔═════════════════╗
              ║ Wheel Velocities║
              ║ (vL, vR)        ║
              ╚═════════════════╝
```

---

## Parameter Configuration Flow

```
╔═══════════════════════════════════════════════════════╗
║  gik9dof_solver.yaml (Configuration File)             ║
╚═══════════════════════════════════════════════════════╝
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ▼               ▼               ▼
┌───────────────┐ ┌─────────────┐ ┌──────────────────┐
│ control_mode  │ │ velocity_   │ │ staged.stage_b.  │
│ "holistic" |  │ │ control_    │ │ submode          │
│ "staged"      │ │ mode        │ │ "pure_hybrid_    │
│               │ │ 0 | 1 | 2   │ │  astar" |        │
│               │ │             │ │ "gik_assisted"   │
└───────┬───────┘ └──────┬──────┘ └────────┬─────────┘
        │                │                 │
        ▼                ▼                 ▼
╔═══════════════════════════════════════════════════════╗
║  GIK9DOFSolverNode::constructor()                     ║
║  • this->get_parameter("control_mode")                ║
║  • this->get_parameter("velocity_control_mode")       ║
║  • this->get_parameter("staged.stage_b.submode")      ║
╚═══════════════════════════════════════════════════════╝
        │
        ▼
┌───────────────────────────────────────────────────────┐
│  Initialization:                                      │
│  • control_mode_ = (str == "staged") ? STAGED         │
│                                      : HOLISTIC       │
│  • current_stage_ = STAGE_A  (if STAGED)              │
│  • velocity_control_mode_ = int value (0/1/2)         │
└───────────────────────────────────────────────────────┘
        │
        ▼
╔═══════════════════════════════════════════════════════╗
║  Runtime Control Loop (10 Hz)                         ║
║  • Uses control_mode_ to switch behavior              ║
║  • Uses current_stage_ to track progress              ║
║  • Uses velocity_control_mode_ for path tracking      ║
╚═══════════════════════════════════════════════════════╝
```

---

## Legend

```
╔════════════╗   Double-line box = Major state/component
║  TITLE     ║
╚════════════╝

┌────────────┐   Single-line box = Function/process
│  Process   │
└────────────┘

      │          Solid arrow = Control flow
      ▼

     ──▶         Dashed arrow = Data flow

```

---

**See Also**:
- 📖 [Full State Machine Documentation](STATE_MACHINE_ARCHITECTURE.md)
- 🚀 [Quick Reference](STATE_MACHINE_QUICKREF.md)
- ⚙️ [Configuration File](../../ros2/gik9dof_solver/config/gik9dof_solver.yaml)
