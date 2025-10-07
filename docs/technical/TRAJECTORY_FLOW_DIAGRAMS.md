# Trajectory Tracking Flow - Visual Diagrams

**Quick Reference**: [`TRAJECTORY_COMPLETION_QUICKREF.md`](TRAJECTORY_COMPLETION_QUICKREF.md)  
**Full Analysis**: [`TRAJECTORY_COMPLETION_BEHAVIOR.md`](TRAJECTORY_COMPLETION_BEHAVIOR.md)

---

## Current Implementation Flow

```
╔═══════════════════════════════════════════════════════════════╗
║            TRAJECTORY TRACKING - CURRENT BEHAVIOR             ║
╚═══════════════════════════════════════════════════════════════╝

┌─────────────────────────────────────────────────────────────┐
│  RECEIVE TRAJECTORY                                         │
│  /gik9dof/target_trajectory                                 │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
        ┌────────────────────────────────┐
        │  trajectoryCallback()          │
        │  • Store current_trajectory_   │
        │  • Store sequence_id           │
        │  • Log waypoint count          │
        └────────────────┬───────────────┘
                         │
                         ▼
        ╔════════════════════════════════╗
        ║  CONTROL LOOP (10 Hz)          ║
        ╚════════════════════════════════╝
                         │
                         ▼
        ┌────────────────────────────────┐
        │  Check Robot State             │
        │  • arm_state_received_?        │
        │  • base_state_received_?       │
        └────────────────┬───────────────┘
                         │
                ┌────────┴────────┐
                │                 │
        ┌───────▼──────┐   ┌──────▼───────┐
        │  NOT READY   │   │    READY     │
        │  Return      │   │  Continue    │
        └──────────────┘   └──────┬───────┘
                                  │
                                  ▼
        ┌─────────────────────────────────────┐
        │  Get Target Pose                    │
        │  ⚠️  ALWAYS: waypoints[0].pose      │
        │  ❌  NEVER: waypoints[1..N-1]       │
        └────────────────┬────────────────────┘
                         │
            ┌────────────┴──────────┐
            │                       │
    ┌───────▼────────┐      ┌───────▼──────────┐
    │  NO TRAJECTORY │      │  HAS TRAJECTORY  │
    │  Log warning   │      │  Use waypoints[0]│
    │  Return        │      │  Continue        │
    └────────────────┘      └───────┬──────────┘
                                    │
                                    ▼
        ┌────────────────────────────────────┐
        │  Execute Control Mode              │
        │  if (STAGED):                      │
        │    executeStagedControl(target)    │
        │  else (HOLISTIC):                  │
        │    executeHolisticControl(target)  │
        └────────────────┬───────────────────┘
                         │
                         ▼
        ┌────────────────────────────────────┐
        │  Solve IK & Publish                │
        │  • solveIK(target_pose)            │
        │  • publishJointCommand()           │
        │  • publishBaseCommand()            │
        └────────────────┬───────────────────┘
                         │
                         ▼
        ┌────────────────────────────────────┐
        │  Wait 100ms                        │
        │  ❌ NO goal-reached check          │
        │  ❌ NO waypoint advancement        │
        │  ❌ NO completion detection        │
        └────────────────┬───────────────────┘
                         │
                         │
                         └──────────┐
                                    │
                                    ▼
                    ╔═══════════════════════════╗
                    ║  REPEAT FOREVER           ║
                    ║  (Until new trajectory    ║
                    ║   or node shutdown)       ║
                    ╚═══════════════════════════╝
```

---

## Single Waypoint Trajectory Timeline

```
╔═══════════════════════════════════════════════════════════════╗
║         SINGLE WAYPOINT - WHAT ACTUALLY HAPPENS               ║
╚═══════════════════════════════════════════════════════════════╝

Input: trajectory.waypoints = [goal_pose]
       trajectory.is_final_segment = True  ← IGNORED


t = 0.0s  ┌──────────────────────────────────────────┐
          │  Receive Trajectory                      │
          │  • Store waypoints[0] = goal_pose        │
          │  • Store is_final_segment = True         │
          └──────────────────────────────────────────┘
                           │
t = 0.1s  ┌────────────────▼──────────────────────────┐
          │  Control Loop #1                          │
          │  • Get waypoints[0]                       │
          │  • Solve IK for goal_pose                 │
          │  • Publish commands (moving toward goal)  │
          └───────────────────────────────────────────┘
                           │
t = 0.2s  ┌────────────────▼──────────────────────────┐
          │  Control Loop #2                          │
          │  • Get waypoints[0]                       │
          │  • Solve IK for goal_pose                 │
          │  • Publish commands (moving toward goal)  │
          └───────────────────────────────────────────┘
                           │
          │                ▼
          │         (Robot moving...)
          │                │
          │                ▼
                           
t = 5.0s  ╔════════════════════════════════════════════╗
          ║  END-EFFECTOR REACHES GOAL                 ║
          ║  ✓ Position within IK tolerance            ║
          ║  ⚠️  No detection or stop logic            ║
          ╚════════════════════════════════════════════╝
                           │
t = 5.1s  ┌────────────────▼──────────────────────────┐
          │  Control Loop #51                         │
          │  • Get waypoints[0]  ← STILL FIRST!       │
          │  • Solve IK for goal_pose                 │
          │  • Publish commands (holding position)    │
          └───────────────────────────────────────────┘
                           │
t = 5.2s  ┌────────────────▼──────────────────────────┐
          │  Control Loop #52                         │
          │  • Get waypoints[0]  ← STILL FIRST!       │
          │  • Solve IK for goal_pose                 │
          │  • Publish commands (holding position)    │
          └───────────────────────────────────────────┘
                           │
                           │
                           ▼
          ╔════════════════════════════════════════════╗
          ║  CONTINUES FOREVER                         ║
          ║  • Solving IK every 100ms                  ║
          ║  • Publishing commands continuously        ║
          ║  • is_final_segment IGNORED                ║
          ║  • No stop condition                       ║
          ╚════════════════════════════════════════════╝
```

---

## Multi-Waypoint Trajectory Timeline

```
╔═══════════════════════════════════════════════════════════════╗
║      MULTI-WAYPOINT - ONLY FIRST WAYPOINT TRACKED             ║
╚═══════════════════════════════════════════════════════════════╝

Input: trajectory.waypoints = [wp1, wp2, wp3, wp4, wp5]
       trajectory.timestamps = [0.0, 0.5, 1.0, 1.5, 2.0]
       trajectory.is_final_segment = True


t = 0.0s  ┌──────────────────────────────────────────┐
          │  Receive Trajectory                      │
          │  • waypoints[0] = wp1                    │
          │  • waypoints[1] = wp2  ← NEVER USED      │
          │  • waypoints[2] = wp3  ← NEVER USED      │
          │  • waypoints[3] = wp4  ← NEVER USED      │
          │  • waypoints[4] = wp5  ← NEVER USED      │
          │  • timestamps[] ← IGNORED                │
          └──────────────────────────────────────────┘
                           │
t = 0.1s  ┌────────────────▼──────────────────────────┐
          │  Control Loop #1                          │
          │  • Get waypoints[0] → wp1                 │
          │  • Solve IK for wp1                       │
          │  • Publish commands (moving toward wp1)   │
          └───────────────────────────────────────────┘
                           │
          │                ▼
          │         (Robot moving to wp1...)
          │                │
          │                ▼
                           
t = 3.0s  ╔════════════════════════════════════════════╗
          ║  END-EFFECTOR REACHES WP1                  ║
          ║  ✓ At first waypoint                       ║
          ║  ❌ Should advance to wp2                  ║
          ║  ❌ No advancement logic                   ║
          ╚════════════════════════════════════════════╝
                           │
t = 3.1s  ┌────────────────▼──────────────────────────┐
          │  Control Loop #31                         │
          │  • Get waypoints[0] → wp1  ← STUCK!       │
          │  • Solve IK for wp1                       │
          │  • Publish commands (holding at wp1)      │
          └───────────────────────────────────────────┘
                           │
t = 3.2s  ┌────────────────▼──────────────────────────┐
          │  Control Loop #32                         │
          │  • Get waypoints[0] → wp1  ← STUCK!       │
          │  • wp2, wp3, wp4, wp5 NEVER tracked       │
          └───────────────────────────────────────────┘
                           │
                           ▼
          ╔════════════════════════════════════════════╗
          ║  STUCK AT WP1 FOREVER                      ║
          ║  • wp2 through wp5 NEVER executed          ║
          ║  • timestamps[] NEVER used                 ║
          ║  • Multi-waypoint trajectory fails         ║
          ╚════════════════════════════════════════════╝
```

---

## Desired Behavior (NOT IMPLEMENTED)

```
╔═══════════════════════════════════════════════════════════════╗
║         DESIRED BEHAVIOR - WHAT SHOULD HAPPEN                 ║
╚═══════════════════════════════════════════════════════════════╝

┌─────────────────────────────────────────────────────────────┐
│  RECEIVE TRAJECTORY                                         │
│  waypoints = [wp1, wp2, wp3, wp4, wp5]                      │
│  is_final_segment = True                                    │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
        ┌────────────────────────────────┐
        │  Initialize Tracking           │
        │  • waypoint_index = 0          │
        │  • trajectory_start_time = now │
        └────────────────┬───────────────┘
                         │
                         ▼
        ╔════════════════════════════════╗
        ║  CONTROL LOOP (10 Hz)          ║
        ╚════════════════════════════════╝
                         │
                         ▼
        ┌────────────────────────────────────┐
        │  Get Current Waypoint              │
        │  target = waypoints[waypoint_index]│
        └────────────────┬───────────────────┘
                         │
                         ▼
        ┌────────────────────────────────────┐
        │  Solve IK & Publish                │
        │  • solveIK(target)                 │
        │  • publishCommands()               │
        └────────────────┬───────────────────┘
                         │
                         ▼
        ┌─────────────────────────────────────┐
        │  Check if Waypoint Reached          │
        │  distance = ||EE_pos - target||     │
        └────────────────┬────────────────────┘
                         │
            ┌────────────┴──────────┐
            │                       │
    ┌───────▼──────┐       ┌────────▼─────────┐
    │ NOT REACHED  │       │    REACHED       │
    │ distance > ε │       │   distance < ε   │
    │ Continue     │       │   Advance!       │
    └──────────────┘       └────────┬─────────┘
            │                       │
            │                       ▼
            │          ┌─────────────────────────┐
            │          │  waypoint_index++       │
            │          │  Check completion       │
            │          └────────┬────────────────┘
            │                   │
            │      ┌────────────┴──────────┐
            │      │                       │
            │  ┌───▼────────┐     ┌────────▼──────────┐
            │  │ MORE WP's  │     │  FINAL WAYPOINT   │
            │  │ Continue   │     │  & is_final_seg   │
            │  └───┬────────┘     └────────┬──────────┘
            │      │                       │
            └──────┘                       ▼
                         ┌──────────────────────────────┐
                         │  Trajectory Complete!        │
                         │  • Publish completion status │
                         │  • Stop OR hold position     │
                         └──────────────────────────────┘
```

---

## Comparison: Current vs Desired

```
╔═══════════════════════════════════════════════════════════════╗
║                    SIDE-BY-SIDE COMPARISON                    ║
╚═══════════════════════════════════════════════════════════════╝

┌────────────────────────────┬──────────────────────────────────┐
│      CURRENT BEHAVIOR      │       DESIRED BEHAVIOR           │
├────────────────────────────┼──────────────────────────────────┤
│                            │                                  │
│  ┌──────────────────────┐  │  ┌────────────────────────────┐ │
│  │ Receive Trajectory   │  │  │ Receive Trajectory         │ │
│  │ waypoints[0..N-1]    │  │  │ waypoints[0..N-1]          │ │
│  └──────────┬───────────┘  │  └──────────┬─────────────────┘ │
│             │              │             │                   │
│             ▼              │             ▼                   │
│  ┌──────────────────────┐  │  ┌────────────────────────────┐ │
│  │ Control Loop         │  │  │ Control Loop               │ │
│  │ Get waypoints[0]     │  │  │ Get waypoints[index]       │ │
│  │                      │  │  │ (index increments)         │ │
│  └──────────┬───────────┘  │  └──────────┬─────────────────┘ │
│             │              │             │                   │
│             ▼              │             ▼                   │
│  ┌──────────────────────┐  │  ┌────────────────────────────┐ │
│  │ Solve IK & Publish   │  │  │ Solve IK & Publish         │ │
│  └──────────┬───────────┘  │  └──────────┬─────────────────┘ │
│             │              │             │                   │
│             ▼              │             ▼                   │
│  ┌──────────────────────┐  │  ┌────────────────────────────┐ │
│  │ ❌ No goal check     │  │  │ ✅ Check goal reached      │ │
│  │ ❌ No advancement    │  │  │ ✅ Advance if reached      │ │
│  └──────────┬───────────┘  │  └──────────┬─────────────────┘ │
│             │              │             │                   │
│             │              │      ┌──────┴──────┐            │
│             │              │      │             │            │
│             │              │   ┌──▼───┐   ┌─────▼────┐      │
│             │              │   │ More │   │ Complete │      │
│             │              │   │ WP's │   │   Stop   │      │
│             │              │   └──┬───┘   └──────────┘      │
│             │              │      │                          │
│             └──────┐       │      └───────┐                 │
│                    │       │              │                 │
│                    ▼       │              ▼                 │
│  ╔═════════════════════╗   │  ╔═══════════════════════════╗ │
│  ║ REPEAT FOREVER      ║   │  ║ STOP WHEN COMPLETE        ║ │
│  ║ Track waypoints[0]  ║   │  ║ All waypoints executed    ║ │
│  ║ Never advances      ║   │  ║ Proper completion         ║ │
│  ╚═════════════════════╝   │  ╚═══════════════════════════╝ │
│                            │                                  │
└────────────────────────────┴──────────────────────────────────┘
```

---

## Key Differences Table

| Aspect | Current | Desired |
|--------|---------|---------|
| **Waypoint Selection** | `waypoints[0]` always | `waypoints[index]` with advancement |
| **Goal Detection** | ❌ None | ✅ Distance-based check |
| **Waypoint Advancement** | ❌ Never | ✅ When goal reached |
| **Completion Detection** | ❌ None | ✅ `is_final_segment` + last waypoint |
| **Stop Condition** | Never (except new trajectory) | When trajectory complete |
| **Multi-Waypoint Support** | ❌ Only first waypoint | ✅ All waypoints tracked |
| **Time-Based Tracking** | ❌ `timestamps[]` ignored | ✅ Optional interpolation |
| **Completion Status** | ❌ No feedback | ✅ Publishes completion message |

---

## Message Field Usage

```
EndEffectorTrajectory Message:
┌─────────────────────────────────────────────────────────────┐
│  Field                  │  Current Usage  │  Should Be      │
├─────────────────────────┼─────────────────┼─────────────────┤
│  header                 │  ✅ Used        │  ✅ Used        │
│  waypoints[]            │  ⚠️ [0] only    │  ✅ All used    │
│  timestamps[]           │  ❌ Ignored     │  ✅ For interp  │
│  sequence_id            │  ✅ Used        │  ✅ Used        │
│  is_final_segment       │  ❌ IGNORED     │  ✅ CRITICAL    │
│  lookahead_time         │  ❌ Ignored     │  ✅ For planning│
│  max_velocity           │  ❌ Ignored     │  ✅ For limits  │
│  max_acceleration       │  ❌ Ignored     │  ✅ For limits  │
└─────────────────────────────────────────────────────────────┘
```

---

## Legend

```
╔════════════╗   Double-line box = Process/state
║  TITLE     ║
╚════════════╝

┌────────────┐   Single-line box = Action/function
│  Process   │
└────────────┘

✅ = Implemented / Correct
❌ = Not implemented / Missing
⚠️ = Partial / Warning

      │          Solid arrow = Control flow
      ▼

```

---

**See Also**:
- 📖 [Full Behavior Analysis](TRAJECTORY_COMPLETION_BEHAVIOR.md)
- 🚀 [Quick Reference](TRAJECTORY_COMPLETION_QUICKREF.md)
- 📊 [Session Summary](TRAJECTORY_ANALYSIS_SESSION_SUMMARY.md)
