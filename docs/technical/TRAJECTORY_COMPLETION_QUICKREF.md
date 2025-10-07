# Trajectory Completion Behavior - Quick Reference

**Full Analysis**: [`TRAJECTORY_COMPLETION_BEHAVIOR.md`](TRAJECTORY_COMPLETION_BEHAVIOR.md)

---

## ⚠️ Critical Finding

**The system DOES NOT stop when reaching the end of the trajectory!**

---

## Current Behavior Summary

```
┌────────────────────────────────────────────────────┐
│  What Actually Happens                             │
├────────────────────────────────────────────────────┤
│                                                    │
│  1. Receive trajectory with N waypoints            │
│     → Stores entire trajectory                     │
│                                                    │
│  2. Control loop (10 Hz)                           │
│     → ALWAYS tracks waypoints[0]                   │
│     → Never advances to waypoints[1..N-1]          │
│                                                    │
│  3. End-effector reaches target                    │
│     → No detection                                 │
│     → Continues tracking waypoints[0]              │
│                                                    │
│  4. Trajectory "completes"                         │
│     → No stop condition                            │
│     → Continues publishing commands indefinitely   │
│                                                    │
│  5. Only stops when:                               │
│     → New trajectory received                      │
│     → Node shutdown                                │
│                                                    │
└────────────────────────────────────────────────────┘
```

---

## Code Evidence

### Always Uses First Waypoint

**File**: `gik9dof_solver_node.cpp` (line 344)

```cpp
// In controlLoop()
if (current_trajectory_ && !current_trajectory_->waypoints.empty()) {
    // ⚠️ CRITICAL: Always uses waypoints[0]
    target_pose = current_trajectory_->waypoints[0].pose;
    has_target = true;
}
```

### No Goal-Reached Checking

**File**: `gik9dof_solver_node.cpp` (lines 483-500)

```cpp
void GIK9DOFSolverNode::executeHolisticControl(const geometry_msgs::msg::Pose& target_pose)
{
    // Solve IK
    bool solve_success = solveIK(target_pose);
    
    if (solve_success) {
        publishJointCommand();
        publishBaseCommand();  // ← Always publishes, no goal check
    }
    // ❌ No check if end-effector reached target
    // ❌ No stop condition
}
```

---

## Missing Features

| Feature | Status | Impact |
|---------|--------|--------|
| **Waypoint advancement** | ❌ Missing | Only tracks first waypoint |
| **Goal-reached detection** | ❌ Missing | Never detects completion |
| **`is_final_segment` handling** | ❌ Ignored | No stop on trajectory end |
| **Time-based interpolation** | ❌ Missing | `timestamps[]` unused |
| **Velocity limits** | ❌ Missing | `max_velocity` unused |

---

## Scenarios

### Single Waypoint Trajectory

```python
# Send single waypoint
trajectory.waypoints = [target_pose]
trajectory.is_final_segment = True
```

**What Happens**:
```
t=0.0s:  Receive trajectory
t=0.1s:  Track waypoints[0] → Publish commands
t=0.2s:  Track waypoints[0] → Publish commands
...
t=5.0s:  End-effector reaches target
t=5.1s:  Track waypoints[0] → Publish commands  ← STILL TRACKING!
t=5.2s:  Track waypoints[0] → Publish commands  ← STILL TRACKING!
...
∞:       Continues forever
```

**Result**: ⚠️ Robot holds position but continuously publishes commands

---

### Multi-Waypoint Trajectory

```python
# Send 5 waypoints
trajectory.waypoints = [pose1, pose2, pose3, pose4, pose5]
trajectory.is_final_segment = True
```

**What Happens**:
```
t=0.0s:  Receive trajectory
t=0.1s:  Track waypoints[0] (pose1) → Publish
...
t=3.0s:  End-effector reaches pose1
t=3.1s:  Track waypoints[0] (pose1) → Publish  ← STUCK!
...
∞:       Never advances to pose2, pose3, pose4, pose5
```

**Result**: ⚠️ Only tracks first waypoint, ignores rest

---

## Quick Fixes

### Fix 1: External Trajectory Manager (Recommended, No Code Change)

Send single-waypoint trajectories sequentially:

```python
# trajectory_manager.py
for waypoint in waypoints:
    # Send one waypoint at a time
    traj = EndEffectorTrajectory()
    traj.waypoints = [waypoint]
    pub.publish(traj)
    
    # Wait for robot to reach
    wait_for_position(waypoint)

# Send stop command (empty trajectory)
stop_traj = EndEffectorTrajectory()
stop_traj.waypoints = []
pub.publish(stop_traj)
```

---

### Fix 2: Minimal Code Patch (Code Change Required)

Add simple waypoint advancement:

```cpp
// In controlLoop() - replace lines 341-348
{
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (current_trajectory_ && !current_trajectory_->waypoints.empty()) {
        
        // NEW: Track waypoint index
        static size_t wp_idx = 0;
        static uint32_t last_seq = 0;
        
        // Reset index on new trajectory
        if (trajectory_sequence_ != last_seq) {
            wp_idx = 0;
            last_seq = trajectory_sequence_;
        }
        
        // Check if close enough to advance
        if (wp_idx < current_trajectory_->waypoints.size()) {
            target_pose = current_trajectory_->waypoints[wp_idx].pose;
            has_target = true;
            
            // Advance if within 5cm
            if (endEffectorDistance(target_pose) < 0.05) {
                wp_idx++;
                
                // Check completion
                if (wp_idx >= current_trajectory_->waypoints.size()) {
                    if (current_trajectory_->is_final_segment) {
                        RCLCPP_INFO(this->get_logger(), "Trajectory complete!");
                        has_target = false;  // Stop
                    } else {
                        wp_idx--;  // Hold last waypoint
                    }
                }
            }
        }
    }
}

// Add helper function
double GIK9DOFSolverNode::endEffectorDistance(const geometry_msgs::msg::Pose& target)
{
    // Compute current EE position from FK
    Eigen::Isometry3d ee_pose = computeForwardKinematics(current_config_);
    Eigen::Vector3d current_pos = ee_pose.translation();
    Eigen::Vector3d target_pos(target.position.x, target.position.y, target.position.z);
    return (current_pos - target_pos).norm();
}
```

---

## Configuration Parameters (Recommended to Add)

```yaml
# Add to gik9dof_solver.yaml
trajectory_tracking:
  waypoint_tolerance: 0.05      # meters (distance to advance waypoint)
  goal_tolerance: 0.02          # meters (final goal tolerance)
  stop_on_completion: true      # Stop when is_final_segment reached
  interpolation_mode: "discrete" # "discrete" | "time_based"
```

---

## Testing Checklist

When implementing fixes, test:

- [ ] **Single waypoint**: Reaches target and stops
- [ ] **Multiple waypoints**: Advances through all waypoints
- [ ] **`is_final_segment=true`**: Stops at end
- [ ] **`is_final_segment=false`**: Holds last waypoint
- [ ] **New trajectory**: Resets waypoint index
- [ ] **Empty trajectory**: Does nothing (current behavior)
- [ ] **Goal tolerance**: Advances at correct distance
- [ ] **Completion status**: Publishes completion message

---

## Related Issues

| Issue | Impact | Priority |
|-------|--------|----------|
| No waypoint advancement | Only tracks first waypoint | 🔴 High |
| No goal detection | Never detects completion | 🔴 High |
| `is_final_segment` ignored | No stop at trajectory end | 🟡 Medium |
| `timestamps[]` unused | No time-based tracking | 🟢 Low |
| `max_velocity` unused | No velocity limiting | 🟢 Low |

---

## Summary

**Problem**: System continuously tracks the first waypoint of a trajectory indefinitely, never advancing or stopping.

**Root Cause**: `controlLoop()` always uses `waypoints[0]`, with no advancement logic or goal-reached checking.

**Impact**: 
- Multi-waypoint trajectories don't work
- No automatic stop at trajectory end
- Continuous command publishing even when at goal

**Recommended Solution**: External trajectory manager that sends single-waypoint trajectories sequentially.

**Alternative**: Add waypoint advancement logic to `controlLoop()`.

---

**See**: [`TRAJECTORY_COMPLETION_BEHAVIOR.md`](TRAJECTORY_COMPLETION_BEHAVIOR.md) for complete analysis and implementation details.
