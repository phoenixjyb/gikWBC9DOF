# Planner Namespace Fix - October 9, 2025

## Problem
After deploying new Linux MATLAB planner to Orin, build failed with:
1. **Type redefinition errors**: Duplicate `struct0_T`, `struct1_T`, `struct2_T`
2. **Function name mismatch**: `b_gik9dof_planHybridAStarCodegen` not found
3. **Include conflicts**: Old `gik9dof_planHybridAStarCodegen_types.h` vs new `planHybridAStarCodegen_types.h`

## Root Cause
Linux MATLAB R2024a generates cleaner names without the `gik9dof_` prefix:
- Old (Windows): `gik9dof_planHybridAStarCodegen_types.h`, `b_gik9dof_planHybridAStarCodegen()`
- New (Linux): `planHybridAStarCodegen_types.h`, `b_planHybridAStarCodegen()`

Stage B controller code was still using old names.

## ✅ Solution Applied

### Files Fixed:
1. `ros2/gik9dof_solver/src/stage_b_chassis_plan.hpp` (line 42)
   ```cpp
   // OLD: #include "gik9dof_planHybridAStarCodegen_types.h"
   // NEW:
   #include "planHybridAStarCodegen_types.h"
   ```

2. `ros2/gik9dof_solver/src/stage_b_chassis_plan.cpp` (line 276)
   ```cpp
   // OLD: planner_->b_gik9dof_planHybridAStarCodegen(&start_state, &goal_state, ...
   // NEW:
   planner_->b_planHybridAStarCodegen(&start_state, &goal_state, ...
   ```

### Deployed to Orin:
```bash
scp stage_b_chassis_plan.* cr@192.168.100.150:/home/nvidia/temp_gikrepo/ros2/gik9dof_solver/src/
```

## Now Ready to Build

On Orin:
```bash
cd /home/nvidia/temp_gikrepo/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select gik9dof_solver
```

Should compile successfully now! 🚀
