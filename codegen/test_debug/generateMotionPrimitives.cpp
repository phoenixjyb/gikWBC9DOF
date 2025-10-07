//
// File: generateMotionPrimitives.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

// Include Files
#include "generateMotionPrimitives.h"
#include "gik9dof_planHybridAStarCodegen_internal_types.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstdio>
#include <cstring>

// Function Definitions
//
// GENERATEMOTIONPRIMITIVES Create motion primitive set for front-diff +
// passive-rear robot
//    primitives = generateMotionPrimitives(params)
//
//    Generates a set of motion primitives for Hybrid A* planning.
//    Each primitive is a (Vx, Wz, dt) command tuple.
//
//    Platform: Front differential + passive rear omniwheels
//    - Min turning radius: ~0.34 m (passive rear constraint)
//    - NO spin-in-place (Vx=0, Wz≠0 not possible)
//    - Control: (Vx, Wz) via front axle differential
//
//    INPUTS:
//        params - Chassis parameters from getChassisParams()
//                 Required fields: .track, .Vwheel_max, .Vx_max, .Wz_max,
//                                  .min_turning_radius, .wheelbase
//
//    OUTPUTS:
//        primitives - FIXED-SIZE struct array with fields:
//            .Vx  - Forward velocity [m/s]
//            .Wz  - Yaw rate [rad/s]
//            .dt  - Duration [s]
//            .arc_length - Approximate distance traveled [m]
//            NOTE: Invalid primitives have Vx=0, Wz=0, dt=0
//                  Check dt > 0 to identify valid primitives
//
//    Primitive design strategy:
//        1. Forward arcs (Vx > 0, Wz varied) - primary motion
//        2. Backward arcs (Vx < 0, Wz varied) - parking/reversing
//        3. NO spin-in-place (passive rear prevents Vx=0 rotation)
//        4. Enforce R = |Vx/Wz| >= min_turning_radius
//
//    Primitive count: ~18-20 primitives (fixed array size: 30)
//        - 8 forward: straight + left/right arcs (3 speeds × 5 yaw rates -
//        duplicates)
//        - 6 backward: straight + left/right arcs
//        - Coverage: left/straight/right, slow/medium/fast
//
//    See also computeMotionPrimitive, HybridState, getChassisParams
//
// Arguments    : struct_T primitives[30]
// Return Type  : void
//
namespace gik9dof {
void generateMotionPrimitives(struct_T primitives[30])
{
  static const double dv[3]{0.4, 0.6, 0.8};
  double Vx;
  int bck_count;
  int i_wz;
  int idx;
  //  Extract parameters
  //  Primitive generation settings
  //  [s] Duration options
  //  [m/s] Forward speeds
  //  [m/s] Backward speeds (slower)
  //  Yaw rate options (enforce min radius constraint)
  //  For R_min = 0.34 m and Vx = 0.8 m/s: Wz_max_for_R_min = 0.8/0.34 = 2.35
  //  rad/s [rad/s] Preallocate primitive array with FIXED SIZE for codegen
  std::memset(&primitives[0], 0, 30U * sizeof(struct_T));
  //  Use fixed index - iterate through ALL slots
  idx = 0;
  //  Generate forward primitives
  for (bck_count = 0; bck_count < 3; bck_count++) {
    Vx = dv[bck_count];
    i_wz = 0;
    while ((i_wz < 5) && (idx + 1 <= 30)) {
      //  Check constraints
      // CHECKPRIMITIVECONSTRAINTS Validate motion primitive satisfies all
      // constraints
      //    Returns true if primitive (Vx, Wz) is feasible
      //  Helper function: Check primitive constraints
      //  1. Wheel speed limits
      //  2. Yaw rate limit
      //  3. Minimum turning radius (if turning)
      //  [rad/s]
      if ((!(std::abs(static_cast<double>(i_wz) - 2.0) >= 0.0001)) ||
          (!(std::abs(Vx / (static_cast<double>(i_wz) - 2.0)) <
             0.34379999999999994))) {
        //  All checks passed
        //  Use medium duration (1.0s) for most primitives
        //  Add primitive at current index
        primitives[idx].Vx = Vx;
        primitives[idx].Wz = static_cast<double>(i_wz) - 2.0;
        primitives[idx].dt = 1.0;
        primitives[idx].arc_length = Vx;
        idx++;
      }
      i_wz++;
    }
  }
  //  Generate backward primitives (fewer, slower)
  bck_count = 0;
  while ((bck_count < 2) && (idx + 1 <= 30)) {
    Vx = -0.2 * static_cast<double>(bck_count) - 0.3;
    //  Fewer yaw options for backward (straight, gentle left/right)
    //  [rad/s]
    i_wz = 0;
    while ((i_wz < 3) && (idx + 1 <= 30)) {
      //  Check constraints
      // CHECKPRIMITIVECONSTRAINTS Validate motion primitive satisfies all
      // constraints
      //    Returns true if primitive (Vx, Wz) is feasible
      //  Helper function: Check primitive constraints
      //  1. Wheel speed limits
      //  2. Yaw rate limit
      //  3. Minimum turning radius (if turning)
      //  [rad/s]
      if ((!(std::abs(static_cast<double>(i_wz) - 1.0) >= 0.0001)) ||
          (!(std::abs(Vx / (static_cast<double>(i_wz) - 1.0)) <
             0.34379999999999994))) {
        //  All checks passed
        //  Standard duration
        //  Add primitive at current index
        primitives[idx].Vx = Vx;
        primitives[idx].Wz = static_cast<double>(i_wz) - 1.0;
        primitives[idx].dt = 1.0;
        primitives[idx].arc_length = std::abs(Vx);
        idx++;
      }
      i_wz++;
    }
    bck_count++;
  }
  //  Add a few longer-duration primitives for straight motion
  if (idx + 1 <= 30) {
    //  [m/s] Fast forward
    //  [rad/s] No turning
    //  [s] Longer duration
    // CHECKPRIMITIVECONSTRAINTS Validate motion primitive satisfies all
    // constraints
    //    Returns true if primitive (Vx, Wz) is feasible
    //  Helper function: Check primitive constraints
    //  1. Wheel speed limits
    //  2. Yaw rate limit
    //  3. Minimum turning radius (if turning)
    //  [rad/s]
    //  All checks passed
    primitives[idx].Vx = 0.8;
    primitives[idx].Wz = 0.0;
    primitives[idx].dt = 1.5;
    primitives[idx].arc_length = 1.2000000000000002;
  }
  //  DO NOT TRIM - Keep fixed-size array for codegen compatibility
  //  Unused slots have dt=0, which caller can filter out
  //  primitives = primitives(1:primitive_count);  % REMOVED for codegen
  //  Display summary
  idx = 0;
  for (i_wz = 0; i_wz < 30; i_wz++) {
    if (primitives[i_wz].dt > 0.0) {
      idx++;
    }
  }
  std::printf("Generated %d motion primitives\n", idx);
  std::fflush(stdout);
  if (idx > 0) {
    idx = 0;
    bck_count = 0;
    for (i_wz = 0; i_wz < 30; i_wz++) {
      if (primitives[i_wz].dt > 0.0) {
        Vx = primitives[i_wz].Vx;
        if (Vx > 0.0) {
          idx++;
        } else if (Vx < 0.0) {
          bck_count++;
        }
      }
    }
    std::printf("  - Forward: %d\n", idx);
    std::fflush(stdout);
    std::printf("  - Backward: %d\n", bck_count);
    std::fflush(stdout);
  }
  std::printf("  - Min radius enforced: %.3f m\n", 0.34379999999999994);
  std::fflush(stdout);
}

} // namespace gik9dof

//
// File trailer for generateMotionPrimitives.cpp
//
// [EOF]
//
