//
// File: getChassisParams.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 14:15:48
//

// Include Files
#include "getChassisParams.h"
#include "rt_nonfinite.h"
#include <cstdio>

// Function Definitions
//
// GETCHASSISPARAMS Return WHEELTEC chassis parameters for path planning
//    params = getChassisParams() returns default "fourwheel" platform
//    params = getChassisParams('compact') - Top diff chassis (0.329m track)
//    params = getChassisParams('fourwheel') - Four-wheel platform (0.573m
//    track) ← DEFAULT
//
//    YOUR PLATFORM: WHEELTEC Four-Wheel Drive with Passive Rear
//    - Front axle: Differential drive (two powered wheels)
//    - Rear axle: Passive omni-wheels (lateral rolling rollers)
//    - Wheelbase: 0.36 m (front-to-rear distance)
//    - Track width: 0.573 m (left-to-right at front wheels)
//    - Wheel radius: 0.1075 m (107.5mm, 215mm diameter drive wheels)
//    - Kinematics: Simplified Ackermann (front-wheel steer via differential)
//
//    This is a HYBRID kinematic model:
//    - Driven like differential drive (front axle, left/right speeds)
//    - Constrained like Ackermann (passive rear creates ICR)
//    - Minimum turning radius exists (NOT zero radius!)
//
//    Based on firmware analysis in docs/chassis-summary.txt
//
//    Returns struct with fields:
//        .chassis_type    - 'front_diff_rear_passive' (unique hybrid)
//        .wheelbase       - Front-to-rear distance [m] ← KEY PARAMETER
//        .track           - Front wheel track width [m]
//        .wheel_radius    - Drive wheel radius [m]
//        .robot_length    - Front-to-back dimension [m]
//        .robot_width     - Side-to-side dimension [m]
//        .robot_radius    - Conservative bounding circle [m]
//        .Vwheel_max      - Max wheel speed [m/s]
//        .Vx_max          - Max forward speed [m/s]
//        .Wz_max          - Max yaw rate [rad/s]
//        .min_turning_radius - Minimum turn radius [m] ← IMPORTANT
//        .accel_max       - Max acceleration [m/s^2]
//
//    See also gik9dof.control.defaultUnifiedParams
//
// Arguments    : void
// Return Type  : void
//
namespace gik9dof {
void getChassisParams()
{
  //  Pre-allocate ALL struct fields for MATLAB Coder compatibility
  //  Four-wheel platform: Front diff + Rear passive omni
  //  [m] Front-to-rear axle distance ← YOUR SPEC
  //  [m] Front track width
  //  [m] 215mm diameter drive wheels
  //  [m] Including bumpers (wheelbase + clearance)
  //  [m] Track + side clearance
  //  Conservative bounding circle (for collision checking)
  //  Velocity limits (from firmware + existing controller)
  //  [m/s] Per-wheel limit (verified in defaultUnifiedParams)
  //  [m/s] Conservative forward speed (existing)
  //  KINEMATICS: Front diff + passive rear creates Instantaneous Center of
  //  Rotation (ICR) The passive rear wheels with lateral rollers allow the
  //  robot to pivot, but the ICR is constrained by geometry (NOT zero radius
  //  like pure diff-drive) Yaw rate limits (differential constraints at front
  //  axle) Front axle differential: |Wz| <= 2 * (Vwheel_max - |Vx|) / track
  //  [rad/s] Pure spin
  //  Conservative yaw limit for mixed motion (Vx = Vx_max)
  //  Use firmware limit (2.5 rad/s) as ultimate cap
  //  [rad/s]
  //  Acceleration limit (from firmware ramp: 0.02 m/s per 10ms = 2 m/s^2)
  //  [m/s^2]
  //  MINIMUM TURNING RADIUS (KEY for Hybrid A*)
  //  With passive rear, the robot pivots around a point related to wheelbase
  //  Approximation: R_min ≈ wheelbase / tan(max_virtual_steer_angle)
  //
  //  For front diff + passive rear, the effective steering angle is:
  //    tan(delta) = (v_R - v_L) / (Vx * track) * wheelbase
  //
  //  At max differential (v_R - v_L = 2*Vwheel_max), assuming Vx ≈ Vwheel_max:
  //    tan(delta_max) ≈ 2 * Vwheel_max / (Vwheel_max * track) * wheelbase
  //                   = 2 * wheelbase / track
  //
  //  Minimum radius: R_min = wheelbase / tan(delta_max)
  //                        = wheelbase / (2 * wheelbase / track)
  //                        = track / 2
  //
  //  This is a SIMPLIFIED estimate. Actual min radius depends on tire slip,
  //  omni-wheel rolling resistance, and speed-dependent effects.
  //  [m] Geometric minimum
  //  Conservative estimate (add 20% margin for real-world effects)
  //  [m]
  //  Alternative calculation based on velocity limits
  //  [m]
  //  Use the larger (more conservative) of the two estimates
  //  Safety margins
  //  [m] Extra clearance (5cm)
  //  Planning parameters
  //  Use conservative forward speed
  //  [m/s] Typical cruising speed
  std::printf("WHEELTEC Chassis Parameters (%s variant):\n", "fourwheel");
  std::fflush(stdout);
  std::printf("  Chassis type:       %s\n", "front_diff_rear_passive");
  std::fflush(stdout);
  std::printf("  Wheelbase:          %.3f m (front-to-rear)\n", 0.36);
  std::fflush(stdout);
  std::printf("  Track width:        %.3f m (front wheels)\n", 0.573);
  std::fflush(stdout);
  std::printf("  Wheel radius:       %.3f m\n", 0.1075);
  std::fflush(stdout);
  std::printf("  Robot radius:       %.3f m (bounding circle)\n",
              0.46097722286464432);
  std::fflush(stdout);
  std::printf("  Inflation radius:   %.3f m (with safety margin)\n",
              0.51097722286464431);
  std::fflush(stdout);
  std::printf("  Max forward speed:  %.2f m/s\n", 0.8);
  std::fflush(stdout);
  std::printf("  Max yaw rate:       %.2f rad/s (%.1f deg/s)\n", 2.5,
              143.23944878270581);
  std::fflush(stdout);
  std::printf("  Yaw (at max speed): %.2f rad/s (%.1f deg/s)\n",
              2.4432809773123911, 139.98968816459904);
  std::fflush(stdout);
  std::printf("  Min turn radius:    %.3f m (passive rear constraint!)\n",
              0.34379999999999994);
  std::fflush(stdout);
  std::printf("  Max acceleration:   %.2f m/s^2\n", 2.0);
  std::fflush(stdout);
}

} // namespace gik9dof

//
// File trailer for getChassisParams.cpp
//
// [EOF]
//
