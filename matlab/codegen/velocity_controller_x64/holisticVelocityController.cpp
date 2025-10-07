//
// File: holisticVelocityController.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 11:21:02
//

// Include Files
#include "holisticVelocityController.h"
#include "holisticVelocityController_types.h"
#include "wrapToPi.h"
#include <cmath>

// Function Definitions
//
// HOLISTICVELOCITYCONTROLLER Simplified wrapper for codegen of
// unifiedChassisCtrl
//    [Vx, Wz, stateOut] = holisticVelocityController(
//        refX, refY, refTheta, refTime,     % Target position from IK solver
//        estX, estY, estYaw,                 % Current robot pose estimate
//        params,                             % Controller parameters (required)
//        stateIn)                            % Controller state (optional, has
//        default)
//
//    Wraps gik9dof.control.unifiedChassisCtrl in "holistic" mode with
//    scalar inputs/outputs for easier C++ code generation and ROS2 integration.
//
//    Inputs:
//        refX, refY, refTheta - Target base pose in world frame [m, m, rad]
//        refTime              - Timestamp of reference [s]
//        estX, estY, estYaw   - Estimated robot pose [m, m, rad]
//        params               - Parameter struct (track, Vx_max, etc.) -
//        REQUIRED stateIn              - State struct (carries prev reference)
//        - OPTIONAL
//
//    Outputs:
//        Vx       - Forward velocity command [m/s]
//        Wz       - Yaw rate command [rad/s]
//        stateOut - Updated state struct for next call
//
//    See also gik9dof.control.unifiedChassisCtrl,
//             gik9dof.control.clampYawByWheelLimit
//
// Arguments    : double refX
//                double refY
//                double refTheta
//                double refTime
//                double estX
//                double estY
//                double estYaw
//                const struct0_T *params
//                const struct1_T *stateIn
//                double *Vx
//                double *Wz
//                struct1_T *stateOut
// Return Type  : void
//
namespace gik9dof_velocity {
void holisticVelocityController(double refX, double refY, double refTheta,
                                double refTime, double, double, double estYaw,
                                const struct0_T *params,
                                const struct1_T *stateIn, double *Vx,
                                double *Wz, struct1_T *stateOut)
{
  double Wcap;
  double Wwheel;
  double dt;
  double dtheta;
  double headingError;
  double vxWorld;
  double vyWorld;
  stateOut->prev.x = refX;
  stateOut->prev.y = refY;
  stateOut->prev.theta = refTheta;
  stateOut->prev.t = refTime;
  //  Build reference struct for unifiedChassisCtrl
  //  Current pose estimate as 1x3 vector
  //  Call main controller in "holistic" mode (params before state)
  //  Use char array 'holistic' instead of string "holistic" for codegen
  //  compatibility
  // UNIFIEDCHASSISCTRL Convert heterogeneous references into unified base
  // commands.
  //    [cmd, state] = gik9dof.control.unifiedChassisCtrl(mode, ref, estPose,
  //    params, state) accepts references from either the holistic GIK pipeline
  //    ("holistic" or "staged-C") or a staged base follower ("staged-B") and
  //    produces a UnifiedCmd struct with fields:
  //        .mode (string)  - input mode echoed back
  //        .time (double)  - ref timestamp [s]
  //        .base.Vx (double) - forward velocity command (m/s)
  //        .base.Vy (double) - always 0 for diff-drive
  //        .base.Wz (double) - yaw rate command (rad/s)
  //        .arm.qdot (double[]) - optional arm joint velocities
  //        (holistic/Stage-C)
  //
  //    Inputs
  //    ------
  //    mode    : string/char, one of "holistic", "staged-C", "staged-B".
  //    ref     : struct containing reference data. For holistic/Stage-C the
  //              struct must include fields x,y,theta,t and optionally
  //              arm_qdot. For staged-B it must include v,w,t.
  //    estPose : 1x3 [x y theta] estimated robot pose in world.
  //    params  : struct with fields:
  //                 track        - wheel track (m)
  //                 Vwheel_max   - per-wheel speed limit (m/s)
  //                 Vx_max       - forward speed clamp (m/s)
  //                 W_max        - yaw rate clamp (rad/s)
  //                 yawKp        - heading proportional gain
  //                 yawKff       - yaw feed-forward gain
  //    state   : struct carrying persistent fields used by the controller.
  //
  //    Outputs
  //    -------
  //    cmd     : struct matching UnifiedCmd schema (see summary doc).
  //    state   : updated state struct (stores previous holistic sample).
  //
  //    Notes
  //    -----
  //    - Differentiation uses timestamps in the reference; ensure consistent
  //      sample times.
  //    - The yaw gate enforces both platform limits and wheel limits derived
  //    from
  //      the diff-drive kinematics.
  //
  //    See also gik9dof.control.defaultUnifiedParams.
  //  Cell array of char arrays for codegen
  //  Ensure required parameter fields exist
  //  Cell array for codegen
  //  Use char arrays for codegen compatibility
  //  Guard: ensure reference carries pose + timestamp
  //  Cell array of char arrays
  dt = std::fmax(0.001, refTime - stateIn->prev.t);
  dtheta = refTheta - stateIn->prev.theta;
  coder::wrapToPi(dtheta);
  vxWorld = (refX - stateIn->prev.x) / dt;
  vyWorld = (refY - stateIn->prev.y) / dt;
  Wcap = std::cos(estYaw) * vxWorld + std::sin(estYaw) * vyWorld;
  //  Desired heading from world-frame velocity
  headingError = std::atan2(vyWorld, vxWorld) - estYaw;
  coder::wrapToPi(headingError);
  if (Wcap < 0.0) {
    Wwheel = -1.0;
  } else {
    Wwheel = (Wcap > 0.0);
  }
  *Vx = Wwheel * std::fmin(std::abs(Wcap), params->Vx_max) *
        std::cos(headingError);
  //  Apply yaw feasibility gate and forward speed clamp
  // CLAMPYAWBYWHEELLIMIT Apply yaw constraints derived from wheel and platform
  // limits.
  //    [WzOut, caps] = gik9dof.control.clampYawByWheelLimit(Vx, Wz, track,
  //    Vwheel_max, Wmax) clamps the yaw command |Wz| so that the
  //    differential-drive wheels remain within their linear speed limits.
  //    Inputs:
  //        Vx          - commanded forward velocity (m/s)
  //        Wz          - commanded yaw rate (rad/s)
  //        track       - wheel track width (m)
  //        Vwheel_max  - maximum per-wheel linear speed (m/s)
  //        Wmax        - platform yaw rate clamp (rad/s)
  //
  //    Outputs:
  //        WzOut       - clamped yaw rate command (rad/s)
  //        caps        - struct with fields:
  //                         .wheel   - yaw limit computed from wheel speeds
  //                         .applied - final yaw limit after combining with
  //                         Wmax
  //
  //    The wheel-derived limit assumes the standard diff-drive relation:
  //        v_left  = Vx - 0.5 * track * Wz
  //        v_right = Vx + 0.5 * track * Wz
  //    Both |v_left| and |v_right| must remain <= Vwheel_max.
  //  Compute wheel-derived yaw cap
  if (params->track <= 0.0) {
    Wwheel = 0.0;
  } else {
    Wwheel = 2.0 * std::fmax(0.0, params->Vwheel_max - std::abs(*Vx)) /
             params->track;
  }
  Wcap = std::fmin(params->W_max, Wwheel);
  //  Populate command struct
  //  Extract velocity commands
  *Vx = std::fmax(-params->Vx_max, std::fmin(params->Vx_max, *Vx));
  *Wz = std::fmax(-Wcap, std::fmin(Wcap, params->yawKp * headingError +
                                             params->yawKff * (dtheta / dt)));
}

} // namespace gik9dof_velocity

//
// File trailer for holisticVelocityController.cpp
//
// [EOF]
//
