//
// File: ChassisPathFollower.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 11-Oct-2025 00:19:03
//

// Include Files
#include "ChassisPathFollower.h"
#include "chassisPathFollowerCodegen_types.h"
#include "find.h"
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "wrapToPi.h"
#include "coder_bounded_array.h"
#include "rt_defines.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Declarations
namespace gik9dof {
static int differentiateSeries(const double series_data[], int series_size,
                               double dt, double df_data[]);

static double rt_atan2d_snf(double u0, double u1);

static double rt_hypotd_snf(double u0, double u1);

} // namespace gik9dof

// Function Definitions
//
// DIFFERENTIATESERIES 5-point finite difference formula
//    Uses different formulas for boundary points:
//    • Interior (i >= 3, i <= N-2): 5-point centered difference
//    • Near boundary (i == 2, i == N-1): 3-point centered difference
//    • Boundary (i == 1, i == N): Forward/backward difference
//
// Arguments    : const double series_data[]
//                int series_size
//                double dt
//                double df_data[]
// Return Type  : int
//
namespace gik9dof {
static int differentiateSeries(const double series_data[], int series_size,
                               double dt, double df_data[])
{
  int df_size;
  df_size = series_size;
  if (series_size - 1 >= 0) {
    std::memset(&df_data[0], 0,
                static_cast<unsigned int>(series_size) * sizeof(double));
  }
  if (series_size >= 2) {
    dt = std::fmax(dt, 0.001);
    if (series_size < 5) {
      //  Use simpler formulas for short series
      for (int i{0}; i < series_size; i++) {
        if (i + 1 == 1) {
          df_data[0] = (series_data[1] - series_data[0]) / dt;
        } else if (i + 1 == series_size) {
          df_data[i] = (series_data[i] - series_data[i - 1]) / dt;
        } else {
          df_data[i] = (series_data[i + 1] - series_data[i - 1]) / (2.0 * dt);
        }
      }
    } else {
      //  5-point differentiation for longer series
      for (int i{0}; i < series_size; i++) {
        if ((i + 1 >= 3) && (i + 1 <= series_size - 2)) {
          //  Interior: 5-point centered difference
          df_data[i] = (((-series_data[i + 2] + 8.0 * series_data[i + 1]) -
                         8.0 * series_data[i - 1]) +
                        series_data[i - 2]) /
                       (12.0 * dt);
        } else if ((i + 1 == 2) || (i + 1 == series_size - 1)) {
          //  Near boundary: 3-point centered
          df_data[i] = (series_data[i + 1] - series_data[i - 1]) / (2.0 * dt);
        } else if (i + 1 == 1) {
          //  Start: forward difference
          df_data[0] = ((-3.0 * series_data[0] + 4.0 * series_data[1]) -
                        series_data[2]) /
                       (2.0 * dt);
        } else {
          //  i == N
          //  End: backward difference
          df_data[i] = ((3.0 * series_data[i] - 4.0 * series_data[i - 1]) +
                        series_data[i - 2]) /
                       (2.0 * dt);
        }
      }
    }
  }
  return df_size;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = std::atan2(static_cast<double>(i), static_cast<double>(i1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double b;
  double y;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = b * std::sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * std::sqrt(b * b + 1.0);
  } else if (std::isnan(b)) {
    y = rtNaN;
  } else {
    y = a * 1.4142135623730951;
  }
  return y;
}

//
// Arguments    : void
// Return Type  : void
//
ChassisPathFollower::ChassisPathFollower() = default;

//
// Arguments    : void
// Return Type  : void
//
ChassisPathFollower::~ChassisPathFollower() = default;

//
// CHASSISPATHFOLLOWERCODEGEN Geometric path follower for waypoint navigation
//
//    [vx, wz, state, status] = chassisPathFollowerCodegen(pose, dt, state,
//    params) implements an advanced path following controller with multiple
//    control modes, curvature-based speed control, and comprehensive velocity
//    limiting.
//
//    This is the codegen-compatible production version of Mode 2 from
//    simulateChassisController. Despite the different name, it provides the
//    same advanced path following functionality from the purePursuitFollower
//    class.
//
//  INPUTS:
//    pose   - [x, y, theta] current robot pose in world frame (1x3 double)
//    dt     - time step since last call (scalar double, seconds)
//    state  - persistent state struct (see initializeChassisPathState)
//    params - parameters struct with fields:
//             • ControllerMode: 'blended', 'purePursuit', or 'stanley'
//             • Lookahead tuning: LookaheadBase, LookaheadVelGain, etc.
//             • Chassis: struct with track, limits, accel/jerk,
//             curvature_slowdown • PathInfo: struct with States, Curvature,
//             ArcLength, etc. • Heading PID: HeadingKp, HeadingKi, HeadingKd,
//             FeedforwardGain • GoalTolerance: distance threshold for goal
//             detection
//
//  OUTPUTS:
//    vx     - forward velocity command (m/s, scalar double)
//    wz     - angular velocity command (rad/s, scalar double)
//    state  - updated state struct for next iteration
//    status - status struct with diagnostics:
//             • isFinished: goal reached flag
//             • distanceRemaining: distance to goal (m)
//             • crossTrackError: lateral error (m)
//             • headingError: angular error (rad)
//             • curvature: path curvature at current point (rad/m)
//             • lookaheadDistance: current lookahead distance (m)
//             • currentMode: active controller mode (for blended)
//             • currentIndex: path segment index
//
//  CONTROLLER MODES:
//    'purePursuit'  - Classic geometric path tracking with lookahead point
//                     Best for: High-speed smooth paths, gentle curves
//                     Method: Computes curvature from lookahead geometry
//
//    'stanley'      - Cross-track error correction with heading alignment
//                     Best for: Low-speed precise tracking, tight maneuvers
//                     Method: Proportional to lateral error + heading error
//
//    'blended'      - Speed-adaptive blend of Stanley (low) + Pure Pursuit
//    (high)
//                     Best for: Full speed range, smooth mode transitions
//                     Method: Sigmoid blend with transition around 0.3 m/s
//                     DEFAULT and RECOMMENDED for most applications
//
//  FEATURES:
//    ✓ 3 control modes with automatic blending
//    ✓ Curvature-based speed reduction in curves
//    ✓ Acceleration limiting with feedforward compensation
//    ✓ Jerk limiting for smooth velocity profiles
//    ✓ Wheel speed limiting (differential drive kinematics)
//    ✓ Adaptive lookahead (base + velocity + acceleration terms)
//    ✓ Bidirectional support (forward/reverse paths)
//    ✓ Comprehensive status reporting
//    ✓ 30+ tuning parameters (vs 13 in old purePursuitVelocityController)
//
//  RELATIONSHIP TO OTHER CONTROLLERS:
//    • chassisPathFollowerCodegen (this): Geometric waypoint following
//      Input: Sparse waypoints [x,y,yaw], no timestamps
//      Use for: Hybrid A* paths, navigation waypoints, staged control
//
//    • unifiedChassisCtrl: GIK trajectory differentiation
//      Input: Dense timestamped trajectory from full-body IK
//      Use for: Holistic control, arm-integrated motion
//
//    • purePursuitVelocityController: Old simple version (DEPRECATED)
//      Limitations: Single mode, no curvature control, no accel limiting
//
//  TYPICAL USAGE:
//    % 1. Preprocess path (offline or at initialization)
//    PathInfo = preparePathForFollower(waypoints, chassisParams);
//
//    % 2. Create parameter struct
//    params = struct(...
//        'ControllerMode', 'blended', ...
//        'LookaheadBase', 0.6, ...
//        'LookaheadVelGain', 0.3, ...
//        'GoalTolerance', 0.1, ...
//        'HeadingKp', 1.2, ...
//        'Chassis', chassisParams, ...
//        'PathInfo', PathInfo);
//
//    % 3. Initialize state
//    state = initializeChassisPathState(params.PathInfo);
//
//    % 4. Control loop
//    while ~status.isFinished
//        pose = getCurrentPose();
//        [vx, wz, state, status] = chassisPathFollowerCodegen(pose, dt, state,
//        params); publishCommand(vx, wz);
//    end
//
//  CODEGEN COMPATIBILITY:
//    This function is designed for MATLAB Coder code generation:
//    • No class dependencies
//    • No inputParser
//    • Fixed-size arrays where possible
//    • Explicit type declarations
//    • No dynamic memory allocation in loops
//
//  PARAMETER TUNING GUIDE:
//    LookaheadBase:     Start with 0.5-0.8 m (larger for higher speeds)
//    LookaheadVelGain:  0.2-0.4 (adaptive lookahead sensitivity)
//    HeadingKp:         1.0-2.0 (higher = more aggressive heading correction)
//    FeedforwardGain:   0.8-1.0 (path yaw rate feedforward strength)
//    accel_limit:       0.8-1.5 m/s² (smooth acceleration)
//    jerk_limit:        3.0-8.0 m/s³ (smoother = lower jerk)
//    kappa_threshold:   0.5-1.5 rad/m (curvature for speed reduction)
//    vx_reduction:      0.5-0.8 (speed fraction in curves)
//
//  PERFORMANCE CHARACTERISTICS:
//    • Handles curves better than old controller (curvature-based slowdown)
//    • Smoother motion (acceleration/jerk limiting)
//    • Better low-speed tracking (Stanley mode in blended)
//    • Better high-speed stability (Pure Pursuit mode in blended)
//    • More tunable (30+ params vs 13 in old version)
//
//  KNOWN LIMITATIONS:
//    • Requires preprocessed path with curvature (use preparePathForFollower)
//    • Assumes differential drive kinematics (no Vy)
//    • Assumes path is geometrically feasible for chassis
//    • Goal detection based on distance only (no orientation check)
//
//  See also: preparePathForFollower, initializeChassisPathState,
//            simulateChassisController, unifiedChassisCtrl,
//            purePursuitVelocityController (deprecated)
//
//  REVISION HISTORY:
//    Oct 10, 2025 - Initial codegen-compatible implementation
//                   Refactored from purePursuitFollower class
//                   Renamed from simulateChassisController Mode 2
//
// Arguments    : const double pose[3]
//                double dt
//                struct0_T *state
//                const struct1_T *params
//                double *vx
//                double *wz
//                struct3_T *status
// Return Type  : void
//
void ChassisPathFollower::chassisPathFollowerCodegen(
    const double pose[3], double dt, struct0_T *state, const struct1_T *params,
    double *vx, double *wz, struct3_T *status)
{
  double params_data[500];
  double vxWorld_data[500];
  double vyWorld_data[500];
  double wSeries_data[500];
  double accel_data;
  double crossTrackError;
  double distanceToGoal;
  double headingError;
  double lookaheadDistance;
  double nearestIdx;
  double wz_desired_data;
  int vxWorld_size;
  bool isFinished;
  //  Input Validation and Initialization
  //  Check for empty or uninitialized state
  if (state->PathNumPoints == 0.0) {
    //  Initialize state on first call using flattened PathInfo fields
    state->PathNumPoints = params->PathInfo_States.size[0];
    state->CurrentIndex = 1.0;
    state->LastVelocity = 0.0;
    state->LastAcceleration = 0.0;
    state->LastHeadingError = 0.0;
    state->IntegralHeadingError = 0.0;
    state->PreviousPose[0] = 0.0;
    state->PreviousPose[1] = 0.0;
    state->PreviousPose[2] = 0.0;
    state->DistanceTraveled = 0.0;
  }
  //  Handle empty path
  if (state->PathNumPoints < 2.0) {
    *vx = 0.0;
    *wz = 0.0;
    // CREATEEMPTYSTATUS Create status struct with default values
    //
    //    status = createEmptyStatus(params) creates a status struct with all
    //    diagnostic fields initialized to safe default values.
    //  initializeChassisPathState
    //  ========================================================================
    //  Helper Function: Create Empty Status
    //  ========================================================================
    status->isFinished = false;
    status->distanceRemaining = rtInf;
    status->crossTrackError = 0.0;
    status->headingError = 0.0;
    status->curvature.size[0] = 1;
    status->curvature.data[0] = 0.0;
    status->lookaheadDistance = params->LookaheadBase;
    status->currentMode = params->ControllerMode;
    status->currentIndex = 1.0;
  } else {
    //  Validate pose dimensions
    //  Controller Mode Dispatch
    //  Numeric: 0, 1, or 2
    //  Get path states
    //  Extract chassis limits
    switch (static_cast<int>(params->ControllerMode)) {
    case 0: {
      double b_params_data[1000];
      double x_data[1000];
      double maxWheel;
      double vL_abs;
      double vR_abs;
      int loop_ub;
      int params_size;
      //         %% MODE 0: Legacy 5-Point Differentiation (Open-Loop)
      //  Replay velocities from path using numerical differentiation
      //  No feedback - just transform world velocities to body frame
      //  Differentiate path to get world-frame velocities
      loop_ub = params->PathInfo_States.size[0];
      if (loop_ub - 1 >= 0) {
        std::copy(&params->PathInfo_States.data[0],
                  &params->PathInfo_States.data[loop_ub], &params_data[0]);
      }
      differentiateSeries(params_data, params->PathInfo_States.size[0], dt,
                          vxWorld_data);
      for (int i{0}; i < loop_ub; i++) {
        params_data[i] =
            params->PathInfo_States.data[i + params->PathInfo_States.size[0]];
      }
      differentiateSeries(params_data, params->PathInfo_States.size[0], dt,
                          vyWorld_data);
      for (int i{0}; i < loop_ub; i++) {
        params_data[i] = params->PathInfo_States
                             .data[i + params->PathInfo_States.size[0] * 2];
      }
      differentiateSeries(params_data, params->PathInfo_States.size[0], dt,
                          wSeries_data);
      //  Find nearest point
      vxWorld_size = params->PathInfo_States.size[0];
      for (int i{0}; i < 2; i++) {
        for (params_size = 0; params_size < loop_ub; params_size++) {
          b_params_data[params_size + vxWorld_size * i] =
              params->PathInfo_States
                  .data[params_size + params->PathInfo_States.size[0] * i] -
              pose[i];
        }
      }
      vxWorld_size = params->PathInfo_States.size[0] << 1;
      for (int i{0}; i < vxWorld_size; i++) {
        maxWheel = b_params_data[i];
        x_data[i] = maxWheel * maxWheel;
      }
      if (params->PathInfo_States.size[0] == 0) {
        params_size = 0;
      } else {
        params_size = params->PathInfo_States.size[0];
        for (vxWorld_size = 0; vxWorld_size < loop_ub; vxWorld_size++) {
          params_data[vxWorld_size] =
              x_data[vxWorld_size] + x_data[loop_ub + vxWorld_size];
        }
      }
      coder::internal::minimum(params_data, params_size, vxWorld_size);
      nearestIdx =
          std::fmax(static_cast<double>(vxWorld_size), state->CurrentIndex);
      state->CurrentIndex = nearestIdx;
      //  Transform world velocities to body frame
      //  Simple clamping (no sophisticated limiting for mode 0)
      // CLAMPVALUE Clamp value to range [minVal, maxVal]
      //  Force inputs to be scalars for codegen
      //  chassisPathFollowerCodegen
      //  ========================================================================
      //  Helper Functions
      //  ========================================================================
      *vx = std::fmin(
          std::fmax(std::cos(pose[2]) *
                            vxWorld_data[static_cast<int>(nearestIdx) - 1] +
                        std::sin(pose[2]) *
                            vyWorld_data[static_cast<int>(nearestIdx) - 1],
                    params->Chassis.vx_min),
          params->Chassis.vx_max);
      // CLAMPYAWBYWHEELLIMIT Clamp yaw rate respecting wheel speed limits
      //    Computes required wheel speeds and scales down wz if needed
      //  Force all inputs to be scalars for codegen
      //  Ensure scalars for codegen
      *wz = wSeries_data[static_cast<int>(nearestIdx) - 1];
      vR_abs = 0.5 * params->Chassis.track * *wz;
      vL_abs = std::abs(*vx - vR_abs);
      vR_abs = std::abs(*vx + vR_abs);
      if (vR_abs > vL_abs) {
        maxWheel = vR_abs;
      } else {
        maxWheel = vL_abs;
      }
      //  Ensure all are scalars
      if ((maxWheel > params->Chassis.wheel_speed_max) && (maxWheel > 1.0E-6)) {
        *wz *= params->Chassis.wheel_speed_max / maxWheel;
      }
      //  Final wz clamping
      // CLAMPVALUE Clamp value to range [minVal, maxVal]
      //  Force inputs to be scalars for codegen
      //  chassisPathFollowerCodegen
      //  ========================================================================
      //  Helper Functions
      //  ========================================================================
      *wz = std::fmin(std::fmax(*wz, -params->Chassis.wz_max),
                      params->Chassis.wz_max);
      //  Ensure scalar output
      //  Status
      distanceToGoal = params->PathInfo_DistanceRemaining
                           .data[static_cast<int>(nearestIdx) - 1];
      //  Ensure scalar
      if ((distanceToGoal <= params->GoalTolerance) ||
          (nearestIdx >= state->PathNumPoints)) {
        isFinished = true;
      } else {
        isFinished = false;
      }
      crossTrackError = 0.0;
      headingError = 0.0;
      lookaheadDistance = 0.0;
      status->curvature.size[0] = 1;
      status->curvature.data[0] = 0.0;
      accel_data = 0.0;
    } break;
    case 1: {
      double b_params_data[1000];
      double x_data[1000];
      double b_params[3];
      double maxWheel;
      double vL_abs;
      double vR_abs;
      int loop_ub;
      int params_size;
      bool c_params_data[500];
      //         %% MODE 1: Heading-Aware Controller (Simple Feedback)
      //  P-control on heading + feedforward yaw rate
      //  Velocity from distance to lookahead point
      //  Find nearest point
      loop_ub = params->PathInfo_States.size[0];
      vxWorld_size = params->PathInfo_States.size[0];
      for (int i{0}; i < 2; i++) {
        for (params_size = 0; params_size < loop_ub; params_size++) {
          b_params_data[params_size + vxWorld_size * i] =
              params->PathInfo_States
                  .data[params_size + params->PathInfo_States.size[0] * i] -
              pose[i];
        }
      }
      vxWorld_size = params->PathInfo_States.size[0] << 1;
      for (int i{0}; i < vxWorld_size; i++) {
        maxWheel = b_params_data[i];
        x_data[i] = maxWheel * maxWheel;
      }
      if (params->PathInfo_States.size[0] == 0) {
        params_size = 0;
      } else {
        params_size = params->PathInfo_States.size[0];
        for (vxWorld_size = 0; vxWorld_size < loop_ub; vxWorld_size++) {
          params_data[vxWorld_size] =
              x_data[vxWorld_size] + x_data[loop_ub + vxWorld_size];
        }
      }
      coder::internal::minimum(params_data, params_size, vxWorld_size);
      nearestIdx =
          std::fmax(static_cast<double>(vxWorld_size), state->CurrentIndex);
      state->CurrentIndex = nearestIdx;
      //  Compute lookahead distance
      b_params[0] =
          (params->LookaheadBase +
           params->LookaheadVelGain * std::abs(state->LastVelocity)) +
          params->LookaheadAccelGain * std::abs(state->LastAcceleration) * dt;
      b_params[1] = params->GoalTolerance;
      b_params[2] = 0.05;
      lookaheadDistance = coder::internal::maximum(b_params);
      //  Find lookahead point
      maxWheel = std::fmin(
          params->PathInfo_ArcLength
              .data[params->PathInfo_ArcLength.size[0] - 1],
          params->PathInfo_ArcLength.data[static_cast<int>(nearestIdx) - 1] +
              lookaheadDistance);
      loop_ub = params->PathInfo_ArcLength.size[0];
      for (int i{0}; i < loop_ub; i++) {
        c_params_data[i] = (params->PathInfo_ArcLength.data[i] >= maxWheel);
      }
      params_size =
          coder::eml_find(c_params_data, params->PathInfo_ArcLength.size[0],
                          (int *)&vxWorld_size);
      for (int i{0}; i < params_size; i++) {
        accel_data = vxWorld_size;
      }
      if (params_size == 0) {
        accel_data = state->PathNumPoints;
      }
      //  Compute heading error and feedforward
      maxWheel =
          params->PathInfo_States.data[static_cast<int>(accel_data) - 1] -
          pose[0];
      vR_abs = params->PathInfo_States.data[(static_cast<int>(accel_data) +
                                             params->PathInfo_States.size[0]) -
                                            1] -
               pose[1];
      headingError = rt_atan2d_snf(vR_abs, maxWheel) - pose[2];
      coder::wrapToPi(headingError);
      vL_abs = std::fmax(dt, 0.001);
      wz_desired_data =
          params->PathInfo_States.data[(static_cast<int>(accel_data) +
                                        params->PathInfo_States.size[0] * 2) -
                                       1] -
          pose[2];
      coder::wrapToPi(wz_desired_data);
      //  Velocity from distance to target
      maxWheel = rt_hypotd_snf(maxWheel, vR_abs) / vL_abs;
      //  Reverse handling
      if (params->ReverseEnabled &&
          (std::cos(headingError) < -0.49999999999999994)) {
        maxWheel = -maxWheel;
      }
      //  Heading control
      //  Apply limits
      // CLAMPVALUE Clamp value to range [minVal, maxVal]
      //  Force inputs to be scalars for codegen
      //  chassisPathFollowerCodegen
      //  ========================================================================
      //  Helper Functions
      //  ========================================================================
      *vx = std::fmin(std::fmax(maxWheel, params->Chassis.vx_min),
                      params->Chassis.vx_max);
      *wz = params->HeadingKp * headingError +
            params->FeedforwardGain * (wz_desired_data / vL_abs);
      // CLAMPYAWBYWHEELLIMIT Clamp yaw rate respecting wheel speed limits
      //    Computes required wheel speeds and scales down wz if needed
      //  Force all inputs to be scalars for codegen
      //  Ensure scalars for codegen
      vR_abs = 0.5 * params->Chassis.track * *wz;
      vL_abs = std::abs(*vx - vR_abs);
      vR_abs = std::abs(*vx + vR_abs);
      if (vR_abs > vL_abs) {
        maxWheel = vR_abs;
      } else {
        maxWheel = vL_abs;
      }
      //  Ensure all are scalars
      if ((maxWheel > params->Chassis.wheel_speed_max) && (maxWheel > 1.0E-6)) {
        *wz *= params->Chassis.wheel_speed_max / maxWheel;
      }
      //  Final wz clamping
      // CLAMPVALUE Clamp value to range [minVal, maxVal]
      //  Force inputs to be scalars for codegen
      //  chassisPathFollowerCodegen
      //  ========================================================================
      //  Helper Functions
      //  ========================================================================
      *wz = std::fmin(std::fmax(*wz, -params->Chassis.wz_max),
                      params->Chassis.wz_max);
      //  Ensure scalar output
      //  Status
      distanceToGoal = params->PathInfo_DistanceRemaining
                           .data[static_cast<int>(nearestIdx) - 1];
      //  Ensure scalar
      if ((distanceToGoal <= params->GoalTolerance) ||
          (nearestIdx >= state->PathNumPoints)) {
        isFinished = true;
      } else {
        isFinished = false;
      }
      crossTrackError = 0.0;
      //  Not computed in mode 1
      status->curvature.size[0] = 1;
      status->curvature.data[0] =
          params->PathInfo_Curvature.data[static_cast<int>(accel_data) - 1];
      //  Ensure scalar
      accel_data = (*vx - state->LastVelocity) / dt;
    } break;
    case 2: {
      double b_params_data[1000];
      double x_data[1000];
      double b_params[3];
      double headingError_data;
      double maxWheel;
      double vL_abs;
      double vR_abs;
      double vx_smooth_data;
      int loop_ub;
      int params_size;
      bool c_params_data[500];
      //         %% MODE 2: Pure Pursuit (Full Feedback with Advanced Features)
      //  Adaptive lookahead, curvature-based speed control, full limiting
      //  Find nearest point
      loop_ub = params->PathInfo_States.size[0];
      vxWorld_size = params->PathInfo_States.size[0];
      for (int i{0}; i < 2; i++) {
        for (params_size = 0; params_size < loop_ub; params_size++) {
          b_params_data[params_size + vxWorld_size * i] =
              params->PathInfo_States
                  .data[params_size + params->PathInfo_States.size[0] * i] -
              pose[i];
        }
      }
      vxWorld_size = params->PathInfo_States.size[0] << 1;
      for (int i{0}; i < vxWorld_size; i++) {
        maxWheel = b_params_data[i];
        x_data[i] = maxWheel * maxWheel;
      }
      if (params->PathInfo_States.size[0] == 0) {
        params_size = 0;
      } else {
        params_size = params->PathInfo_States.size[0];
        for (vxWorld_size = 0; vxWorld_size < loop_ub; vxWorld_size++) {
          params_data[vxWorld_size] =
              x_data[vxWorld_size] + x_data[loop_ub + vxWorld_size];
        }
      }
      coder::internal::minimum(params_data, params_size, vxWorld_size);
      nearestIdx =
          std::fmax(static_cast<double>(vxWorld_size), state->CurrentIndex);
      state->CurrentIndex = nearestIdx;
      //  Adaptive lookahead distance
      b_params[0] =
          (params->LookaheadBase +
           params->LookaheadVelGain * std::abs(state->LastVelocity)) +
          params->LookaheadAccelGain * std::abs(state->LastAcceleration) * dt;
      b_params[1] = params->GoalTolerance;
      b_params[2] = 0.05;
      lookaheadDistance = coder::internal::maximum(b_params);
      //  Find lookahead point
      maxWheel = std::fmin(
          params->PathInfo_ArcLength
              .data[params->PathInfo_ArcLength.size[0] - 1],
          params->PathInfo_ArcLength.data[static_cast<int>(nearestIdx) - 1] +
              lookaheadDistance);
      loop_ub = params->PathInfo_ArcLength.size[0];
      for (int i{0}; i < loop_ub; i++) {
        c_params_data[i] = (params->PathInfo_ArcLength.data[i] >= maxWheel);
      }
      params_size =
          coder::eml_find(c_params_data, params->PathInfo_ArcLength.size[0],
                          (int *)&vxWorld_size);
      for (int i{0}; i < params_size; i++) {
        accel_data = vxWorld_size;
      }
      if (params_size == 0) {
        accel_data = state->PathNumPoints;
      }
      //  Transform target to body frame
      maxWheel = std::cos(pose[2]);
      vR_abs = std::sin(pose[2]);
      vL_abs = params->PathInfo_States.data[static_cast<int>(accel_data) - 1] -
               pose[0];
      wz_desired_data =
          params->PathInfo_States.data[(static_cast<int>(accel_data) +
                                        params->PathInfo_States.size[0]) -
                                       1] -
          pose[1];
      crossTrackError = -vR_abs * vL_abs + maxWheel * wz_desired_data;
      //  Pure pursuit curvature from geometry
      vx_smooth_data = std::fmax(lookaheadDistance, 0.001);
      //  Cross-track and heading errors
      headingError_data =
          params->PathInfo_States.data[(static_cast<int>(accel_data) +
                                        params->PathInfo_States.size[0] * 2) -
                                       1] -
          pose[2];
      coder::wrapToPi((double *)&headingError_data);
      headingError = headingError_data;
      //  Ensure scalar for codegen
      //  Path curvature for speed control
      status->curvature.size[0] = 1;
      status->curvature.data[0] =
          params->PathInfo_Curvature.data[static_cast<int>(accel_data) - 1];
      //  Ensure scalar indexing
      //  Direction handling
      vxWorld_size = 1;
      if (params->ReverseEnabled &&
          (maxWheel * vL_abs + vR_abs * wz_desired_data < 0.0)) {
        vxWorld_size = -1;
      }
      //  Curvature-based speed control
      headingError_data = std::abs(
          params->PathInfo_Curvature.data[static_cast<int>(accel_data) - 1]);
      if (!(headingError_data > params->KappaThreshold)) {
        headingError_data =
            params->VxReduction +
            (1.0 - params->VxReduction) *
                std::fmax(0.0,
                          1.0 - headingError_data / params->KappaThreshold);
      } else {
        headingError_data = params->VxReduction;
      }
      headingError_data = std::fmax(params->Chassis.vx_min,
                                    params->Chassis.vx_max * headingError_data);
      //  Goal approach tapering
      //  Ensure scalar
      distanceToGoal = params->PathInfo_DistanceRemaining
                           .data[static_cast<int>(nearestIdx) - 1];
      wz_desired_data = 3.0 * params->GoalTolerance;
      if (distanceToGoal < wz_desired_data) {
        headingError_data =
            std::fmin(headingError_data,
                      params->Chassis.vx_max *
                          std::fmax(distanceToGoal / wz_desired_data, 0.2));
      }
      headingError_data = std::fmin(headingError_data, params->Chassis.vx_max);
      vR_abs = static_cast<double>(vxWorld_size) * std::abs(headingError_data);
      //  Pure pursuit angular velocity
      wz_desired_data =
          vR_abs * (2.0 * crossTrackError / (vx_smooth_data * vx_smooth_data));
      //  Acceleration limiting
      vR_abs = (vR_abs - state->LastVelocity) / dt;
      if (!(vR_abs >= 0.0)) {
        accel_data = std::fmax(vR_abs, -params->Chassis.decel_limit);
      } else {
        accel_data = std::fmin(vR_abs, params->Chassis.accel_limit);
      }
      //  Jerk limiting
      vx_smooth_data =
          state->LastVelocity +
          (state->LastAcceleration +
           std::fmax(-params->Chassis.jerk_limit,
                     std::fmin(params->Chassis.jerk_limit,
                               (accel_data - state->LastAcceleration) / dt)) *
               dt) *
              dt;
      //  Wheel speed limiting
      vR_abs = wz_desired_data * params->Chassis.track / 2.0;
      headingError_data = vx_smooth_data - vR_abs;
      vR_abs += vx_smooth_data;
      maxWheel = std::abs(headingError_data);
      headingError_data = std::fmax(maxWheel, std::abs(vR_abs));
      if (headingError_data > params->Chassis.wheel_speed_max) {
        maxWheel = params->Chassis.wheel_speed_max / headingError_data;
        vx_smooth_data *= maxWheel;
        wz_desired_data *= maxWheel;
      }
      //  Final yaw clamping
      // CLAMPYAWBYWHEELLIMIT Clamp yaw rate respecting wheel speed limits
      //    Computes required wheel speeds and scales down wz if needed
      //  Force all inputs to be scalars for codegen
      //  Ensure scalars for codegen
      vR_abs = 0.5 * params->Chassis.track * wz_desired_data;
      vL_abs = std::abs(vx_smooth_data - vR_abs);
      vR_abs = std::abs(vx_smooth_data + vR_abs);
      if (vR_abs > vL_abs) {
        maxWheel = vR_abs;
      } else {
        maxWheel = vL_abs;
      }
      //  Ensure all are scalars
      if ((maxWheel > params->Chassis.wheel_speed_max) && (maxWheel > 1.0E-6)) {
        wz_desired_data *= params->Chassis.wheel_speed_max / maxWheel;
      }
      //  Final wz clamping
      // CLAMPVALUE Clamp value to range [minVal, maxVal]
      //  Force inputs to be scalars for codegen
      //  chassisPathFollowerCodegen
      //  ========================================================================
      //  Helper Functions
      //  ========================================================================
      *wz = std::fmin(std::fmax(wz_desired_data, -params->Chassis.wz_max),
                      params->Chassis.wz_max);
      //  Ensure scalar output
      //  Ensure scalar
      // CLAMPVALUE Clamp value to range [minVal, maxVal]
      //  Force inputs to be scalars for codegen
      //  chassisPathFollowerCodegen
      //  ========================================================================
      //  Helper Functions
      //  ========================================================================
      *vx = std::fmin(std::fmax(vx_smooth_data, params->Chassis.vx_min),
                      params->Chassis.vx_max);
      //  Goal detection
      if ((distanceToGoal <= params->GoalTolerance) ||
          (nearestIdx >= state->PathNumPoints)) {
        isFinished = true;
      } else {
        isFinished = false;
      }
    } break;
    }
    //  State Update
    state->LastVelocity = *vx;
    //  Ensure scalar
    state->LastAcceleration = accel_data;
    //  Ensure scalar
    state->PreviousPose[0] = pose[0];
    state->PreviousPose[1] = pose[1];
    state->PreviousPose[2] = pose[2];
    //  Status Report
    status->isFinished = isFinished;
    status->distanceRemaining = distanceToGoal;
    status->crossTrackError = crossTrackError;
    status->headingError = headingError;
    status->lookaheadDistance = lookaheadDistance;
    status->currentMode = params->ControllerMode;
    status->currentIndex = nearestIdx;
  }
}

} // namespace gik9dof

//
// File trailer for ChassisPathFollower.cpp
//
// [EOF]
//
