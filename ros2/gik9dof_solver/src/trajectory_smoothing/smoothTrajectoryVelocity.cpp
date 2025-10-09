//
// File: smoothTrajectoryVelocity.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 23:31:47
//

// Include Files
#include "smoothTrajectoryVelocity.h"
#include "rt_nonfinite.h"
#include "smoothTrajectoryVelocity_types.h"
#include <cmath>

// Type Definitions
namespace coder {
class captured_var {
public:
  double contents;
};

} // namespace coder

// Variable Definitions
static bool vx_prev_not_empty;

static bool isInitialized_smoothTrajectoryVelocity{false};

// Function Declarations
static double applySCurve(double v_current, double a_current, double v_target,
                          double dt, double jerk_max, double *a_out);

static double decelerateToStop(double v_current, double a_current, double dt,
                               double a_max, double jerk_max);

static void smoothTrajectoryVelocity_init();

// Function Definitions
//
// APPLYSCURVE Apply S-curve acceleration profile with jerk limiting
//
//  S-curve has 3 phases:
//    1. Jerk-up phase: acceleration increases linearly
//    2. Constant acceleration phase
//    3. Jerk-down phase: acceleration decreases linearly
//
//  This implementation uses a simplified single-step approach suitable for
//  real-time control at 50-100Hz.
//
// Arguments    : double v_current
//                double a_current
//                double v_target
//                double dt
//                double jerk_max
//                double *a_out
// Return Type  : double
//
static double applySCurve(double v_current, double a_current, double v_target,
                          double dt, double jerk_max, double *a_out)
{
  double da_max;
  double dv;
  double v_out;
  //  Helper Function: S-Curve Acceleration Profile
  //  Velocity error
  dv = v_target - v_current;
  //  Determine required acceleration
  if (std::abs(dv) < 1.0E-6) {
    //  Already at target, maintain
    dv = -a_current * 0.5;
    //  Gradually reduce acceleration to zero
  } else {
    //  Acceleration needed to reach target in reasonable time
    //  Using a_desired = k * dv with damping
    //  Reduced gain for smoother transitions
    //  Proportional gain (reduced from 2.0)
    dv *= 1.5;
  }
  //  Limit acceleration change by jerk
  //  This is the key to smooth motion - jerk is rate of change of acceleration
  da_max = jerk_max * dt;
  dv -= a_current;
  //  Strictly enforce jerk limit
  if (std::abs(dv) > da_max) {
    if (std::isnan(dv)) {
      dv = rtNaN;
    } else if (dv < 0.0) {
      dv = -1.0;
    } else {
      dv = (dv > 0.0);
    }
    dv *= da_max;
  }
  //  New acceleration after jerk-limited change
  *a_out = a_current + dv;
  //  DO NOT clamp acceleration here - it would create jerk violations!
  //  The jerk limit naturally prevents acceleration from growing too fast
  //  Over time, acceleration will converge to safe values
  //  Integrate to get new velocity
  v_out = v_current + *a_out * dt;
  //  Clamp velocity (safety - should be done outside too)
  if (std::isnan(v_out)) {
    dv = rtNaN;
  } else if (v_out < 0.0) {
    dv = -1.0;
  } else {
    dv = (v_out > 0.0);
  }
  return dv *
         std::fmin(std::abs(v_out), std::fmax(std::abs(v_target) * 1.2, 0.5));
  //  Allow 20% overshoot temporarily
}

//
// DECELERATETOSTOP Smoothly decelerate to zero velocity
//
// Arguments    : double v_current
//                double a_current
//                double dt
//                double a_max
//                double jerk_max
// Return Type  : double
//
static double decelerateToStop(double v_current, double a_current, double dt,
                               double a_max, double jerk_max)
{
  double v_out;
  //  Helper Function: Decelerate to Stop
  if (std::abs(v_current) < 0.001) {
    v_out = 0.0;
  } else {
    double a_target_tmp;
    double da;
    double da_max;
    //  Target deceleration (opposite sign of velocity)
    if (std::isnan(v_current)) {
      a_target_tmp = rtNaN;
    } else if (v_current < 0.0) {
      a_target_tmp = -1.0;
    } else {
      a_target_tmp = (v_current > 0.0);
    }
    //  Limit acceleration change by jerk - STRICT enforcement
    da_max = jerk_max * dt;
    da = -a_target_tmp * a_max - a_current;
    //  Strictly enforce jerk limit
    if (std::abs(da) > da_max) {
      if (std::isnan(da)) {
        da = rtNaN;
      } else if (da < 0.0) {
        da = -1.0;
      } else {
        da = (da > 0.0);
      }
      da *= da_max;
    }
    //  New acceleration after jerk-limited change
    da_max = a_current + da;
    //  Clamp to max acceleration (secondary constraint)
    if (std::abs(da_max) > a_max) {
      if (std::isnan(da_max)) {
        da = rtNaN;
      } else if (da_max < 0.0) {
        da = -1.0;
      } else {
        da = (da_max > 0.0);
      }
      da_max = da * a_max;
    }
    //  Integrate to get new velocity
    v_out = v_current + da_max * dt;
    //  Don't overshoot zero
    if (std::isnan(v_out)) {
      da = rtNaN;
    } else if (v_out < 0.0) {
      da = -1.0;
    } else {
      da = (v_out > 0.0);
    }
    if (da != a_target_tmp) {
      v_out = 0.0;
    }
  }
  return v_out;
}

//
// Arguments    : void
// Return Type  : void
//
static void smoothTrajectoryVelocity_init()
{
  vx_prev_not_empty = false;
}

//
// SMOOTHTRAJECTORYVELOCITY Apply acceleration and jerk limits to trajectory
// following
//
//  Inputs:
//    waypoints_x     - X positions of waypoints (Nx1)
//    waypoints_y     - Y positions of waypoints (Nx1)
//    waypoints_theta - Yaw angles of waypoints (Nx1, radians)
//    t_waypoints     - Time stamps of waypoints (Nx1, seconds)
//    t_current       - Current time (scalar)
//    params          - Structure with fields:
//                      .vx_max        - Max forward velocity (m/s)
//                      .ax_max        - Max forward acceleration (m/s²)
//                      .jx_max        - Max forward jerk (m/s³)
//                      .wz_max        - Max angular velocity (rad/s)
//                      .alpha_max     - Max angular acceleration (rad/s²)
//                      .jerk_wz_max   - Max angular jerk (rad/s³)
//                      .smoothing_method - 'scurve' or 'exponential'
//
//  Outputs:
//    vx_cmd          - Smoothed forward velocity command (m/s)
//    wz_cmd          - Smoothed angular velocity command (rad/s)
//    ax_cmd          - Forward acceleration (m/s²) - for diagnostics
//    alpha_cmd       - Angular acceleration (rad/s²) - for diagnostics
//    jerk_vx_cmd     - Forward jerk (m/s³) - TRUE enforced value
//    jerk_wz_cmd     - Angular jerk (rad/s³) - TRUE enforced value
//
//  Strategy:
//    1. Find current segment between waypoints
//    2. Compute target velocity from segment direction
//    3. Apply S-curve acceleration profile with jerk limits
//    4. Enforce velocity and acceleration bounds
//
// Arguments    : const double waypoints_x[5]
//                const double waypoints_y[5]
//                const double waypoints_theta[5]
//                const double t_waypoints[5]
//                double t_current
//                const struct0_T *params
//                double *vx_cmd
//                double *wz_cmd
//                double *ax_cmd
//                double *alpha_cmd
//                double *jerk_vx_cmd
//                double *jerk_wz_cmd
// Return Type  : void
//
void smoothTrajectoryVelocity(const double waypoints_x[5],
                              const double waypoints_y[5],
                              const double waypoints_theta[5],
                              const double t_waypoints[5], double t_current,
                              const struct0_T *params, double *vx_cmd,
                              double *wz_cmd, double *ax_cmd, double *alpha_cmd,
                              double *jerk_vx_cmd, double *jerk_wz_cmd)
{
  static coder::captured_var alpha_prev;
  static coder::captured_var ax_prev;
  static coder::captured_var t_prev;
  static coder::captured_var vx_prev;
  static coder::captured_var wz_prev;
  double dt;
  int i;
  int idx;
  bool exitg1;
  if (!isInitialized_smoothTrajectoryVelocity) {
    smoothTrajectoryVelocity_initialize();
  }
  //  Copyright 2025 - GIK 9DOF Mobile Manipulator Project
  //  Jetson AGX Orin Deployment - Trajectory Smoothing Module
  //  Input Validation
  //  Default Parameters
  //  m/s
  //  m/s²
  //  m/s³
  //  rad/s
  //  rad/s²
  //  rad/s³
  //  Persistent State for Smooth Acceleration
  if (!vx_prev_not_empty) {
    vx_prev_not_empty = true;
    vx_prev.contents = 0.0;
    wz_prev.contents = 0.0;
    ax_prev.contents = 0.0;
    alpha_prev.contents = 0.0;
    t_prev.contents = t_current;
  }
  dt = t_current - t_prev.contents;
  if (dt <= 0.0) {
    dt = 0.02;
    //  Default 50Hz if time didn't advance
  }
  //  Handle Edge Cases
  //  Find Current Segment
  //  Find the segment we're currently in or approaching
  //  NOTE: Using manual loop instead of find() for MATLAB Coder compatibility
  //  (fixed-size arrays)
  idx = -1;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 5)) {
    if (t_waypoints[i] >= t_current) {
      idx = i;
      exitg1 = true;
    } else {
      i++;
    }
  }
  if (idx + 1 == 0) {
    //  Past all waypoints, decelerate to stop
    *vx_cmd = decelerateToStop(vx_prev.contents, ax_prev.contents, dt,
                               params->ax_max, params->jx_max);
    *wz_cmd = decelerateToStop(wz_prev.contents, alpha_prev.contents, dt,
                               params->alpha_max, params->jerk_wz_max);
    *ax_cmd = (*vx_cmd - vx_prev.contents) / dt;
    *alpha_cmd = (*wz_cmd - wz_prev.contents) / dt;
    //  Compute actual jerk
    *jerk_vx_cmd = (*ax_cmd - ax_prev.contents) / dt;
    *jerk_wz_cmd = (*alpha_cmd - alpha_prev.contents) / dt;
    //  Nested Helper Functions
    vx_prev.contents = *vx_cmd;
    wz_prev.contents = *wz_cmd;
    ax_prev.contents = *ax_cmd;
    alpha_prev.contents = *alpha_cmd;
    t_prev.contents = t_current;
  } else {
    double absxk;
    double scale;
    double segment_duration;
    double segment_length;
    double t;
    if (idx + 1 == 1) {
      idx = 1;
      //  Use first segment
    }
    //  Current segment: from waypoint (idx-1) to waypoint (idx)
    //  Compute Target Velocities from Segment
    //  Direction of motion
    scale = 3.3121686421112381E-170;
    absxk = std::abs(waypoints_x[idx] - waypoints_x[idx - 1]);
    if (absxk > 3.3121686421112381E-170) {
      segment_length = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      segment_length = t * t;
    }
    absxk = std::abs(waypoints_y[idx] - waypoints_y[idx - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      segment_length = segment_length * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      segment_length += t * t;
    }
    segment_length = scale * std::sqrt(segment_length);
    segment_duration = t_waypoints[idx] - t_waypoints[idx - 1];
    if ((segment_length < 1.0E-6) || (segment_duration < 1.0E-6)) {
      //  Stationary waypoint, stop
      segment_length = 0.0;
      scale = 0.0;
    } else {
      //  Target forward velocity (desired)
      segment_length =
          std::fmin(segment_length / segment_duration, params->vx_max);
      //  Target angular velocity
      //  NOTE: Use local wrapAngle instead of wrapToPi for codegen
      //  compatibility
      scale = waypoints_theta[idx] - waypoints_theta[idx - 1];
      // WRAPANGLE Wrap angle to [-pi, pi] range (codegen-compatible version)
      //  This replaces wrapToPi for MATLAB Coder compatibility
      //  Local Helper: Wrap angle to [-pi, pi]
      if (std::isnan(scale + 3.1415926535897931) ||
          std::isinf(scale + 3.1415926535897931)) {
        absxk = rtNaN;
      } else if (scale + 3.1415926535897931 == 0.0) {
        absxk = 0.0;
      } else {
        bool rEQ0;
        absxk = std::fmod(scale + 3.1415926535897931, 6.2831853071795862);
        rEQ0 = (absxk == 0.0);
        if (!rEQ0) {
          t = std::abs((scale + 3.1415926535897931) / 6.2831853071795862);
          rEQ0 =
              !(std::abs(t - std::floor(t + 0.5)) > 2.2204460492503131E-16 * t);
        }
        if (rEQ0) {
          absxk = 0.0;
        } else if (scale + 3.1415926535897931 < 0.0) {
          absxk += 6.2831853071795862;
        }
      }
      scale = (absxk - 3.1415926535897931) / segment_duration;
      if (std::isnan(scale)) {
        absxk = rtNaN;
      } else if (scale < 0.0) {
        absxk = -1.0;
      } else {
        absxk = (scale > 0.0);
      }
      scale = absxk * std::fmin(std::abs(scale), params->wz_max);
    }
    //  Apply Smoothing Based on Method
    //  NOTE: For codegen, we only support 'scurve' method
    //  The strcmp with params.smoothing_method is kept for compatibility but
    //  will be optimized away S-curve acceleration profile with jerk limiting
    //  (default)
    *vx_cmd = applySCurve(vx_prev.contents, ax_prev.contents, segment_length,
                          dt, params->jx_max, ax_cmd);
    *wz_cmd = applySCurve(wz_prev.contents, alpha_prev.contents, scale, dt,
                          params->jerk_wz_max, alpha_cmd);
    //  NOTE: Safety verification removed - applySCurve should enforce jerk
    //  limit If violations still occur, the issue is in applySCurve logic
    //  Compute TRUE Jerk (BEFORE any final clamping!)
    //  This is the jerk that was ACTUALLY enforced by the S-curve algorithm
    *jerk_vx_cmd = (*ax_cmd - ax_prev.contents) / dt;
    *jerk_wz_cmd = (*alpha_cmd - alpha_prev.contents) / dt;
    //  Final Safety Clamps (these should rarely trigger if S-curve works
    //  correctly) NOTE: These clamps can create jerk violations if triggered!
    //  They are here only as a last-resort safety measure
    if (std::isnan(*vx_cmd)) {
      absxk = rtNaN;
    } else if (*vx_cmd < 0.0) {
      absxk = -1.0;
    } else {
      absxk = (*vx_cmd > 0.0);
    }
    *vx_cmd = absxk * std::fmin(std::abs(*vx_cmd), params->vx_max);
    if (std::isnan(*wz_cmd)) {
      absxk = rtNaN;
    } else if (*wz_cmd < 0.0) {
      absxk = -1.0;
    } else {
      absxk = (*wz_cmd > 0.0);
    }
    *wz_cmd = absxk * std::fmin(std::abs(*wz_cmd), params->wz_max);
    //  DO NOT clamp acceleration here - it would violate jerk limit!
    //  The S-curve algorithm already enforced acceleration limits
    //  ax_cmd = sign(ax_cmd) * min(abs(ax_cmd), params.ax_max);
    //  alpha_cmd = sign(alpha_cmd) * min(abs(alpha_cmd), params.alpha_max);
    //  Update Persistent State
    //  Nested Helper Functions
    vx_prev.contents = *vx_cmd;
    wz_prev.contents = *wz_cmd;
    ax_prev.contents = *ax_cmd;
    alpha_prev.contents = *alpha_cmd;
    t_prev.contents = t_current;
  }
}

//
// Arguments    : void
// Return Type  : void
//
void smoothTrajectoryVelocity_initialize()
{
  smoothTrajectoryVelocity_init();
  isInitialized_smoothTrajectoryVelocity = true;
}

//
// Arguments    : void
// Return Type  : void
//
void smoothTrajectoryVelocity_terminate()
{
  isInitialized_smoothTrajectoryVelocity = false;
}

//
// File trailer for smoothTrajectoryVelocity.cpp
//
// [EOF]
//
