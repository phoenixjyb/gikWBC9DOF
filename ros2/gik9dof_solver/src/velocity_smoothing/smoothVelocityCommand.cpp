//
// File: smoothVelocityCommand.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 00:47:40
//

// Include Files
#include "smoothVelocityCommand.h"
#include "rt_nonfinite.h"
#include "smoothVelocityCommand_types.h"
#include <cmath>

// Function Definitions
//
// SMOOTHVELOCITYCOMMAND Apply S-curve smoothing to velocity commands
//
//  This function applies acceleration and jerk limits to raw velocity commands
//  from any controller (Pure Pursuit, Heading Controller, etc.)
//
//  Inputs:
//    vx_target    - Target forward velocity (m/s)
//    wz_target    - Target angular velocity (rad/s)
//    vx_prev      - Previous forward velocity (m/s)
//    wz_prev      - Previous angular velocity (rad/s)
//    ax_prev      - Previous forward acceleration (m/s²)
//    alpha_prev   - Previous angular acceleration (rad/s²)
//    dt           - Time step (seconds)
//    params       - Structure with fields:
//                   .vx_max        - Max forward velocity (m/s)
//                   .ax_max        - Max forward acceleration (m/s²)
//                   .jx_max        - Max forward jerk (m/s³)
//                   .wz_max        - Max angular velocity (rad/s)
//                   .alpha_max     - Max angular acceleration (rad/s²)
//                   .jerk_wz_max   - Max angular jerk (rad/s³)
//
//  Outputs:
//    vx_smooth    - Smoothed forward velocity (m/s)
//    wz_smooth    - Smoothed angular velocity (rad/s)
//    ax_out       - Forward acceleration (m/s²)
//    alpha_out    - Angular acceleration (rad/s²)
//
//  Strategy:
//    1. Compute desired acceleration to reach target velocity
//    2. Apply jerk limit to acceleration change
//    3. Apply acceleration limit
//    4. Integrate to get smoothed velocity
//    5. Apply velocity limit
//
//  Copyright 2025 - GIK 9DOF Mobile Manipulator Project
//  Jetson AGX Orin Deployment - Velocity Smoothing Module
//
// Arguments    : double vx_target
//                double wz_target
//                double vx_prev
//                double wz_prev
//                double ax_prev
//                double alpha_prev
//                double dt
//                const struct0_T *params
//                double *vx_smooth
//                double *wz_smooth
//                double *ax_out
//                double *alpha_out
// Return Type  : void
//
void smoothVelocityCommand(double vx_target, double wz_target, double vx_prev,
                           double wz_prev, double ax_prev, double alpha_prev,
                           double dt, const VelSmoothParams_T *params,
                           double *vx_smooth, double *wz_smooth, double *ax_out,
                           double *alpha_out)
{
  double d;
  double jerk_vx;
  double jerk_wz;
  //  Input Validation
  //  Default Parameters
  //  m/s
  //  m/s²
  //  m/s³
  //  rad/s
  //  rad/s²
  //  rad/s³
  //  Clamp targets to velocity limits
  vx_target = std::fmax(-params->vx_max, std::fmin(params->vx_max, vx_target));
  wz_target = std::fmax(-params->wz_max, std::fmin(params->wz_max, wz_target));
  //  Forward Velocity Smoothing with S-Curve
  //  Compute desired acceleration
  //  Apply jerk limit to acceleration change
  jerk_vx = ((vx_target - vx_prev) / dt - ax_prev) / dt;
  if (std::abs(jerk_vx) > params->jx_max) {
    if (std::isnan(jerk_vx)) {
      d = rtNaN;
    } else if (jerk_vx < 0.0) {
      d = -1.0;
    } else {
      d = (jerk_vx > 0.0);
    }
    jerk_vx = d * params->jx_max;
  }
  //  Integrate jerk to get acceleration
  *ax_out = ax_prev + jerk_vx * dt;
  //  Apply acceleration limit
  if (std::abs(*ax_out) > params->ax_max) {
    if (std::isnan(*ax_out)) {
      d = rtNaN;
    } else if (*ax_out < 0.0) {
      d = -1.0;
    } else {
      d = (*ax_out > 0.0);
    }
    *ax_out = d * params->ax_max;
  }
  //  Integrate acceleration to get velocity
  *vx_smooth = vx_prev + *ax_out * dt;
  //  Apply velocity limit
  if (std::abs(*vx_smooth) > params->vx_max) {
    if (std::isnan(*vx_smooth)) {
      d = rtNaN;
    } else if (*vx_smooth < 0.0) {
      d = -1.0;
    } else {
      d = (*vx_smooth > 0.0);
    }
    *vx_smooth = d * params->vx_max;
    //  If we hit velocity limit, set acceleration to zero
    *ax_out = 0.0;
  }
  //  Angular Velocity Smoothing with S-Curve
  //  Compute desired angular acceleration
  //  Apply angular jerk limit
  jerk_wz = ((wz_target - wz_prev) / dt - alpha_prev) / dt;
  if (std::abs(jerk_wz) > params->jerk_wz_max) {
    if (std::isnan(jerk_wz)) {
      d = rtNaN;
    } else if (jerk_wz < 0.0) {
      d = -1.0;
    } else {
      d = (jerk_wz > 0.0);
    }
    jerk_wz = d * params->jerk_wz_max;
  }
  //  Integrate jerk to get angular acceleration
  *alpha_out = alpha_prev + jerk_wz * dt;
  //  Apply angular acceleration limit
  if (std::abs(*alpha_out) > params->alpha_max) {
    if (std::isnan(*alpha_out)) {
      d = rtNaN;
    } else if (*alpha_out < 0.0) {
      d = -1.0;
    } else {
      d = (*alpha_out > 0.0);
    }
    *alpha_out = d * params->alpha_max;
  }
  //  Integrate angular acceleration to get angular velocity
  *wz_smooth = wz_prev + *alpha_out * dt;
  //  Apply angular velocity limit
  if (std::abs(*wz_smooth) > params->wz_max) {
    if (std::isnan(*wz_smooth)) {
      d = rtNaN;
    } else if (*wz_smooth < 0.0) {
      d = -1.0;
    } else {
      d = (*wz_smooth > 0.0);
    }
    *wz_smooth = d * params->wz_max;
    //  If we hit velocity limit, set acceleration to zero
    *alpha_out = 0.0;
  }
}

//
// File trailer for smoothVelocityCommand.cpp
//
// [EOF]
//
