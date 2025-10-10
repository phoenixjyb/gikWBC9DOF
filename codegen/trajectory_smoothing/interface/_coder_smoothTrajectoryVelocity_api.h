//
// File: _coder_smoothTrajectoryVelocity_api.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 15:00:48
//

#ifndef _CODER_SMOOTHTRAJECTORYVELOCITY_API_H
#define _CODER_SMOOTHTRAJECTORYVELOCITY_API_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct struct0_T {
  real_T vx_max;
  real_T ax_max;
  real_T jx_max;
  real_T wz_max;
  real_T alpha_max;
  real_T jerk_wz_max;
  char_T smoothing_method[10];
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void smoothTrajectoryVelocity(real_T waypoints_x[5], real_T waypoints_y[5],
                              real_T waypoints_theta[5], real_T t_waypoints[5],
                              real_T t_current, struct0_T *params,
                              real_T *vx_cmd, real_T *wz_cmd, real_T *ax_cmd,
                              real_T *alpha_cmd, real_T *jerk_vx_cmd,
                              real_T *jerk_wz_cmd);

void smoothTrajectoryVelocity_api(const mxArray *const prhs[6], int32_T nlhs,
                                  const mxArray *plhs[6]);

void smoothTrajectoryVelocity_atexit();

void smoothTrajectoryVelocity_initialize();

void smoothTrajectoryVelocity_terminate();

void smoothTrajectoryVelocity_xil_shutdown();

void smoothTrajectoryVelocity_xil_terminate();

#endif
//
// File trailer for _coder_smoothTrajectoryVelocity_api.h
//
// [EOF]
//
