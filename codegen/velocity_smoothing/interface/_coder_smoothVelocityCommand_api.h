//
// File: _coder_smoothVelocityCommand_api.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 15:09:44
//

#ifndef _CODER_SMOOTHVELOCITYCOMMAND_API_H
#define _CODER_SMOOTHVELOCITYCOMMAND_API_H

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
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void smoothVelocityCommand(real_T vx_target, real_T wz_target, real_T vx_prev,
                           real_T wz_prev, real_T ax_prev, real_T alpha_prev,
                           real_T dt, struct0_T *params, real_T *vx_smooth,
                           real_T *wz_smooth, real_T *ax_out,
                           real_T *alpha_out);

void smoothVelocityCommand_api(const mxArray *const prhs[8], int32_T nlhs,
                               const mxArray *plhs[4]);

void smoothVelocityCommand_atexit();

void smoothVelocityCommand_initialize();

void smoothVelocityCommand_terminate();

void smoothVelocityCommand_xil_shutdown();

void smoothVelocityCommand_xil_terminate();

#endif
//
// File trailer for _coder_smoothVelocityCommand_api.h
//
// [EOF]
//
