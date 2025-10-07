//
// File: _coder_holisticVelocityController_api.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 11:21:02
//

#ifndef _CODER_HOLISTICVELOCITYCONTROLLER_API_H
#define _CODER_HOLISTICVELOCITYCONTROLLER_API_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct struct0_T {
  real_T track;
  real_T Vwheel_max;
  real_T Vx_max;
  real_T W_max;
  real_T yawKp;
  real_T yawKff;
};

struct struct2_T {
  real_T x;
  real_T y;
  real_T theta;
  real_T t;
};

struct struct1_T {
  struct2_T prev;
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void holisticVelocityController(real_T refX, real_T refY, real_T refTheta,
                                real_T refTime, real_T estX, real_T estY,
                                real_T estYaw, struct0_T *params,
                                struct1_T *stateIn, real_T *Vx, real_T *Wz,
                                struct1_T *stateOut);

void holisticVelocityController_api(const mxArray *const prhs[9], int32_T nlhs,
                                    const mxArray *plhs[3]);

void holisticVelocityController_atexit();

void holisticVelocityController_initialize();

void holisticVelocityController_terminate();

void holisticVelocityController_xil_shutdown();

void holisticVelocityController_xil_terminate();

#endif
//
// File trailer for _coder_holisticVelocityController_api.h
//
// [EOF]
//
