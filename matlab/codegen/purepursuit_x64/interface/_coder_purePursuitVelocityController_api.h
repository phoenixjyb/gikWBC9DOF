//
// File: _coder_purePursuitVelocityController_api.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 03:43:59
//

#ifndef _CODER_PUREPURSUITVELOCITYCONTROLLER_API_H
#define _CODER_PUREPURSUITVELOCITYCONTROLLER_API_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct struct0_T {
  real_T lookaheadBase;
  real_T lookaheadVelGain;
  real_T lookaheadTimeGain;
  real_T vxNominal;
  real_T vxMax;
  real_T vxMin;
  real_T wzMax;
  real_T track;
  real_T vwheelMax;
  real_T waypointSpacing;
  real_T pathBufferSize;
  real_T goalTolerance;
  real_T interpSpacing;
};

struct struct1_T {
  real_T pathX[30];
  real_T pathY[30];
  real_T pathTheta[30];
  real_T pathTime[30];
  uint32_T numWaypoints;
  real_T prevVx;
  real_T prevWz;
  real_T prevPoseX;
  real_T prevPoseY;
  real_T prevPoseYaw;
  real_T lastRefTime;
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void purePursuitVelocityController(real_T refX, real_T refY, real_T refTheta,
                                   real_T refTime, real_T estX, real_T estY,
                                   real_T estYaw, struct0_T *params,
                                   struct1_T *stateIn, real_T *vx, real_T *wz,
                                   struct1_T *stateOut);

void purePursuitVelocityController_api(const mxArray *const prhs[9],
                                       int32_T nlhs, const mxArray *plhs[3]);

void purePursuitVelocityController_atexit();

void purePursuitVelocityController_initialize();

void purePursuitVelocityController_terminate();

void purePursuitVelocityController_xil_shutdown();

void purePursuitVelocityController_xil_terminate();

#endif
//
// File trailer for _coder_purePursuitVelocityController_api.h
//
// [EOF]
//
