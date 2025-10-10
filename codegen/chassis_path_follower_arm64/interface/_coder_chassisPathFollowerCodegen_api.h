//
// File: _coder_chassisPathFollowerCodegen_api.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 11-Oct-2025 00:19:03
//

#ifndef _CODER_CHASSISPATHFOLLOWERCODEGEN_API_H
#define _CODER_CHASSISPATHFOLLOWERCODEGEN_API_H

// Include Files
#include "coder_bounded_array.h"
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct struct2_T {
  real_T track;
  real_T wheel_speed_max;
  real_T vx_max;
  real_T vx_min;
  real_T wz_max;
  real_T accel_limit;
  real_T decel_limit;
  real_T jerk_limit;
  real_T wheel_base;
  boolean_T reverse_enabled;
};

struct struct0_T {
  real_T PathNumPoints;
  real_T CurrentIndex;
  real_T LastVelocity;
  real_T LastAcceleration;
  real_T LastHeadingError;
  real_T IntegralHeadingError;
  real_T PreviousPose[3];
  real_T DistanceTraveled;
};

struct struct1_T {
  real_T ControllerMode;
  boolean_T ReverseEnabled;
  real_T LookaheadBase;
  real_T LookaheadVelGain;
  real_T LookaheadAccelGain;
  real_T GoalTolerance;
  real_T HeadingKp;
  real_T HeadingKi;
  real_T HeadingKd;
  real_T FeedforwardGain;
  real_T KappaThreshold;
  real_T VxReduction;
  struct2_T Chassis;
  coder::bounded_array<real_T, 1500U, 2U> PathInfo_States;
  coder::bounded_array<real_T, 500U, 1U> PathInfo_Curvature;
  coder::bounded_array<real_T, 500U, 1U> PathInfo_ArcLength;
  coder::bounded_array<real_T, 500U, 1U> PathInfo_DistanceRemaining;
};

struct struct3_T {
  boolean_T isFinished;
  real_T distanceRemaining;
  real_T crossTrackError;
  real_T headingError;
  coder::bounded_array<real_T, 1U, 1U> curvature;
  real_T lookaheadDistance;
  real_T currentMode;
  real_T currentIndex;
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void chassisPathFollowerCodegen(real_T pose[3], real_T dt, struct0_T *state,
                                struct1_T *params, real_T *vx, real_T *wz,
                                struct3_T *status);

void chassisPathFollowerCodegen_api(const mxArray *const prhs[4], int32_T nlhs,
                                    const mxArray *plhs[4]);

void chassisPathFollowerCodegen_atexit();

void chassisPathFollowerCodegen_initialize();

void chassisPathFollowerCodegen_terminate();

void chassisPathFollowerCodegen_xil_shutdown();

void chassisPathFollowerCodegen_xil_terminate();

#endif
//
// File trailer for _coder_chassisPathFollowerCodegen_api.h
//
// [EOF]
//
