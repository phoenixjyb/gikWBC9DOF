//
// File: _coder_solveGIKStepWrapper_api.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 13:57:39
//

#ifndef _CODER_SOLVEGIKSTEPWRAPPER_API_H
#define _CODER_SOLVEGIKSTEPWRAPPER_API_H

// Include Files
#include "coder_array_mex.h"
#include "coder_bounded_array.h"
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct struct1_T {
  coder::bounded_array<char_T, 8U, 2U> Type;
  coder::array<real_T, 2U> Violation;
};

struct struct0_T {
  real_T Iterations;
  real_T NumRandomRestarts;
  struct1_T ConstraintViolations[22];
  real_T ExitFlag;
  coder::bounded_array<char_T, 14U, 2U> Status;
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void solveGIKStepWrapper(real_T qCurrent[9], real_T targetPose[16],
                         int32_T distBodyIndices[20],
                         int32_T distRefBodyIndices[20],
                         real_T distBoundsLower[20], real_T distBoundsUpper[20],
                         real_T distWeights[20], real_T qNext[9],
                         struct0_T *solverInfo);

void solveGIKStepWrapper_api(const mxArray *const prhs[7], int32_T nlhs,
                             const mxArray *plhs[2]);

void solveGIKStepWrapper_atexit();

void solveGIKStepWrapper_initialize();

void solveGIKStepWrapper_terminate();

void solveGIKStepWrapper_xil_shutdown();

void solveGIKStepWrapper_xil_terminate();

#endif
//
// File trailer for _coder_solveGIKStepWrapper_api.h
//
// [EOF]
//
