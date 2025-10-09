//
// File: _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_api.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 18:33:39
//

#ifndef _CODER_GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_API_H
#define _CODER_GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_API_H

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
void gik9dof_codegen_inuse_solveGIKStepWrapper(
    real_T qCurrent[9], real_T targetPose[16], int32_T distBodyIndices[20],
    int32_T distRefBodyIndices[20], real_T distBoundsLower[20],
    real_T distBoundsUpper[20], real_T distWeights[20], real_T qNext[9],
    struct0_T *solverInfo);

void gik9dof_codegen_inuse_solveGIKStepWrapper_api(const mxArray *const prhs[7],
                                                   int32_T nlhs,
                                                   const mxArray *plhs[2]);

void gik9dof_codegen_inuse_solveGIKStepWrapper_atexit();

void gik9dof_codegen_inuse_solveGIKStepWrapper_initialize();

void gik9dof_codegen_inuse_solveGIKStepWrapper_terminate();

void gik9dof_codegen_inuse_solveGIKStepWrapper_xil_shutdown();

void gik9dof_codegen_inuse_solveGIKStepWrapper_xil_terminate();

#endif
//
// File trailer for _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_api.h
//
// [EOF]
//
