//
// _coder_gik9dof_codegen_followTrajectory_api.h
//
// Code generation for function 'gik9dof.codegen.followTrajectory'
//

#ifndef _CODER_GIK9DOF_CODEGEN_FOLLOWTRAJECTORY_API_H
#define _CODER_GIK9DOF_CODEGEN_FOLLOWTRAJECTORY_API_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void followTrajectory(real_T q0[9], real_T poses_data[], int32_T poses_size[3],
                      real_T distanceLower, real_T distanceWeight,
                      real_T qOut[9]);

void gik9dof_codegen_followTrajectory_api(const mxArray *const prhs[4],
                                          const mxArray **plhs);

void gik9dof_codegen_followTrajectory_atexit();

void gik9dof_codegen_followTrajectory_initialize();

void gik9dof_codegen_followTrajectory_terminate();

void gik9dof_codegen_followTrajectory_xil_shutdown();

void gik9dof_codegen_followTrajectory_xil_terminate();

#endif
// End of code generation (_coder_gik9dof_codegen_followTrajectory_api.h)
