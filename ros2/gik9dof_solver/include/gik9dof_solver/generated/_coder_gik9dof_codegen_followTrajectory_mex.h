//
// _coder_gik9dof_codegen_followTrajectory_mex.h
//
// Code generation for function 'gik9dof.codegen.followTrajectory'
//

#ifndef _CODER_GIK9DOF_CODEGEN_FOLLOWTRAJECTORY_MEX_H
#define _CODER_GIK9DOF_CODEGEN_FOLLOWTRAJECTORY_MEX_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_gik9dof_codegen_followTrajectory_mexFunction(
    int32_T nlhs, mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[4]);

#endif
// End of code generation (_coder_gik9dof_codegen_followTrajectory_mex.h)
