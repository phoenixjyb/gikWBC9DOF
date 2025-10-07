//
// File: _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_mex.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "_coder_gik9dof_codegen_inuse_solveGIKStepWrapper_mex.h"
#include "_coder_gik9dof_codegen_inuse_solveGIKStepWrapper_api.h"

// Function Definitions
//
// Arguments    : int32_T nlhs
//                mxArray *plhs[]
//                int32_T nrhs
//                const mxArray *prhs[]
// Return Type  : void
//
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&gik9dof_codegen_inuse_solveGIKStepWrapper_atexit);
  gik9dof_codegen_inuse_solveGIKStepWrapper_initialize();
  unsafe_gik9dof_codegen_inuse_solveGIKStepWrapper_mexFunction(nlhs, plhs, nrhs,
                                                               prhs);
  gik9dof_codegen_inuse_solveGIKStepWrapper_terminate();
}

//
// Arguments    : void
// Return Type  : emlrtCTX
//
emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[2]
//                int32_T nrhs
//                const mxArray *prhs[4]
// Return Type  : void
//
void unsafe_gik9dof_codegen_inuse_solveGIKStepWrapper_mexFunction(
    int32_T nlhs, mxArray *plhs[2], int32_T nrhs, const mxArray *prhs[4])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[4];
  const mxArray *outputs[2];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        41, "gik9dof.codegen_inuse.solveGIKStepWrapper");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 41,
                        "gik9dof.codegen_inuse.solveGIKStepWrapper");
  }
  // Call the function.
  b_prhs[0] = prhs[0];
  b_prhs[1] = prhs[1];
  b_prhs[2] = prhs[2];
  b_prhs[3] = prhs[3];
  gik9dof_codegen_inuse_solveGIKStepWrapper_api(b_prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

//
// File trailer for _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_mex.cpp
//
// [EOF]
//
