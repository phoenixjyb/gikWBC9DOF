//
// File: _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_mex.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
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
//                const mxArray *prhs[7]
// Return Type  : void
//
void unsafe_gik9dof_codegen_inuse_solveGIKStepWrapper_mexFunction(
    int32_T nlhs, mxArray *plhs[2], int32_T nrhs, const mxArray *prhs[7])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[7];
  const mxArray *outputs[2];
  int32_T i1;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 7, 4,
                        41, "gik9dof.codegen_inuse.solveGIKStepWrapper");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 41,
                        "gik9dof.codegen_inuse.solveGIKStepWrapper");
  }
  // Call the function.
  for (int32_T i{0}; i < 7; i++) {
    b_prhs[i] = prhs[i];
  }
  gik9dof_codegen_inuse_solveGIKStepWrapper_api(b_prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i1 = 1;
  } else {
    i1 = nlhs;
  }
  emlrtReturnArrays(i1, &plhs[0], &outputs[0]);
}

//
// File trailer for _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_mex.cpp
//
// [EOF]
//
