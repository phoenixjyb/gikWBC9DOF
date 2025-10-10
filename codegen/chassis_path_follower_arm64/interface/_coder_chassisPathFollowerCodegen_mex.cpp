//
// File: _coder_chassisPathFollowerCodegen_mex.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 11-Oct-2025 00:19:03
//

// Include Files
#include "_coder_chassisPathFollowerCodegen_mex.h"
#include "_coder_chassisPathFollowerCodegen_api.h"

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
  mexAtExit(&chassisPathFollowerCodegen_atexit);
  // Module initialization.
  chassisPathFollowerCodegen_initialize();
  // Dispatch the entry-point.
  unsafe_chassisPathFollowerCodegen_mexFunction(nlhs, plhs, nrhs, prhs);
  // Module termination.
  chassisPathFollowerCodegen_terminate();
}

//
// Arguments    : void
// Return Type  : emlrtCTX
//
emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, "UTF-8", true);
  return emlrtRootTLSGlobal;
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[4]
//                int32_T nrhs
//                const mxArray *prhs[4]
// Return Type  : void
//
void unsafe_chassisPathFollowerCodegen_mexFunction(int32_T nlhs,
                                                   mxArray *plhs[4],
                                                   int32_T nrhs,
                                                   const mxArray *prhs[4])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[4];
  const mxArray *outputs[4];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        26, "chassisPathFollowerCodegen");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 26,
                        "chassisPathFollowerCodegen");
  }
  // Call the function.
  b_prhs[0] = prhs[0];
  b_prhs[1] = prhs[1];
  b_prhs[2] = prhs[2];
  b_prhs[3] = prhs[3];
  chassisPathFollowerCodegen_api(b_prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

//
// File trailer for _coder_chassisPathFollowerCodegen_mex.cpp
//
// [EOF]
//
