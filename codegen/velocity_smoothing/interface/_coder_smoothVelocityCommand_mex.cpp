//
// File: _coder_smoothVelocityCommand_mex.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 00:47:40
//

// Include Files
#include "_coder_smoothVelocityCommand_mex.h"
#include "_coder_smoothVelocityCommand_api.h"

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
  mexAtExit(&smoothVelocityCommand_atexit);
  // Module initialization.
  smoothVelocityCommand_initialize();
  // Dispatch the entry-point.
  unsafe_smoothVelocityCommand_mexFunction(nlhs, plhs, nrhs, prhs);
  // Module termination.
  smoothVelocityCommand_terminate();
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
//                const mxArray *prhs[8]
// Return Type  : void
//
void unsafe_smoothVelocityCommand_mexFunction(int32_T nlhs, mxArray *plhs[4],
                                              int32_T nrhs,
                                              const mxArray *prhs[8])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[8];
  const mxArray *outputs[4];
  int32_T i1;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 8) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 8, 4,
                        21, "smoothVelocityCommand");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 21,
                        "smoothVelocityCommand");
  }
  // Call the function.
  for (int32_T i{0}; i < 8; i++) {
    b_prhs[i] = prhs[i];
  }
  smoothVelocityCommand_api(b_prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i1 = 1;
  } else {
    i1 = nlhs;
  }
  emlrtReturnArrays(i1, &plhs[0], &outputs[0]);
}

//
// File trailer for _coder_smoothVelocityCommand_mex.cpp
//
// [EOF]
//
