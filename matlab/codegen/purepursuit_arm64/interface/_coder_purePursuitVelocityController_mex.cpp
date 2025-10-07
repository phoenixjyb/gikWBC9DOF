//
// File: _coder_purePursuitVelocityController_mex.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 03:43:41
//

// Include Files
#include "_coder_purePursuitVelocityController_mex.h"
#include "_coder_purePursuitVelocityController_api.h"

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
  mexAtExit(&purePursuitVelocityController_atexit);
  purePursuitVelocityController_initialize();
  unsafe_purePursuitVelocityController_mexFunction(nlhs, plhs, nrhs, prhs);
  purePursuitVelocityController_terminate();
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
//                mxArray *plhs[3]
//                int32_T nrhs
//                const mxArray *prhs[9]
// Return Type  : void
//
void unsafe_purePursuitVelocityController_mexFunction(int32_T nlhs,
                                                      mxArray *plhs[3],
                                                      int32_T nrhs,
                                                      const mxArray *prhs[9])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[9];
  const mxArray *outputs[3];
  int32_T i1;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 9) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 9, 4,
                        29, "purePursuitVelocityController");
  }
  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 29,
                        "purePursuitVelocityController");
  }
  // Call the function.
  for (int32_T i{0}; i < 9; i++) {
    b_prhs[i] = prhs[i];
  }
  purePursuitVelocityController_api(b_prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i1 = 1;
  } else {
    i1 = nlhs;
  }
  emlrtReturnArrays(i1, &plhs[0], &outputs[0]);
}

//
// File trailer for _coder_purePursuitVelocityController_mex.cpp
//
// [EOF]
//
