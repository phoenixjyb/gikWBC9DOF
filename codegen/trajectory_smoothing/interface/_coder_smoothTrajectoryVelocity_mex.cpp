//
// File: _coder_smoothTrajectoryVelocity_mex.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 15:00:48
//

// Include Files
#include "_coder_smoothTrajectoryVelocity_mex.h"
#include "_coder_smoothTrajectoryVelocity_api.h"

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
  mexAtExit(&smoothTrajectoryVelocity_atexit);
  // Module initialization.
  smoothTrajectoryVelocity_initialize();
  // Dispatch the entry-point.
  unsafe_smoothTrajectoryVelocity_mexFunction(nlhs, plhs, nrhs, prhs);
  // Module termination.
  smoothTrajectoryVelocity_terminate();
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
//                mxArray *plhs[6]
//                int32_T nrhs
//                const mxArray *prhs[6]
// Return Type  : void
//
void unsafe_smoothTrajectoryVelocity_mexFunction(int32_T nlhs, mxArray *plhs[6],
                                                 int32_T nrhs,
                                                 const mxArray *prhs[6])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[6];
  const mxArray *outputs[6];
  int32_T i1;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        24, "smoothTrajectoryVelocity");
  }
  if (nlhs > 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 24,
                        "smoothTrajectoryVelocity");
  }
  // Call the function.
  for (int32_T i{0}; i < 6; i++) {
    b_prhs[i] = prhs[i];
  }
  smoothTrajectoryVelocity_api(b_prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i1 = 1;
  } else {
    i1 = nlhs;
  }
  emlrtReturnArrays(i1, &plhs[0], &outputs[0]);
}

//
// File trailer for _coder_smoothTrajectoryVelocity_mex.cpp
//
// [EOF]
//
