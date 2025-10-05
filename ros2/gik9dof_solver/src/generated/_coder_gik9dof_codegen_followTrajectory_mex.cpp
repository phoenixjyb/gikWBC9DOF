//
// _coder_gik9dof_codegen_followTrajectory_mex.cpp
//
// Code generation for function 'gik9dof.codegen.followTrajectory'
//

// Include files
#include "_coder_gik9dof_codegen_followTrajectory_mex.h"
#include "_coder_gik9dof_codegen_followTrajectory_api.h"

// Function Definitions
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&gik9dof_codegen_followTrajectory_atexit);
  gik9dof_codegen_followTrajectory_initialize();
  unsafe_gik9dof_codegen_followTrajectory_mexFunction(nlhs, plhs, nrhs, prhs);
  gik9dof_codegen_followTrajectory_terminate();
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void unsafe_gik9dof_codegen_followTrajectory_mexFunction(int32_T nlhs,
                                                         mxArray *plhs[1],
                                                         int32_T nrhs,
                                                         const mxArray *prhs[4])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[4];
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        32, "gik9dof.codegen.followTrajectory");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 32,
                        "gik9dof.codegen.followTrajectory");
  }
  // Call the function.
  b_prhs[0] = prhs[0];
  b_prhs[1] = prhs[1];
  b_prhs[2] = prhs[2];
  b_prhs[3] = prhs[3];
  gik9dof_codegen_followTrajectory_api(b_prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

// End of code generation (_coder_gik9dof_codegen_followTrajectory_mex.cpp)
