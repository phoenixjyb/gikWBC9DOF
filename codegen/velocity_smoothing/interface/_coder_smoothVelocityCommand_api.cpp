//
// File: _coder_smoothVelocityCommand_api.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 00:47:40
//

// Include Files
#include "_coder_smoothVelocityCommand_api.h"
#include "_coder_smoothVelocityCommand_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131643U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "smoothVelocityCommand",                              // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static struct0_T b_emlrt_marshallIn(const emlrtStack &sp,
                                    const mxArray *b_nullptr,
                                    const char_T *identifier);

static struct0_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static real_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                               const char_T *identifier);

static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(const real_T u);

// Function Definitions
//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
// Return Type  : struct0_T
//
static struct0_T b_emlrt_marshallIn(const emlrtStack &sp,
                                    const mxArray *b_nullptr,
                                    const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  struct0_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : struct0_T
//
static struct0_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[6]{"vx_max", "ax_max",    "jx_max",
                                     "wz_max", "alpha_max", "jerk_wz_max"};
  emlrtMsgIdentifier thisId;
  struct0_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 6,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "vx_max";
  y.vx_max = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "vx_max")),
      &thisId);
  thisId.fIdentifier = "ax_max";
  y.ax_max = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1, "ax_max")),
      &thisId);
  thisId.fIdentifier = "jx_max";
  y.jx_max = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2, "jx_max")),
      &thisId);
  thisId.fIdentifier = "wz_max";
  y.wz_max = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3, "wz_max")),
      &thisId);
  thisId.fIdentifier = "alpha_max";
  y.alpha_max = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4, "alpha_max")),
      &thisId);
  thisId.fIdentifier = "jerk_wz_max";
  y.jerk_wz_max =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      5, "jerk_wz_max")),
                       &thisId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T
//
static real_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const void *r
// Return Type  : void
//
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
// Return Type  : real_T
//
static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T
//
static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const real_T u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const mxArray * const prhs[8]
//                int32_T nlhs
//                const mxArray *plhs[4]
// Return Type  : void
//
void smoothVelocityCommand_api(const mxArray *const prhs[8], int32_T nlhs,
                               const mxArray *plhs[4])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  struct0_T params;
  real_T alpha_out;
  real_T alpha_prev;
  real_T ax_out;
  real_T ax_prev;
  real_T dt;
  real_T vx_prev;
  real_T vx_smooth;
  real_T vx_target;
  real_T wz_prev;
  real_T wz_smooth;
  real_T wz_target;
  st.tls = emlrtRootTLSGlobal;
  // Marshall function inputs
  vx_target = emlrt_marshallIn(st, emlrtAliasP(prhs[0]), "vx_target");
  wz_target = emlrt_marshallIn(st, emlrtAliasP(prhs[1]), "wz_target");
  vx_prev = emlrt_marshallIn(st, emlrtAliasP(prhs[2]), "vx_prev");
  wz_prev = emlrt_marshallIn(st, emlrtAliasP(prhs[3]), "wz_prev");
  ax_prev = emlrt_marshallIn(st, emlrtAliasP(prhs[4]), "ax_prev");
  alpha_prev = emlrt_marshallIn(st, emlrtAliasP(prhs[5]), "alpha_prev");
  dt = emlrt_marshallIn(st, emlrtAliasP(prhs[6]), "dt");
  params = b_emlrt_marshallIn(st, emlrtAliasP(prhs[7]), "params");
  // Invoke the target function
  smoothVelocityCommand(vx_target, wz_target, vx_prev, wz_prev, ax_prev,
                        alpha_prev, dt, &params, &vx_smooth, &wz_smooth,
                        &ax_out, &alpha_out);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(vx_smooth);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(wz_smooth);
  }
  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(ax_out);
  }
  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(alpha_out);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void smoothVelocityCommand_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(&st, false, nullptr,
                                    (void *)&emlrtExitTimeCleanupDtorFcn,
                                    nullptr, nullptr, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  smoothVelocityCommand_xil_terminate();
  smoothVelocityCommand_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void smoothVelocityCommand_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void smoothVelocityCommand_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_smoothVelocityCommand_api.cpp
//
// [EOF]
//
