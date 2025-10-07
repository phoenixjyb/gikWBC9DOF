//
// File: _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_api.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "_coder_gik9dof_codegen_inuse_solveGIKStepWrapper_api.h"
#include "_coder_gik9dof_codegen_inuse_solveGIKStepWrapper_mex.h"
#include "coder_array_mex.h"
#include "coder_bounded_array.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131659U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "gik9dof_codegen_inuse_solveGIKStepWrapper",          // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[16];

static real_T (*b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[16];

static real_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier);

static real_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T (*d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[9];

static real_T (*e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16];

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[9];

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[9];

static const mxArray *emlrt_marshallOut(real_T u[9]);

static const mxArray *emlrt_marshallOut(const emlrtStack &sp,
                                        const struct0_T &u);

static real_T f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

// Function Definitions
//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
// Return Type  : real_T (*)[16]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[16]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[16];
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
// Return Type  : real_T (*)[16]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[16]
{
  real_T(*y)[16];
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
// Return Type  : real_T
//
static real_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T
//
static real_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[9]
//
static real_T (*d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[9]
{
  static const int32_T dims{9};
  real_T(*ret)[9];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[9])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[16]
//
static real_T (*e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16]
{
  static const int32_T dims[2]{4, 4};
  real_T(*ret)[16];
  int32_T iv[2];
  boolean_T bv[2]{false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[16])emlrtMxGetData(src);
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
// Return Type  : real_T (*)[9]
//
static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[9]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[9];
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
// Return Type  : real_T (*)[9]
//
static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[9]
{
  real_T(*y)[9];
  y = d_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : real_T u[9]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(real_T u[9])
{
  static const int32_T i{0};
  static const int32_T i1{9};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const struct0_T &u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const emlrtStack &sp,
                                        const struct0_T &u)
{
  static const char_T *sv[5]{"Iterations", "NumRandomRestarts",
                             "ConstraintViolations", "ExitFlag", "Status"};
  static const char_T *sv1[2]{"Type", "Violation"};
  coder::array<real_T, 2U> c_u;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *m;
  const mxArray *y;
  struct1_T b_u[3];
  const struct1_T *r;
  real_T *pData;
  int32_T iv[2];
  char_T u_data[8];
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)&sp);
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 5, (const char_T **)&sv[0]));
  b_y = nullptr;
  m = emlrtCreateDoubleScalar(u.Iterations);
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "Iterations", b_y, 0);
  c_y = nullptr;
  m = emlrtCreateDoubleScalar(u.NumRandomRestarts);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "NumRandomRestarts", c_y, 1);
  b_u[0] = u.ConstraintViolations[0];
  b_u[1] = u.ConstraintViolations[1];
  b_u[2] = u.ConstraintViolations[2];
  d_y = nullptr;
  iv[0] = 1;
  iv[1] = 3;
  emlrtAssign(&d_y,
              emlrtCreateStructArray(2, &iv[0], 2, (const char_T **)&sv1[0]));
  for (int32_T i{0}; i < 3; i++) {
    int32_T i1;
    int32_T loop_ub;
    int32_T u_size_idx_1;
    r = &b_u[i];
    u_size_idx_1 = r->Type.size[1];
    loop_ub = r->Type.size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      u_data[i1] = r->Type.data[i1];
    }
    g_y = nullptr;
    iv[0] = 1;
    iv[1] = u_size_idx_1;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a((emlrtConstCTX)&sp, u_size_idx_1, m, &u_data[0]);
    emlrtAssign(&g_y, m);
    emlrtSetFieldR2017b(d_y, i, "Type", g_y, 0);
    i1 = r->Violation.size(1);
    c_u.set_size(1, i1);
    loop_ub = r->Violation.size(1);
    for (u_size_idx_1 = 0; u_size_idx_1 < loop_ub; u_size_idx_1++) {
      c_u[u_size_idx_1] = r->Violation[u_size_idx_1];
    }
    h_y = nullptr;
    iv[0] = 1;
    iv[1] = i1;
    m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    i1 = 0;
    for (u_size_idx_1 = 0; u_size_idx_1 < c_u.size(1); u_size_idx_1++) {
      pData[i1] = c_u[u_size_idx_1];
      i1++;
    }
    emlrtAssign(&h_y, m);
    emlrtSetFieldR2017b(d_y, i, "Violation", h_y, 1);
  }
  emlrtSetFieldR2017b(y, 0, "ConstraintViolations", d_y, 2);
  e_y = nullptr;
  m = emlrtCreateDoubleScalar(u.ExitFlag);
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "ExitFlag", e_y, 3);
  f_y = nullptr;
  iv[0] = 1;
  iv[1] = u.Status.size[1];
  m = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)&sp, u.Status.size[1], m,
                           &u.Status.data[0]);
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, "Status", f_y, 4);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)&sp);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T
//
static real_T f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
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
// Arguments    : const mxArray * const prhs[4]
//                int32_T nlhs
//                const mxArray *plhs[2]
// Return Type  : void
//
void gik9dof_codegen_inuse_solveGIKStepWrapper_api(const mxArray *const prhs[4],
                                                   int32_T nlhs,
                                                   const mxArray *plhs[2])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  struct0_T solverInfo;
  real_T(*targetPose)[16];
  real_T(*qCurrent)[9];
  real_T(*qNext)[9];
  real_T distanceLower;
  real_T distanceWeight;
  st.tls = emlrtRootTLSGlobal;
  qNext = (real_T(*)[9])mxMalloc(sizeof(real_T[9]));
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  // Marshall function inputs
  qCurrent = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "qCurrent");
  targetPose = b_emlrt_marshallIn(st, emlrtAlias(prhs[1]), "targetPose");
  distanceLower = c_emlrt_marshallIn(st, emlrtAliasP(prhs[2]), "distanceLower");
  distanceWeight =
      c_emlrt_marshallIn(st, emlrtAliasP(prhs[3]), "distanceWeight");
  // Invoke the target function
  gik9dof_codegen_inuse_solveGIKStepWrapper(*qCurrent, *targetPose,
                                            distanceLower, distanceWeight,
                                            *qNext, &solverInfo);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(*qNext);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(st, solverInfo);
  }
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

//
// Arguments    : void
// Return Type  : void
//
void gik9dof_codegen_inuse_solveGIKStepWrapper_atexit()
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
  gik9dof_codegen_inuse_solveGIKStepWrapper_xil_terminate();
  gik9dof_codegen_inuse_solveGIKStepWrapper_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void gik9dof_codegen_inuse_solveGIKStepWrapper_initialize()
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
void gik9dof_codegen_inuse_solveGIKStepWrapper_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_gik9dof_codegen_inuse_solveGIKStepWrapper_api.cpp
//
// [EOF]
//
