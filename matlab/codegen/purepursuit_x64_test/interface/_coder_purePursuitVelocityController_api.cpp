//
// File: _coder_purePursuitVelocityController_api.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 03:34:11
//

// Include Files
#include "_coder_purePursuitVelocityController_api.h"
#include "_coder_purePursuitVelocityController_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131659U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "purePursuitVelocityController",                      // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static void b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[30]);

static uint32_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static real_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static uint32_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct0_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct0_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct1_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct1_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[30]);

static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                               const char_T *identifier);

static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(const struct1_T &u);

static const mxArray *emlrt_marshallOut(const real_T u[30]);

static const mxArray *emlrt_marshallOut(const real_T u);

// Function Definitions
//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
//                real_T ret[30]
// Return Type  : void
//
static void b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[30])
{
  static const int32_T dims[2]{1, 30};
  real_T(*r)[30];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[30])emlrtMxGetData(src);
  for (int32_T i{0}; i < 30; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : uint32_T
//
static uint32_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  uint32_T y;
  y = d_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
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
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : uint32_T
//
static uint32_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  uint32_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "uint32", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<uint32_T *>(emlrtMxGetData(src));
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
//                struct0_T &y
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct0_T &y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId, y);
  emlrtDestroyArray(&b_nullptr);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
//                struct0_T &y
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct0_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[13]{"lookaheadBase",
                                      "lookaheadVelGain",
                                      "lookaheadTimeGain",
                                      "vxNominal",
                                      "vxMax",
                                      "vxMin",
                                      "wzMax",
                                      "track",
                                      "vwheelMax",
                                      "waypointSpacing",
                                      "pathBufferSize",
                                      "goalTolerance",
                                      "interpSpacing"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 13,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "lookaheadBase";
  y.lookaheadBase =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      0, "lookaheadBase")),
                       &thisId);
  thisId.fIdentifier = "lookaheadVelGain";
  y.lookaheadVelGain =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      1, "lookaheadVelGain")),
                       &thisId);
  thisId.fIdentifier = "lookaheadTimeGain";
  y.lookaheadTimeGain =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      2, "lookaheadTimeGain")),
                       &thisId);
  thisId.fIdentifier = "vxNominal";
  y.vxNominal = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3, "vxNominal")),
      &thisId);
  thisId.fIdentifier = "vxMax";
  y.vxMax = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4, "vxMax")),
      &thisId);
  thisId.fIdentifier = "vxMin";
  y.vxMin = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 5, "vxMin")),
      &thisId);
  thisId.fIdentifier = "wzMax";
  y.wzMax = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 6, "wzMax")),
      &thisId);
  thisId.fIdentifier = "track";
  y.track = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 7, "track")),
      &thisId);
  thisId.fIdentifier = "vwheelMax";
  y.vwheelMax = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 8, "vwheelMax")),
      &thisId);
  thisId.fIdentifier = "waypointSpacing";
  y.waypointSpacing =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      9, "waypointSpacing")),
                       &thisId);
  thisId.fIdentifier = "pathBufferSize";
  y.pathBufferSize =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      10, "pathBufferSize")),
                       &thisId);
  thisId.fIdentifier = "goalTolerance";
  y.goalTolerance =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      11, "goalTolerance")),
                       &thisId);
  thisId.fIdentifier = "interpSpacing";
  y.interpSpacing =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      12, "interpSpacing")),
                       &thisId);
  emlrtDestroyArray(&u);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
//                struct1_T &y
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct1_T &y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId, y);
  emlrtDestroyArray(&b_nullptr);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
//                struct1_T &y
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct1_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[11]{
      "pathX",        "pathY",       "pathTheta",  "pathTime",
      "numWaypoints", "prevVx",      "prevWz",     "prevPoseX",
      "prevPoseY",    "prevPoseYaw", "lastRefTime"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 11,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "pathX";
  emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "pathX")),
      &thisId, y.pathX);
  thisId.fIdentifier = "pathY";
  emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1, "pathY")),
      &thisId, y.pathY);
  thisId.fIdentifier = "pathTheta";
  emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2, "pathTheta")),
      &thisId, y.pathTheta);
  thisId.fIdentifier = "pathTime";
  emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3, "pathTime")),
      &thisId, y.pathTime);
  thisId.fIdentifier = "numWaypoints";
  y.numWaypoints =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 4, "numWaypoints")),
                         &thisId);
  thisId.fIdentifier = "prevVx";
  y.prevVx = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 5, "prevVx")),
      &thisId);
  thisId.fIdentifier = "prevWz";
  y.prevWz = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 6, "prevWz")),
      &thisId);
  thisId.fIdentifier = "prevPoseX";
  y.prevPoseX = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 7, "prevPoseX")),
      &thisId);
  thisId.fIdentifier = "prevPoseY";
  y.prevPoseY = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 8, "prevPoseY")),
      &thisId);
  thisId.fIdentifier = "prevPoseYaw";
  y.prevPoseYaw =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      9, "prevPoseYaw")),
                       &thisId);
  thisId.fIdentifier = "lastRefTime";
  y.lastRefTime =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      10, "lastRefTime")),
                       &thisId);
  emlrtDestroyArray(&u);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
//                real_T y[30]
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[30])
{
  b_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
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
// Arguments    : const struct1_T &u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const struct1_T &u)
{
  static const char_T *sv[11]{"pathX",       "pathY",        "pathTheta",
                              "pathTime",    "numWaypoints", "prevVx",
                              "prevWz",      "prevPoseX",    "prevPoseY",
                              "prevPoseYaw", "lastRefTime"};
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 11, (const char_T **)&sv[0]));
  emlrtSetFieldR2017b(y, 0, "pathX", emlrt_marshallOut(u.pathX), 0);
  emlrtSetFieldR2017b(y, 0, "pathY", emlrt_marshallOut(u.pathY), 1);
  emlrtSetFieldR2017b(y, 0, "pathTheta", emlrt_marshallOut(u.pathTheta), 2);
  emlrtSetFieldR2017b(y, 0, "pathTime", emlrt_marshallOut(u.pathTime), 3);
  b_y = nullptr;
  m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
  *static_cast<uint32_T *>(emlrtMxGetData(m)) = u.numWaypoints;
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "numWaypoints", b_y, 4);
  emlrtSetFieldR2017b(y, 0, "prevVx", emlrt_marshallOut(u.prevVx), 5);
  emlrtSetFieldR2017b(y, 0, "prevWz", emlrt_marshallOut(u.prevWz), 6);
  emlrtSetFieldR2017b(y, 0, "prevPoseX", emlrt_marshallOut(u.prevPoseX), 7);
  emlrtSetFieldR2017b(y, 0, "prevPoseY", emlrt_marshallOut(u.prevPoseY), 8);
  emlrtSetFieldR2017b(y, 0, "prevPoseYaw", emlrt_marshallOut(u.prevPoseYaw), 9);
  emlrtSetFieldR2017b(y, 0, "lastRefTime", emlrt_marshallOut(u.lastRefTime),
                      10);
  return y;
}

//
// Arguments    : const real_T u[30]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u[30])
{
  static const int32_T iv[2]{1, 30};
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (int32_T i{0}; i < 30; i++) {
    pData[i] = u[i];
  }
  emlrtAssign(&y, m);
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
// Arguments    : const mxArray * const prhs[9]
//                int32_T nlhs
//                const mxArray *plhs[3]
// Return Type  : void
//
void purePursuitVelocityController_api(const mxArray *const prhs[9],
                                       int32_T nlhs, const mxArray *plhs[3])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  struct0_T params;
  struct1_T stateIn;
  struct1_T stateOut;
  real_T estX;
  real_T estY;
  real_T estYaw;
  real_T refTheta;
  real_T refTime;
  real_T refX;
  real_T refY;
  real_T vx;
  real_T wz;
  st.tls = emlrtRootTLSGlobal;
  // Marshall function inputs
  refX = emlrt_marshallIn(st, emlrtAliasP(prhs[0]), "refX");
  refY = emlrt_marshallIn(st, emlrtAliasP(prhs[1]), "refY");
  refTheta = emlrt_marshallIn(st, emlrtAliasP(prhs[2]), "refTheta");
  refTime = emlrt_marshallIn(st, emlrtAliasP(prhs[3]), "refTime");
  estX = emlrt_marshallIn(st, emlrtAliasP(prhs[4]), "estX");
  estY = emlrt_marshallIn(st, emlrtAliasP(prhs[5]), "estY");
  estYaw = emlrt_marshallIn(st, emlrtAliasP(prhs[6]), "estYaw");
  emlrt_marshallIn(st, emlrtAliasP(prhs[7]), "params", params);
  emlrt_marshallIn(st, emlrtAliasP(prhs[8]), "stateIn", stateIn);
  // Invoke the target function
  purePursuitVelocityController(refX, refY, refTheta, refTime, estX, estY,
                                estYaw, &params, &stateIn, &vx, &wz, &stateOut);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(vx);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(wz);
  }
  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(stateOut);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void purePursuitVelocityController_atexit()
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
  purePursuitVelocityController_xil_terminate();
  purePursuitVelocityController_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void purePursuitVelocityController_initialize()
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
void purePursuitVelocityController_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_purePursuitVelocityController_api.cpp
//
// [EOF]
//
