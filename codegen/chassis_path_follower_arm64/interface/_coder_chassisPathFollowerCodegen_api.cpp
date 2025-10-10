//
// File: _coder_chassisPathFollowerCodegen_api.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 11-Oct-2025 00:19:03
//

// Include Files
#include "_coder_chassisPathFollowerCodegen_api.h"
#include "_coder_chassisPathFollowerCodegen_mex.h"
#include "coder_bounded_array.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131643U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "chassisPathFollowerCodegen",                         // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier);

static int32_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId,
                                  real_T y_data[]);

static void b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret_data[], int32_T ret_size[2]);

static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static boolean_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static void c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3]);

static int32_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  real_T ret_data[]);

static real_T (*d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3];

static real_T e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[3]);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct1_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct1_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct2_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct0_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             real_T y_data[], int32_T y_size[2]);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct0_T &y);

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[3];

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[3];

static const mxArray *emlrt_marshallOut(const struct3_T &u);

static const mxArray *emlrt_marshallOut(const real_T u);

static const mxArray *emlrt_marshallOut(const struct0_T &u);

static boolean_T f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

// Function Definitions
//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T
//
static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
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
static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
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
//                real_T y_data[]
// Return Type  : int32_T
//
static int32_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId,
                                  real_T y_data[])
{
  int32_T y_size;
  y_size = d_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data);
  emlrtDestroyArray(&u);
  return y_size;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
//                real_T ret_data[]
//                int32_T ret_size[2]
// Return Type  : void
//
static void b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret_data[], int32_T ret_size[2])
{
  static const int32_T dims[2]{500, 3};
  int32_T iv[2];
  boolean_T bv[2]{true, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret_size[0] = iv[0];
  ret_size[1] = iv[1];
  emlrtImportArrayR2015b((emlrtConstCTX)&sp, src, &ret_data[0], 8, false);
  emlrtDestroyArray(&src);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
//                real_T ret[3]
// Return Type  : void
//
static void c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims[2]{1, 3};
  real_T(*r)[3];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : boolean_T
//
static boolean_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[3]
//
static real_T (*d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims[2]{1, 3};
  real_T(*ret)[3];
  int32_T iv[2];
  boolean_T bv[2]{false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
//                real_T ret_data[]
// Return Type  : int32_T
//
static int32_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId,
                                  real_T ret_data[])
{
  static const int32_T dims{500};
  int32_T ret_size;
  boolean_T b{true};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &ret_size);
  emlrtImportArrayR2015b((emlrtConstCTX)&sp, src, &ret_data[0], 8, false);
  emlrtDestroyArray(&src);
  return ret_size;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T
//
static real_T e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
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
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
//                struct2_T &y
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct2_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[10]{
      "track",      "wheel_speed_max", "vx_max",      "vx_min",
      "wz_max",     "accel_limit",     "decel_limit", "jerk_limit",
      "wheel_base", "reverse_enabled"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 10,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "track";
  y.track = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "track")),
      &thisId);
  thisId.fIdentifier = "wheel_speed_max";
  y.wheel_speed_max =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 1, "wheel_speed_max")),
                         &thisId);
  thisId.fIdentifier = "vx_max";
  y.vx_max = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2, "vx_max")),
      &thisId);
  thisId.fIdentifier = "vx_min";
  y.vx_min = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3, "vx_min")),
      &thisId);
  thisId.fIdentifier = "wz_max";
  y.wz_max = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4, "wz_max")),
      &thisId);
  thisId.fIdentifier = "accel_limit";
  y.accel_limit =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 5, "accel_limit")),
                         &thisId);
  thisId.fIdentifier = "decel_limit";
  y.decel_limit =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 6, "decel_limit")),
                         &thisId);
  thisId.fIdentifier = "jerk_limit";
  y.jerk_limit =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 7, "jerk_limit")),
                         &thisId);
  thisId.fIdentifier = "wheel_base";
  y.wheel_base =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 8, "wheel_base")),
                         &thisId);
  thisId.fIdentifier = "reverse_enabled";
  y.reverse_enabled =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 9, "reverse_enabled")),
                         &thisId);
  emlrtDestroyArray(&u);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[3]
//
static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[3]
{
  real_T(*y)[3];
  y = d_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
// Return Type  : real_T (*)[3]
//
static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3];
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
//                real_T y_data[]
//                int32_T y_size[2]
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             real_T y_data[], int32_T y_size[2])
{
  b_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
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
  static const char_T *fieldNames[17]{"ControllerMode",
                                      "ReverseEnabled",
                                      "LookaheadBase",
                                      "LookaheadVelGain",
                                      "LookaheadAccelGain",
                                      "GoalTolerance",
                                      "HeadingKp",
                                      "HeadingKi",
                                      "HeadingKd",
                                      "FeedforwardGain",
                                      "KappaThreshold",
                                      "VxReduction",
                                      "Chassis",
                                      "PathInfo_States",
                                      "PathInfo_Curvature",
                                      "PathInfo_ArcLength",
                                      "PathInfo_DistanceRemaining"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 17,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "ControllerMode";
  y.ControllerMode =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 0, "ControllerMode")),
                         &thisId);
  thisId.fIdentifier = "ReverseEnabled";
  y.ReverseEnabled =
      c_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 1, "ReverseEnabled")),
                         &thisId);
  thisId.fIdentifier = "LookaheadBase";
  y.LookaheadBase =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 2, "LookaheadBase")),
                         &thisId);
  thisId.fIdentifier = "LookaheadVelGain";
  y.LookaheadVelGain =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 3, "LookaheadVelGain")),
                         &thisId);
  thisId.fIdentifier = "LookaheadAccelGain";
  y.LookaheadAccelGain = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4,
                                     "LookaheadAccelGain")),
      &thisId);
  thisId.fIdentifier = "GoalTolerance";
  y.GoalTolerance =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 5, "GoalTolerance")),
                         &thisId);
  thisId.fIdentifier = "HeadingKp";
  y.HeadingKp = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 6, "HeadingKp")),
      &thisId);
  thisId.fIdentifier = "HeadingKi";
  y.HeadingKi = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 7, "HeadingKi")),
      &thisId);
  thisId.fIdentifier = "HeadingKd";
  y.HeadingKd = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 8, "HeadingKd")),
      &thisId);
  thisId.fIdentifier = "FeedforwardGain";
  y.FeedforwardGain =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 9, "FeedforwardGain")),
                         &thisId);
  thisId.fIdentifier = "KappaThreshold";
  y.KappaThreshold =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 10, "KappaThreshold")),
                         &thisId);
  thisId.fIdentifier = "VxReduction";
  y.VxReduction =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 11, "VxReduction")),
                         &thisId);
  thisId.fIdentifier = "Chassis";
  emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 12, "Chassis")),
      &thisId, y.Chassis);
  thisId.fIdentifier = "PathInfo_States";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 13,
                                                  "PathInfo_States")),
                   &thisId, y.PathInfo_States.data, y.PathInfo_States.size);
  thisId.fIdentifier = "PathInfo_Curvature";
  y.PathInfo_Curvature.size[0] = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 14,
                                     "PathInfo_Curvature")),
      &thisId, y.PathInfo_Curvature.data);
  thisId.fIdentifier = "PathInfo_ArcLength";
  y.PathInfo_ArcLength.size[0] = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 15,
                                     "PathInfo_ArcLength")),
      &thisId, y.PathInfo_ArcLength.data);
  thisId.fIdentifier = "PathInfo_DistanceRemaining";
  y.PathInfo_DistanceRemaining.size[0] = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 16,
                                     "PathInfo_DistanceRemaining")),
      &thisId, y.PathInfo_DistanceRemaining.data);
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
//                real_T y[3]
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[3])
{
  c_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
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
  static const char_T *fieldNames[8]{"PathNumPoints",    "CurrentIndex",
                                     "LastVelocity",     "LastAcceleration",
                                     "LastHeadingError", "IntegralHeadingError",
                                     "PreviousPose",     "DistanceTraveled"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 8,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "PathNumPoints";
  y.PathNumPoints =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 0, "PathNumPoints")),
                         &thisId);
  thisId.fIdentifier = "CurrentIndex";
  y.CurrentIndex =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 1, "CurrentIndex")),
                         &thisId);
  thisId.fIdentifier = "LastVelocity";
  y.LastVelocity =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 2, "LastVelocity")),
                         &thisId);
  thisId.fIdentifier = "LastAcceleration";
  y.LastAcceleration =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 3, "LastAcceleration")),
                         &thisId);
  thisId.fIdentifier = "LastHeadingError";
  y.LastHeadingError =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 4, "LastHeadingError")),
                         &thisId);
  thisId.fIdentifier = "IntegralHeadingError";
  y.IntegralHeadingError = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 5,
                                     "IntegralHeadingError")),
      &thisId);
  thisId.fIdentifier = "PreviousPose";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 6,
                                                  "PreviousPose")),
                   &thisId, y.PreviousPose);
  thisId.fIdentifier = "DistanceTraveled";
  y.DistanceTraveled =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 7, "DistanceTraveled")),
                         &thisId);
  emlrtDestroyArray(&u);
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
// Arguments    : const struct3_T &u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const struct3_T &u)
{
  static const char_T *sv[8]{
      "isFinished", "distanceRemaining", "crossTrackError", "headingError",
      "curvature",  "lookaheadDistance", "currentMode",     "currentIndex"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T i;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&sv[0]));
  b_y = nullptr;
  m = emlrtCreateLogicalScalar(u.isFinished);
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "isFinished", b_y, 0);
  emlrtSetFieldR2017b(y, 0, "distanceRemaining",
                      emlrt_marshallOut(u.distanceRemaining), 1);
  emlrtSetFieldR2017b(y, 0, "crossTrackError",
                      emlrt_marshallOut(u.crossTrackError), 2);
  emlrtSetFieldR2017b(y, 0, "headingError", emlrt_marshallOut(u.headingError),
                      3);
  c_y = nullptr;
  i = 1;
  m = emlrtCreateNumericArray(1, &i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.curvature.data[0];
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "curvature", c_y, 4);
  emlrtSetFieldR2017b(y, 0, "lookaheadDistance",
                      emlrt_marshallOut(u.lookaheadDistance), 5);
  emlrtSetFieldR2017b(y, 0, "currentMode", emlrt_marshallOut(u.currentMode), 6);
  emlrtSetFieldR2017b(y, 0, "currentIndex", emlrt_marshallOut(u.currentIndex),
                      7);
  return y;
}

//
// Arguments    : const struct0_T &u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const struct0_T &u)
{
  static const int32_T iv[2]{1, 3};
  static const char_T *sv[8]{"PathNumPoints",    "CurrentIndex",
                             "LastVelocity",     "LastAcceleration",
                             "LastHeadingError", "IntegralHeadingError",
                             "PreviousPose",     "DistanceTraveled"};
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&sv[0]));
  emlrtSetFieldR2017b(y, 0, "PathNumPoints", emlrt_marshallOut(u.PathNumPoints),
                      0);
  emlrtSetFieldR2017b(y, 0, "CurrentIndex", emlrt_marshallOut(u.CurrentIndex),
                      1);
  emlrtSetFieldR2017b(y, 0, "LastVelocity", emlrt_marshallOut(u.LastVelocity),
                      2);
  emlrtSetFieldR2017b(y, 0, "LastAcceleration",
                      emlrt_marshallOut(u.LastAcceleration), 3);
  emlrtSetFieldR2017b(y, 0, "LastHeadingError",
                      emlrt_marshallOut(u.LastHeadingError), 4);
  emlrtSetFieldR2017b(y, 0, "IntegralHeadingError",
                      emlrt_marshallOut(u.IntegralHeadingError), 5);
  b_y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.PreviousPose[0];
  pData[1] = u.PreviousPose[1];
  pData[2] = u.PreviousPose[2];
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "PreviousPose", b_y, 6);
  emlrtSetFieldR2017b(y, 0, "DistanceTraveled",
                      emlrt_marshallOut(u.DistanceTraveled), 7);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : boolean_T
//
static boolean_T f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  boolean_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "logical", false, 0U,
                          (const void *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const mxArray * const prhs[4]
//                int32_T nlhs
//                const mxArray *plhs[4]
// Return Type  : void
//
void chassisPathFollowerCodegen_api(const mxArray *const prhs[4], int32_T nlhs,
                                    const mxArray *plhs[4])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  struct0_T state;
  struct1_T params;
  struct3_T status;
  real_T(*pose)[3];
  real_T dt;
  real_T vx;
  real_T wz;
  st.tls = emlrtRootTLSGlobal;
  // Marshall function inputs
  pose = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "pose");
  dt = b_emlrt_marshallIn(st, emlrtAliasP(prhs[1]), "dt");
  emlrt_marshallIn(st, emlrtAliasP(prhs[2]), "state", state);
  emlrt_marshallIn(st, emlrtAliasP(prhs[3]), "params", params);
  // Invoke the target function
  chassisPathFollowerCodegen(*pose, dt, &state, &params, &vx, &wz, &status);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(vx);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(wz);
  }
  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(state);
  }
  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(status);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void chassisPathFollowerCodegen_atexit()
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
  chassisPathFollowerCodegen_xil_terminate();
  chassisPathFollowerCodegen_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void chassisPathFollowerCodegen_initialize()
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
void chassisPathFollowerCodegen_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_chassisPathFollowerCodegen_api.cpp
//
// [EOF]
//
