//
// File: _coder_gik9dof_planHybridAStarCodegen_api.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:31:57
//

// Include Files
#include "_coder_gik9dof_planHybridAStarCodegen_api.h"
#include "_coder_gik9dof_planHybridAStarCodegen_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131659U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "gik9dof_planHybridAStarCodegen",                     // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static int32_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId);

static boolean_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static void d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               boolean_T y[40000]);

static void e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct0_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct0_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, OccupancyGrid2D &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             OccupancyGrid2D &y);

static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier);

static const mxArray *emlrt_marshallOut(const struct1_T u[500]);

static const mxArray *emlrt_marshallOut(const struct2_T u);

static real_T f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static int32_T g_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId);

static boolean_T h_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static void i_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               boolean_T ret[40000]);

// Function Definitions
//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : int32_T
//
static int32_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId)
{
  int32_T y;
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
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
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
//                boolean_T y[40000]
// Return Type  : void
//
static void d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               boolean_T y[40000])
{
  i_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : void
//
static void e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 0, nullptr, 0U,
                         (const void *)&dims);
  emlrtDestroyArray(&u);
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
  static const char_T *fieldNames[14]{
      "x",  "y",  "theta", "grid_x", "grid_y", "theta_bin",  "Vx",
      "Wz", "dt", "g",     "h",      "f",      "parent_idx", "is_valid"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 14,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "x";
  y.x = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "x")),
      &thisId);
  thisId.fIdentifier = "y";
  y.y = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1, "y")),
      &thisId);
  thisId.fIdentifier = "theta";
  y.theta = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2, "theta")),
      &thisId);
  thisId.fIdentifier = "grid_x";
  y.grid_x = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3, "grid_x")),
      &thisId);
  thisId.fIdentifier = "grid_y";
  y.grid_y = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4, "grid_y")),
      &thisId);
  thisId.fIdentifier = "theta_bin";
  y.theta_bin = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 5, "theta_bin")),
      &thisId);
  thisId.fIdentifier = "Vx";
  y.Vx = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 6, "Vx")),
      &thisId);
  thisId.fIdentifier = "Wz";
  y.Wz = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 7, "Wz")),
      &thisId);
  thisId.fIdentifier = "dt";
  y.dt = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 8, "dt")),
      &thisId);
  thisId.fIdentifier = "g";
  y.g = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 9, "g")),
      &thisId);
  thisId.fIdentifier = "h";
  y.h = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 10, "h")),
      &thisId);
  thisId.fIdentifier = "f";
  y.f = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 11, "f")),
      &thisId);
  thisId.fIdentifier = "parent_idx";
  y.parent_idx =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 12, "parent_idx")),
                         &thisId);
  thisId.fIdentifier = "is_valid";
  y.is_valid = c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 13, "is_valid")),
      &thisId);
  emlrtDestroyArray(&u);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
//                OccupancyGrid2D &y
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, OccupancyGrid2D &y)
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
//                OccupancyGrid2D &y
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             OccupancyGrid2D &y)
{
  emlrtMsgIdentifier thisId;
  const mxArray *propValues[6];
  const char_T *propClasses[6]{
      "gik9dof.OccupancyGrid2D", "gik9dof.OccupancyGrid2D",
      "gik9dof.OccupancyGrid2D", "gik9dof.OccupancyGrid2D",
      "gik9dof.OccupancyGrid2D", "gik9dof.OccupancyGrid2D"};
  const char_T *propNames[6]{"data",     "resolution", "origin_x",
                             "origin_y", "size_x",     "size_y"};
  for (int32_T i{0}; i < 6; i++) {
    propValues[i] = nullptr;
  }
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckMcosClass2017a((emlrtCTX)&sp, parentId, u,
                           "gik9dof.OccupancyGrid2D");
  emlrtGetAllProperties((emlrtCTX)&sp, u, 0, 6, (const char_T **)&propNames[0],
                        (const char_T **)&propClasses[0], &propValues[0]);
  thisId.fIdentifier = "data";
  d_emlrt_marshallIn(sp, emlrtAlias(propValues[0]), &thisId, y.data);
  thisId.fIdentifier = "resolution";
  y.resolution = emlrt_marshallIn(sp, emlrtAlias(propValues[1]), &thisId);
  thisId.fIdentifier = "origin_x";
  y.origin_x = emlrt_marshallIn(sp, emlrtAlias(propValues[2]), &thisId);
  thisId.fIdentifier = "origin_y";
  y.origin_y = emlrt_marshallIn(sp, emlrtAlias(propValues[3]), &thisId);
  thisId.fIdentifier = "size_x";
  y.size_x = b_emlrt_marshallIn(sp, emlrtAlias(propValues[4]), &thisId);
  thisId.fIdentifier = "size_y";
  y.size_y = b_emlrt_marshallIn(sp, emlrtAlias(propValues[5]), &thisId);
  emlrtDestroyArrays(6, &propValues[0]);
  emlrtDestroyArray(&u);
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
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  e_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
}

//
// Arguments    : const struct1_T u[500]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const struct1_T u[500])
{
  static const char_T *sv[6]{"x", "y", "theta", "Vx", "Wz", "dt"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *m;
  const mxArray *y;
  const struct1_T *r;
  int32_T i;
  y = nullptr;
  i = 500;
  emlrtAssign(&y, emlrtCreateStructArray(1, &i, 6, (const char_T **)&sv[0]));
  for (i = 0; i < 500; i++) {
    real_T b_u;
    r = &u[i];
    b_u = r->x;
    b_y = nullptr;
    m = emlrtCreateDoubleScalar(b_u);
    emlrtAssign(&b_y, m);
    emlrtSetFieldR2017b(y, i, "x", b_y, 0);
    b_u = r->y;
    c_y = nullptr;
    m = emlrtCreateDoubleScalar(b_u);
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(y, i, "y", c_y, 1);
    b_u = r->theta;
    d_y = nullptr;
    m = emlrtCreateDoubleScalar(b_u);
    emlrtAssign(&d_y, m);
    emlrtSetFieldR2017b(y, i, "theta", d_y, 2);
    b_u = r->Vx;
    e_y = nullptr;
    m = emlrtCreateDoubleScalar(b_u);
    emlrtAssign(&e_y, m);
    emlrtSetFieldR2017b(y, i, "Vx", e_y, 3);
    b_u = r->Wz;
    f_y = nullptr;
    m = emlrtCreateDoubleScalar(b_u);
    emlrtAssign(&f_y, m);
    emlrtSetFieldR2017b(y, i, "Wz", f_y, 4);
    b_u = r->dt;
    g_y = nullptr;
    m = emlrtCreateDoubleScalar(b_u);
    emlrtAssign(&g_y, m);
    emlrtSetFieldR2017b(y, i, "dt", g_y, 5);
  }
  return y;
}

//
// Arguments    : const struct2_T u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const struct2_T u)
{
  static const char_T *sv[6]{"success",        "iterations",
                             "nodes_expanded", "planning_time_sec",
                             "path_cost",      "path_length"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 6, (const char_T **)&sv[0]));
  b_y = nullptr;
  m = emlrtCreateLogicalScalar(u.success);
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "success", b_y, 0);
  c_y = nullptr;
  m = emlrtCreateDoubleScalar(u.iterations);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "iterations", c_y, 1);
  d_y = nullptr;
  m = emlrtCreateDoubleScalar(u.nodes_expanded);
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, "nodes_expanded", d_y, 2);
  e_y = nullptr;
  m = emlrtCreateDoubleScalar(u.planning_time_sec);
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "planning_time_sec", e_y, 3);
  f_y = nullptr;
  m = emlrtCreateDoubleScalar(u.path_cost);
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, "path_cost", f_y, 4);
  g_y = nullptr;
  m = emlrtCreateDoubleScalar(u.path_length);
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, "path_length", g_y, 5);
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
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : int32_T
//
static int32_T g_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  int32_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "int32", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<int32_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : boolean_T
//
static boolean_T h_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
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
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
//                boolean_T ret[40000]
// Return Type  : void
//
static void i_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               boolean_T ret[40000])
{
  static const int32_T dims[2]{200, 200};
  boolean_T(*r)[40000];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "logical", false, 2U,
                          (const void *)&dims[0]);
  r = (boolean_T(*)[40000])emlrtMxGetLogicals(src);
  for (int32_T i{0}; i < 40000; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

//
// Arguments    : const mxArray * const prhs[4]
//                int32_T nlhs
//                const mxArray *plhs[2]
// Return Type  : void
//
void gik9dof_planHybridAStarCodegen_api(const mxArray *const prhs[4],
                                        int32_T nlhs, const mxArray *plhs[2])
{
  OccupancyGrid2D occupancy_grid;
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  struct0_T goal_state;
  struct0_T start_state;
  struct1_T path[500];
  struct2_T search_stats;
  st.tls = emlrtRootTLSGlobal;
  // Marshall function inputs
  emlrt_marshallIn(st, emlrtAliasP(prhs[0]), "start_state", start_state);
  emlrt_marshallIn(st, emlrtAliasP(prhs[1]), "goal_state", goal_state);
  emlrt_marshallIn(st, emlrtAliasP(prhs[2]), "occupancy_grid", occupancy_grid);
  emlrt_marshallIn(st, emlrtAliasP(prhs[3]), "options");
  // Invoke the target function
  gik9dof_planHybridAStarCodegen(&start_state, &goal_state, &occupancy_grid,
                                 path, &search_stats);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(path);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(search_stats);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void gik9dof_planHybridAStarCodegen_atexit()
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
  gik9dof_planHybridAStarCodegen_xil_terminate();
  gik9dof_planHybridAStarCodegen_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void gik9dof_planHybridAStarCodegen_initialize()
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
void gik9dof_planHybridAStarCodegen_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_gik9dof_planHybridAStarCodegen_api.cpp
//
// [EOF]
//
