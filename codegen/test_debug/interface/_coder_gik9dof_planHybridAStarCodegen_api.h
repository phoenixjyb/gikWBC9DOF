//
// File: _coder_gik9dof_planHybridAStarCodegen_api.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

#ifndef _CODER_GIK9DOF_PLANHYBRIDASTARCODEGEN_API_H
#define _CODER_GIK9DOF_PLANHYBRIDASTARCODEGEN_API_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct struct0_T {
  real_T x;
  real_T y;
  real_T theta;
  int32_T grid_x;
  int32_T grid_y;
  int32_T theta_bin;
  real_T Vx;
  real_T Wz;
  real_T dt;
  real_T g;
  real_T h;
  real_T f;
  int32_T parent_idx;
  boolean_T is_valid;
};

struct struct1_T {
  real_T x;
  real_T y;
  real_T theta;
  real_T Vx;
  real_T Wz;
  real_T dt;
};

struct struct2_T {
  boolean_T success;
  real_T iterations;
  real_T nodes_expanded;
  real_T planning_time_sec;
  real_T path_cost;
  real_T path_length;
};

struct gik9dof_OccupancyGrid2D_tag_0 {
  boolean_T data[40000];
  real_T resolution;
  real_T origin_x;
  real_T origin_y;
  int32_T size_x;
  int32_T size_y;
};
using OccupancyGrid2D = gik9dof_OccupancyGrid2D_tag_0;

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void gik9dof_planHybridAStarCodegen_api(const mxArray *const prhs[4],
                                        int32_T nlhs, const mxArray *plhs[2]);

void gik9dof_planHybridAStarCodegen_atexit();

void gik9dof_planHybridAStarCodegen_initialize();

void gik9dof_planHybridAStarCodegen_terminate();

void gik9dof_planHybridAStarCodegen_xil_shutdown();

void gik9dof_planHybridAStarCodegen_xil_terminate();

void planHybridAStarCodegen(struct0_T *start_state, struct0_T *goal_state,
                            OccupancyGrid2D *occupancy_grid,
                            struct1_T path[500], struct2_T *search_stats);

#endif
//
// File trailer for _coder_gik9dof_planHybridAStarCodegen_api.h
//
// [EOF]
//
