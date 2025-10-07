//
// File: gik9dof_planHybridAStarCodegen_types.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

#ifndef GIK9DOF_PLANHYBRIDASTARCODEGEN_TYPES_H
#define GIK9DOF_PLANHYBRIDASTARCODEGEN_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
struct struct0_T {
  double x;
  double y;
  double theta;
  int grid_x;
  int grid_y;
  int theta_bin;
  double Vx;
  double Wz;
  double dt;
  double g;
  double h;
  double f;
  int parent_idx;
  boolean_T is_valid;
};

struct struct1_T {
  double x;
  double y;
  double theta;
  double Vx;
  double Wz;
  double dt;
};

struct struct2_T {
  boolean_T success;
  double iterations;
  double nodes_expanded;
  double planning_time_sec;
  double path_cost;
  double path_length;
};

#endif
//
// File trailer for gik9dof_planHybridAStarCodegen_types.h
//
// [EOF]
//
