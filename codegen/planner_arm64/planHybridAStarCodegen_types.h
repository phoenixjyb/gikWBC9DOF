//
// File: planHybridAStarCodegen_types.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 14:15:48
//

#ifndef PLANHYBRIDASTARCODEGEN_TYPES_H
#define PLANHYBRIDASTARCODEGEN_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
namespace gik9dof {
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
  bool is_valid;
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
  bool success;
  double iterations;
  double nodes_expanded;
  double planning_time_sec;
  double path_cost;
  double path_length;
};

struct planHybridAStarCodegen {
  struct0_T state_list[50000];
  int heap_indices[50000];
  int parent_indices[50000];
};

struct planHybridAStarCodegenPersistentData {
  double freq;
  bool freq_not_empty;
  bool savedTime_not_empty;
};

struct planHybridAStarCodegenStackData {
  planHybridAStarCodegen f0;
  planHybridAStarCodegenPersistentData *pd;
};

} // namespace gik9dof

#endif
//
// File trailer for planHybridAStarCodegen_types.h
//
// [EOF]
//
