//
// File: gik9dof_planHybridAStarCodegen.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

#ifndef GIK9DOF_PLANHYBRIDASTARCODEGEN_H
#define GIK9DOF_PLANHYBRIDASTARCODEGEN_H

// Include Files
#include "OccupancyGrid2D.h"
#include "gik9dof_planHybridAStarCodegen_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof {
extern void planHybridAStarCodegen(struct0_T *start_state,
                                   const struct0_T *goal_state,
                                   const OccupancyGrid2D *occupancy_grid,
                                   struct1_T path[500],
                                   struct2_T *search_stats);

}

#endif
//
// File trailer for gik9dof_planHybridAStarCodegen.h
//
// [EOF]
//
