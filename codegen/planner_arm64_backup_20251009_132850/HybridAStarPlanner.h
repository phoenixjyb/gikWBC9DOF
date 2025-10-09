//
// File: HybridAStarPlanner.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:31:57
//

#ifndef HYBRIDASTARPLANNER_H
#define HYBRIDASTARPLANNER_H

// Include Files
#include "gik9dof_planHybridAStarCodegen_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
class OccupancyGrid2D;

}

// Type Definitions
namespace gik9dof {
class HybridAStarPlanner {
public:
  HybridAStarPlanner();
  ~HybridAStarPlanner();
  void b_gik9dof_planHybridAStarCodegen(struct0_T *start_state,
                                        const struct0_T *goal_state,
                                        const OccupancyGrid2D *occupancy_grid,
                                        struct1_T path[500],
                                        struct2_T *search_stats);
  gik9dof_planHybridAStarCodegenStackData *getStackData();

private:
  gik9dof_planHybridAStarCodegenPersistentData pd_;
  gik9dof_planHybridAStarCodegenStackData SD_;
};

} // namespace gik9dof

#endif
//
// File trailer for HybridAStarPlanner.h
//
// [EOF]
//
