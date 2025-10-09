//
// File: HybridAStarPlanner.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 13:46:29
//

#ifndef HYBRIDASTARPLANNER_H
#define HYBRIDASTARPLANNER_H

// Include Files
#include "planHybridAStarCodegen_types.h"
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
  void b_planHybridAStarCodegen(struct0_T *start_state,
                                const struct0_T *goal_state,
                                const OccupancyGrid2D *occupancy_grid,
                                struct1_T path[500], struct2_T *search_stats);
  planHybridAStarCodegenStackData *getStackData();

private:
  planHybridAStarCodegenPersistentData pd_;
  planHybridAStarCodegenStackData SD_;
};

} // namespace gik9dof

#endif
//
// File trailer for HybridAStarPlanner.h
//
// [EOF]
//
