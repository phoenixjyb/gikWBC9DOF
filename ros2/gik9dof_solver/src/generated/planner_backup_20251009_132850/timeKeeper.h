//
// File: timeKeeper.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:31:57
//

#ifndef TIMEKEEPER_H
#define TIMEKEEPER_H

// Include Files
#include "rtwtypes.h"
#include "coder_posix_time.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
class HybridAStarPlanner;

}

// Function Declarations
namespace gik9dof {
namespace coder {
namespace internal {
namespace b_time {
namespace impl {
double timeKeeper(const coderTimespec &savedTime, double &outTime_tv_nsec);

void timeKeeper(HybridAStarPlanner *aInstancePtr, double newTime_tv_sec,
                double newTime_tv_nsec, coderTimespec &savedTime);

} // namespace impl
} // namespace b_time
} // namespace internal
} // namespace coder
void timeKeeper_init(HybridAStarPlanner *aInstancePtr);

} // namespace gik9dof

#endif
//
// File trailer for timeKeeper.h
//
// [EOF]
//
