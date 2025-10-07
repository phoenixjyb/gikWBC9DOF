//
// File: gik9dof_planHybridAStarCodegen_initialize.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

// Include Files
#include "gik9dof_planHybridAStarCodegen_initialize.h"
#include "CoderTimeAPI.h"
#include "gik9dof_planHybridAStarCodegen_data.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void gik9dof_planHybridAStarCodegen_initialize()
{
  CoderTimeAPI::callCoderClockGettime_init();
  timeKeeper_init();
  isInitialized_gik9dof_planHybridAStarCodegen = true;
}

//
// File trailer for gik9dof_planHybridAStarCodegen_initialize.cpp
//
// [EOF]
//
