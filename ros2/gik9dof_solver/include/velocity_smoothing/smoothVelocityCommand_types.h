//
// File: smoothVelocityCommand_types.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 00:47:40
//

#ifndef SMOOTHVELOCITYCOMMAND_TYPES_H
#define SMOOTHVELOCITYCOMMAND_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
// Renamed to avoid conflict with smoothTrajectoryVelocity_types.h
struct VelSmoothParams_T {
  double vx_max;
  double ax_max;
  double jx_max;
  double wz_max;
  double alpha_max;
  double jerk_wz_max;
};

#endif
//
// File trailer for smoothVelocityCommand_types.h
//
// [EOF]
//
