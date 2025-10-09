//
// File: smoothVelocityCommand.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 00:47:40
//

#ifndef SMOOTHVELOCITYCOMMAND_H
#define SMOOTHVELOCITYCOMMAND_H

// Include Files
#include "rtwtypes.h"
#include "smoothVelocityCommand_types.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void smoothVelocityCommand(double vx_target, double wz_target,
                                  double vx_prev, double wz_prev,
                                  double ax_prev, double alpha_prev, double dt,
                                  const VelSmoothParams_T *params, double *vx_smooth,
                                  double *wz_smooth, double *ax_out,
                                  double *alpha_out);

#endif
//
// File trailer for smoothVelocityCommand.h
//
// [EOF]
//
