//
// File: smoothTrajectoryVelocity.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 15:00:48
//

#ifndef SMOOTHTRAJECTORYVELOCITY_H
#define SMOOTHTRAJECTORYVELOCITY_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct0_T;

// Function Declarations
extern void smoothTrajectoryVelocity(const double waypoints_x[5],
                                     const double waypoints_y[5],
                                     const double waypoints_theta[5],
                                     const double t_waypoints[5],
                                     double t_current, const struct0_T *params,
                                     double *vx_cmd, double *wz_cmd,
                                     double *ax_cmd, double *alpha_cmd,
                                     double *jerk_vx_cmd, double *jerk_wz_cmd);

extern void smoothTrajectoryVelocity_initialize();

extern void smoothTrajectoryVelocity_terminate();

#endif
//
// File trailer for smoothTrajectoryVelocity.h
//
// [EOF]
//
