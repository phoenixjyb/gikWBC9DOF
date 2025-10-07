//
// File: purePursuitVelocityController.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 03:34:11
//

#ifndef PUREPURSUITVELOCITYCONTROLLER_H
#define PUREPURSUITVELOCITYCONTROLLER_H

// Include Files
#include "purePursuitVelocityController_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof_purepursuit {
extern void purePursuitVelocityController(
    double refX, double refY, double refTheta, double refTime, double estX,
    double estY, double estYaw, const struct0_T *params,
    const struct1_T *stateIn, double *vx, double *wz, struct1_T *stateOut);

}

#endif
//
// File trailer for purePursuitVelocityController.h
//
// [EOF]
//
