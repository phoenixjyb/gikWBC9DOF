//
// File: holisticVelocityController.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 11:21:02
//

#ifndef HOLISTICVELOCITYCONTROLLER_H
#define HOLISTICVELOCITYCONTROLLER_H

// Include Files
#include "holisticVelocityController_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof_velocity {
extern void holisticVelocityController(double refX, double refY,
                                       double refTheta, double refTime,
                                       double estX, double estY, double estYaw,
                                       const struct0_T *params,
                                       const struct1_T *stateIn, double *Vx,
                                       double *Wz, struct1_T *stateOut);

}

#endif
//
// File trailer for holisticVelocityController.h
//
// [EOF]
//
