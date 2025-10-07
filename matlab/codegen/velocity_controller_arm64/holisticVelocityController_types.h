//
// File: holisticVelocityController_types.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 11:20:37
//

#ifndef HOLISTICVELOCITYCONTROLLER_TYPES_H
#define HOLISTICVELOCITYCONTROLLER_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
namespace gik9dof_velocity {
struct struct0_T {
  double track;
  double Vwheel_max;
  double Vx_max;
  double W_max;
  double yawKp;
  double yawKff;
};

struct struct2_T {
  double x;
  double y;
  double theta;
  double t;
};

struct struct1_T {
  struct2_T prev;
};

} // namespace gik9dof_velocity

#endif
//
// File trailer for holisticVelocityController_types.h
//
// [EOF]
//
