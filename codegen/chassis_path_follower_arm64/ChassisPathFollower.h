//
// File: ChassisPathFollower.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 11-Oct-2025 00:19:03
//

#ifndef CHASSISPATHFOLLOWER_H
#define CHASSISPATHFOLLOWER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
struct struct0_T;

struct struct1_T;

struct struct3_T;

} // namespace gik9dof

// Type Definitions
namespace gik9dof {
class ChassisPathFollower {
public:
  ChassisPathFollower();
  ~ChassisPathFollower();
  void chassisPathFollowerCodegen(const double pose[3], double dt,
                                  struct0_T *state, const struct1_T *params,
                                  double *vx, double *wz, struct3_T *status);
};

} // namespace gik9dof

#endif
//
// File trailer for ChassisPathFollower.h
//
// [EOF]
//
