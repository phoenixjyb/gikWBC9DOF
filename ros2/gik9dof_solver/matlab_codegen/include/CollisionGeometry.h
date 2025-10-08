//
// File: CollisionGeometry.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef COLLISIONGEOMETRY_H
#define COLLISIONGEOMETRY_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class CollisionGeometry {
public:
  void *CollisionPrimitive;
  double LocalPose[16];
  double WorldPose[16];
  double MeshScale[3];
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

#endif
//
// File trailer for CollisionGeometry.h
//
// [EOF]
//
