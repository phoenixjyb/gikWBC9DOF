//
// File: CollisionGeometry.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 18:19:21
//

#ifndef COLLISIONGEOMETRY_H
#define COLLISIONGEOMETRY_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class CollisionGeometry {
public:
  CollisionGeometry();
  ~CollisionGeometry();
  void *CollisionPrimitive;
  double LocalPose[16];
  double WorldPose[16];
  double MeshScale[3];
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for CollisionGeometry.h
//
// [EOF]
//
