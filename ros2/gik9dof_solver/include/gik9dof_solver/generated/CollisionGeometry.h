//
// CollisionGeometry.h
//
// Code generation for function 'CollisionGeometry'
//

#ifndef COLLISIONGEOMETRY_H
#define COLLISIONGEOMETRY_H

// Include files
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

#endif
// End of code generation (CollisionGeometry.h)
