//
// CollisionSet.h
//
// Code generation for function 'CollisionSet'
//

#ifndef COLLISIONSET_H
#define COLLISIONSET_H

// Include files
#include "CollisionGeometry.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
class CollisionSet {
public:
  CollisionSet *init(double maxElements);
  void matlabCodegenDestructor();
  ~CollisionSet();
  CollisionSet();
  bool matlabCodegenIsDeleted;
  array<CollisionGeometry, 2U> CollisionGeometries;
  double MaxElements;
  double Size;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

#endif
// End of code generation (CollisionSet.h)
