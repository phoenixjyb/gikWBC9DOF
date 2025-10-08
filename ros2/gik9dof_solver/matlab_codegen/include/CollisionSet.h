//
// File: CollisionSet.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

#ifndef COLLISIONSET_H
#define COLLISIONSET_H

// Include Files
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
  boolean_T matlabCodegenIsDeleted;
  array<CollisionGeometry, 2U> CollisionGeometries;
  double MaxElements;
  double Size;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

#endif
//
// File trailer for CollisionSet.h
//
// [EOF]
//
