//
// File: CollisionSet.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

#ifndef COLLISIONSET_H
#define COLLISIONSET_H

// Include Files
#include "CollisionGeometry.h"
#include "rtwtypes.h"
#include "string1.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
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
  ::coder::array<CollisionGeometry, 2U> CollisionGeometries;
  double MaxElements;
  double Size;
  rtString Tags;
};

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for CollisionSet.h
//
// [EOF]
//
