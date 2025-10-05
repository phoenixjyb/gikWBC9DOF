//
// CollisionSet.cpp
//
// Code generation for function 'CollisionSet'
//

// Include files
#include "CollisionSet.h"
#include "CollisionGeometry.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include "collisioncodegen_api.hpp"

// Function Definitions
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
CollisionSet::CollisionSet()
{
  matlabCodegenIsDeleted = true;
}

CollisionSet::~CollisionSet()
{
  matlabCodegenDestructor();
}

CollisionSet *CollisionSet::init(double maxElements)
{
  static const void *t0_GeometryInternal{nullptr};
  CollisionGeometry defaultGeometry;
  CollisionSet *obj;
  int size_tmp_idx_1;
  obj = this;
  obj->Size = 0.0;
  obj->MaxElements = maxElements;
  size_tmp_idx_1 = static_cast<int>(obj->MaxElements);
  obj->CollisionGeometries.set_size(1, size_tmp_idx_1);
  defaultGeometry.CollisionPrimitive = const_cast<void *>(t0_GeometryInternal);
  for (size_tmp_idx_1 = 0; size_tmp_idx_1 < 16; size_tmp_idx_1++) {
    defaultGeometry.LocalPose[size_tmp_idx_1] = iv[size_tmp_idx_1];
  }
  for (size_tmp_idx_1 = 0; size_tmp_idx_1 < 16; size_tmp_idx_1++) {
    defaultGeometry.WorldPose[size_tmp_idx_1] = iv[size_tmp_idx_1];
  }
  double d;
  defaultGeometry.MeshScale[0] = 1.0;
  defaultGeometry.MeshScale[1] = 1.0;
  defaultGeometry.MeshScale[2] = 1.0;
  d = obj->MaxElements;
  size_tmp_idx_1 = static_cast<int>(d);
  for (int i{0}; i < size_tmp_idx_1; i++) {
    obj->CollisionGeometries[i] = defaultGeometry;
  }
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

void CollisionSet::matlabCodegenDestructor()
{
  CollisionGeometry obj;
  if (!matlabCodegenIsDeleted) {
    double d;
    int i;
    matlabCodegenIsDeleted = true;
    d = Size;
    i = static_cast<int>(d);
    for (int b_i{0}; b_i < i; b_i++) {
      obj = CollisionGeometries[b_i];
      collisioncodegen_destructGeometry(&obj.CollisionPrimitive);
      CollisionGeometries[b_i] = obj;
    }
  }
}

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

// End of code generation (CollisionSet.cpp)
