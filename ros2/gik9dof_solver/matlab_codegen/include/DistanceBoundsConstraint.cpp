//
// File: DistanceBoundsConstraint.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "DistanceBoundsConstraint.h"
#include "CharacterVector.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "rt_nonfinite.h"
#include <algorithm>

// Function Definitions
//
// Arguments    : void
// Return Type  : DistanceBoundsConstraint
//
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
DistanceBoundsConstraint::DistanceBoundsConstraint() = default;

//
// Arguments    : void
// Return Type  : void
//
DistanceBoundsConstraint::~DistanceBoundsConstraint() = default;

//
// Arguments    : char value_data[]
//                int value_size[2]
// Return Type  : void
//
void DistanceBoundsConstraint::get_EndEffector(char value_data[],
                                               int value_size[2])
{
  CharacterVector c_obj;
  RigidBody *b_obj;
  RigidBodyTree *obj;
  if (EndEffectorIndex > 0.0) {
    int loop_ub;
    b_obj = Tree->Bodies[static_cast<int>(EndEffectorIndex) - 1];
    c_obj = b_obj->NameInternal;
    if (c_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(c_obj.Length);
    }
    value_size[0] = 1;
    value_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&c_obj.Vector[0], &c_obj.Vector[loop_ub], &value_data[0]);
    }
  } else {
    int loop_ub;
    obj = Tree;
    c_obj = obj->Base.NameInternal;
    if (c_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(c_obj.Length);
    }
    value_size[0] = 1;
    value_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&c_obj.Vector[0], &c_obj.Vector[loop_ub], &value_data[0]);
    }
  }
}

//
// Arguments    : char value_data[]
//                int value_size[2]
// Return Type  : void
//
void DistanceBoundsConstraint::get_ReferenceBody(char value_data[],
                                                 int value_size[2])
{
  CharacterVector c_obj;
  RigidBody *b_obj;
  RigidBodyTree *obj;
  if (ReferenceBodyIndex > 0.0) {
    int loop_ub;
    b_obj = Tree->Bodies[static_cast<int>(ReferenceBodyIndex) - 1];
    c_obj = b_obj->NameInternal;
    if (c_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(c_obj.Length);
    }
    value_size[0] = 1;
    value_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&c_obj.Vector[0], &c_obj.Vector[loop_ub], &value_data[0]);
    }
  } else {
    int loop_ub;
    obj = Tree;
    c_obj = obj->Base.NameInternal;
    if (c_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(c_obj.Length);
    }
    value_size[0] = 1;
    value_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&c_obj.Vector[0], &c_obj.Vector[loop_ub], &value_data[0]);
    }
  }
}

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

//
// File trailer for DistanceBoundsConstraint.cpp
//
// [EOF]
//
