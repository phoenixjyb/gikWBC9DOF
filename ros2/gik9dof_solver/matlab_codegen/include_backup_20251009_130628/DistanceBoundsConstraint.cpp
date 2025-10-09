//
// File: DistanceBoundsConstraint.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

// Include Files
#include "DistanceBoundsConstraint.h"
#include "CharacterVector.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "constraintDistanceBounds.h"
#include "rt_nonfinite.h"
#include "strcmp.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const array<double, 1U> &q
//                double J_data[]
//                int J_size[2]
// Return Type  : double
//
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
double DistanceBoundsConstraint::evaluate(const array<double, 1U> &q,
                                          double J_data[], int J_size[2])
{
  __m128d b_r;
  array<double, 2U> C;
  array<double, 2U> Jrobot;
  double T_data[16];
  double A[6];
  double r[3];
  double d;
  double g;
  double s;
  int T_size[2];
  int boffset;
  int n_tmp;
  Tree->efficientFKAndJacobianForIK(q, EndEffectorIndex, ReferenceBodyIndex,
                                    T_data, T_size, Jrobot);
  boffset = T_size[0] * (T_size[1] - 1);
  d = T_data[boffset];
  r[0] = d;
  s = d * d;
  d = T_data[boffset + 1];
  r[1] = d;
  s += d * d;
  d = T_data[boffset + 2];
  r[2] = d;
  s += d * d;
  g = std::sqrt(s + 2.2204460492503131E-16);
  _mm_storeu_pd(&A[0], _mm_set1_pd(0.0));
  b_r = _mm_loadu_pd(&r[0]);
  _mm_storeu_pd(&A[3], _mm_div_pd(b_r, _mm_set1_pd(g)));
  A[2] = 0.0;
  A[5] = d / g;
  n_tmp = Jrobot.size(1);
  C.set_size(1, Jrobot.size(1));
  for (int j{0}; j < n_tmp; j++) {
    boffset = j * 6;
    s = 0.0;
    for (int k{0}; k < 6; k++) {
      s += A[k] * Jrobot[boffset + k];
    }
    C[j] = s;
  }
  J_size[0] = 1;
  J_size[1] = Jrobot.size(1);
  for (boffset = 0; boffset < n_tmp; boffset++) {
    J_data[boffset] = C[boffset];
  }
  return g;
}

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

//
// Arguments    : RigidBodyTree *tree
// Return Type  : DistanceBoundsConstraint *
//
DistanceBoundsConstraint *DistanceBoundsConstraint::init(RigidBodyTree *tree)
{
  DistanceBoundsConstraint *obj;
  int obj_idx_0;
  obj = this;
  obj->Tree = tree;
  obj->NumElements = 1.0;
  obj_idx_0 = static_cast<int>(obj->NumElements);
  obj->BoundsInternal.set_size(obj_idx_0, 2);
  obj_idx_0 <<= 1;
  for (int i{0}; i < obj_idx_0; i++) {
    obj->BoundsInternal[i] = 0.0;
  }
  obj_idx_0 = static_cast<int>(obj->NumElements);
  obj->Weights.set_size(1, obj_idx_0);
  for (int i{0}; i < obj_idx_0; i++) {
    obj->Weights[i] = 1.0;
  }
  obj->EndEffectorIndex = 1.0;
  obj->ReferenceBodyIndex = 0.0;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : const constraintDistanceBounds *other
// Return Type  : void
//
void DistanceBoundsConstraint::update(const constraintDistanceBounds *other)
{
  CharacterVector b_obj;
  RigidBody *c_obj;
  RigidBodyTree *obj;
  int a_size[2];
  int kstr;
  char a_data[200];
  boolean_T b_bool;
  Weights.set_size(1, 1);
  Weights[0] = other->Weights;
  get_EndEffector(a_data, a_size);
  b_bool = false;
  if (a_size[1] == 17) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 17) {
        if (a_data[kstr] != other->EndEffector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    EndEffectorIndex = Tree->validateInputBodyName(other->EndEffector);
  }
  get_ReferenceBody(a_data, a_size);
  if (!::coder::internal::b_strcmp(a_data, a_size, other->ReferenceBody)) {
    if ((other->ReferenceBody.size(0) == 0) ||
        (other->ReferenceBody.size(1) == 0)) {
      ReferenceBodyIndex = 0.0;
    } else {
      int loop_ub;
      obj = Tree;
      kstr = -1;
      b_obj = obj->Base.NameInternal;
      if (b_obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(b_obj.Length);
      }
      a_size[0] = 1;
      a_size[1] = loop_ub;
      if (loop_ub - 1 >= 0) {
        ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &a_data[0]);
      }
      if (::coder::internal::b_strcmp(a_data, a_size, other->ReferenceBody)) {
        kstr = 0;
      } else {
        double d;
        int i;
        boolean_T exitg2;
        d = obj->NumBodies;
        i = 0;
        exitg2 = false;
        while ((!exitg2) && (i <= static_cast<int>(d) - 1)) {
          c_obj = obj->Bodies[i];
          b_obj = c_obj->NameInternal;
          if (b_obj.Length < 1.0) {
            loop_ub = 0;
          } else {
            loop_ub = static_cast<int>(b_obj.Length);
          }
          a_size[0] = 1;
          a_size[1] = loop_ub;
          if (loop_ub - 1 >= 0) {
            ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &a_data[0]);
          }
          if (::coder::internal::b_strcmp(a_data, a_size,
                                          other->ReferenceBody)) {
            kstr = i + 1;
            exitg2 = true;
          } else {
            i++;
          }
        }
      }
      ReferenceBodyIndex = kstr;
    }
  }
  BoundsInternal.set_size(1, 2);
  BoundsInternal[0] = other->Bounds[0];
  BoundsInternal[1] = other->Bounds[1];
}

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

//
// File trailer for DistanceBoundsConstraint.cpp
//
// [EOF]
//
