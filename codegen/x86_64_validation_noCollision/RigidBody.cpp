//
// File: RigidBody.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "RigidBody.h"
#include "CharacterVector.h"
#include "CollisionGeometry.h"
#include "CollisionSet.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWrapper_data.h"
#include "string1.h"
#include "coder_array.h"
#include "collisioncodegen_api.hpp"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// Arguments    : void
// Return Type  : RigidBody
//
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
RigidBody::RigidBody()
{
  matlabCodegenIsDeleted = true;
}

//
// Arguments    : void
// Return Type  : b_RigidBody
//
b_RigidBody::b_RigidBody()
{
  matlabCodegenIsDeleted = true;
}

//
// Arguments    : CollisionSet &iobj_0
//                rigidBodyJoint &iobj_1
//                RigidBody &iobj_2
// Return Type  : RigidBody *
//
RigidBody *RigidBody::copy(CollisionSet &iobj_0, rigidBodyJoint &iobj_1,
                           RigidBody &iobj_2)
{
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv3[5]{'f', 'i', 'x', 'e', 'd'};
  void *copyGeometryInternal;
  CharacterVector b_obj;
  CharacterVector obj;
  CollisionGeometry b_newObj;
  CollisionGeometry e_obj;
  CollisionSet *d_obj;
  CollisionSet *newObj;
  RigidBody *newbody;
  ::coder::array<char, 2U> vec;
  double msubspace_data[36];
  double poslim_data[14];
  double c_obj[9];
  double obj_idx_0;
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int loop_ub;
  int poslim_size_idx_0;
  signed char b_I[36];
  signed char c_I[9];
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  newbody = &iobj_2;
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  iobj_2.NameInternal = b_obj;
  b_obj = iobj_2.NameInternal;
  b_obj.Length = loop_ub;
  if (loop_ub < 1) {
    ibmat = 0;
  } else {
    ibmat = loop_ub;
  }
  if (ibmat - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[ibmat], &b_obj.Vector[0]);
  }
  iobj_2.NameInternal = b_obj;
  (&iobj_1)[0].InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    (&iobj_1)[0].JointToParentTransform[i] = b_tmp;
    (&iobj_1)[0].ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    (&iobj_1)[0].PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    (&iobj_1)[0].HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    (&iobj_1)[0].MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  (&iobj_1)[0].NameInternal = b_obj;
  (&iobj_1)[0].TypeInternal = b_obj;
  b_obj = (&iobj_1)[0].NameInternal;
  vec.set_size(1, loop_ub + 4);
  for (i = 0; i < loop_ub; i++) {
    vec[i] = obj.Vector[i];
  }
  vec[loop_ub] = '_';
  vec[loop_ub + 1] = 'j';
  vec[loop_ub + 2] = 'n';
  vec[loop_ub + 3] = 't';
  b_obj.Length = vec.size(1);
  loop_ub = vec.size(1);
  for (i = 0; i < loop_ub; i++) {
    b_obj.Vector[i] = vec[i];
  }
  (&iobj_1)[0].NameInternal = b_obj;
  obj = (&iobj_1)[0].TypeInternal;
  obj.Length = 5.0;
  for (i = 0; i < 5; i++) {
    obj.Vector[i] = b_cv[i];
  }
  (&iobj_1)[0].TypeInternal = obj;
  obj = (&iobj_1)[0].TypeInternal;
  if (obj.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(obj.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv1[ibmat] != obj.Vector[ibmat]) {
          exitg1 = 1;
        } else {
          ibmat++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    ibmat = 0;
  } else {
    result = false;
    if (i == 9) {
      ibmat = 0;
      do {
        exitg1 = 0;
        if (ibmat < 9) {
          if (cv6[ibmat] != obj.Vector[ibmat]) {
            exitg1 = 1;
          } else {
            ibmat++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (result) {
      ibmat = 1;
    } else {
      result = false;
      if (i == 8) {
        ibmat = 0;
        do {
          exitg1 = 0;
          if (ibmat < 8) {
            if (b_cv2[ibmat] != obj.Vector[ibmat]) {
              exitg1 = 1;
            } else {
              ibmat++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (result) {
        ibmat = 2;
      } else {
        ibmat = -1;
      }
    }
  }
  switch (ibmat) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    (&iobj_1)[0].VelocityNumber = 1.0;
    (&iobj_1)[0].PositionNumber = 1.0;
    (&iobj_1)[0].JointAxisInternal[0] = 0.0;
    (&iobj_1)[0].JointAxisInternal[1] = 0.0;
    (&iobj_1)[0].JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv1[i];
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    (&iobj_1)[0].VelocityNumber = 1.0;
    (&iobj_1)[0].PositionNumber = 1.0;
    (&iobj_1)[0].JointAxisInternal[0] = 0.0;
    (&iobj_1)[0].JointAxisInternal[1] = 0.0;
    (&iobj_1)[0].JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b[6];
    for (i = 0; i < 36; i++) {
      b_I[i] = 0;
    }
    for (ibmat = 0; ibmat < 6; ibmat++) {
      b_I[ibmat + 6 * ibmat] = 1;
    }
    for (i = 0; i < 36; i++) {
      msubspace_data[i] = b_I[i];
    }
    poslim_size_idx_0 = 7;
    for (loop_ub = 0; loop_ub < 2; loop_ub++) {
      ibmat = loop_ub * 3;
      b_tmp = static_cast<signed char>(10 * loop_ub - 5);
      b[ibmat] = b_tmp;
      b[ibmat + 1] = b_tmp;
      b[ibmat + 2] = b_tmp;
      poslim_data[7 * loop_ub] = rtNaN;
      poslim_data[7 * loop_ub + 1] = rtNaN;
      poslim_data[7 * loop_ub + 2] = rtNaN;
      poslim_data[7 * loop_ub + 3] = rtNaN;
    }
    for (i = 0; i < 2; i++) {
      poslim_data[7 * i + 4] = b[3 * i];
      poslim_data[7 * i + 5] = b[3 * i + 1];
      poslim_data[7 * i + 6] = b[3 * i + 2];
    }
    homepos_size_idx_1 = 7;
    for (i = 0; i < 7; i++) {
      homepos_data[i] = iv2[i];
    }
    (&iobj_1)[0].VelocityNumber = 6.0;
    (&iobj_1)[0].PositionNumber = 7.0;
    (&iobj_1)[0].JointAxisInternal[0] = rtNaN;
    (&iobj_1)[0].JointAxisInternal[1] = rtNaN;
    (&iobj_1)[0].JointAxisInternal[2] = rtNaN;
  } break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0.0;
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    (&iobj_1)[0].VelocityNumber = 0.0;
    (&iobj_1)[0].PositionNumber = 0.0;
    (&iobj_1)[0].JointAxisInternal[0] = 0.0;
    (&iobj_1)[0].JointAxisInternal[1] = 0.0;
    (&iobj_1)[0].JointAxisInternal[2] = 0.0;
    break;
  }
  (&iobj_1)[0].set_MotionSubspace(msubspace_data);
  obj = (&iobj_1)[0].TypeInternal;
  if (obj.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(obj.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (obj.Vector[ibmat] != b_cv3[ibmat]) {
          exitg1 = 1;
        } else {
          ibmat++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!result) {
    obj_idx_0 = (&iobj_1)[0].PositionNumber;
    if (obj_idx_0 < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(obj_idx_0);
    }
    for (i = 0; i < 2; i++) {
      for (ibmat = 0; ibmat < loop_ub; ibmat++) {
        (&iobj_1)[0].PositionLimitsInternal[ibmat + 7 * i] =
            poslim_data[ibmat + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      (&iobj_1)[0].HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    (&iobj_1)[0].PositionLimitsInternal[0] = poslim_data[0];
    (&iobj_1)[0].PositionLimitsInternal[7] = poslim_data[1];
    (&iobj_1)[0].HomePositionInternal[0] = homepos_data[0];
  }
  iobj_2.JointInternal = &(&iobj_1)[0];
  iobj_2.Index = -1.0;
  iobj_2.ParentIndex = -1.0;
  iobj_2.MassInternal = 1.0;
  iobj_2.CenterOfMassInternal[0] = 0.0;
  iobj_2.CenterOfMassInternal[1] = 0.0;
  iobj_2.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_2.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.SpatialInertia[i] = b_I[i];
  }
  double obj_idx_1;
  double obj_idx_2;
  iobj_2.CollisionsInternal = (&iobj_0)[0].init(static_cast<double>(0.0));
  iobj_2.matlabCodegenIsDeleted = false;
  iobj_2.JointInternal = JointInternal->copy((&iobj_1)[1]);
  iobj_2.MassInternal = MassInternal;
  obj_idx_0 = CenterOfMassInternal[0];
  obj_idx_1 = CenterOfMassInternal[1];
  obj_idx_2 = CenterOfMassInternal[2];
  iobj_2.CenterOfMassInternal[0] = obj_idx_0;
  iobj_2.CenterOfMassInternal[1] = obj_idx_1;
  iobj_2.CenterOfMassInternal[2] = obj_idx_2;
  for (i = 0; i < 9; i++) {
    c_obj[i] = InertiaInternal[i];
  }
  for (i = 0; i < 9; i++) {
    iobj_2.InertiaInternal[i] = c_obj[i];
  }
  for (i = 0; i < 36; i++) {
    msubspace_data[i] = SpatialInertia[i];
  }
  for (i = 0; i < 36; i++) {
    iobj_2.SpatialInertia[i] = msubspace_data[i];
  }
  d_obj = CollisionsInternal;
  newObj = (&iobj_0)[1].init(d_obj->MaxElements);
  newObj->Size = d_obj->Size;
  newObj->Tags = d_obj->Tags;
  obj_idx_0 = d_obj->Size;
  i = static_cast<int>(obj_idx_0);
  for (ibmat = 0; ibmat < i; ibmat++) {
    e_obj = d_obj->CollisionGeometries[ibmat];
    copyGeometryInternal =
        collisioncodegen_copyGeometry(e_obj.CollisionPrimitive);
    b_newObj.CollisionPrimitive = copyGeometryInternal;
    ::std::copy(&e_obj.LocalPose[0], &e_obj.LocalPose[16],
                &b_newObj.LocalPose[0]);
    b_newObj.MeshScale[0] = e_obj.MeshScale[0];
    b_newObj.MeshScale[1] = e_obj.MeshScale[1];
    b_newObj.MeshScale[2] = e_obj.MeshScale[2];
    ::std::copy(&e_obj.WorldPose[0], &e_obj.WorldPose[16],
                &b_newObj.WorldPose[0]);
    newObj->CollisionGeometries[ibmat] = b_newObj;
  }
  iobj_2.CollisionsInternal = newObj;
  return newbody;
}

//
// Arguments    : void
// Return Type  : void
//
RigidBody::~RigidBody()
{
  matlabCodegenDestructor();
}

//
// Arguments    : void
// Return Type  : void
//
b_RigidBody::~b_RigidBody()
{
  matlabCodegenDestructor();
}

//
// Arguments    : const char bodyInput[10]
// Return Type  : RigidBody *
//
RigidBody *RigidBody::init(const char bodyInput[10])
{
  CharacterVector b_obj;
  RigidBody *obj;
  signed char c_I[36];
  char b_bodyInput[14];
  signed char b_I[9];
  obj = this;
  for (int k{0}; k < 200; k++) {
    b_obj.Vector[k] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 10.0;
  for (int k{0}; k < 10; k++) {
    b_obj.Vector[k] = bodyInput[k];
  }
  obj->NameInternal = b_obj;
  for (int k{0}; k < 10; k++) {
    b_bodyInput[k] = bodyInput[k];
  }
  b_bodyInput[10] = '_';
  b_bodyInput[11] = 'j';
  b_bodyInput[12] = 'n';
  b_bodyInput[13] = 't';
  obj->JointInternal = obj->_pobj1.init(b_bodyInput);
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (int k{0}; k < 9; k++) {
    b_I[k] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (int k{0}; k < 9; k++) {
    obj->InertiaInternal[k] = b_I[k];
  }
  for (int k{0}; k < 36; k++) {
    c_I[k] = 0;
  }
  for (int k{0}; k < 6; k++) {
    c_I[k + 6 * k] = 1;
  }
  for (int k{0}; k < 36; k++) {
    obj->SpatialInertia[k] = c_I[k];
  }
  obj->CollisionsInternal = obj->_pobj0.init(static_cast<double>(0.0));
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : const char bodyInput[11]
//                CollisionSet &iobj_0
//                rigidBodyJoint &iobj_1
// Return Type  : RigidBody *
//
RigidBody *RigidBody::init(const char bodyInput[11], CollisionSet &iobj_0,
                           rigidBodyJoint &iobj_1)
{
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv3[5]{'f', 'i', 'x', 'e', 'd'};
  CharacterVector b_obj;
  RigidBody *obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int i1;
  int ibmat;
  int poslim_size_idx_0;
  signed char b_I[36];
  signed char c_I[9];
  signed char homepos_data[7];
  boolean_T result;
  obj = this;
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 11.0;
  for (i = 0; i < 11; i++) {
    b_obj.Vector[i] = bodyInput[i];
  }
  obj->NameInternal = b_obj;
  iobj_1.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_1.JointToParentTransform[i] = i1;
    iobj_1.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_1.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_1.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_1.MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  iobj_1.NameInternal = b_obj;
  iobj_1.TypeInternal = b_obj;
  b_obj = iobj_1.NameInternal;
  b_obj.Length = 15.0;
  for (i = 0; i < 11; i++) {
    b_obj.Vector[i] = bodyInput[i];
  }
  b_obj.Vector[11] = '_';
  b_obj.Vector[12] = 'j';
  b_obj.Vector[13] = 'n';
  b_obj.Vector[14] = 't';
  iobj_1.NameInternal = b_obj;
  b_obj = iobj_1.TypeInternal;
  b_obj.Length = 5.0;
  for (i = 0; i < 5; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  iobj_1.TypeInternal = b_obj;
  b_obj = iobj_1.TypeInternal;
  if (b_obj.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(b_obj.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv1[ibmat] != b_obj.Vector[ibmat]) {
          exitg1 = 1;
        } else {
          ibmat++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    ibmat = 0;
  } else {
    result = false;
    if (i == 9) {
      ibmat = 0;
      do {
        exitg1 = 0;
        if (ibmat < 9) {
          if (cv6[ibmat] != b_obj.Vector[ibmat]) {
            exitg1 = 1;
          } else {
            ibmat++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (result) {
      ibmat = 1;
    } else {
      result = false;
      if (i == 8) {
        ibmat = 0;
        do {
          exitg1 = 0;
          if (ibmat < 8) {
            if (b_cv2[ibmat] != b_obj.Vector[ibmat]) {
              exitg1 = 1;
            } else {
              ibmat++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (result) {
        ibmat = 2;
      } else {
        ibmat = -1;
      }
    }
  }
  switch (ibmat) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv[i];
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    iobj_1.VelocityNumber = 1.0;
    iobj_1.PositionNumber = 1.0;
    iobj_1.JointAxisInternal[0] = 0.0;
    iobj_1.JointAxisInternal[1] = 0.0;
    iobj_1.JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv1[i];
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    iobj_1.VelocityNumber = 1.0;
    iobj_1.PositionNumber = 1.0;
    iobj_1.JointAxisInternal[0] = 0.0;
    iobj_1.JointAxisInternal[1] = 0.0;
    iobj_1.JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b[6];
    for (i = 0; i < 36; i++) {
      b_I[i] = 0;
    }
    for (ibmat = 0; ibmat < 6; ibmat++) {
      b_I[ibmat + 6 * ibmat] = 1;
    }
    for (i = 0; i < 36; i++) {
      msubspace_data[i] = b_I[i];
    }
    poslim_size_idx_0 = 7;
    for (homepos_size_idx_1 = 0; homepos_size_idx_1 < 2; homepos_size_idx_1++) {
      signed char b_tmp;
      ibmat = homepos_size_idx_1 * 3;
      b_tmp = static_cast<signed char>(10 * homepos_size_idx_1 - 5);
      b[ibmat] = b_tmp;
      b[ibmat + 1] = b_tmp;
      b[ibmat + 2] = b_tmp;
      poslim_data[7 * homepos_size_idx_1] = rtNaN;
      poslim_data[7 * homepos_size_idx_1 + 1] = rtNaN;
      poslim_data[7 * homepos_size_idx_1 + 2] = rtNaN;
      poslim_data[7 * homepos_size_idx_1 + 3] = rtNaN;
    }
    for (i = 0; i < 2; i++) {
      poslim_data[7 * i + 4] = b[3 * i];
      poslim_data[7 * i + 5] = b[3 * i + 1];
      poslim_data[7 * i + 6] = b[3 * i + 2];
    }
    homepos_size_idx_1 = 7;
    for (i = 0; i < 7; i++) {
      homepos_data[i] = iv2[i];
    }
    iobj_1.VelocityNumber = 6.0;
    iobj_1.PositionNumber = 7.0;
    iobj_1.JointAxisInternal[0] = rtNaN;
    iobj_1.JointAxisInternal[1] = rtNaN;
    iobj_1.JointAxisInternal[2] = rtNaN;
  } break;
  default:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0.0;
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    iobj_1.VelocityNumber = 0.0;
    iobj_1.PositionNumber = 0.0;
    iobj_1.JointAxisInternal[0] = 0.0;
    iobj_1.JointAxisInternal[1] = 0.0;
    iobj_1.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_1.set_MotionSubspace(msubspace_data);
  b_obj = iobj_1.TypeInternal;
  if (b_obj.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(b_obj.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (b_obj.Vector[ibmat] != b_cv3[ibmat]) {
          exitg1 = 1;
        } else {
          ibmat++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!result) {
    double d;
    d = iobj_1.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_1.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_1.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_1.PositionLimitsInternal[0] = poslim_data[0];
    iobj_1.PositionLimitsInternal[7] = poslim_data[1];
    iobj_1.HomePositionInternal[0] = homepos_data[0];
  }
  obj->JointInternal = &iobj_1;
  obj->Index = -1.0;
  obj->ParentIndex = -1.0;
  obj->MassInternal = 1.0;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    obj->InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    obj->SpatialInertia[i] = b_I[i];
  }
  obj->CollisionsInternal = iobj_0.init(static_cast<double>(0.0));
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void RigidBody::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

//
// Arguments    : void
// Return Type  : void
//
void b_RigidBody::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

//
// File trailer for RigidBody.cpp
//
// [EOF]
//
