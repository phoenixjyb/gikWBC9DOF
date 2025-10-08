//
// File: rigidBody1.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

// Include Files
#include "rigidBody1.h"
#include "CharacterVector.h"
#include "CollisionSet.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include <cstring>

// Variable Definitions
static const signed char iv1[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};

// Function Definitions
//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
namespace coder {
rigidBody *rigidBody::b_init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                             robotics::manip::internal::CollisionSet &iobj_1,
                             rigidBodyJoint &iobj_2,
                             robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[15]{'b', 'a', 's', 'e', '_', 'l', 'i', 'n',
                             'k', '_', 'y', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 11.0;
  for (i = 0; i < 11; i++) {
    s.Vector[i] = cv3[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 15.0;
  for (i = 0; i < 15; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::b_set_CenterOfMass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->CenterOfMassInternal[0] = 0.00703065;
  obj->CenterOfMassInternal[1] = -1.01E-5;
  obj->CenterOfMassInternal[2] = 0.21569964;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::b_set_Inertia()
{
  static const double inertiaInternal[9]{3.27070589, 0.00035735, -0.184686,
                                         0.00035735, 3.37047588, 0.00010827,
                                         -0.184686,  0.00010827, 2.72731093};
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  for (int i{0}; i < 9; i++) {
    obj->InertiaInternal[i] = inertiaInternal[i];
  }
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::b_set_Mass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->MassInternal = 50.96231322;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
rigidBody *rigidBody::c_init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                             robotics::manip::internal::CollisionSet &iobj_1,
                             rigidBodyJoint &iobj_2,
                             robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[25]{'a', 'b', 's', 't', 'r', 'a', 'c', 't', '_',
                             'c', 'h', 'a', 's', 's', 'i', 's', '_', 'l',
                             'i', 'n', 'k', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 21.0;
  for (i = 0; i < 21; i++) {
    s.Vector[i] = cv4[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 25.0;
  for (i = 0; i < 25; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::c_set_CenterOfMass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->CenterOfMassInternal[0] = -0.0005634;
  obj->CenterOfMassInternal[1] = 0.038934;
  obj->CenterOfMassInternal[2] = 3.1874E-6;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::c_set_Inertia()
{
  static const double inertiaInternal[9]{0.0010597,  -1.9146E-7, -1.6752E-7,
                                         -1.9146E-7, 0.0011787,  1.9821E-5,
                                         -1.6752E-7, 1.9821E-5,  0.0010647};
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  for (int i{0}; i < 9; i++) {
    obj->InertiaInternal[i] = inertiaInternal[i];
  }
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::c_set_Mass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->MassInternal = 1.658;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : rigidBody
//
rigidBody::rigidBody()
{
  matlabCodegenIsDeleted = true;
}

//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
rigidBody *rigidBody::d_init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                             robotics::manip::internal::CollisionSet &iobj_1,
                             rigidBodyJoint &iobj_2,
                             robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[22]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm',
                             '_', 'b', 'a', 's', 'e', '_', 'l', 'i',
                             'n', 'k', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 18.0;
  for (i = 0; i < 18; i++) {
    s.Vector[i] = cv5[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 22.0;
  for (i = 0; i < 22; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::d_set_CenterOfMass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->CenterOfMassInternal[0] = 1.5E-5;
  obj->CenterOfMassInternal[1] = 0.105259;
  obj->CenterOfMassInternal[2] = -0.001954;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::d_set_Inertia()
{
  static const double inertiaInternal[9]{
      0.001125, -2.3E-5, 0.0, -2.3E-5, 0.001084, 0.0, 0.0, 0.0, 0.001158};
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  for (int i{0}; i < 9; i++) {
    obj->InertiaInternal[i] = inertiaInternal[i];
  }
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::d_set_Mass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->MassInternal = 1.164;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
rigidBody::~rigidBody()
{
  matlabCodegenDestructor();
}

//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
rigidBody *rigidBody::e_init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                             robotics::manip::internal::CollisionSet &iobj_1,
                             rigidBodyJoint &iobj_2,
                             robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[18]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
                             'l', 'i', 'n', 'k', '1', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 14.0;
  for (i = 0; i < 14; i++) {
    s.Vector[i] = cv6[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 18.0;
  for (i = 0; i < 18; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::e_set_CenterOfMass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->CenterOfMassInternal[0] = -0.23622;
  obj->CenterOfMassInternal[1] = 0.016352;
  obj->CenterOfMassInternal[2] = -0.00013275;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::e_set_Inertia()
{
  static const double inertiaInternal[9]{0.00060638, -8.0916E-6, 0.00014956,
                                         -8.0916E-6, 0.0075936,  0.00041817,
                                         0.00014956, 0.00041817, 0.0075712};
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  for (int i{0}; i < 9; i++) {
    obj->InertiaInternal[i] = inertiaInternal[i];
  }
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::e_set_Mass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->MassInternal = 1.3;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
rigidBody *rigidBody::f_init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                             robotics::manip::internal::CollisionSet &iobj_1,
                             rigidBodyJoint &iobj_2,
                             robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[18]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
                             'l', 'i', 'n', 'k', '2', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 14.0;
  for (i = 0; i < 14; i++) {
    s.Vector[i] = cv7[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 18.0;
  for (i = 0; i < 18; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::f_set_CenterOfMass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->CenterOfMassInternal[0] = 0.045114;
  obj->CenterOfMassInternal[1] = 0.054616;
  obj->CenterOfMassInternal[2] = -0.00045593;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::f_set_Inertia()
{
  static const double inertiaInternal[9]{0.00060107, -9.7503E-6,  -7.1194E-6,
                                         -9.7503E-6, 0.0013959,   -0.00022467,
                                         -7.1194E-6, -0.00022467, 0.0015027};
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  for (int i{0}; i < 9; i++) {
    obj->InertiaInternal[i] = inertiaInternal[i];
  }
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::f_set_Mass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->MassInternal = 0.818;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
rigidBody *rigidBody::g_init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                             robotics::manip::internal::CollisionSet &iobj_1,
                             rigidBodyJoint &iobj_2,
                             robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[18]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
                             'l', 'i', 'n', 'k', '3', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 14.0;
  for (i = 0; i < 14; i++) {
    s.Vector[i] = cv8[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 18.0;
  for (i = 0; i < 18; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::g_set_CenterOfMass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->CenterOfMassInternal[0] = 0.24285;
  obj->CenterOfMassInternal[1] = 0.0023784;
  obj->CenterOfMassInternal[2] = 1.279E-6;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::g_set_Inertia()
{
  static const double inertiaInternal[9]{8.45E-5,    5.3612E-9,  -2.2607E-9,
                                         5.3612E-9,  0.00010174, -8.2627E-7,
                                         -2.2607E-9, -8.2627E-7, 9.7044E-5};
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  for (int i{0}; i < 9; i++) {
    obj->InertiaInternal[i] = inertiaInternal[i];
  }
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::g_set_Mass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->MassInternal = 0.698;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
rigidBody *rigidBody::h_init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                             robotics::manip::internal::CollisionSet &iobj_1,
                             rigidBodyJoint &iobj_2,
                             robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[18]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
                             'l', 'i', 'n', 'k', '4', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 14.0;
  for (i = 0; i < 14; i++) {
    s.Vector[i] = cv9[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 18.0;
  for (i = 0; i < 18; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::h_set_CenterOfMass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->CenterOfMassInternal[0] = 0.054309;
  obj->CenterOfMassInternal[1] = 0.0041811;
  obj->CenterOfMassInternal[2] = 4.0699E-6;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::h_set_Inertia()
{
  static const double inertiaInternal[9]{8.3999E-5,  -1.3811E-8, 7.4127E-8,
                                         -1.3811E-8, 9.8498E-5,  1.6234E-5,
                                         7.4127E-8,  1.6234E-5,  0.00011333};
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  for (int i{0}; i < 9; i++) {
    obj->InertiaInternal[i] = inertiaInternal[i];
  }
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::h_set_Mass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->MassInternal = 0.417;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
rigidBody *rigidBody::i_init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                             robotics::manip::internal::CollisionSet &iobj_1,
                             rigidBodyJoint &iobj_2,
                             robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[18]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
                             'l', 'i', 'n', 'k', '5', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 14.0;
  for (i = 0; i < 14; i++) {
    s.Vector[i] = cv10[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 18.0;
  for (i = 0; i < 18; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::i_set_CenterOfMass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->CenterOfMassInternal[0] = 0.028138;
  obj->CenterOfMassInternal[1] = 1.2134E-7;
  obj->CenterOfMassInternal[2] = 5.4049E-8;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::i_set_Inertia()
{
  static const double inertiaInternal[9]{3.5662E-6,   -4.1666E-12, 2.9628E-12,
                                         -4.1666E-12, 2.0238E-6,   6.6514E-12,
                                         2.9628E-12,  6.6514E-12,  2.0238E-6};
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  for (int i{0}; i < 9; i++) {
    obj->InertiaInternal[i] = inertiaInternal[i];
  }
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::i_set_Mass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->MassInternal = 0.037;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
rigidBody *rigidBody::init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                           robotics::manip::internal::CollisionSet &iobj_1,
                           rigidBodyJoint &iobj_2,
                           robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[15]{'b', 'a', 's', 'e', '_', 'l', 'i', 'n',
                             'k', '_', 'x', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 11.0;
  for (i = 0; i < 11; i++) {
    s.Vector[i] = cv2[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 15.0;
  for (i = 0; i < 15; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
rigidBody *rigidBody::j_init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                             robotics::manip::internal::CollisionSet &iobj_1,
                             rigidBodyJoint &iobj_2,
                             robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[18]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm', '_',
                             'l', 'i', 'n', 'k', '6', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 14.0;
  for (i = 0; i < 14; i++) {
    s.Vector[i] = cv11[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 18.0;
  for (i = 0; i < 18; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::j_set_CenterOfMass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->CenterOfMassInternal[0] = -0.031107;
  obj->CenterOfMassInternal[1] = -1.3893E-7;
  obj->CenterOfMassInternal[2] = -1.437E-7;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::j_set_Inertia()
{
  static const double inertiaInternal[9]{0.00017588,  -8.1856E-8, -5.3493E-10,
                                         -8.1856E-8,  9.8637E-5,  4.1789E-10,
                                         -5.3493E-10, 4.1789E-10, 0.00016512};
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  for (int i{0}; i < 9; i++) {
    obj->InertiaInternal[i] = inertiaInternal[i];
  }
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::j_set_Mass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->MassInternal = 0.604;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : robotics::manip::internal::b_RigidBodyTree &iobj_0
//                robotics::manip::internal::CollisionSet &iobj_1
//                rigidBodyJoint &iobj_2
//                robotics::manip::internal::RigidBody &iobj_3
// Return Type  : rigidBody *
//
rigidBody *rigidBody::k_init(robotics::manip::internal::b_RigidBodyTree &iobj_0,
                             robotics::manip::internal::CollisionSet &iobj_1,
                             rigidBodyJoint &iobj_2,
                             robotics::manip::internal::RigidBody &iobj_3)
{
  static const char b_cv[21]{'l', 'e', 'f', 't', '_', 'g', 'r',
                             'i', 'p', 'p', 'e', 'r', '_', 'l',
                             'i', 'n', 'k', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::b_RigidBodyTree *b_default;
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
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_3.NameInternal = s;
  s = iobj_3.NameInternal;
  s.Length = 17.0;
  for (i = 0; i < 17; i++) {
    s.Vector[i] = cv12[i];
  }
  iobj_3.NameInternal = s;
  iobj_2.InTree = false;
  for (i = 0; i < 16; i++) {
    i1 = iv[i];
    iobj_2.JointToParentTransform[i] = i1;
    iobj_2.ChildToJointTransform[i] = i1;
  }
  for (i = 0; i < 14; i++) {
    iobj_2.PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    iobj_2.HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    iobj_2.MotionSubspaceInternal[i] = 0.0;
  }
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.NameInternal = s;
  s.Length = 200.0;
  for (i = 0; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.NameInternal;
  s.Length = 21.0;
  for (i = 0; i < 21; i++) {
    s.Vector[i] = b_cv[i];
  }
  iobj_2.NameInternal = s;
  s = iobj_2.TypeInternal;
  s.Length = 5.0;
  for (i = 0; i < 5; i++) {
    s.Vector[i] = b_cv1[i];
  }
  iobj_2.TypeInternal = s;
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 8) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 8) {
        if (b_cv2[ibmat] != s.Vector[ibmat]) {
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
          if (cv1[ibmat] != s.Vector[ibmat]) {
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
            if (b_cv3[ibmat] != s.Vector[ibmat]) {
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 1.0;
    iobj_2.PositionNumber = 1.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 1.0;
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
    iobj_2.VelocityNumber = 6.0;
    iobj_2.PositionNumber = 7.0;
    iobj_2.JointAxisInternal[0] = rtNaN;
    iobj_2.JointAxisInternal[1] = rtNaN;
    iobj_2.JointAxisInternal[2] = rtNaN;
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
    iobj_2.VelocityNumber = 0.0;
    iobj_2.PositionNumber = 0.0;
    iobj_2.JointAxisInternal[0] = 0.0;
    iobj_2.JointAxisInternal[1] = 0.0;
    iobj_2.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_2.set_MotionSubspace(msubspace_data);
  s = iobj_2.TypeInternal;
  if (s.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(s.Length);
  }
  result = false;
  if (i == 5) {
    ibmat = 0;
    do {
      exitg1 = 0;
      if (ibmat < 5) {
        if (s.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = iobj_2.PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (i1 = 0; i1 < ibmat; i1++) {
        iobj_2.PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_2.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_2.PositionLimitsInternal[0] = poslim_data[0];
    iobj_2.PositionLimitsInternal[7] = poslim_data[1];
    iobj_2.HomePositionInternal[0] = homepos_data[0];
  }
  iobj_3.JointInternal = &iobj_2;
  iobj_3.Index = -1.0;
  iobj_3.ParentIndex = -1.0;
  iobj_3.MassInternal = 1.0;
  iobj_3.CenterOfMassInternal[0] = 0.0;
  iobj_3.CenterOfMassInternal[1] = 0.0;
  iobj_3.CenterOfMassInternal[2] = 0.0;
  for (i = 0; i < 9; i++) {
    c_I[i] = 0;
  }
  c_I[0] = 1;
  c_I[4] = 1;
  c_I[8] = 1;
  for (i = 0; i < 9; i++) {
    iobj_3.InertiaInternal[i] = c_I[i];
  }
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    b_I[ibmat + 6 * ibmat] = 1;
  }
  for (i = 0; i < 36; i++) {
    iobj_3.SpatialInertia[i] = b_I[i];
  }
  iobj_3.CollisionsInternal = iobj_1.init(static_cast<double>(0.0));
  iobj_3.matlabCodegenIsDeleted = false;
  b_default = iobj_0.init();
  obj->BodyInternal = &iobj_3;
  obj->TreeInternal = b_default;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::set_CenterOfMass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->CenterOfMassInternal[0] = 0.0;
  obj->CenterOfMassInternal[1] = 0.0;
  obj->CenterOfMassInternal[2] = 0.0;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::set_Inertia()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  for (int i{0}; i < 9; i++) {
    obj->InertiaInternal[i] = iv1[i];
  }
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBody::set_Mass()
{
  robotics::manip::internal::RigidBody *obj;
  double inertia[9];
  double sc[9];
  double com_idx_0;
  double com_idx_1;
  double com_idx_2;
  double mass;
  int i1;
  obj = BodyInternal;
  obj->MassInternal = 1.0;
  mass = obj->MassInternal;
  com_idx_0 = obj->CenterOfMassInternal[0];
  com_idx_1 = obj->CenterOfMassInternal[1];
  com_idx_2 = obj->CenterOfMassInternal[2];
  for (int i{0}; i < 9; i++) {
    inertia[i] = obj->InertiaInternal[i];
  }
  sc[0] = 0.0;
  sc[3] = -com_idx_2;
  sc[6] = com_idx_1;
  sc[1] = com_idx_2;
  sc[4] = 0.0;
  sc[7] = -com_idx_0;
  sc[2] = -com_idx_1;
  sc[5] = com_idx_0;
  sc[8] = 0.0;
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i] = inertia[3 * i];
    obj->SpatialInertia[6 * i + 1] = inertia[3 * i + 1];
    obj->SpatialInertia[6 * i + 2] = inertia[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1] = mass * sc[3 * i];
    obj->SpatialInertia[i1 + 1] = mass * sc[3 * i + 1];
    obj->SpatialInertia[i1 + 2] = mass * sc[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    obj->SpatialInertia[6 * i + 3] = mass * sc[i];
    obj->SpatialInertia[6 * i + 4] = mass * sc[i + 3];
    obj->SpatialInertia[6 * i + 5] = mass * sc[i + 6];
  }
  for (int i{0}; i < 3; i++) {
    i1 = 6 * (i + 3);
    obj->SpatialInertia[i1 + 3] = mass * static_cast<double>(iv1[3 * i]);
    obj->SpatialInertia[i1 + 4] = mass * static_cast<double>(iv1[3 * i + 1]);
    obj->SpatialInertia[i1 + 5] = mass * static_cast<double>(iv1[3 * i + 2]);
  }
}

} // namespace coder

//
// File trailer for rigidBody1.cpp
//
// [EOF]
//
