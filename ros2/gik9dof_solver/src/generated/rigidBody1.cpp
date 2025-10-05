//
// rigidBody1.cpp
//
// Code generation for function 'rigidBody1'
//

// Include files
#include "rigidBody1.h"
#include "CharacterVector.h"
#include "CollisionSet.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
rigidBody *rigidBody::b_init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

rigidBody *rigidBody::c_init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

rigidBody::rigidBody()
{
  matlabCodegenIsDeleted = true;
}

rigidBody *rigidBody::d_init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

rigidBody::~rigidBody()
{
  matlabCodegenDestructor();
}

rigidBody *rigidBody::e_init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

rigidBody *rigidBody::f_init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

rigidBody *rigidBody::g_init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

rigidBody *rigidBody::h_init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

rigidBody *rigidBody::i_init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

rigidBody *rigidBody::init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

rigidBody *rigidBody::j_init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

rigidBody *rigidBody::k_init(robotics::manip::internal::RigidBodyTree &iobj_0,
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
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBody *obj;
  robotics::manip::internal::CharacterVector s;
  robotics::manip::internal::RigidBodyTree *b_default;
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
  bool result;
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
      msubspace_data[i] = iv1[i];
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

void rigidBody::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

} // namespace coder

// End of code generation (rigidBody1.cpp)
