//
// File: rigidBodyJoint.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "rigidBodyJoint.h"
#include "CharacterVector.h"
#include "GIKSolver.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWrapper_data.h"
#include "solveGIKStepWrapper_types.h"
#include "validatestring.h"
#include "coder_array.h"
#include "rtGetInf.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const double msubspace[6]
// Return Type  : void
//
namespace gik9dof {
namespace coder {
void rigidBodyJoint::b_set_MotionSubspace(const double msubspace[6])
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  int i;
  int kstr;
  boolean_T b_bool;
  if (TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(TypeInternal.Length);
  }
  b_bool = false;
  if (i == 5) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (TypeInternal.Vector[kstr] != b_cv[kstr]) {
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
    double d;
    d = VelocityNumber;
    if (d < 1.0) {
      kstr = 0;
    } else {
      kstr = static_cast<int>(d);
    }
    for (i = 0; i < kstr; i++) {
      for (int i1{0}; i1 < 6; i1++) {
        int i2;
        i2 = i1 + 6 * i;
        MotionSubspaceInternal[i2] = msubspace[i2];
      }
    }
  } else {
    for (i = 0; i < 6; i++) {
      MotionSubspaceInternal[i] = 0.0;
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBodyJoint::resetHomePosition()
{
  ::coder::array<boolean_T, 1U> x;
  double ub_data[7];
  double d;
  int loop_ub_tmp;
  d = PositionNumber;
  if (d < 1.0) {
    loop_ub_tmp = 0;
  } else {
    loop_ub_tmp = static_cast<int>(d);
  }
  if (loop_ub_tmp - 1 >= 0) {
    ::std::copy(&PositionLimitsInternal[7],
                &PositionLimitsInternal[7 + loop_ub_tmp], &ub_data[0]);
  }
  if (PositionNumber == 1.0) {
    double d1;
    int ix;
    boolean_T x_data[7];
    boolean_T exitg1;
    boolean_T guard1;
    boolean_T guard2;
    boolean_T guard3;
    boolean_T y;
    x.set_size(loop_ub_tmp);
    for (int i{0}; i < loop_ub_tmp; i++) {
      d1 = PositionLimitsInternal[i];
      x[i] = ((!std::isinf(d1)) && (!std::isnan(d1)));
    }
    y = true;
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix <= x.size(0))) {
      if (!x[ix - 1]) {
        y = false;
        exitg1 = true;
      } else {
        ix++;
      }
    }
    guard1 = false;
    guard2 = false;
    guard3 = false;
    if (y) {
      x.set_size(loop_ub_tmp);
      for (int i{0}; i < loop_ub_tmp; i++) {
        d1 = PositionLimitsInternal[i + 7];
        x[i] = ((!std::isinf(d1)) && (!std::isnan(d1)));
      }
      y = true;
      ix = 1;
      exitg1 = false;
      while ((!exitg1) && (ix <= x.size(0))) {
        if (!x[ix - 1]) {
          y = false;
          exitg1 = true;
        } else {
          ix++;
        }
      }
      if (y) {
        int vectorUB;
        ix = (loop_ub_tmp / 2) << 1;
        vectorUB = ix - 2;
        for (int i{0}; i <= vectorUB; i += 2) {
          __m128d r;
          __m128d r1;
          r = _mm_loadu_pd(&PositionLimitsInternal[i]);
          r1 = _mm_loadu_pd(&PositionLimitsInternal[i + 7]);
          _mm_storeu_pd(&ub_data[i],
                        _mm_mul_pd(_mm_set1_pd(0.5), _mm_add_pd(r, r1)));
        }
        for (int i{ix}; i < loop_ub_tmp; i++) {
          ub_data[i] =
              0.5 * (PositionLimitsInternal[i] + PositionLimitsInternal[i + 7]);
        }
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
    if (guard3) {
      x.set_size(loop_ub_tmp);
      for (int i{0}; i < loop_ub_tmp; i++) {
        d1 = PositionLimitsInternal[i];
        x[i] = ((!std::isinf(d1)) && (!std::isnan(d1)));
      }
      y = true;
      ix = 1;
      exitg1 = false;
      while ((!exitg1) && (ix <= x.size(0))) {
        if (!x[ix - 1]) {
          y = false;
          exitg1 = true;
        } else {
          ix++;
        }
      }
      if (y) {
        for (int i{0}; i < loop_ub_tmp; i++) {
          d1 = PositionLimitsInternal[i + 7];
          x_data[i] = (std::isinf(d1) || std::isnan(d1));
        }
        y = false;
        ix = 1;
        exitg1 = false;
        while ((!exitg1) && (ix <= loop_ub_tmp)) {
          if (x_data[ix - 1]) {
            y = true;
            exitg1 = true;
          } else {
            ix++;
          }
        }
        if (y) {
          if (loop_ub_tmp - 1 >= 0) {
            ::std::copy(&PositionLimitsInternal[0],
                        &PositionLimitsInternal[loop_ub_tmp], &ub_data[0]);
          }
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    }
    if (guard2) {
      for (int i{0}; i < loop_ub_tmp; i++) {
        d1 = PositionLimitsInternal[i];
        x_data[i] = (std::isinf(d1) || std::isnan(d1));
      }
      y = false;
      ix = 1;
      exitg1 = false;
      while ((!exitg1) && (ix <= loop_ub_tmp)) {
        if (x_data[ix - 1]) {
          y = true;
          exitg1 = true;
        } else {
          ix++;
        }
      }
      if (y) {
        x.set_size(loop_ub_tmp);
        for (int i{0}; i < loop_ub_tmp; i++) {
          d1 = PositionLimitsInternal[i + 7];
          x[i] = ((!std::isinf(d1)) && (!std::isnan(d1)));
        }
        y = true;
        ix = 1;
        exitg1 = false;
        while ((!exitg1) && (ix <= x.size(0))) {
          if (!x[ix - 1]) {
            y = false;
            exitg1 = true;
          } else {
            ix++;
          }
        }
        if (!y) {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    }
    if (guard1) {
      if (loop_ub_tmp - 1 >= 0) {
        std::memset(&ub_data[0], 0,
                    static_cast<unsigned int>(loop_ub_tmp) * sizeof(double));
      }
    }
    if (d < 1.0) {
      ix = 0;
    } else {
      ix = static_cast<int>(d);
    }
    if (ix - 1 >= 0) {
      ::std::copy(&ub_data[0], &ub_data[ix], &HomePositionInternal[0]);
    }
  }
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::b_init()
{
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const char b_cv[7]{'j', 'o', 'i', 'n', 't', '_', 'x'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv3[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 7.0;
  for (i = 0; i < 7; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 9.0;
  for (i = 0; i < 9; i++) {
    b_obj.Vector[i] = cv6[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBodyJoint::b_set_JointAxis()
{
  static const double dv[6]{0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  static const double dv1[6]{0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
  static const char b_cv[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  int kstr;
  boolean_T result;
  JointAxisInternal[0] = 0.0;
  JointAxisInternal[1] = 1.0;
  JointAxisInternal[2] = 0.0;
  if (TypeInternal.Length < 1.0) {
    kstr = 0;
  } else {
    kstr = static_cast<int>(TypeInternal.Length);
  }
  result = false;
  if (kstr == 8) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (b_cv[kstr] != TypeInternal.Vector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    kstr = 0;
  } else {
    kstr = -1;
  }
  if (kstr == 0) {
    b_set_MotionSubspace(dv1);
  } else {
    b_set_MotionSubspace(dv);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBodyJoint::b_set_PositionLimits()
{
  double dv[2];
  double d;
  int ix;
  int loop_ub;
  boolean_T resetHome;
  resetHome = false;
  switch (static_cast<int>(PositionNumber)) {
  case 0:
  case 7:
    break;
  default: {
    boolean_T x_data[7];
    boolean_T exitg1;
    boolean_T y;
    d = PositionNumber;
    if (d < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(d);
    }
    for (ix = 0; ix < loop_ub; ix++) {
      x_data[ix] = (HomePositionInternal[ix] > 2.8798);
    }
    y = false;
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix <= loop_ub)) {
      if (x_data[ix - 1]) {
        y = true;
        exitg1 = true;
      } else {
        ix++;
      }
    }
    if (y) {
      resetHome = true;
    } else {
      for (ix = 0; ix < loop_ub; ix++) {
        x_data[ix] = (HomePositionInternal[ix] < -2.8798);
      }
      y = false;
      ix = 1;
      exitg1 = false;
      while ((!exitg1) && (ix <= loop_ub)) {
        if (x_data[ix - 1]) {
          y = true;
          exitg1 = true;
        } else {
          ix++;
        }
      }
      if (y) {
        resetHome = true;
      }
    }
  } break;
  }
  d = PositionNumber;
  dv[0] = -2.8798;
  dv[1] = 2.8798;
  if (d < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(d);
  }
  for (ix = 0; ix < 2; ix++) {
    for (int i{0}; i < loop_ub; i++) {
      PositionLimitsInternal[i + 7 * ix] = dv[i + loop_ub * ix];
    }
  }
  if (resetHome) {
    resetHomePosition();
  }
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::c_init()
{
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const char b_cv[7]{'j', 'o', 'i', 'n', 't', '_', 'y'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv3[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 7.0;
  for (i = 0; i < 7; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 9.0;
  for (i = 0; i < 9; i++) {
    b_obj.Vector[i] = cv6[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBodyJoint::c_set_JointAxis()
{
  static const double dv[6]{0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  static const double dv1[6]{0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
  static const char b_cv[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  int kstr;
  boolean_T result;
  JointAxisInternal[0] = 0.0;
  JointAxisInternal[1] = 0.0;
  JointAxisInternal[2] = 1.0;
  if (TypeInternal.Length < 1.0) {
    kstr = 0;
  } else {
    kstr = static_cast<int>(TypeInternal.Length);
  }
  result = false;
  if (kstr == 8) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (b_cv[kstr] != TypeInternal.Vector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    kstr = 0;
  } else {
    kstr = -1;
  }
  if (kstr == 0) {
    b_set_MotionSubspace(dv1);
  } else {
    b_set_MotionSubspace(dv);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBodyJoint::c_set_PositionLimits()
{
  double dv[2];
  double d;
  int ix;
  int loop_ub;
  boolean_T resetHome;
  resetHome = false;
  switch (static_cast<int>(PositionNumber)) {
  case 0:
  case 7:
    break;
  default: {
    boolean_T x_data[7];
    boolean_T exitg1;
    boolean_T y;
    d = PositionNumber;
    if (d < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(d);
    }
    for (ix = 0; ix < loop_ub; ix++) {
      x_data[ix] = (HomePositionInternal[ix] > 3.2289);
    }
    y = false;
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix <= loop_ub)) {
      if (x_data[ix - 1]) {
        y = true;
        exitg1 = true;
      } else {
        ix++;
      }
    }
    if (y) {
      resetHome = true;
    } else {
      for (ix = 0; ix < loop_ub; ix++) {
        x_data[ix] = (HomePositionInternal[ix] < 0.0);
      }
      y = false;
      ix = 1;
      exitg1 = false;
      while ((!exitg1) && (ix <= loop_ub)) {
        if (x_data[ix - 1]) {
          y = true;
          exitg1 = true;
        } else {
          ix++;
        }
      }
      if (y) {
        resetHome = true;
      }
    }
  } break;
  }
  d = PositionNumber;
  dv[0] = 0.0;
  dv[1] = 3.2289;
  if (d < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(d);
  }
  for (ix = 0; ix < 2; ix++) {
    for (int i{0}; i < loop_ub; i++) {
      PositionLimitsInternal[i + 7 * ix] = dv[i + loop_ub * ix];
    }
  }
  if (resetHome) {
    resetHomePosition();
  }
}

//
// Arguments    : rigidBodyJoint &iobj_0
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::copy(rigidBodyJoint &iobj_0) const
{
  static const char b_cv[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv1[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv3[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 1, 0, 0, 0};
  static const signed char iv2[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv2[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *newjoint;
  robotics::manip::internal::CharacterVector b_obj;
  robotics::manip::internal::CharacterVector c_obj;
  robotics::manip::internal::CharacterVector obj;
  ::coder::array<char, 2U> vec;
  double msubspace_data[36];
  double e_obj[16];
  double poslim_data[14];
  double d_obj[7];
  double obj_idx_0;
  int b_iv[2];
  int obj_size[2];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int loop_ub;
  int poslim_size_idx_0;
  char obj_data[200];
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_obj = NameInternal;
  if (b_obj.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(b_obj.Length);
  }
  iobj_0.InTree = false;
  for (ibmat = 0; ibmat < 16; ibmat++) {
    b_tmp = iv[ibmat];
    iobj_0.JointToParentTransform[ibmat] = b_tmp;
    iobj_0.ChildToJointTransform[ibmat] = b_tmp;
  }
  for (ibmat = 0; ibmat < 14; ibmat++) {
    iobj_0.PositionLimitsInternal[ibmat] = 0.0;
  }
  for (ibmat = 0; ibmat < 7; ibmat++) {
    iobj_0.HomePositionInternal[ibmat] = 0.0;
  }
  for (ibmat = 0; ibmat < 36; ibmat++) {
    iobj_0.MotionSubspaceInternal[ibmat] = 0.0;
  }
  newjoint = &iobj_0;
  for (ibmat = 0; ibmat < 200; ibmat++) {
    c_obj.Vector[ibmat] = ' ';
  }
  c_obj.Length = 200.0;
  iobj_0.NameInternal = c_obj;
  iobj_0.TypeInternal = c_obj;
  c_obj = iobj_0.NameInternal;
  c_obj.Length = i;
  if (i < 1) {
    ibmat = 0;
  } else {
    ibmat = i;
  }
  if (ibmat - 1 >= 0) {
    ::std::copy(&b_obj.Vector[0], &b_obj.Vector[ibmat], &c_obj.Vector[0]);
  }
  iobj_0.NameInternal = c_obj;
  b_obj = iobj_0.TypeInternal;
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  vec.reserve(9);
  validatestring(obj_data, obj_size, (char *)vec.data(), b_iv);
  (*(int(*)[2])vec.size())[0] = b_iv[0];
  (*(int(*)[2])vec.size())[1] = b_iv[1];
  vec.set_size(vec.size(0), vec.size(1));
  b_obj.Length = vec.size(1);
  loop_ub = vec.size(1);
  for (i = 0; i < loop_ub; i++) {
    b_obj.Vector[i] = vec[i];
  }
  iobj_0.TypeInternal = b_obj;
  obj = iobj_0.TypeInternal;
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
        if (b_cv[ibmat] != obj.Vector[ibmat]) {
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
        ibmat = 2;
      } else {
        ibmat = -1;
      }
    }
  }
  switch (ibmat) {
  case 0:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = b_iv1[i];
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    iobj_0.VelocityNumber = 1.0;
    iobj_0.PositionNumber = 1.0;
    iobj_0.JointAxisInternal[0] = 0.0;
    iobj_0.JointAxisInternal[1] = 0.0;
    iobj_0.JointAxisInternal[2] = 1.0;
    break;
  case 1:
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = iv2[i];
    }
    poslim_size_idx_0 = 1;
    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    homepos_size_idx_1 = 1;
    homepos_data[0] = 0;
    iobj_0.VelocityNumber = 1.0;
    iobj_0.PositionNumber = 1.0;
    iobj_0.JointAxisInternal[0] = 0.0;
    iobj_0.JointAxisInternal[1] = 0.0;
    iobj_0.JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
      homepos_data[i] = iv3[i];
    }
    iobj_0.VelocityNumber = 6.0;
    iobj_0.PositionNumber = 7.0;
    iobj_0.JointAxisInternal[0] = rtNaN;
    iobj_0.JointAxisInternal[1] = rtNaN;
    iobj_0.JointAxisInternal[2] = rtNaN;
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
    iobj_0.VelocityNumber = 0.0;
    iobj_0.PositionNumber = 0.0;
    iobj_0.JointAxisInternal[0] = 0.0;
    iobj_0.JointAxisInternal[1] = 0.0;
    iobj_0.JointAxisInternal[2] = 0.0;
    break;
  }
  iobj_0.set_MotionSubspace(msubspace_data);
  obj = iobj_0.TypeInternal;
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
        if (obj.Vector[ibmat] != b_cv2[ibmat]) {
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
    obj_idx_0 = iobj_0.PositionNumber;
    if (obj_idx_0 < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(obj_idx_0);
    }
    for (i = 0; i < 2; i++) {
      for (ibmat = 0; ibmat < loop_ub; ibmat++) {
        iobj_0.PositionLimitsInternal[ibmat + 7 * i] =
            poslim_data[ibmat + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      iobj_0.HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    iobj_0.PositionLimitsInternal[0] = poslim_data[0];
    iobj_0.PositionLimitsInternal[7] = poslim_data[1];
    iobj_0.HomePositionInternal[0] = homepos_data[0];
  }
  obj = NameInternal;
  if (obj.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(obj.Length);
  }
  if (i != 0) {
    obj = NameInternal;
    if (obj.Length < 1.0) {
      i = 0;
    } else {
      i = static_cast<int>(obj.Length);
    }
    if (!iobj_0.InTree) {
      b_obj = iobj_0.NameInternal;
      b_obj.Length = i;
      if (i < 1) {
        loop_ub = 0;
      } else {
        loop_ub = i;
      }
      if (loop_ub - 1 >= 0) {
        ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &b_obj.Vector[0]);
      }
      iobj_0.NameInternal = b_obj;
    }
  }
  for (i = 0; i < 14; i++) {
    poslim_data[i] = PositionLimitsInternal[i];
  }
  for (i = 0; i < 14; i++) {
    iobj_0.PositionLimitsInternal[i] = poslim_data[i];
  }
  for (i = 0; i < 7; i++) {
    d_obj[i] = HomePositionInternal[i];
  }
  for (i = 0; i < 7; i++) {
    iobj_0.HomePositionInternal[i] = d_obj[i];
  }
  double obj_idx_1;
  double obj_idx_2;
  obj_idx_0 = JointAxisInternal[0];
  obj_idx_1 = JointAxisInternal[1];
  obj_idx_2 = JointAxisInternal[2];
  iobj_0.JointAxisInternal[0] = obj_idx_0;
  iobj_0.JointAxisInternal[1] = obj_idx_1;
  iobj_0.JointAxisInternal[2] = obj_idx_2;
  get_MotionSubspace(msubspace_data, obj_size);
  iobj_0.set_MotionSubspace(msubspace_data);
  for (i = 0; i < 16; i++) {
    e_obj[i] = JointToParentTransform[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_0.JointToParentTransform[i] = e_obj[i];
  }
  for (i = 0; i < 16; i++) {
    e_obj[i] = ChildToJointTransform[i];
  }
  for (i = 0; i < 16; i++) {
    iobj_0.ChildToJointTransform[i] = e_obj[i];
  }
  return newjoint;
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::d_init()
{
  static const char b_cv[11]{'j', 'o', 'i', 'n', 't', '_',
                             't', 'h', 'e', 't', 'a'};
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 11.0;
  for (i = 0; i < 11; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 8.0;
  for (i = 0; i < 8; i++) {
    b_obj.Vector[i] = b_cv1[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
            if (b_cv3[ibmat] != b_obj.Vector[ibmat]) {
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
        if (b_obj.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBodyJoint::d_set_JointAxis()
{
  static const double dv[6]{0.0, 0.0, 0.0, 0.0, 0.0, -1.0};
  static const double dv1[6]{0.0, 0.0, -1.0, 0.0, 0.0, 0.0};
  static const char b_cv[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  int kstr;
  boolean_T result;
  JointAxisInternal[0] = 0.0;
  JointAxisInternal[1] = 0.0;
  JointAxisInternal[2] = -1.0;
  if (TypeInternal.Length < 1.0) {
    kstr = 0;
  } else {
    kstr = static_cast<int>(TypeInternal.Length);
  }
  result = false;
  if (kstr == 8) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (b_cv[kstr] != TypeInternal.Vector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    kstr = 0;
  } else {
    kstr = -1;
  }
  if (kstr == 0) {
    b_set_MotionSubspace(dv1);
  } else {
    b_set_MotionSubspace(dv);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBodyJoint::d_set_PositionLimits()
{
  double dv[2];
  double d;
  int ix;
  int loop_ub;
  boolean_T resetHome;
  resetHome = false;
  switch (static_cast<int>(PositionNumber)) {
  case 0:
  case 7:
    break;
  default: {
    boolean_T x_data[7];
    boolean_T exitg1;
    boolean_T y;
    d = PositionNumber;
    if (d < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(d);
    }
    for (ix = 0; ix < loop_ub; ix++) {
      x_data[ix] = (HomePositionInternal[ix] > 0.0);
    }
    y = false;
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix <= loop_ub)) {
      if (x_data[ix - 1]) {
        y = true;
        exitg1 = true;
      } else {
        ix++;
      }
    }
    if (y) {
      resetHome = true;
    } else {
      for (ix = 0; ix < loop_ub; ix++) {
        x_data[ix] = (HomePositionInternal[ix] < -3.3161);
      }
      y = false;
      ix = 1;
      exitg1 = false;
      while ((!exitg1) && (ix <= loop_ub)) {
        if (x_data[ix - 1]) {
          y = true;
          exitg1 = true;
        } else {
          ix++;
        }
      }
      if (y) {
        resetHome = true;
      }
    }
  } break;
  }
  d = PositionNumber;
  dv[0] = -3.3161;
  dv[1] = 0.0;
  if (d < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(d);
  }
  for (ix = 0; ix < 2; ix++) {
    for (int i{0}; i < loop_ub; i++) {
      PositionLimitsInternal[i + 7 * ix] = dv[i + loop_ub * ix];
    }
  }
  if (resetHome) {
    resetHomePosition();
  }
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::e_init()
{
  static const char b_cv[15]{'a', 'r', 'm', '_', 'm', 'o', 'u', 'n',
                             't', '_', 'j', 'o', 'i', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 15.0;
  for (i = 0; i < 15; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 5.0;
  for (i = 0; i < 5; i++) {
    b_obj.Vector[i] = b_cv1[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
            if (b_cv3[ibmat] != b_obj.Vector[ibmat]) {
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
        if (b_obj.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBodyJoint::e_set_PositionLimits()
{
  double dv[2];
  double d;
  int ix;
  int loop_ub;
  boolean_T resetHome;
  resetHome = false;
  switch (static_cast<int>(PositionNumber)) {
  case 0:
  case 7:
    break;
  default: {
    boolean_T x_data[7];
    boolean_T exitg1;
    boolean_T y;
    d = PositionNumber;
    if (d < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(d);
    }
    for (ix = 0; ix < loop_ub; ix++) {
      x_data[ix] = (HomePositionInternal[ix] > 1.6581);
    }
    y = false;
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix <= loop_ub)) {
      if (x_data[ix - 1]) {
        y = true;
        exitg1 = true;
      } else {
        ix++;
      }
    }
    if (y) {
      resetHome = true;
    } else {
      for (ix = 0; ix < loop_ub; ix++) {
        x_data[ix] = (HomePositionInternal[ix] < -1.6581);
      }
      y = false;
      ix = 1;
      exitg1 = false;
      while ((!exitg1) && (ix <= loop_ub)) {
        if (x_data[ix - 1]) {
          y = true;
          exitg1 = true;
        } else {
          ix++;
        }
      }
      if (y) {
        resetHome = true;
      }
    }
  } break;
  }
  d = PositionNumber;
  dv[0] = -1.6581;
  dv[1] = 1.6581;
  if (d < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(d);
  }
  for (ix = 0; ix < 2; ix++) {
    for (int i{0}; i < loop_ub; i++) {
      PositionLimitsInternal[i + 7 * ix] = dv[i + loop_ub * ix];
    }
  }
  if (resetHome) {
    resetHomePosition();
  }
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::f_init()
{
  static const char b_cv[15]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm',
                             '_', 'j', 'o', 'i', 'n', 't', '1'};
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 15.0;
  for (i = 0; i < 15; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 8.0;
  for (i = 0; i < 8; i++) {
    b_obj.Vector[i] = b_cv1[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
            if (b_cv3[ibmat] != b_obj.Vector[ibmat]) {
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
        if (b_obj.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::g_init()
{
  static const char b_cv[15]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm',
                             '_', 'j', 'o', 'i', 'n', 't', '2'};
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 15.0;
  for (i = 0; i < 15; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 8.0;
  for (i = 0; i < 8; i++) {
    b_obj.Vector[i] = b_cv1[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
            if (b_cv3[ibmat] != b_obj.Vector[ibmat]) {
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
        if (b_obj.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : double ax[3]
// Return Type  : void
//
void rigidBodyJoint::get_JointAxis(double ax[3]) const
{
  static const char b_cv[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  int exitg1;
  int i;
  int kstr;
  boolean_T b_bool;
  boolean_T guard1;
  if (TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(TypeInternal.Length);
  }
  b_bool = false;
  if (i == 8) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (TypeInternal.Vector[kstr] != b_cv[kstr]) {
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
  guard1 = false;
  if (b_bool) {
    guard1 = true;
  } else {
    b_bool = false;
    if (i == 9) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 9) {
          if (TypeInternal.Vector[kstr] != cv6[kstr]) {
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
    if (b_bool) {
      guard1 = true;
    } else {
      ax[0] = rtNaN;
      ax[1] = rtNaN;
      ax[2] = rtNaN;
    }
  }
  if (guard1) {
    ax[0] = JointAxisInternal[0];
    ax[1] = JointAxisInternal[1];
    ax[2] = JointAxisInternal[2];
  }
}

//
// Arguments    : double msubspace_data[]
//                int msubspace_size[2]
// Return Type  : void
//
void rigidBodyJoint::get_MotionSubspace(double msubspace_data[],
                                        int msubspace_size[2]) const
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  int i;
  int kstr;
  boolean_T b_bool;
  if (TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(TypeInternal.Length);
  }
  b_bool = false;
  if (i == 5) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (TypeInternal.Vector[kstr] != b_cv[kstr]) {
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
    double d;
    d = VelocityNumber;
    if (d < 1.0) {
      kstr = 0;
    } else {
      kstr = static_cast<int>(d);
    }
    msubspace_size[0] = 6;
    msubspace_size[1] = kstr;
    for (i = 0; i < kstr; i++) {
      for (int i1{0}; i1 < 6; i1++) {
        int msubspace_data_tmp;
        msubspace_data_tmp = i1 + 6 * i;
        msubspace_data[msubspace_data_tmp] =
            MotionSubspaceInternal[msubspace_data_tmp];
      }
    }
  } else {
    msubspace_size[0] = 6;
    msubspace_size[1] = 1;
    for (i = 0; i < 6; i++) {
      msubspace_data[i] = 0.0;
    }
  }
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::h_init()
{
  static const char b_cv[15]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm',
                             '_', 'j', 'o', 'i', 'n', 't', '3'};
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 15.0;
  for (i = 0; i < 15; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 8.0;
  for (i = 0; i < 8; i++) {
    b_obj.Vector[i] = b_cv1[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
            if (b_cv3[ibmat] != b_obj.Vector[ibmat]) {
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
        if (b_obj.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::i_init()
{
  static const char b_cv[15]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm',
                             '_', 'j', 'o', 'i', 'n', 't', '4'};
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 15.0;
  for (i = 0; i < 15; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 8.0;
  for (i = 0; i < 8; i++) {
    b_obj.Vector[i] = b_cv1[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
            if (b_cv3[ibmat] != b_obj.Vector[ibmat]) {
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
        if (b_obj.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : const char jname[14]
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::init(const char jname[14])
{
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv3[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 14.0;
  for (i = 0; i < 14; i++) {
    b_obj.Vector[i] = jname[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 5.0;
  for (i = 0; i < 5; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::init()
{
  static const char b_cv[8]{'b', 'a', 's', 'e', '_', 'j', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 8.0;
  for (i = 0; i < 8; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 5.0;
  for (i = 0; i < 5; i++) {
    b_obj.Vector[i] = b_cv1[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
            if (b_cv3[ibmat] != b_obj.Vector[ibmat]) {
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
        if (b_obj.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::j_init()
{
  static const char b_cv[15]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm',
                             '_', 'j', 'o', 'i', 'n', 't', '5'};
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 15.0;
  for (i = 0; i < 15; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 8.0;
  for (i = 0; i < 8; i++) {
    b_obj.Vector[i] = b_cv1[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
            if (b_cv3[ibmat] != b_obj.Vector[ibmat]) {
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
        if (b_obj.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::k_init()
{
  static const char b_cv[15]{'l', 'e', 'f', 't', '_', 'a', 'r', 'm',
                             '_', 'j', 'o', 'i', 'n', 't', '6'};
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 15.0;
  for (i = 0; i < 15; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 8.0;
  for (i = 0; i < 8; i++) {
    b_obj.Vector[i] = b_cv1[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
            if (b_cv3[ibmat] != b_obj.Vector[ibmat]) {
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
        if (b_obj.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint *
//
rigidBodyJoint *rigidBodyJoint::l_init()
{
  static const char b_cv[18]{'l', 'e', 'f', 't', '_', 'g', 'r', 'i', 'p',
                             'p', 'e', 'r', '_', 'j', 'o', 'i', 'n', 't'};
  static const char b_cv2[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv3[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const signed char iv2[7]{1, 0, 0, 0, 0, 0, 0};
  static const signed char b_iv[6]{0, 0, 1, 0, 0, 0};
  static const signed char b_iv1[6]{0, 0, 0, 0, 0, 1};
  static const char b_cv1[5]{'f', 'i', 'x', 'e', 'd'};
  static const char b_cv4[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  robotics::manip::internal::CharacterVector b_obj;
  double msubspace_data[36];
  double poslim_data[14];
  int exitg1;
  int homepos_size_idx_1;
  int i;
  int ibmat;
  int poslim_size_idx_0;
  signed char homepos_data[7];
  signed char b_tmp;
  boolean_T result;
  obj = this;
  obj->InTree = false;
  for (i = 0; i < 16; i++) {
    b_tmp = iv[i];
    obj->JointToParentTransform[i] = b_tmp;
    obj->ChildToJointTransform[i] = b_tmp;
  }
  for (i = 0; i < 14; i++) {
    obj->PositionLimitsInternal[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    obj->HomePositionInternal[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    obj->MotionSubspaceInternal[i] = 0.0;
  }
  for (i = 0; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->NameInternal = b_obj;
  obj->TypeInternal = b_obj;
  b_obj = obj->NameInternal;
  b_obj.Length = 18.0;
  for (i = 0; i < 18; i++) {
    b_obj.Vector[i] = b_cv[i];
  }
  obj->NameInternal = b_obj;
  b_obj = obj->TypeInternal;
  b_obj.Length = 5.0;
  for (i = 0; i < 5; i++) {
    b_obj.Vector[i] = b_cv1[i];
  }
  obj->TypeInternal = b_obj;
  b_obj = obj->TypeInternal;
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
            if (b_cv3[ibmat] != b_obj.Vector[ibmat]) {
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
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
    obj->VelocityNumber = 1.0;
    obj->PositionNumber = 1.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 1.0;
    break;
  case 2: {
    signed char b_I[36];
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
    obj->VelocityNumber = 6.0;
    obj->PositionNumber = 7.0;
    obj->JointAxisInternal[0] = rtNaN;
    obj->JointAxisInternal[1] = rtNaN;
    obj->JointAxisInternal[2] = rtNaN;
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
    obj->VelocityNumber = 0.0;
    obj->PositionNumber = 0.0;
    obj->JointAxisInternal[0] = 0.0;
    obj->JointAxisInternal[1] = 0.0;
    obj->JointAxisInternal[2] = 0.0;
    break;
  }
  obj->set_MotionSubspace(msubspace_data);
  b_obj = obj->TypeInternal;
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
        if (b_obj.Vector[ibmat] != b_cv4[ibmat]) {
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
    d = obj->PositionNumber;
    if (d < 1.0) {
      ibmat = 0;
    } else {
      ibmat = static_cast<int>(d);
    }
    for (i = 0; i < 2; i++) {
      for (int i1{0}; i1 < ibmat; i1++) {
        obj->PositionLimitsInternal[i1 + 7 * i] =
            poslim_data[i1 + poslim_size_idx_0 * i];
      }
    }
    for (i = 0; i < homepos_size_idx_1; i++) {
      obj->HomePositionInternal[i] = homepos_data[i];
    }
  } else {
    obj->PositionLimitsInternal[0] = poslim_data[0];
    obj->PositionLimitsInternal[7] = poslim_data[1];
    obj->HomePositionInternal[0] = homepos_data[0];
  }
  return obj;
}

//
// Arguments    : void
// Return Type  : rigidBodyJoint
//
rigidBodyJoint::rigidBodyJoint() = default;

//
// Arguments    : void
// Return Type  : void
//
rigidBodyJoint::~rigidBodyJoint() = default;

//
// Arguments    : void
// Return Type  : void
//
void rigidBodyJoint::set_JointAxis()
{
  static const double dv[6]{0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
  static const double dv1[6]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static const char b_cv[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  int kstr;
  boolean_T result;
  JointAxisInternal[0] = 1.0;
  JointAxisInternal[1] = 0.0;
  JointAxisInternal[2] = 0.0;
  if (TypeInternal.Length < 1.0) {
    kstr = 0;
  } else {
    kstr = static_cast<int>(TypeInternal.Length);
  }
  result = false;
  if (kstr == 8) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (b_cv[kstr] != TypeInternal.Vector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    kstr = 0;
  } else {
    kstr = -1;
  }
  if (kstr == 0) {
    b_set_MotionSubspace(dv1);
  } else {
    b_set_MotionSubspace(dv);
  }
}

//
// Arguments    : const double msubspace_data[]
// Return Type  : void
//
void rigidBodyJoint::set_MotionSubspace(const double msubspace_data[])
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  int i;
  int kstr;
  boolean_T b_bool;
  if (TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(TypeInternal.Length);
  }
  b_bool = false;
  if (i == 5) {
    kstr = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (TypeInternal.Vector[kstr] != b_cv[kstr]) {
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
    double d;
    d = VelocityNumber;
    if (d < 1.0) {
      kstr = 0;
    } else {
      kstr = static_cast<int>(d);
    }
    for (i = 0; i < kstr; i++) {
      for (int i1{0}; i1 < 6; i1++) {
        int i2;
        i2 = i1 + 6 * i;
        MotionSubspaceInternal[i2] = msubspace_data[i2];
      }
    }
  } else {
    for (i = 0; i < 6; i++) {
      MotionSubspaceInternal[i] = 0.0;
    }
  }
}

//
// Arguments    : GIKSolver *aInstancePtr
// Return Type  : void
//
void rigidBodyJoint::set_PositionLimits(GIKSolver *aInstancePtr)
{
  static const double lims[2]{0.0, 0.0};
  solveGIKStepWrapperStackData *localSD;
  double d;
  int loop_ub;
  localSD = aInstancePtr->getStackData();
  for (int b_j0{0}; b_j0 < 2; b_j0++) {
    localSD->pd->lims[b_j0] = lims[b_j0];
  }
  localSD->pd->lims[0U] = rtGetMinusInf();
  localSD->pd->lims[1U] = rtGetInf();
  d = PositionNumber;
  if (d < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(d);
  }
  for (int i{0}; i < 2; i++) {
    for (int i1{0}; i1 < loop_ub; i1++) {
      PositionLimitsInternal[i1 + 7 * i] = localSD->pd->lims[i1 + loop_ub * i];
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void rigidBodyJoint::set_PositionLimits()
{
  double d;
  int ix;
  int loop_ub;
  signed char b_iv[2];
  boolean_T resetHome;
  resetHome = false;
  switch (static_cast<int>(PositionNumber)) {
  case 0:
  case 7:
    break;
  default: {
    boolean_T x_data[7];
    boolean_T exitg1;
    boolean_T y;
    d = PositionNumber;
    if (d < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(d);
    }
    for (ix = 0; ix < loop_ub; ix++) {
      x_data[ix] = (HomePositionInternal[ix] > 50.0);
    }
    y = false;
    ix = 1;
    exitg1 = false;
    while ((!exitg1) && (ix <= loop_ub)) {
      if (x_data[ix - 1]) {
        y = true;
        exitg1 = true;
      } else {
        ix++;
      }
    }
    if (y) {
      resetHome = true;
    } else {
      for (ix = 0; ix < loop_ub; ix++) {
        x_data[ix] = (HomePositionInternal[ix] < -50.0);
      }
      y = false;
      ix = 1;
      exitg1 = false;
      while ((!exitg1) && (ix <= loop_ub)) {
        if (x_data[ix - 1]) {
          y = true;
          exitg1 = true;
        } else {
          ix++;
        }
      }
      if (y) {
        resetHome = true;
      }
    }
  } break;
  }
  d = PositionNumber;
  b_iv[0] = -50;
  b_iv[1] = 50;
  if (d < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(d);
  }
  for (ix = 0; ix < 2; ix++) {
    for (int i{0}; i < loop_ub; i++) {
      PositionLimitsInternal[i + 7 * ix] = b_iv[i + loop_ub * ix];
    }
  }
  if (resetHome) {
    resetHomePosition();
  }
}

//
// Arguments    : const ::coder::array<double, 1U> &q
//                double T[16]
// Return Type  : void
//
void rigidBodyJoint::transformBodyToParent(const ::coder::array<double, 1U> &q,
                                           double T[16]) const
{
  static const char b_cv[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv1[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  double b[16];
  double b_I[16];
  double b_b[16];
  double tempR[9];
  double result_data[4];
  double v[3];
  double b_tempR_tmp;
  double cth;
  double sth;
  double tempR_tmp;
  int exitg1;
  int i;
  int kstr;
  boolean_T result;
  if (TypeInternal.Length < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(TypeInternal.Length);
  }
  result = false;
  if (i == 8) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (b_cv[kstr] != TypeInternal.Vector[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        result = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (result) {
    kstr = 0;
  } else {
    result = false;
    if (i == 9) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 9) {
          if (cv6[kstr] != TypeInternal.Vector[kstr]) {
            exitg1 = 1;
          } else {
            kstr++;
          }
        } else {
          result = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    if (result) {
      kstr = 1;
    } else {
      result = false;
      if (i == 8) {
        kstr = 0;
        do {
          exitg1 = 0;
          if (kstr < 8) {
            if (b_cv1[kstr] != TypeInternal.Vector[kstr]) {
              exitg1 = 1;
            } else {
              kstr++;
            }
          } else {
            result = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (result) {
        kstr = 2;
      } else {
        kstr = -1;
      }
    }
  }
  switch (kstr) {
  case 0: {
    __m128d r;
    double R[9];
    double c_tempR_tmp;
    double d_tempR_tmp;
    double e_tempR_tmp;
    double f_tempR_tmp;
    double g_tempR_tmp;
    double h_tempR_tmp;
    double i_tempR_tmp;
    double j_tempR_tmp;
    double k_tempR_tmp;
    signed char input_sizes_idx_1;
    get_JointAxis(v);
    input_sizes_idx_1 = static_cast<signed char>(q.size(0) != 0);
    result_data[0] = v[0];
    result_data[1] = v[1];
    result_data[2] = v[2];
    kstr = input_sizes_idx_1;
    if (kstr - 1 >= 0) {
      result_data[3] = q[0];
    }
    r = _mm_loadu_pd(&result_data[0]);
    _mm_storeu_pd(&v[0], _mm_mul_pd(r, r));
    sth = 1.0 / std::sqrt((v[0] + v[1]) + result_data[2] * result_data[2]);
    r = _mm_loadu_pd(&result_data[0]);
    _mm_storeu_pd(&v[0], _mm_mul_pd(r, _mm_set1_pd(sth)));
    v[2] = result_data[2] * sth;
    cth = std::cos(result_data[3]);
    sth = std::sin(result_data[3]);
    c_tempR_tmp = v[0] * v[0] * (1.0 - cth) + cth;
    tempR[0] = c_tempR_tmp;
    d_tempR_tmp = v[0] * v[1] * (1.0 - cth);
    e_tempR_tmp = v[2] * sth;
    f_tempR_tmp = d_tempR_tmp - e_tempR_tmp;
    tempR[1] = f_tempR_tmp;
    g_tempR_tmp = v[0] * v[2] * (1.0 - cth);
    h_tempR_tmp = v[1] * sth;
    i_tempR_tmp = g_tempR_tmp + h_tempR_tmp;
    tempR[2] = i_tempR_tmp;
    d_tempR_tmp += e_tempR_tmp;
    tempR[3] = d_tempR_tmp;
    e_tempR_tmp = v[1] * v[1] * (1.0 - cth) + cth;
    tempR[4] = e_tempR_tmp;
    j_tempR_tmp = v[1] * v[2] * (1.0 - cth);
    sth *= v[0];
    k_tempR_tmp = j_tempR_tmp - sth;
    tempR[5] = k_tempR_tmp;
    g_tempR_tmp -= h_tempR_tmp;
    tempR[6] = g_tempR_tmp;
    h_tempR_tmp = j_tempR_tmp + sth;
    tempR[7] = h_tempR_tmp;
    j_tempR_tmp = v[2] * v[2] * (1.0 - cth) + cth;
    tempR[8] = j_tempR_tmp;
    R[0] = c_tempR_tmp;
    R[1] = f_tempR_tmp;
    R[2] = i_tempR_tmp;
    R[3] = d_tempR_tmp;
    R[4] = e_tempR_tmp;
    R[5] = k_tempR_tmp;
    R[6] = g_tempR_tmp;
    R[7] = h_tempR_tmp;
    R[8] = j_tempR_tmp;
    for (kstr = 0; kstr < 3; kstr++) {
      R[kstr] = tempR[3 * kstr];
      R[kstr + 3] = tempR[3 * kstr + 1];
      R[kstr + 6] = tempR[3 * kstr + 2];
    }
    std::memset(&b[0], 0, 16U * sizeof(double));
    for (i = 0; i < 3; i++) {
      kstr = i << 2;
      b[kstr] = R[3 * i];
      b[kstr + 1] = R[3 * i + 1];
      b[kstr + 2] = R[3 * i + 2];
    }
    b[15] = 1.0;
  } break;
  case 1:
    get_JointAxis(v);
    std::memset(&tempR[0], 0, 9U * sizeof(double));
    tempR[0] = 1.0;
    tempR[4] = 1.0;
    tempR[8] = 1.0;
    for (i = 0; i < 3; i++) {
      kstr = i << 2;
      b[kstr] = tempR[3 * i];
      b[kstr + 1] = tempR[3 * i + 1];
      b[kstr + 2] = tempR[3 * i + 2];
      b[i + 12] = v[i] * q[0];
    }
    b[3] = 0.0;
    b[7] = 0.0;
    b[11] = 0.0;
    b[15] = 1.0;
    break;
  case 2: {
    __m128d r;
    double R[9];
    double c_tempR_tmp;
    double d_tempR_tmp;
    double e_tempR_tmp;
    double f_tempR_tmp;
    double g_tempR_tmp;
    double h_tempR_tmp;
    double i_tempR_tmp;
    double j_tempR_tmp;
    double k_tempR_tmp;
    std::memset(&b_I[0], 0, 16U * sizeof(double));
    b_I[0] = 1.0;
    b_I[5] = 1.0;
    b_I[10] = 1.0;
    b_I[15] = 1.0;
    b_I[12] = q[4];
    b_I[13] = q[5];
    b_I[14] = q[6];
    r = _mm_loadu_pd(&(((::coder::array<double, 1U> *)&q)->data())[0]);
    _mm_storeu_pd(&result_data[0], _mm_mul_pd(r, r));
    r = _mm_loadu_pd(&q[2]);
    _mm_storeu_pd(&result_data[2], _mm_mul_pd(r, r));
    sth = ((result_data[0] + result_data[1]) + result_data[2]) + result_data[3];
    r = _mm_set1_pd(1.0 / std::sqrt(sth));
    _mm_storeu_pd(
        &result_data[0],
        _mm_mul_pd(
            _mm_loadu_pd(&(((::coder::array<double, 1U> *)&q)->data())[0]), r));
    _mm_storeu_pd(&result_data[2], _mm_mul_pd(_mm_loadu_pd(&q[2]), r));
    c_tempR_tmp = result_data[3] * result_data[3];
    d_tempR_tmp = result_data[2] * result_data[2];
    e_tempR_tmp = 1.0 - 2.0 * (d_tempR_tmp + c_tempR_tmp);
    tempR[0] = e_tempR_tmp;
    f_tempR_tmp = result_data[1] * result_data[2];
    g_tempR_tmp = result_data[0] * result_data[3];
    h_tempR_tmp = 2.0 * (f_tempR_tmp - g_tempR_tmp);
    tempR[1] = h_tempR_tmp;
    i_tempR_tmp = result_data[1] * result_data[3];
    j_tempR_tmp = result_data[0] * result_data[2];
    sth = 2.0 * (i_tempR_tmp + j_tempR_tmp);
    tempR[2] = sth;
    f_tempR_tmp = 2.0 * (f_tempR_tmp + g_tempR_tmp);
    tempR[3] = f_tempR_tmp;
    g_tempR_tmp = result_data[1] * result_data[1];
    c_tempR_tmp = 1.0 - 2.0 * (g_tempR_tmp + c_tempR_tmp);
    tempR[4] = c_tempR_tmp;
    k_tempR_tmp = result_data[2] * result_data[3];
    tempR_tmp = result_data[0] * result_data[1];
    b_tempR_tmp = 2.0 * (k_tempR_tmp - tempR_tmp);
    tempR[5] = b_tempR_tmp;
    i_tempR_tmp = 2.0 * (i_tempR_tmp - j_tempR_tmp);
    tempR[6] = i_tempR_tmp;
    j_tempR_tmp = 2.0 * (k_tempR_tmp + tempR_tmp);
    tempR[7] = j_tempR_tmp;
    d_tempR_tmp = 1.0 - 2.0 * (g_tempR_tmp + d_tempR_tmp);
    tempR[8] = d_tempR_tmp;
    R[0] = e_tempR_tmp;
    R[1] = h_tempR_tmp;
    R[2] = sth;
    R[3] = f_tempR_tmp;
    R[4] = c_tempR_tmp;
    R[5] = b_tempR_tmp;
    R[6] = i_tempR_tmp;
    R[7] = j_tempR_tmp;
    R[8] = d_tempR_tmp;
    for (kstr = 0; kstr < 3; kstr++) {
      R[kstr] = tempR[3 * kstr];
      R[kstr + 3] = tempR[3 * kstr + 1];
      R[kstr + 6] = tempR[3 * kstr + 2];
    }
    std::memset(&b_b[0], 0, 16U * sizeof(double));
    for (i = 0; i < 3; i++) {
      kstr = i << 2;
      b_b[kstr] = R[3 * i];
      b_b[kstr + 1] = R[3 * i + 1];
      b_b[kstr + 2] = R[3 * i + 2];
    }
    b_b[15] = 1.0;
    for (i = 0; i < 4; i++) {
      sth = b_I[i];
      cth = b_I[i + 4];
      tempR_tmp = b_I[i + 8];
      b_tempR_tmp = b_I[i + 12];
      for (int i1{0}; i1 < 4; i1++) {
        kstr = i1 << 2;
        b[i + kstr] = ((sth * b_b[kstr] + cth * b_b[kstr + 1]) +
                       tempR_tmp * b_b[kstr + 2]) +
                      b_tempR_tmp * b_b[kstr + 3];
      }
    }
  } break;
  default:
    std::memset(&b[0], 0, 16U * sizeof(double));
    b[0] = 1.0;
    b[5] = 1.0;
    b[10] = 1.0;
    b[15] = 1.0;
    break;
  }
  for (i = 0; i < 4; i++) {
    sth = JointToParentTransform[i];
    cth = JointToParentTransform[i + 4];
    tempR_tmp = JointToParentTransform[i + 8];
    b_tempR_tmp = JointToParentTransform[i + 12];
    for (int i1{0}; i1 < 4; i1++) {
      kstr = i1 << 2;
      b_I[i + kstr] =
          ((sth * b[kstr] + cth * b[kstr + 1]) + tempR_tmp * b[kstr + 2]) +
          b_tempR_tmp * b[kstr + 3];
    }
    sth = b_I[i];
    cth = b_I[i + 4];
    tempR_tmp = b_I[i + 8];
    b_tempR_tmp = b_I[i + 12];
    for (int i1{0}; i1 < 4; i1++) {
      kstr = i1 << 2;
      T[i + kstr] = ((sth * ChildToJointTransform[kstr] +
                      cth * ChildToJointTransform[kstr + 1]) +
                     tempR_tmp * ChildToJointTransform[kstr + 2]) +
                    b_tempR_tmp * ChildToJointTransform[kstr + 3];
    }
  }
}

//
// Arguments    : ::coder::array<double, 1U> &in1
//                const double in2_data[]
//                const int in2_size[2]
// Return Type  : void
//
} // namespace coder
void binary_expand_op_5(::coder::array<double, 1U> &in1,
                        const double in2_data[], const int in2_size[2])
{
  double b_in2_data[7];
  int i;
  int loop_ub;
  int stride_0_0_tmp;
  int stride_1_0;
  if (in2_size[0] == 1) {
    i = in1.size(0);
  } else {
    i = in2_size[0];
  }
  if (i == 1) {
    loop_ub = in2_size[0];
  } else {
    loop_ub = i;
  }
  stride_0_0_tmp = (in2_size[0] != 1);
  stride_1_0 = (in1.size(0) != 1);
  for (i = 0; i < loop_ub; i++) {
    double b_in2_tmp;
    int in2_tmp;
    in2_tmp = i * stride_0_0_tmp;
    b_in2_tmp = in2_data[in2_tmp];
    b_in2_data[i] =
        b_in2_tmp +
        in1[i * stride_1_0] * (in2_data[in2_tmp + in2_size[0]] - b_in2_tmp);
  }
  in1.set_size(loop_ub);
  for (i = 0; i < loop_ub; i++) {
    in1[i] = b_in2_data[i];
  }
}

//
// Arguments    : ::coder::array<double, 1U> &in1
//                const double in2_data[]
//                const int in2_size[2]
//                const ::coder::array<double, 1U> &in3
// Return Type  : void
//
void binary_expand_op_6(::coder::array<double, 1U> &in1,
                        const double in2_data[], const int in2_size[2],
                        const ::coder::array<double, 1U> &in3)
{
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in3.size(0) == 1) {
    loop_ub = in2_size[0];
  } else {
    loop_ub = in3.size(0);
  }
  in1.set_size(loop_ub);
  stride_0_0 = (in2_size[0] != 1);
  stride_1_0 = (in3.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    in1[i] = in2_data[i * stride_0_0] + in3[i * stride_1_0];
  }
}

//
// Arguments    : ::coder::array<double, 1U> &in1
//                const double in2_data[]
//                const int in2_size[2]
//                const ::coder::array<double, 1U> &in3
// Return Type  : void
//
void binary_expand_op_7(::coder::array<double, 1U> &in1,
                        const double in2_data[], const int in2_size[2],
                        const ::coder::array<double, 1U> &in3)
{
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  if (in3.size(0) == 1) {
    loop_ub = in2_size[0];
  } else {
    loop_ub = in3.size(0);
  }
  in1.set_size(loop_ub);
  stride_0_0 = (in2_size[0] != 1);
  stride_1_0 = (in3.size(0) != 1);
  for (int i{0}; i < loop_ub; i++) {
    in1[i] = in2_data[i * stride_0_0 + in2_size[0]] - in3[i * stride_1_0];
  }
}

} // namespace gik9dof

//
// File trailer for rigidBodyJoint.cpp
//
// [EOF]
//
