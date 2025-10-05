//
// PoseTarget.cpp
//
// Code generation for function 'PoseTarget'
//

// Include files
#include "PoseTarget.h"
#include "CharacterVector.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "svd.h"
#include "rt_defines.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
PoseTarget::PoseTarget() = default;

PoseTarget::~PoseTarget() = default;

void PoseTarget::evaluateFromTransform(const double T_data[],
                                       const int T_size[2], double g[2],
                                       double JTwist[12]) const
{
  creal_T u;
  double T[9];
  double y[9];
  double vspecial_data[3];
  double b_err;
  double d;
  double d1;
  double err;
  double err_idx_1;
  double err_idx_3;
  double err_idx_4;
  double q;
  double u_tmp;
  int i;
  int i1;
  signed char R_tmp[3];
  bool exitg1;
  bool rEQ0;
  i = T_size[0];
  for (i1 = 0; i1 < 3; i1++) {
    R_tmp[i1] = static_cast<signed char>(i1 + 1);
    T[3 * i1] = T_data[static_cast<signed char>(i1 + 1) - 1];
    T[3 * i1 + 1] = T_data[(static_cast<signed char>(i1 + 1) + i) - 1];
    T[3 * i1 + 2] = T_data[(static_cast<signed char>(i1 + 1) + i * 2) - 1];
  }
  for (i = 0; i < 3; i++) {
    d = TargetTransform[i];
    d1 = TargetTransform[i + 4];
    u_tmp = TargetTransform[i + 8];
    for (i1 = 0; i1 < 3; i1++) {
      y[i + 3 * i1] =
          (d * T[3 * i1] + d1 * T[3 * i1 + 1]) + u_tmp * T[3 * i1 + 2];
    }
  }
  u_tmp = 0.5 * (((y[0] + y[4]) + y[8]) - 1.0);
  if (!(std::abs(u_tmp) > 1.0)) {
    u.re = std::acos(u_tmp);
  } else {
    creal_T v;
    bool b;
    v.re = u_tmp + 1.0;
    v.im = 0.0;
    ::coder::internal::scalar::b_sqrt(v);
    u.re = 1.0 - u_tmp;
    u.im = 0.0;
    ::coder::internal::scalar::b_sqrt(u);
    rEQ0 = std::isnan(v.re);
    b = std::isnan(u.re);
    if (b || rEQ0) {
      u_tmp = rtNaN;
    } else if (std::isinf(u.re) && std::isinf(v.re)) {
      if (u.re > 0.0) {
        i = 1;
      } else {
        i = -1;
      }
      if (v.re > 0.0) {
        i1 = 1;
      } else {
        i1 = -1;
      }
      u_tmp = std::atan2(static_cast<double>(i), static_cast<double>(i1));
    } else if (v.re == 0.0) {
      if (u.re > 0.0) {
        u_tmp = RT_PI / 2.0;
      } else if (u.re < 0.0) {
        u_tmp = -(RT_PI / 2.0);
      } else {
        u_tmp = 0.0;
      }
    } else {
      u_tmp = std::atan2(u.re, v.re);
    }
    u.re = 2.0 * u_tmp;
  }
  u_tmp = 2.0 * std::sin(u.re);
  vspecial_data[0] = (y[5] - y[7]) / u_tmp;
  vspecial_data[1] = (y[6] - y[2]) / u_tmp;
  vspecial_data[2] = (y[1] - y[3]) / u_tmp;
  if (std::isnan(u.re) || std::isinf(u.re)) {
    u_tmp = rtNaN;
  } else if (u.re == 0.0) {
    u_tmp = 0.0;
  } else {
    u_tmp = std::fmod(u.re, 3.1415926535897931);
    rEQ0 = (u_tmp == 0.0);
    if (!rEQ0) {
      q = std::abs(u.re / 3.1415926535897931);
      rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }
    if (rEQ0) {
      u_tmp = 0.0;
    } else if (u_tmp < 0.0) {
      u_tmp += 3.1415926535897931;
    }
  }
  rEQ0 = true;
  i1 = 0;
  exitg1 = false;
  while ((!exitg1) && (i1 < 3)) {
    if (!(vspecial_data[i1] == 0.0)) {
      rEQ0 = false;
      exitg1 = true;
    } else {
      i1++;
    }
  }
  if ((u_tmp == 0.0) || rEQ0) {
    for (i = 0; i < 3; i++) {
      vspecial_data[i] = 0.0;
    }
    for (int b_i{0}; b_i < 1; b_i++) {
      double V[9];
      std::memset(&T[0], 0, 9U * sizeof(double));
      T[0] = 1.0;
      T[4] = 1.0;
      T[8] = 1.0;
      rEQ0 = true;
      for (i1 = 0; i1 < 9; i1++) {
        d = T[i1] - y[i1];
        T[i1] = d;
        if ((!rEQ0) || (std::isinf(d) || std::isnan(d))) {
          rEQ0 = false;
        }
      }
      if (rEQ0) {
        double U[9];
        ::coder::internal::svd(T, U, vspecial_data, V);
      } else {
        for (i = 0; i < 9; i++) {
          V[i] = rtNaN;
        }
      }
      vspecial_data[0] = V[6];
      vspecial_data[1] = V[7];
      vspecial_data[2] = V[8];
    }
  }
  u_tmp = 1.0 / std::sqrt((vspecial_data[0] * vspecial_data[0] +
                           vspecial_data[1] * vspecial_data[1]) +
                          vspecial_data[2] * vspecial_data[2]);
  d = u.re * (vspecial_data[0] * u_tmp);
  q = d;
  d1 = TargetTransform[12] - T_data[(R_tmp[0] + T_size[0] * 3) - 1];
  err_idx_3 = d1;
  err = d * d;
  b_err = d1 * d1;
  d = u.re * (vspecial_data[1] * u_tmp);
  err_idx_1 = d;
  d1 = TargetTransform[13] - T_data[(R_tmp[1] + T_size[0] * 3) - 1];
  err_idx_4 = d1;
  err += d * d;
  b_err += d1 * d1;
  d = u.re * (vspecial_data[2] * u_tmp);
  d1 = TargetTransform[14] - T_data[(R_tmp[2] + T_size[0] * 3) - 1];
  err += d * d;
  b_err += d1 * d1;
  g[0] = std::sqrt(err + 2.2204460492503131E-16);
  g[1] = std::sqrt(b_err + 2.2204460492503131E-16);
  JTwist[0] = -(q / g[0]);
  JTwist[6] = -0.0;
  JTwist[1] = -0.0;
  JTwist[7] = -(err_idx_3 / g[1]);
  JTwist[2] = -(err_idx_1 / g[0]);
  JTwist[8] = -0.0;
  JTwist[3] = -0.0;
  JTwist[9] = -(err_idx_4 / g[1]);
  JTwist[4] = -(d / g[0]);
  JTwist[10] = -0.0;
  JTwist[5] = -0.0;
  JTwist[11] = -(d1 / g[1]);
}

void PoseTarget::get_EndEffector(char value_data[], int value_size[2])
{
  CharacterVector c_obj;
  RigidBody *b_obj;
  b_RigidBodyTree *obj;
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

void PoseTarget::get_ReferenceBody(char value_data[], int value_size[2])
{
  CharacterVector c_obj;
  RigidBody *b_obj;
  b_RigidBodyTree *obj;
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

// End of code generation (PoseTarget.cpp)
