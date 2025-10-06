//
// File: GIKProblem.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 06-Oct-2025 17:03:24
//

// Include Files
#include "GIKProblem.h"
#include "CharacterVector.h"
#include "DistanceBoundsConstraint.h"
#include "JointPositionBounds.h"
#include "PoseTarget.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "gik9dof_codegen_realtime_solveGIKStepWrapper_types1.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "strcmp.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Type Definitions
namespace gik9dof {
struct cell_wrap_63 {
  ::coder::array<double, 2U> f1;
};

} // namespace gik9dof

// Function Declarations
namespace gik9dof {
static void binary_expand_op_1(::coder::array<double, 1U> &in1,
                               const ::coder::array<int, 1U> &in2,
                               const ::coder::array<double, 1U> &in3, int in4,
                               const ::coder::array<double, 1U> &in5);

static void binary_expand_op_2(::coder::array<double, 1U> &in1,
                               const ::coder::array<int, 1U> &in2,
                               const double in3[2],
                               const ::coder::array<double, 1U> &in4);

} // namespace gik9dof

// Function Definitions
//
// Arguments    : ::coder::array<double, 1U> &in1
//                const ::coder::array<int, 1U> &in2
//                const ::coder::array<double, 1U> &in3
//                int in4
//                const ::coder::array<double, 1U> &in5
// Return Type  : void
//
namespace gik9dof {
static void binary_expand_op_1(::coder::array<double, 1U> &in1,
                               const ::coder::array<int, 1U> &in2,
                               const ::coder::array<double, 1U> &in3, int in4,
                               const ::coder::array<double, 1U> &in5)
{
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  stride_0_0 = (in4 + 1 != 1);
  stride_1_0 = (in5.size(0) != 1);
  loop_ub = in2.size(0);
  for (int i{0}; i < loop_ub; i++) {
    in1[in2[i]] = in3[i * stride_0_0] - in5[i * stride_1_0];
  }
}

//
// Arguments    : ::coder::array<double, 1U> &in1
//                const ::coder::array<int, 1U> &in2
//                const double in3[2]
//                const ::coder::array<double, 1U> &in4
// Return Type  : void
//
static void binary_expand_op_2(::coder::array<double, 1U> &in1,
                               const ::coder::array<int, 1U> &in2,
                               const double in3[2],
                               const ::coder::array<double, 1U> &in4)
{
  int loop_ub;
  int stride_0_0;
  stride_0_0 = (in4.size(0) != 1);
  loop_ub = in2.size(0);
  for (int i{0}; i < loop_ub; i++) {
    in1[in2[i]] = in3[i] - in4[i * stride_0_0];
  }
}

//
// Arguments    : void
// Return Type  : GIKProblem
//
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
GIKProblem::GIKProblem() = default;

//
// Arguments    : void
// Return Type  : void
//
GIKProblem::~GIKProblem() = default;

//
// Arguments    : const ::coder::array<double, 1U> &x
//                ::coder::array<double, 1U> &ev
// Return Type  : double
//
double GIKProblem::evaluateSolution(const ::coder::array<double, 1U> &x,
                                    ::coder::array<double, 1U> &ev)
{
  ::coder::array<double, 2U> a;
  ::coder::array<double, 1U> y;
  int inner;
  int mc_tmp;
  residuals(x, ev);
  get_WeightMatrix(a);
  mc_tmp = a.size(0);
  inner = a.size(1);
  y.set_size(a.size(0));
  for (int i{0}; i < mc_tmp; i++) {
    y[i] = 0.0;
  }
  for (int k{0}; k < inner; k++) {
    int aoffset;
    int scalarLB;
    int vectorUB;
    aoffset = k * a.size(0);
    scalarLB = (mc_tmp / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int i{0}; i <= vectorUB; i += 2) {
      __m128d r;
      __m128d r1;
      r = _mm_loadu_pd(&a[aoffset + i]);
      r1 = _mm_loadu_pd(&y[i]);
      _mm_storeu_pd(&y[i], _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(ev[k]))));
    }
    for (int i{scalarLB}; i < mc_tmp; i++) {
      y[i] = y[i] + a[aoffset + i] * ev[k];
    }
  }
  return b_norm(y);
}

//
// Arguments    : double value_data[]
//                int value_size[2]
// Return Type  : void
//
void GIKProblem::get_KinematicPath(double value_data[], int value_size[2])
{
  DistanceBoundsConstraint *c_obj;
  PoseTarget *obj;
  RigidBody *body1;
  RigidBody *body2;
  RigidBodyTree *b_obj;
  ::coder::array<double, 2U> b_value;
  ::coder::array<double, 2U> c_value;
  ::coder::array<double, 2U> r;
  double y_data[36];
  double bid1;
  double bid2;
  int idx_data[36];
  int iwork_data[36];
  int body2Name_size[2];
  int y_size[2];
  int b_i;
  int i;
  int j;
  int k;
  int n;
  int nb;
  int nd2;
  int qEnd;
  char body1Name_data[200];
  char body2Name_data[200];
  boolean_T exitg1;
  obj = Constraints.f1;
  b_obj = obj->Tree;
  obj->get_EndEffector(body1Name_data, y_size);
  obj->get_ReferenceBody(body2Name_data, body2Name_size);
  bid1 = b_obj->findBodyIndexByName(body1Name_data, y_size);
  bid2 = b_obj->findBodyIndexByName(body2Name_data, body2Name_size);
  if (bid1 == 0.0) {
    body1 = &b_obj->Base;
  } else {
    body1 = b_obj->Bodies[static_cast<int>(bid1) - 1];
  }
  if (bid2 == 0.0) {
    body2 = &b_obj->Base;
  } else {
    body2 = b_obj->Bodies[static_cast<int>(bid2) - 1];
  }
  b_obj->kinematicPathInternal(body1, body2, b_value);
  Constraints.f2->get_KinematicPath(r);
  i = b_value.size(1) + r.size(1);
  c_value.set_size(1, i);
  nd2 = b_value.size(1);
  for (b_i = 0; b_i < nd2; b_i++) {
    c_value[b_i] = b_value[b_i];
  }
  nd2 = r.size(1);
  for (b_i = 0; b_i < nd2; b_i++) {
    c_value[b_i + b_value.size(1)] = r[b_i];
  }
  c_obj = Constraints.f3;
  b_obj = c_obj->Tree;
  c_obj->get_EndEffector(body1Name_data, y_size);
  c_obj->get_ReferenceBody(body2Name_data, body2Name_size);
  bid1 = b_obj->findBodyIndexByName(body1Name_data, y_size);
  bid2 = b_obj->findBodyIndexByName(body2Name_data, body2Name_size);
  if (bid1 == 0.0) {
    body1 = &b_obj->Base;
  } else {
    body1 = b_obj->Bodies[static_cast<int>(bid1) - 1];
  }
  if (bid2 == 0.0) {
    body2 = &b_obj->Base;
  } else {
    body2 = b_obj->Bodies[static_cast<int>(bid2) - 1];
  }
  b_obj->kinematicPathInternal(body1, body2, b_value);
  y_size[0] = 1;
  y_size[1] = c_value.size(1) + b_value.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    y_data[b_i] = c_value[b_i];
  }
  i = b_value.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    y_data[b_i + c_value.size(1)] = b_value[b_i];
  }
  ::gik9dof::coder::internal::b_sort(y_data, y_size);
  nd2 = y_size[1];
  n = y_size[1] + 1;
  if (nd2 - 1 >= 0) {
    std::memset(&idx_data[0], 0, static_cast<unsigned int>(nd2) * sizeof(int));
  }
  if (y_size[1] != 0) {
    b_i = y_size[1] - 1;
    for (k = 1; k <= b_i; k += 2) {
      bid1 = y_data[k];
      if ((y_data[k - 1] <= bid1) || std::isnan(bid1)) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }
    if ((static_cast<unsigned int>(y_size[1]) & 1U) != 0U) {
      idx_data[y_size[1] - 1] = y_size[1];
    }
    i = 2;
    while (i < n - 1) {
      nb = i << 1;
      j = 1;
      for (int pEnd{i + 1}; pEnd < n; pEnd = qEnd + i) {
        int kEnd;
        int p;
        int q;
        p = j;
        q = pEnd - 1;
        qEnd = j + nb;
        if (qEnd > n) {
          qEnd = n;
        }
        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          bid1 = y_data[idx_data[q] - 1];
          b_i = idx_data[p - 1];
          if ((y_data[b_i - 1] <= bid1) || std::isnan(bid1)) {
            iwork_data[k] = b_i;
            p++;
            if (p == pEnd) {
              while (q + 1 < qEnd) {
                k++;
                iwork_data[k] = idx_data[q];
                q++;
              }
            }
          } else {
            iwork_data[k] = idx_data[q];
            q++;
            if (q + 1 == qEnd) {
              while (p < pEnd) {
                k++;
                iwork_data[k] = idx_data[p - 1];
                p++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          idx_data[(j + k) - 1] = iwork_data[k];
        }
        j = qEnd;
      }
      i = nb;
    }
  }
  b_value.set_size(1, y_size[1]);
  for (k = 0; k < nd2; k++) {
    b_value[k] = y_data[idx_data[k] - 1];
  }
  k = 0;
  while ((k + 1 <= nd2) && std::isinf(b_value[k]) && (b_value[k] < 0.0)) {
    k++;
  }
  n = k;
  k = y_size[1];
  while ((k >= 1) && std::isnan(b_value[k - 1])) {
    k--;
  }
  i = y_size[1] - k;
  exitg1 = false;
  while ((!exitg1) && (k >= 1)) {
    bid1 = b_value[k - 1];
    if (std::isinf(bid1) && (bid1 > 0.0)) {
      k--;
    } else {
      exitg1 = true;
    }
  }
  nd2 = (y_size[1] - k) - i;
  nb = -1;
  if (n > 0) {
    nb = 0;
  }
  while (n + 1 <= k) {
    bid1 = b_value[n];
    do {
      n++;
    } while (!((n + 1 > k) || (b_value[n] != bid1)));
    nb++;
    b_value[nb] = bid1;
  }
  if (nd2 > 0) {
    nb++;
    b_value[nb] = b_value[k];
  }
  n = k + nd2;
  for (j = 0; j < i; j++) {
    b_value[(nb + j) + 1] = b_value[n + j];
  }
  if (i - 1 >= 0) {
    nb += i;
  }
  if (nb + 1 < 1) {
    i = 0;
  } else {
    i = nb + 1;
  }
  b_value.set_size(b_value.size(0), i);
  nd2 = b_value.size(1) >> 1;
  for (nb = 0; nb < nd2; nb++) {
    n = (b_value.size(1) - nb) - 1;
    bid1 = b_value[nb];
    b_value[nb] = b_value[n];
    b_value[n] = bid1;
  }
  value_size[0] = 1;
  value_size[1] = i;
  for (b_i = 0; b_i < i; b_i++) {
    value_data[b_i] = b_value[b_i];
  }
}

//
// Arguments    : ::coder::array<double, 2U> &b_value
// Return Type  : void
//
void GIKProblem::get_WeightMatrix(::coder::array<double, 2U> &b_value) const
{
  ::coder::array<double, 2U> r3;
  ::coder::array<double, 2U> v;
  ::coder::array<double, 1U> r;
  ::coder::array<int, 1U> r1;
  ::coder::array<int, 1U> r2;
  double t;
  int i;
  int i1;
  int loop_ub;
  int loop_ub_tmp;
  int m_tmp;
  t = NumResiduals;
  if (t < 0.0) {
    t = 0.0;
  }
  m_tmp = static_cast<int>(t);
  b_value.set_size(static_cast<int>(t), static_cast<int>(t));
  loop_ub_tmp = static_cast<int>(t) * static_cast<int>(t);
  for (i = 0; i < loop_ub_tmp; i++) {
    b_value[i] = 0.0;
  }
  if (static_cast<int>(t) > 0) {
    for (loop_ub_tmp = 0; loop_ub_tmp < m_tmp; loop_ub_tmp++) {
      b_value[loop_ub_tmp + b_value.size(0) * loop_ub_tmp] = 1.0;
    }
  }
  loop_ub = ResidualIndices[0].f1.size(1);
  r.set_size(loop_ub);
  m_tmp = ResidualIndices[0].f1.size(1);
  for (i = 0; i < m_tmp; i++) {
    r[i] = ResidualIndices[0].f1[i];
  }
  r1.set_size(loop_ub);
  r2.set_size(loop_ub);
  for (i = 0; i < loop_ub; i++) {
    i1 = static_cast<int>(r[i]) - 1;
    r1[i] = i1;
    r2[i] = i1;
  }
  i = Constraints.f1->Weights.size(1);
  v.set_size(1, i);
  m_tmp = Constraints.f1->Weights.size(1);
  for (i1 = 0; i1 < m_tmp; i1++) {
    v[i1] = Constraints.f1->Weights[i1];
  }
  r3.set_size(i, i);
  loop_ub_tmp = v.size(1) * v.size(1);
  for (i1 = 0; i1 < loop_ub_tmp; i1++) {
    r3[i1] = 0.0;
  }
  for (m_tmp = 0; m_tmp < i; m_tmp++) {
    r3[m_tmp + r3.size(0) * m_tmp] = v[m_tmp];
  }
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
    }
  }
  loop_ub = ResidualIndices[1].f1.size(1);
  r.set_size(loop_ub);
  m_tmp = ResidualIndices[1].f1.size(1);
  for (i = 0; i < m_tmp; i++) {
    r[i] = ResidualIndices[1].f1[i];
  }
  r1.set_size(loop_ub);
  r2.set_size(loop_ub);
  for (i = 0; i < loop_ub; i++) {
    i1 = static_cast<int>(r[i]) - 1;
    r1[i] = i1;
    r2[i] = i1;
  }
  i = Constraints.f2->Weights.size(1);
  v.set_size(1, i);
  m_tmp = Constraints.f2->Weights.size(1);
  for (i1 = 0; i1 < m_tmp; i1++) {
    v[i1] = Constraints.f2->Weights[i1];
  }
  r3.set_size(i, i);
  loop_ub_tmp = v.size(1) * v.size(1);
  for (i1 = 0; i1 < loop_ub_tmp; i1++) {
    r3[i1] = 0.0;
  }
  for (m_tmp = 0; m_tmp < i; m_tmp++) {
    r3[m_tmp + r3.size(0) * m_tmp] = v[m_tmp];
  }
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
    }
  }
  loop_ub = ResidualIndices[2].f1.size(1);
  r.set_size(loop_ub);
  m_tmp = ResidualIndices[2].f1.size(1);
  for (i = 0; i < m_tmp; i++) {
    r[i] = ResidualIndices[2].f1[i];
  }
  r1.set_size(loop_ub);
  r2.set_size(loop_ub);
  for (i = 0; i < loop_ub; i++) {
    i1 = static_cast<int>(r[i]) - 1;
    r1[i] = i1;
    r2[i] = i1;
  }
  i = Constraints.f3->Weights.size(1);
  v.set_size(1, i);
  m_tmp = Constraints.f3->Weights.size(1);
  for (i1 = 0; i1 < m_tmp; i1++) {
    v[i1] = Constraints.f3->Weights[i1];
  }
  r3.set_size(i, i);
  loop_ub_tmp = v.size(1) * v.size(1);
  for (i1 = 0; i1 < loop_ub_tmp; i1++) {
    r3[i1] = 0.0;
  }
  for (m_tmp = 0; m_tmp < i; m_tmp++) {
    r3[m_tmp + r3.size(0) * m_tmp] = v[m_tmp];
  }
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_value[r1[i1] + b_value.size(0) * r2[i]] = r3[i1 + loop_ub * i];
    }
  }
}

//
// Arguments    : const ::coder::array<double, 1U> &x
//                ::coder::array<double, 1U> &f
//                ::coder::array<double, 2U> &J
// Return Type  : void
//
void GIKProblem::residuals(const ::coder::array<double, 1U> &x,
                           ::coder::array<double, 1U> &f,
                           ::coder::array<double, 2U> &J)
{
  double d;
  int i;
  boolean_T flag;
  flag = true;
  d = NumVariables;
  i = 0;
  int exitg1;
  do {
    exitg1 = 0;
    if (i <= static_cast<int>(d) - 1) {
      if (x[i] != LastX[i]) {
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      flag = false;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  if (flag) {
    residualsInternal(x, f, J);
    i = x.size(0);
    LastX.set_size(x.size(0));
    for (int b_i{0}; b_i < i; b_i++) {
      LastX[b_i] = x[b_i];
    }
    i = f.size(0);
    LastF.set_size(f.size(0));
    for (int b_i{0}; b_i < i; b_i++) {
      LastF[b_i] = f[b_i];
    }
    LastJ.set_size(J.size(0), J.size(1));
    i = J.size(0) * J.size(1);
    for (int b_i{0}; b_i < i; b_i++) {
      LastJ[b_i] = J[b_i];
    }
  } else {
    f.set_size(LastF.size(0));
    i = LastF.size(0);
    for (int b_i{0}; b_i < i; b_i++) {
      f[b_i] = LastF[b_i];
    }
    J.set_size(LastJ.size(0), LastJ.size(1));
    i = LastJ.size(0) * LastJ.size(1);
    for (int b_i{0}; b_i < i; b_i++) {
      J[b_i] = LastJ[b_i];
    }
  }
}

//
// Arguments    : const ::coder::array<double, 1U> &x
//                ::coder::array<double, 1U> &f
// Return Type  : void
//
void GIKProblem::residuals(const ::coder::array<double, 1U> &x,
                           ::coder::array<double, 1U> &f)
{
  ::coder::array<double, 2U> J;
  double d;
  int i;
  boolean_T flag;
  flag = true;
  d = NumVariables;
  i = 0;
  int exitg1;
  do {
    exitg1 = 0;
    if (i <= static_cast<int>(d) - 1) {
      if (x[i] != LastX[i]) {
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      flag = false;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  if (flag) {
    residualsInternal(x, f, J);
    i = x.size(0);
    LastX.set_size(x.size(0));
    for (int b_i{0}; b_i < i; b_i++) {
      LastX[b_i] = x[b_i];
    }
    i = f.size(0);
    LastF.set_size(f.size(0));
    for (int b_i{0}; b_i < i; b_i++) {
      LastF[b_i] = f[b_i];
    }
    LastJ.set_size(J.size(0), J.size(1));
    i = J.size(0) * J.size(1);
    for (int b_i{0}; b_i < i; b_i++) {
      LastJ[b_i] = J[b_i];
    }
  } else {
    f.set_size(LastF.size(0));
    i = LastF.size(0);
    for (int b_i{0}; b_i < i; b_i++) {
      f[b_i] = LastF[b_i];
    }
  }
}

//
// Arguments    : const ::coder::array<double, 1U> &x
//                ::coder::array<double, 1U> &f
//                ::coder::array<double, 2U> &J
// Return Type  : void
//
void GIKProblem::residualsInternal(const ::coder::array<double, 1U> &x,
                                   ::coder::array<double, 1U> &f,
                                   ::coder::array<double, 2U> &J)
{
  DistanceBoundsConstraint *c_obj;
  JointPositionBounds *b_obj;
  PoseTarget *obj;
  ::coder::array<double, 2U> C;
  ::coder::array<double, 2U> Jrobot;
  ::coder::array<double, 2U> b_C;
  ::coder::array<double, 2U> b_Jrobot;
  ::coder::array<double, 2U> cols;
  ::coder::array<double, 2U> r3;
  ::coder::array<double, 2U> rows;
  ::coder::array<double, 1U> q;
  ::coder::array<double, 1U> slacks;
  ::coder::array<int, 2U> r1;
  ::coder::array<int, 1U> b_r;
  ::coder::array<int, 1U> r10;
  ::coder::array<int, 1U> r4;
  ::coder::array<int, 1U> r5;
  ::coder::array<int, 1U> r6;
  ::coder::array<int, 1U> r7;
  ::coder::array<int, 1U> r9;
  ::coder::array<signed char, 2U> r2;
  cell_wrap_63 JCell[3];
  double T_data[16];
  double JTwist[12];
  double A[6];
  double r[3];
  double gCell_f1[2];
  double d;
  double s;
  double t;
  double varargin_1;
  int T_size[2];
  int b_loop_ub;
  int boffset;
  int c_loop_ub;
  int coffset;
  int loop_ub;
  int m_tmp_tmp;
  int ndx_tmp_idx_0;
  d = NumPositions;
  if (d < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(d);
  }
  q.set_size(loop_ub);
  for (boffset = 0; boffset < loop_ub; boffset++) {
    q[boffset] = x[boffset];
  }
  obj = Constraints.f1;
  obj->Tree->efficientFKAndJacobianForIK(q, obj->EndEffectorIndex,
                                         obj->ReferenceBodyIndex, T_data,
                                         T_size, Jrobot);
  obj->evaluateFromTransform(T_data, T_size, gCell_f1, JTwist);
  b_obj = Constraints.f2;
  varargin_1 = b_obj->NumElements;
  if (varargin_1 < 0.0) {
    t = 0.0;
    m_tmp_tmp = 0;
  } else {
    t = varargin_1;
    m_tmp_tmp = static_cast<int>(varargin_1);
  }
  JCell[1].f1.set_size(m_tmp_tmp, m_tmp_tmp);
  b_loop_ub = static_cast<int>(t) * static_cast<int>(t);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    JCell[1].f1[boffset] = 0.0;
  }
  if (static_cast<int>(t) > 0) {
    for (int k{0}; k < m_tmp_tmp; k++) {
      JCell[1].f1[k + JCell[1].f1.size(0) * k] = 1.0;
    }
  }
  c_obj = Constraints.f3;
  c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
                                           c_obj->ReferenceBodyIndex, T_data,
                                           T_size, b_Jrobot);
  boffset = T_size[0] * (T_size[1] - 1);
  d = T_data[boffset];
  r[0] = d;
  varargin_1 = d * d;
  d = T_data[boffset + 1];
  r[1] = d;
  varargin_1 += d * d;
  d = T_data[boffset + 2];
  r[2] = d;
  varargin_1 += d * d;
  varargin_1 = std::sqrt(varargin_1 + 2.2204460492503131E-16);
  f.set_size(static_cast<int>(NumResiduals));
  b_loop_ub = static_cast<int>(NumResiduals);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    f[boffset] = 0.0;
  }
  ndx_tmp_idx_0 = static_cast<int>(NumResiduals);
  J.set_size(ndx_tmp_idx_0, static_cast<int>(NumVariables));
  b_loop_ub = static_cast<int>(NumResiduals) * static_cast<int>(NumVariables);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    J[boffset] = 0.0;
  }
  b_loop_ub = ResidualIndices[0].f1.size(1);
  rows.set_size(1, b_loop_ub);
  c_loop_ub = ResidualIndices[0].f1.size(1);
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    rows[boffset] = ResidualIndices[0].f1[boffset];
  }
  c_loop_ub = SlackIndices[0].f1.size(1);
  cols.set_size(1, c_loop_ub);
  m_tmp_tmp = SlackIndices[0].f1.size(1);
  for (boffset = 0; boffset < m_tmp_tmp; boffset++) {
    cols[boffset] = SlackIndices[0].f1[boffset];
  }
  t = NumPositions;
  q.set_size(b_loop_ub);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    q[boffset] = rows[boffset];
  }
  b_r.set_size(b_loop_ub);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    b_r[boffset] = static_cast<int>(q[boffset]) - 1;
  }
  m_tmp_tmp = Jrobot.size(1);
  C.set_size(2, Jrobot.size(1));
  for (int j{0}; j < m_tmp_tmp; j++) {
    coffset = j << 1;
    boffset = j * 6;
    for (int i{0}; i < 2; i++) {
      s = 0.0;
      for (int k{0}; k < 6; k++) {
        s += JTwist[(k << 1) + i] * Jrobot[boffset + k];
      }
      C[coffset + i] = s;
    }
  }
  if (t < 1.0) {
    m_tmp_tmp = 0;
  } else {
    m_tmp_tmp = static_cast<int>(t);
  }
  for (boffset = 0; boffset < m_tmp_tmp; boffset++) {
    for (coffset = 0; coffset < b_loop_ub; coffset++) {
      J[b_r[coffset] + J.size(0) * boffset] = C[coffset + b_loop_ub * boffset];
    }
  }
  slacks.set_size(c_loop_ub);
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    slacks[boffset] = x[static_cast<int>(cols[boffset]) - 1];
  }
  r1.set_size(1, b_loop_ub);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    r1[boffset] = static_cast<int>(rows[boffset]) +
                  ndx_tmp_idx_0 * (static_cast<int>(cols[boffset]) - 1);
  }
  r2.set_size(EqualityFlags[0].f1.size(0), EqualityFlags[0].f1.size(1));
  c_loop_ub = EqualityFlags[0].f1.size(0) * EqualityFlags[0].f1.size(1);
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    r2[boffset] = static_cast<signed char>(-!EqualityFlags[0].f1[boffset]);
  }
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    J[r1[boffset] - 1] = r2[boffset];
  }
  r3.set_size(Constraints.f1->BoundsInternal.size(0), 2);
  c_loop_ub = Constraints.f1->BoundsInternal.size(0) << 1;
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    r3[boffset] = Constraints.f1->BoundsInternal[boffset];
  }
  coffset = EqualityFlags[0].f1.size(0) * EqualityFlags[0].f1.size(1);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[0].f1[i]) {
      m_tmp_tmp++;
    }
  }
  r4.set_size(m_tmp_tmp);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[0].f1[i]) {
      r4[m_tmp_tmp] = i;
      m_tmp_tmp++;
    }
  }
  coffset = EqualityFlags[0].f1.size(0) * EqualityFlags[0].f1.size(1);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[0].f1[i]) {
      m_tmp_tmp++;
    }
  }
  r5.set_size(m_tmp_tmp);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[0].f1[i]) {
      r5[m_tmp_tmp] = i;
      m_tmp_tmp++;
    }
  }
  c_loop_ub = r5.size(0);
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    slacks[r4[boffset]] = r3[r5[boffset]];
  }
  b_r.set_size(b_loop_ub);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    b_r[boffset] = static_cast<int>(q[boffset]) - 1;
  }
  if (slacks.size(0) == 2) {
    for (boffset = 0; boffset < b_loop_ub; boffset++) {
      f[b_r[boffset]] = gCell_f1[boffset] - slacks[boffset];
    }
  } else {
    binary_expand_op_2(f, b_r, gCell_f1, slacks);
  }
  b_loop_ub = ResidualIndices[1].f1.size(1);
  rows.set_size(1, b_loop_ub);
  c_loop_ub = ResidualIndices[1].f1.size(1);
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    rows[boffset] = ResidualIndices[1].f1[boffset];
  }
  c_loop_ub = SlackIndices[1].f1.size(1);
  cols.set_size(1, c_loop_ub);
  m_tmp_tmp = SlackIndices[1].f1.size(1);
  for (boffset = 0; boffset < m_tmp_tmp; boffset++) {
    cols[boffset] = SlackIndices[1].f1[boffset];
  }
  t = NumPositions;
  q.set_size(b_loop_ub);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    q[boffset] = rows[boffset];
  }
  b_r.set_size(b_loop_ub);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    b_r[boffset] = static_cast<int>(q[boffset]) - 1;
  }
  if (t < 1.0) {
    m_tmp_tmp = 0;
  } else {
    m_tmp_tmp = static_cast<int>(t);
  }
  for (boffset = 0; boffset < m_tmp_tmp; boffset++) {
    for (coffset = 0; coffset < b_loop_ub; coffset++) {
      J[b_r[coffset] + J.size(0) * boffset] =
          JCell[1].f1[coffset + b_loop_ub * boffset];
    }
  }
  slacks.set_size(c_loop_ub);
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    slacks[boffset] = x[static_cast<int>(cols[boffset]) - 1];
  }
  r1.set_size(1, b_loop_ub);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    r1[boffset] = static_cast<int>(rows[boffset]) +
                  ndx_tmp_idx_0 * (static_cast<int>(cols[boffset]) - 1);
  }
  r2.set_size(EqualityFlags[1].f1.size(0), EqualityFlags[1].f1.size(1));
  c_loop_ub = EqualityFlags[1].f1.size(0) * EqualityFlags[1].f1.size(1);
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    r2[boffset] = static_cast<signed char>(-!EqualityFlags[1].f1[boffset]);
  }
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    J[r1[boffset] - 1] = r2[boffset];
  }
  r3.set_size(Constraints.f2->BoundsInternal.size(0), 2);
  c_loop_ub = Constraints.f2->BoundsInternal.size(0) << 1;
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    r3[boffset] = Constraints.f2->BoundsInternal[boffset];
  }
  coffset = EqualityFlags[1].f1.size(0) * EqualityFlags[1].f1.size(1);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[1].f1[i]) {
      m_tmp_tmp++;
    }
  }
  r6.set_size(m_tmp_tmp);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[1].f1[i]) {
      r6[m_tmp_tmp] = i;
      m_tmp_tmp++;
    }
  }
  coffset = EqualityFlags[1].f1.size(0) * EqualityFlags[1].f1.size(1);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[1].f1[i]) {
      m_tmp_tmp++;
    }
  }
  r7.set_size(m_tmp_tmp);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[1].f1[i]) {
      r7[m_tmp_tmp] = i;
      m_tmp_tmp++;
    }
  }
  c_loop_ub = r7.size(0);
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    slacks[r6[boffset]] = r3[r7[boffset]];
  }
  b_r.set_size(b_loop_ub);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    b_r[boffset] = static_cast<int>(q[boffset]) - 1;
  }
  if (loop_ub == slacks.size(0)) {
    for (boffset = 0; boffset < b_loop_ub; boffset++) {
      f[b_r[boffset]] = x[boffset] - slacks[boffset];
    }
  } else {
    binary_expand_op_1(f, b_r, x, loop_ub - 1, slacks);
  }
  loop_ub = ResidualIndices[2].f1.size(1);
  rows.set_size(1, loop_ub);
  b_loop_ub = ResidualIndices[2].f1.size(1);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    rows[boffset] = ResidualIndices[2].f1[boffset];
  }
  b_loop_ub = SlackIndices[2].f1.size(1);
  cols.set_size(1, b_loop_ub);
  c_loop_ub = SlackIndices[2].f1.size(1);
  for (boffset = 0; boffset < c_loop_ub; boffset++) {
    cols[boffset] = SlackIndices[2].f1[boffset];
  }
  t = NumPositions;
  q.set_size(loop_ub);
  for (boffset = 0; boffset < loop_ub; boffset++) {
    q[boffset] = rows[boffset];
  }
  b_r.set_size(loop_ub);
  for (boffset = 0; boffset < loop_ub; boffset++) {
    b_r[boffset] = static_cast<int>(q[boffset]) - 1;
  }
  __m128d r8;
  _mm_storeu_pd(&A[0], _mm_set1_pd(0.0));
  r8 = _mm_loadu_pd(&r[0]);
  _mm_storeu_pd(&A[3], _mm_div_pd(r8, _mm_set1_pd(varargin_1)));
  A[2] = 0.0;
  A[5] = d / varargin_1;
  m_tmp_tmp = b_Jrobot.size(1);
  b_C.set_size(1, b_Jrobot.size(1));
  for (int j{0}; j < m_tmp_tmp; j++) {
    boffset = j * 6;
    s = 0.0;
    for (int k{0}; k < 6; k++) {
      s += A[k] * b_Jrobot[boffset + k];
    }
    b_C[j] = s;
  }
  if (t < 1.0) {
    m_tmp_tmp = 0;
  } else {
    m_tmp_tmp = static_cast<int>(t);
  }
  for (boffset = 0; boffset < m_tmp_tmp; boffset++) {
    for (coffset = 0; coffset < loop_ub; coffset++) {
      J[b_r[coffset] + J.size(0) * boffset] = b_C[coffset + loop_ub * boffset];
    }
  }
  slacks.set_size(b_loop_ub);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    slacks[boffset] = x[static_cast<int>(cols[boffset]) - 1];
  }
  r1.set_size(1, loop_ub);
  for (boffset = 0; boffset < loop_ub; boffset++) {
    r1[boffset] = static_cast<int>(rows[boffset]) +
                  ndx_tmp_idx_0 * (static_cast<int>(cols[boffset]) - 1);
  }
  r2.set_size(EqualityFlags[2].f1.size(0), EqualityFlags[2].f1.size(1));
  b_loop_ub = EqualityFlags[2].f1.size(0) * EqualityFlags[2].f1.size(1);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    r2[boffset] = static_cast<signed char>(-!EqualityFlags[2].f1[boffset]);
  }
  for (boffset = 0; boffset < loop_ub; boffset++) {
    J[r1[boffset] - 1] = r2[boffset];
  }
  r3.set_size(Constraints.f3->BoundsInternal.size(0), 2);
  b_loop_ub = Constraints.f3->BoundsInternal.size(0) << 1;
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    r3[boffset] = Constraints.f3->BoundsInternal[boffset];
  }
  coffset = EqualityFlags[2].f1.size(0) * EqualityFlags[2].f1.size(1);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[2].f1[i]) {
      m_tmp_tmp++;
    }
  }
  r9.set_size(m_tmp_tmp);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[2].f1[i]) {
      r9[m_tmp_tmp] = i;
      m_tmp_tmp++;
    }
  }
  coffset = EqualityFlags[2].f1.size(0) * EqualityFlags[2].f1.size(1);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[2].f1[i]) {
      m_tmp_tmp++;
    }
  }
  r10.set_size(m_tmp_tmp);
  m_tmp_tmp = 0;
  for (int i{0}; i < coffset; i++) {
    if (EqualityFlags[2].f1[i]) {
      r10[m_tmp_tmp] = i;
      m_tmp_tmp++;
    }
  }
  b_loop_ub = r10.size(0);
  for (boffset = 0; boffset < b_loop_ub; boffset++) {
    slacks[r9[boffset]] = r3[r10[boffset]];
  }
  b_r.set_size(loop_ub);
  for (boffset = 0; boffset < loop_ub; boffset++) {
    b_r[boffset] = static_cast<int>(q[boffset]) - 1;
  }
  for (boffset = 0; boffset < loop_ub; boffset++) {
    f[b_r[boffset]] = varargin_1 - slacks[boffset];
  }
}

//
// Arguments    : boolean_T b_value
// Return Type  : void
//
void GIKProblem::set_EnforceJointLimits(boolean_T b_value)
{
  ::coder::array<double, 2U> A;
  ::coder::array<double, 2U> r;
  ::coder::array<double, 2U> r1;
  ::coder::array<double, 1U> b;
  EnforceJointLimitsInternal = b_value;
  if (EnforceJointLimitsInternal) {
    double d;
    int b_loop_ub;
    int c_loop_ub;
    int loop_ub;
    loop_ub = DesignVariableBoundsInternal.size(0);
    r.set_size(loop_ub, 2);
    b_loop_ub = DesignVariableBoundsInternal.size(0) << 1;
    for (int i{0}; i < b_loop_ub; i++) {
      r[i] = DesignVariableBoundsInternal[i];
    }
    d = NumPositions;
    Tree->get_JointPositionLimits(r1);
    if (d < 1.0) {
      b_loop_ub = 0;
    } else {
      b_loop_ub = static_cast<int>(d);
    }
    for (int i{0}; i < 2; i++) {
      for (c_loop_ub = 0; c_loop_ub < b_loop_ub; c_loop_ub++) {
        r[c_loop_ub + r.size(0) * i] = r1[c_loop_ub + r1.size(0) * i];
      }
    }
    DesignVariableBoundsInternal.set_size(loop_ub, 2);
    c_loop_ub = r.size(0) << 1;
    for (int i{0}; i < c_loop_ub; i++) {
      DesignVariableBoundsInternal[i] = r[i];
    }
    b_loop_ub = 2 * r.size(0);
    A.set_size(b_loop_ub, loop_ub);
    c_loop_ub = b_loop_ub * r.size(0);
    for (int i{0}; i < c_loop_ub; i++) {
      A[i] = 0.0;
    }
    b.set_size(b_loop_ub);
    for (int i{0}; i < b_loop_ub; i++) {
      b[i] = 0.0;
    }
    for (int i{0}; i < loop_ub; i++) {
      c_loop_ub = static_cast<int>(static_cast<unsigned int>(i + 1) << 1);
      A[(c_loop_ub + A.size(0) * i) - 2] = -1.0;
      A[(c_loop_ub + A.size(0) * i) - 1] = 1.0;
      b[c_loop_ub - 2] = -r[i];
      b[c_loop_ub - 1] = r[i + r.size(0)];
    }
    ConstraintMatrixInternal.set_size(loop_ub, b_loop_ub);
    for (int i{0}; i < b_loop_ub; i++) {
      for (c_loop_ub = 0; c_loop_ub < loop_ub; c_loop_ub++) {
        ConstraintMatrixInternal[c_loop_ub +
                                 ConstraintMatrixInternal.size(0) * i] =
            A[i + A.size(0) * c_loop_ub];
      }
    }
    ConstraintBoundInternal.set_size(b_loop_ub);
    for (int i{0}; i < b_loop_ub; i++) {
      ConstraintBoundInternal[i] = b[i];
    }
  } else {
    int b_loop_ub;
    int c_loop_ub;
    int loop_ub;
    loop_ub = DesignVariableBoundsInternal.size(0);
    r.set_size(loop_ub, 2);
    b_loop_ub = DesignVariableBoundsInternal.size(0) << 1;
    for (int i{0}; i < b_loop_ub; i++) {
      r[i] = DesignVariableBoundsInternal[i];
    }
    b_loop_ub = static_cast<int>(NumPositions);
    c_loop_ub = static_cast<int>(NumPositions);
    for (int i{0}; i < b_loop_ub; i++) {
      r[i] = rtMinusInf;
    }
    for (int i{0}; i < c_loop_ub; i++) {
      r[i + r.size(0)] = rtInf;
    }
    DesignVariableBoundsInternal.set_size(loop_ub, 2);
    c_loop_ub = r.size(0) << 1;
    for (int i{0}; i < c_loop_ub; i++) {
      DesignVariableBoundsInternal[i] = r[i];
    }
    b_loop_ub = 2 * r.size(0);
    A.set_size(b_loop_ub, loop_ub);
    c_loop_ub = b_loop_ub * r.size(0);
    for (int i{0}; i < c_loop_ub; i++) {
      A[i] = 0.0;
    }
    b.set_size(b_loop_ub);
    for (int i{0}; i < b_loop_ub; i++) {
      b[i] = 0.0;
    }
    for (int i{0}; i < loop_ub; i++) {
      c_loop_ub = static_cast<int>(static_cast<unsigned int>(i + 1) << 1);
      A[(c_loop_ub + A.size(0) * i) - 2] = -1.0;
      A[(c_loop_ub + A.size(0) * i) - 1] = 1.0;
      b[c_loop_ub - 2] = -r[i];
      b[c_loop_ub - 1] = r[i + r.size(0)];
    }
    ConstraintMatrixInternal.set_size(loop_ub, b_loop_ub);
    for (int i{0}; i < b_loop_ub; i++) {
      for (c_loop_ub = 0; c_loop_ub < loop_ub; c_loop_ub++) {
        ConstraintMatrixInternal[c_loop_ub +
                                 ConstraintMatrixInternal.size(0) * i] =
            A[i + A.size(0) * c_loop_ub];
      }
    }
    ConstraintBoundInternal.set_size(b_loop_ub);
    for (int i{0}; i < b_loop_ub; i++) {
      ConstraintBoundInternal[i] = b[i];
    }
  }
}

//
// Arguments    : const constraintPoseTarget &varargin_1
//                const constraintJointBounds &varargin_2
//                const constraintDistanceBounds &varargin_3
// Return Type  : void
//
void GIKProblem::update(const constraintPoseTarget &varargin_1,
                        const constraintJointBounds &varargin_2,
                        const constraintDistanceBounds &varargin_3)
{
  CharacterVector e_obj;
  DistanceBoundsConstraint *c_obj;
  JointPositionBounds *b_obj;
  PoseTarget *obj;
  RigidBody *g_obj;
  RigidBodyTree *d_obj;
  ::coder::array<double, 2U> r1;
  ::coder::array<double, 1U> f_obj;
  ::coder::array<double, 1U> r;
  int a_size[2];
  int exitg1;
  int i;
  int kstr;
  int loop_ub;
  char a_data[200];
  boolean_T b_bool;
  obj = Constraints.f1;
  obj->Weights.set_size(1, 2);
  obj->Weights[0] = varargin_1.Weights[0];
  obj->Weights[1] = varargin_1.Weights[1];
  obj->get_EndEffector(a_data, a_size);
  b_bool = false;
  if (a_size[1] == 17) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 17) {
        if (a_data[kstr] != varargin_1.EndEffector[kstr]) {
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
    obj->EndEffectorIndex =
        obj->Tree->validateInputBodyName(varargin_1.EndEffector);
  }
  obj->get_ReferenceBody(a_data, a_size);
  if (static_cast<unsigned char>(a_size[1]) != 0) {
    obj->ReferenceBodyIndex = 0.0;
  }
  for (i = 0; i < 16; i++) {
    obj->TargetTransform[i] = varargin_1.TargetTransform[i];
  }
  obj->BoundsInternal[obj->BoundsInternal.size(0)] =
      varargin_1.OrientationTolerance;
  obj->BoundsInternal[obj->BoundsInternal.size(0) + 1] =
      varargin_1.PositionTolerance;
  b_obj = Constraints.f2;
  b_obj->Weights.set_size(1, b_obj->Weights.size(1));
  loop_ub = varargin_2.WeightsInternal.size(1);
  b_obj->Weights.set_size(b_obj->Weights.size(0), loop_ub);
  for (i = 0; i < loop_ub; i++) {
    b_obj->Weights[i] = varargin_2.WeightsInternal[i];
  }
  b_obj->BoundsInternal.set_size(varargin_2.BoundsInternal.size(0), 2);
  kstr = varargin_2.BoundsInternal.size(0) << 1;
  for (i = 0; i < kstr; i++) {
    b_obj->BoundsInternal[i] = varargin_2.BoundsInternal[i];
  }
  c_obj = Constraints.f3;
  c_obj->Weights.set_size(1, 1);
  c_obj->Weights[0] = varargin_3.Weights;
  c_obj->get_EndEffector(a_data, a_size);
  b_bool = false;
  if (a_size[1] == 17) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 17) {
        if (a_data[kstr] != varargin_3.EndEffector[kstr]) {
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
    c_obj->EndEffectorIndex =
        c_obj->Tree->validateInputBodyName(varargin_3.EndEffector);
  }
  c_obj->get_ReferenceBody(a_data, a_size);
  if (!::gik9dof::coder::internal::b_strcmp(a_data, a_size,
                                            varargin_3.ReferenceBody)) {
    if ((varargin_3.ReferenceBody.size(0) == 0) ||
        (varargin_3.ReferenceBody.size(1) == 0)) {
      c_obj->ReferenceBodyIndex = 0.0;
    } else {
      d_obj = c_obj->Tree;
      kstr = -1;
      e_obj = d_obj->Base.NameInternal;
      if (e_obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(e_obj.Length);
      }
      a_size[0] = 1;
      a_size[1] = loop_ub;
      if (loop_ub - 1 >= 0) {
        ::std::copy(&e_obj.Vector[0], &e_obj.Vector[loop_ub], &a_data[0]);
      }
      if (::gik9dof::coder::internal::b_strcmp(a_data, a_size,
                                               varargin_3.ReferenceBody)) {
        kstr = 0;
      } else {
        double d;
        boolean_T exitg2;
        d = d_obj->NumBodies;
        i = 0;
        exitg2 = false;
        while ((!exitg2) && (i <= static_cast<int>(d) - 1)) {
          g_obj = d_obj->Bodies[i];
          e_obj = g_obj->NameInternal;
          if (e_obj.Length < 1.0) {
            loop_ub = 0;
          } else {
            loop_ub = static_cast<int>(e_obj.Length);
          }
          a_size[0] = 1;
          a_size[1] = loop_ub;
          if (loop_ub - 1 >= 0) {
            ::std::copy(&e_obj.Vector[0], &e_obj.Vector[loop_ub], &a_data[0]);
          }
          if (::gik9dof::coder::internal::b_strcmp(a_data, a_size,
                                                   varargin_3.ReferenceBody)) {
            kstr = i + 1;
            exitg2 = true;
          } else {
            i++;
          }
        }
      }
      c_obj->ReferenceBodyIndex = kstr;
    }
  }
  c_obj->BoundsInternal.set_size(1, 2);
  c_obj->BoundsInternal[0] = varargin_3.Bounds[0];
  c_obj->BoundsInternal[1] = varargin_3.Bounds[1];
  updateDesignVariableBounds();
  kstr = static_cast<int>(NumVariables);
  LastX.set_size(kstr);
  for (i = 0; i < kstr; i++) {
    LastX[i] = 0.0;
  }
  f_obj.set_size(LastX.size(0));
  loop_ub = LastX.size(0) - 1;
  for (i = 0; i <= loop_ub; i++) {
    f_obj[i] = LastX[i];
  }
  residualsInternal(f_obj, r, r1);
  loop_ub = r.size(0);
  LastF.set_size(r.size(0));
  for (i = 0; i < loop_ub; i++) {
    LastF[i] = r[i];
  }
  LastJ.set_size(r1.size(0), r1.size(1));
  kstr = r1.size(0) * r1.size(1);
  for (i = 0; i < kstr; i++) {
    LastJ[i] = r1[i];
  }
}

//
// Arguments    : void
// Return Type  : void
//
void GIKProblem::updateDesignVariableBounds()
{
  __m128d r;
  __m128d r1;
  ::coder::array<double, 2U> A;
  ::coder::array<double, 2U> bounds;
  ::coder::array<double, 1U> y;
  ::coder::array<int, 1U> r2;
  ::coder::array<int, 1U> r3;
  ::coder::array<int, 1U> r4;
  int end;
  int i;
  int loop_ub;
  int nx;
  int s;
  i = Constraints.f1->BoundsInternal.size(0);
  bounds.set_size(i, 2);
  loop_ub = Constraints.f1->BoundsInternal.size(0) << 1;
  for (nx = 0; nx < loop_ub; nx++) {
    bounds[nx] = Constraints.f1->BoundsInternal[nx];
  }
  ConstraintBoundInternal.set_size(i);
  if (bounds.size(0) != 0) {
    nx = (bounds.size(0) / 2) << 1;
    loop_ub = nx - 2;
    for (s = 0; s <= loop_ub; s += 2) {
      r = _mm_loadu_pd(&bounds[s + i]);
      r1 = _mm_loadu_pd(&bounds[s]);
      _mm_storeu_pd(&ConstraintBoundInternal[s], _mm_sub_pd(r, r1));
    }
    for (s = nx; s < i; s++) {
      ConstraintBoundInternal[s] = bounds[s + i] - bounds[s];
    }
  }
  nx = ConstraintBoundInternal.size(0);
  end = ConstraintBoundInternal.size(0);
  y.set_size(end);
  for (loop_ub = 0; loop_ub < nx; loop_ub++) {
    y[loop_ub] = std::abs(ConstraintBoundInternal[loop_ub]);
  }
  nx = 0;
  for (int b_i{0}; b_i < end; b_i++) {
    if (y[b_i] < 2.2204460492503131E-16) {
      nx++;
    }
  }
  r2.set_size(nx);
  nx = 0;
  for (int b_i{0}; b_i < end; b_i++) {
    if (y[b_i] < 2.2204460492503131E-16) {
      r2[nx] = b_i;
      nx++;
    }
  }
  loop_ub = r2.size(0);
  for (i = 0; i < loop_ub; i++) {
    bounds[r2[i]] = rtMinusInf;
    bounds[r2[i] + bounds.size(0)] = rtInf;
  }
  loop_ub = SlackIndices[0].f1.size(1);
  r2.set_size(loop_ub);
  s = SlackIndices[0].f1.size(1);
  for (i = 0; i < s; i++) {
    r2[i] = static_cast<int>(SlackIndices[0].f1[i]) - 1;
  }
  for (i = 0; i < 2; i++) {
    for (nx = 0; nx < loop_ub; nx++) {
      DesignVariableBoundsInternal[r2[nx] +
                                   DesignVariableBoundsInternal.size(0) * i] =
          bounds[nx + bounds.size(0) * i];
    }
  }
  nx = 2 * DesignVariableBoundsInternal.size(0);
  ConstraintBoundInternal.set_size(nx);
  for (i = 0; i < nx; i++) {
    ConstraintBoundInternal[i] = 0.0;
  }
  i = DesignVariableBoundsInternal.size(0);
  for (int b_i{0}; b_i < i; b_i++) {
    nx = static_cast<int>(static_cast<unsigned int>(b_i + 1) << 1);
    ConstraintBoundInternal[nx - 2] = -DesignVariableBoundsInternal[b_i];
    ConstraintBoundInternal[nx - 1] =
        DesignVariableBoundsInternal[b_i +
                                     DesignVariableBoundsInternal.size(0)];
  }
  EqualityFlags[0].f1.set_size(end, 1);
  for (i = 0; i < end; i++) {
    EqualityFlags[0].f1[i] = (y[i] < 2.2204460492503131E-16);
  }
  i = Constraints.f2->BoundsInternal.size(0);
  bounds.set_size(i, 2);
  loop_ub = Constraints.f2->BoundsInternal.size(0) << 1;
  for (nx = 0; nx < loop_ub; nx++) {
    bounds[nx] = Constraints.f2->BoundsInternal[nx];
  }
  ConstraintBoundInternal.set_size(i);
  if (bounds.size(0) != 0) {
    nx = (bounds.size(0) / 2) << 1;
    loop_ub = nx - 2;
    for (s = 0; s <= loop_ub; s += 2) {
      r = _mm_loadu_pd(&bounds[s + i]);
      r1 = _mm_loadu_pd(&bounds[s]);
      _mm_storeu_pd(&ConstraintBoundInternal[s], _mm_sub_pd(r, r1));
    }
    for (s = nx; s < i; s++) {
      ConstraintBoundInternal[s] = bounds[s + i] - bounds[s];
    }
  }
  nx = ConstraintBoundInternal.size(0);
  end = ConstraintBoundInternal.size(0);
  y.set_size(end);
  for (loop_ub = 0; loop_ub < nx; loop_ub++) {
    y[loop_ub] = std::abs(ConstraintBoundInternal[loop_ub]);
  }
  nx = 0;
  for (int b_i{0}; b_i < end; b_i++) {
    if (y[b_i] < 2.2204460492503131E-16) {
      nx++;
    }
  }
  r3.set_size(nx);
  nx = 0;
  for (int b_i{0}; b_i < end; b_i++) {
    if (y[b_i] < 2.2204460492503131E-16) {
      r3[nx] = b_i;
      nx++;
    }
  }
  loop_ub = r3.size(0);
  for (i = 0; i < loop_ub; i++) {
    bounds[r3[i]] = rtMinusInf;
    bounds[r3[i] + bounds.size(0)] = rtInf;
  }
  loop_ub = SlackIndices[1].f1.size(1);
  r2.set_size(loop_ub);
  s = SlackIndices[1].f1.size(1);
  for (i = 0; i < s; i++) {
    r2[i] = static_cast<int>(SlackIndices[1].f1[i]) - 1;
  }
  for (i = 0; i < 2; i++) {
    for (nx = 0; nx < loop_ub; nx++) {
      DesignVariableBoundsInternal[r2[nx] +
                                   DesignVariableBoundsInternal.size(0) * i] =
          bounds[nx + bounds.size(0) * i];
    }
  }
  nx = 2 * DesignVariableBoundsInternal.size(0);
  ConstraintBoundInternal.set_size(nx);
  for (i = 0; i < nx; i++) {
    ConstraintBoundInternal[i] = 0.0;
  }
  i = DesignVariableBoundsInternal.size(0);
  for (int b_i{0}; b_i < i; b_i++) {
    nx = static_cast<int>(static_cast<unsigned int>(b_i + 1) << 1);
    ConstraintBoundInternal[nx - 2] = -DesignVariableBoundsInternal[b_i];
    ConstraintBoundInternal[nx - 1] =
        DesignVariableBoundsInternal[b_i +
                                     DesignVariableBoundsInternal.size(0)];
  }
  EqualityFlags[1].f1.set_size(end, 1);
  for (i = 0; i < end; i++) {
    EqualityFlags[1].f1[i] = (y[i] < 2.2204460492503131E-16);
  }
  i = Constraints.f3->BoundsInternal.size(0);
  bounds.set_size(i, 2);
  loop_ub = Constraints.f3->BoundsInternal.size(0) << 1;
  for (nx = 0; nx < loop_ub; nx++) {
    bounds[nx] = Constraints.f3->BoundsInternal[nx];
  }
  ConstraintBoundInternal.set_size(i);
  if (bounds.size(0) != 0) {
    nx = (bounds.size(0) / 2) << 1;
    loop_ub = nx - 2;
    for (s = 0; s <= loop_ub; s += 2) {
      r = _mm_loadu_pd(&bounds[s + i]);
      r1 = _mm_loadu_pd(&bounds[s]);
      _mm_storeu_pd(&ConstraintBoundInternal[s], _mm_sub_pd(r, r1));
    }
    for (s = nx; s < i; s++) {
      ConstraintBoundInternal[s] = bounds[s + i] - bounds[s];
    }
  }
  nx = ConstraintBoundInternal.size(0);
  end = ConstraintBoundInternal.size(0);
  y.set_size(end);
  for (loop_ub = 0; loop_ub < nx; loop_ub++) {
    y[loop_ub] = std::abs(ConstraintBoundInternal[loop_ub]);
  }
  nx = 0;
  for (int b_i{0}; b_i < end; b_i++) {
    if (y[b_i] < 2.2204460492503131E-16) {
      nx++;
    }
  }
  r4.set_size(nx);
  nx = 0;
  for (int b_i{0}; b_i < end; b_i++) {
    if (y[b_i] < 2.2204460492503131E-16) {
      r4[nx] = b_i;
      nx++;
    }
  }
  loop_ub = r4.size(0);
  for (i = 0; i < loop_ub; i++) {
    bounds[r4[i]] = rtMinusInf;
    bounds[r4[i] + bounds.size(0)] = rtInf;
  }
  loop_ub = SlackIndices[2].f1.size(1);
  r2.set_size(loop_ub);
  s = SlackIndices[2].f1.size(1);
  for (i = 0; i < s; i++) {
    r2[i] = static_cast<int>(SlackIndices[2].f1[i]) - 1;
  }
  for (i = 0; i < 2; i++) {
    for (nx = 0; nx < loop_ub; nx++) {
      DesignVariableBoundsInternal[r2[nx] +
                                   DesignVariableBoundsInternal.size(0) * i] =
          bounds[nx + bounds.size(0) * i];
    }
  }
  loop_ub = 2 * DesignVariableBoundsInternal.size(0);
  s = DesignVariableBoundsInternal.size(0);
  A.set_size(loop_ub, s);
  nx = 2 * DesignVariableBoundsInternal.size(0) *
       DesignVariableBoundsInternal.size(0);
  for (i = 0; i < nx; i++) {
    A[i] = 0.0;
  }
  nx = 2 * DesignVariableBoundsInternal.size(0);
  ConstraintBoundInternal.set_size(nx);
  for (i = 0; i < nx; i++) {
    ConstraintBoundInternal[i] = 0.0;
  }
  i = DesignVariableBoundsInternal.size(0);
  for (int b_i{0}; b_i < i; b_i++) {
    nx = static_cast<int>(static_cast<unsigned int>(b_i + 1) << 1);
    A[(nx + A.size(0) * b_i) - 2] = -1.0;
    A[(nx + A.size(0) * b_i) - 1] = 1.0;
    ConstraintBoundInternal[nx - 2] = -DesignVariableBoundsInternal[b_i];
    ConstraintBoundInternal[nx - 1] =
        DesignVariableBoundsInternal[b_i +
                                     DesignVariableBoundsInternal.size(0)];
  }
  ConstraintMatrixInternal.set_size(s, loop_ub);
  for (i = 0; i < loop_ub; i++) {
    for (nx = 0; nx < s; nx++) {
      ConstraintMatrixInternal[nx + ConstraintMatrixInternal.size(0) * i] =
          A[i + A.size(0) * nx];
    }
  }
  EqualityFlags[2].f1.set_size(end, 1);
  for (i = 0; i < end; i++) {
    EqualityFlags[2].f1[i] = (y[i] < 2.2204460492503131E-16);
  }
}

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

//
// File trailer for GIKProblem.cpp
//
// [EOF]
//
