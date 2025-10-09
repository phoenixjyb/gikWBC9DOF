//
// File: JointPositionBounds.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "JointPositionBounds.h"
#include "RigidBodyTree.h"
#include "ismember.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : void
// Return Type  : JointPositionBounds
//
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
JointPositionBounds::JointPositionBounds() = default;

//
// Arguments    : void
// Return Type  : void
//
JointPositionBounds::~JointPositionBounds() = default;

//
// Arguments    : ::coder::array<double, 2U> &b_value
// Return Type  : void
//
void JointPositionBounds::get_KinematicPath(
    ::coder::array<double, 2U> &b_value) const
{
  ::coder::array<double, 1U> b;
  ::coder::array<double, 1U> b_ii;
  ::coder::array<int, 1U> ii;
  ::coder::array<int, 1U> iwork;
  ::coder::array<boolean_T, 2U> x;
  ::coder::array<boolean_T, 1U> b_x;
  double xtmp;
  int b_i;
  int i;
  int i1;
  int i2;
  int j;
  int j2;
  int k;
  int pEnd;
  int qEnd;
  boolean_T exitg1;
  i = BoundsInternal.size(0);
  x.set_size(i, 2);
  i1 = BoundsInternal.size(0) << 1;
  for (b_i = 0; b_i < i1; b_i++) {
    x[b_i] = !std::isinf(BoundsInternal[b_i]);
  }
  b_x.set_size(i);
  for (b_i = 0; b_i < i; b_i++) {
    b_x[b_i] = false;
  }
  i2 = i;
  i1 = 0;
  for (j = 0; j < i; j++) {
    i1++;
    i2++;
    j2 = i1;
    exitg1 = false;
    while ((!exitg1) && ((i > 0) && (j2 <= i2))) {
      if (x[j2 - 1]) {
        b_x[j] = true;
        exitg1 = true;
      } else {
        j2 += i;
      }
    }
  }
  i1 = 0;
  ii.set_size(i);
  j2 = 0;
  exitg1 = false;
  while ((!exitg1) && (j2 <= i - 1)) {
    if (b_x[j2]) {
      i1++;
      ii[i1 - 1] = j2 + 1;
      if (i1 >= i) {
        exitg1 = true;
      } else {
        j2++;
      }
    } else {
      j2++;
    }
  }
  if (b_x.size(0) == 1) {
    if (i1 == 0) {
      ii.set_size(0);
    }
  } else {
    if (i1 < 1) {
      i1 = 0;
    }
    ii.set_size(i1);
  }
  i = ii.size(0);
  b_ii.set_size(ii.size(0));
  for (b_i = 0; b_i < i; b_i++) {
    b_ii[b_i] = ii[b_i];
  }
  isMember(b_ii, Tree->PositionDoFMap, b_x, ii);
  b_ii.set_size(ii.size(0) + 1);
  b_ii[0] = 0.0;
  i = ii.size(0);
  for (b_i = 0; b_i < i; b_i++) {
    b_ii[b_i + 1] = ii[b_i];
  }
  ::gik9dof::coder::internal::sort(b_ii);
  j2 = b_ii.size(0);
  i1 = b_ii.size(0) + 1;
  ii.set_size(b_ii.size(0));
  for (b_i = 0; b_i < j2; b_i++) {
    ii[b_i] = 0;
  }
  iwork.set_size(b_ii.size(0));
  b_i = b_ii.size(0) - 1;
  for (k = 1; k <= b_i; k += 2) {
    if ((b_ii[k - 1] <= b_ii[k]) || std::isnan(b_ii[k])) {
      ii[k - 1] = k;
      ii[k] = k + 1;
    } else {
      ii[k - 1] = k + 1;
      ii[k] = k;
    }
  }
  if ((b_ii.size(0) & 1) != 0) {
    ii[b_ii.size(0) - 1] = b_ii.size(0);
  }
  i = 2;
  while (i < i1 - 1) {
    i2 = i << 1;
    j = 1;
    for (pEnd = i + 1; pEnd < i1; pEnd = qEnd + i) {
      int kEnd;
      int p;
      int q;
      p = j;
      q = pEnd - 1;
      qEnd = j + i2;
      if (qEnd > i1) {
        qEnd = i1;
      }
      k = 0;
      kEnd = qEnd - j;
      while (k + 1 <= kEnd) {
        xtmp = b_ii[ii[q] - 1];
        b_i = ii[p - 1];
        if ((b_ii[b_i - 1] <= xtmp) || std::isnan(xtmp)) {
          iwork[k] = b_i;
          p++;
          if (p == pEnd) {
            while (q + 1 < qEnd) {
              k++;
              iwork[k] = ii[q];
              q++;
            }
          }
        } else {
          iwork[k] = ii[q];
          q++;
          if (q + 1 == qEnd) {
            while (p < pEnd) {
              k++;
              iwork[k] = ii[p - 1];
              p++;
            }
          }
        }
        k++;
      }
      for (k = 0; k < kEnd; k++) {
        ii[(j + k) - 1] = iwork[k];
      }
      j = qEnd;
    }
    i = i2;
  }
  b.set_size(b_ii.size(0));
  for (k = 0; k < j2; k++) {
    b[k] = b_ii[ii[k] - 1];
  }
  k = b_ii.size(0);
  while ((k >= 1) && std::isnan(b[k - 1])) {
    k--;
  }
  i1 = b_ii.size(0) - k;
  exitg1 = false;
  while ((!exitg1) && (k >= 1)) {
    xtmp = b[k - 1];
    if (std::isinf(xtmp) && (xtmp > 0.0)) {
      k--;
    } else {
      exitg1 = true;
    }
  }
  j2 = (b_ii.size(0) - k) - i1;
  i = -1;
  pEnd = 1;
  while (pEnd <= k) {
    xtmp = b[pEnd - 1];
    do {
      pEnd++;
    } while (!((pEnd > k) || (b[pEnd - 1] != xtmp)));
    i++;
    b[i] = xtmp;
  }
  if (j2 > 0) {
    i++;
    b[i] = b[k];
  }
  pEnd = k + j2;
  for (j = 0; j < i1; j++) {
    b[(i + j) + 1] = b[pEnd + j];
  }
  if (i1 - 1 >= 0) {
    i += i1;
  }
  if (i + 1 < 1) {
    b_i = -1;
  } else {
    b_i = i;
  }
  b.set_size(b_i + 1);
  b_value.set_size(1, b_i + 1);
  i = b_i + 1;
  for (j2 = 0; j2 < i; j2++) {
    b_value[j2] = b[j2];
  }
  i1 = (b_i + 1) >> 1;
  for (i = 0; i < i1; i++) {
    j2 = b_i - i;
    xtmp = b_value[i];
    b_value[i] = b_value[j2];
    b_value[j2] = xtmp;
  }
}

//
// Arguments    : b_RigidBodyTree *tree
// Return Type  : JointPositionBounds *
//
JointPositionBounds *JointPositionBounds::init(b_RigidBodyTree *tree)
{
  JointPositionBounds *obj;
  double numElements;
  int obj_idx_0;
  obj = this;
  numElements = tree->PositionNumber;
  obj->Tree = tree;
  obj->NumElements = numElements;
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
  obj->Tree->get_JointPositionLimits(obj->BoundsInternal);
  obj_idx_0 = static_cast<int>(obj->NumElements);
  obj->Weights.set_size(1, obj_idx_0);
  for (int i{0}; i < obj_idx_0; i++) {
    obj->Weights[i] = 1.0;
  }
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
} // namespace gik9dof

//
// File trailer for JointPositionBounds.cpp
//
// [EOF]
//
