//
// File: JointPositionBounds.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
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
  ::coder::array<bool, 2U> x;
  ::coder::array<bool, 1U> b_x;
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
  bool exitg1;
  pEnd = BoundsInternal.size(0);
  x.set_size(pEnd, 2);
  i1 = BoundsInternal.size(0) << 1;
  for (i = 0; i < i1; i++) {
    x[i] = !std::isinf(BoundsInternal[i]);
  }
  b_x.set_size(pEnd);
  for (i = 0; i < pEnd; i++) {
    b_x[i] = false;
  }
  i2 = pEnd;
  i1 = 1;
  for (j = 0; j < pEnd; j++) {
    j2 = i1;
    i1++;
    i2++;
    b_i = j2;
    exitg1 = false;
    while ((!exitg1) && ((pEnd > 0) && (b_i <= i2))) {
      if (x[b_i - 1]) {
        b_x[j2 - 1] = true;
        exitg1 = true;
      } else {
        b_i += pEnd;
      }
    }
  }
  i1 = 0;
  ii.set_size(pEnd);
  j2 = 0;
  exitg1 = false;
  while ((!exitg1) && (j2 <= pEnd - 1)) {
    if (b_x[j2]) {
      i1++;
      ii[i1 - 1] = j2 + 1;
      if (i1 >= pEnd) {
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
  pEnd = ii.size(0);
  b_ii.set_size(ii.size(0));
  for (i = 0; i < pEnd; i++) {
    b_ii[i] = ii[i];
  }
  isMember(b_ii, Tree->PositionDoFMap, b_x, ii);
  b_ii.set_size(ii.size(0) + 1);
  b_ii[0] = 0.0;
  pEnd = ii.size(0);
  for (i = 0; i < pEnd; i++) {
    b_ii[i + 1] = ii[i];
  }
  ::gik9dof::coder::internal::sort(b_ii);
  j2 = b_ii.size(0);
  i1 = b_ii.size(0) + 1;
  ii.set_size(b_ii.size(0));
  for (i = 0; i < j2; i++) {
    ii[i] = 0;
  }
  iwork.set_size(b_ii.size(0));
  i = b_ii.size(0) - 1;
  for (k = 1; k <= i; k += 2) {
    if ((b_ii[k - 1] <= b_ii[k]) || std::isnan(b_ii[k])) {
      ii[k - 1] = k;
      ii[k] = k + 1;
    } else {
      ii[k - 1] = k + 1;
      ii[k] = k;
    }
  }
  if ((static_cast<unsigned int>(b_ii.size(0)) & 1U) != 0U) {
    ii[b_ii.size(0) - 1] = b_ii.size(0);
  }
  b_i = 2;
  while (b_i < i1 - 1) {
    i2 = b_i << 1;
    j = 1;
    for (pEnd = b_i + 1; pEnd < i1; pEnd = qEnd + b_i) {
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
        i = ii[p - 1];
        if ((b_ii[i - 1] <= xtmp) || std::isnan(xtmp)) {
          iwork[k] = i;
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
    b_i = i2;
  }
  b.set_size(b_ii.size(0));
  for (k = 0; k < j2; k++) {
    b[k] = b_ii[ii[k] - 1];
  }
  k = b_ii.size(0);
  while ((k >= 1) && std::isnan(b[k - 1])) {
    k--;
  }
  j2 = b_ii.size(0) - k;
  exitg1 = false;
  while ((!exitg1) && (k >= 1)) {
    xtmp = b[k - 1];
    if (std::isinf(xtmp) && (xtmp > 0.0)) {
      k--;
    } else {
      exitg1 = true;
    }
  }
  i1 = (b_ii.size(0) - k) - j2;
  b_i = -1;
  pEnd = 1;
  while (pEnd <= k) {
    xtmp = b[pEnd - 1];
    do {
      pEnd++;
    } while (!((pEnd > k) || (b[pEnd - 1] != xtmp)));
    b_i++;
    b[b_i] = xtmp;
  }
  if (i1 > 0) {
    b_i++;
    b[b_i] = b[k];
  }
  pEnd = k + i1;
  for (j = 0; j < j2; j++) {
    b[(b_i + j) + 1] = b[pEnd + j];
  }
  if (j2 - 1 >= 0) {
    b_i += j2;
  }
  if (b_i + 1 < 1) {
    i = -1;
  } else {
    i = b_i;
  }
  b.set_size(i + 1);
  b_value.set_size(1, i + 1);
  pEnd = i + 1;
  for (i1 = 0; i1 < pEnd; i1++) {
    b_value[i1] = b[i1];
  }
  i1 = (i + 1) >> 1;
  for (b_i = 0; b_i < i1; b_i++) {
    j2 = i - b_i;
    xtmp = b_value[b_i];
    b_value[b_i] = b_value[j2];
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
