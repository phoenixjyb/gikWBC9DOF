//
// File: RigidBodyTree.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

// Include Files
#include "RigidBodyTree.h"
#include "CharacterVector.h"
#include "CollisionSet.h"
#include "RigidBody.h"
#include "find.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "rand.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "strcmp.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Variable Definitions
static const char cv[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '1'};

// Function Declarations
static void binary_expand_op_29(boolean_T in1[9], const double in2[9],
                                const coder::array<double, 2U> &in3);

static void binary_expand_op_30(boolean_T in1[9], const double in2[9],
                                const coder::array<double, 2U> &in3);

static int div_s32(int numerator, int denominator);

// Function Definitions
//
// Arguments    : RigidBody *body
//                array<double, 2U> &indices
// Return Type  : void
//
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
void RigidBodyTree::ancestorIndices(RigidBody *body, array<double, 2U> &indices)
{
  unsigned int b_i;
  int loop_ub;
  indices.set_size(1, static_cast<int>(NumBodies + 1.0));
  loop_ub = static_cast<int>(NumBodies + 1.0);
  for (int i{0}; i < loop_ub; i++) {
    indices[i] = 0.0;
  }
  b_i = 2U;
  indices[0] = body->Index;
  while (body->ParentIndex > 0.0) {
    body = Bodies[static_cast<int>(body->ParentIndex) - 1];
    indices[static_cast<int>(b_i) - 1] = body->Index;
    b_i++;
  }
  if (body->Index > 0.0) {
    indices[static_cast<int>(b_i) - 1] = body->ParentIndex;
    b_i++;
  }
  indices.set_size(indices.size(0), static_cast<int>(b_i - 1U));
}

//
// Arguments    : RigidBody *body1
//                RigidBody *body2
//                array<double, 2U> &indices
// Return Type  : void
//
void RigidBodyTree::kinematicPathInternal(RigidBody *body1, RigidBody *body2,
                                          array<double, 2U> &indices)
{
  array<double, 2U> ancestorIndices1;
  array<double, 2U> ancestorIndices2;
  int b_i;
  int b_loop_ub;
  int i;
  int i1;
  int loop_ub;
  int minPathLength;
  boolean_T exitg1;
  ancestorIndices(body1, ancestorIndices1);
  ancestorIndices(body2, ancestorIndices2);
  minPathLength = static_cast<int>(
      std::fmin(static_cast<double>(ancestorIndices1.size(1)),
                static_cast<double>(ancestorIndices2.size(1))));
  i = 2;
  exitg1 = false;
  while ((!exitg1) && (i - 2 <= minPathLength - 2)) {
    if (ancestorIndices1[ancestorIndices1.size(1) - i] !=
        ancestorIndices2[ancestorIndices2.size(1) - i]) {
      minPathLength = i - 1;
      exitg1 = true;
    } else {
      i++;
    }
  }
  b_i = ancestorIndices1.size(1) - minPathLength;
  if (b_i < 1) {
    loop_ub = 0;
  } else {
    loop_ub = b_i;
  }
  i = ancestorIndices2.size(1) - minPathLength;
  if (i < 1) {
    i = 0;
    minPathLength = 1;
    i1 = -1;
  } else {
    i--;
    minPathLength = -1;
    i1 = 0;
  }
  b_loop_ub = div_s32(i1 - i, minPathLength);
  indices.set_size(1, (loop_ub + b_loop_ub) + 2);
  for (i1 = 0; i1 < loop_ub; i1++) {
    indices[i1] = ancestorIndices1[i1];
  }
  indices[loop_ub] = ancestorIndices1[b_i];
  for (b_i = 0; b_i <= b_loop_ub; b_i++) {
    indices[(b_i + loop_ub) + 1] = ancestorIndices2[i + minPathLength * b_i];
  }
}

//
// Arguments    : boolean_T in1[9]
//                const double in2[9]
//                const coder::array<double, 2U> &in3
// Return Type  : void
//
} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
static void binary_expand_op_29(boolean_T in1[9], const double in2[9],
                                const coder::array<double, 2U> &in3)
{
  int stride_0_0;
  stride_0_0 = (in3.size(0) != 1);
  for (int i{0}; i < 9; i++) {
    in1[i] = (in2[i] >= in3[i * stride_0_0] - 4.4408920985006262E-16);
  }
}

//
// Arguments    : boolean_T in1[9]
//                const double in2[9]
//                const coder::array<double, 2U> &in3
// Return Type  : void
//
static void binary_expand_op_30(boolean_T in1[9], const double in2[9],
                                const coder::array<double, 2U> &in3)
{
  int stride_0_0;
  stride_0_0 = (in3.size(0) != 1);
  for (int i{0}; i < 9; i++) {
    in1[i] =
        (in2[i] <= in3[i * stride_0_0 + in3.size(0)] + 4.4408920985006262E-16);
  }
}

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_s32(int numerator, int denominator)
{
  int quotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    unsigned int u;
    unsigned int u1;
    if (numerator < 0) {
      u = ~static_cast<unsigned int>(numerator) + 1U;
    } else {
      u = static_cast<unsigned int>(numerator);
    }
    if (denominator < 0) {
      u1 = ~static_cast<unsigned int>(denominator) + 1U;
    } else {
      u1 = static_cast<unsigned int>(denominator);
    }
    u /= u1;
    if ((numerator < 0) != (denominator < 0)) {
      quotient = -static_cast<int>(u);
    } else {
      quotient = static_cast<int>(u);
    }
  }
  return quotient;
}

//
// Arguments    : RigidBody *bodyin
//                const char parentName_data[]
//                const int parentName_size[2]
//                CollisionSet &iobj_0
//                rigidBodyJoint &iobj_1
//                RigidBody &iobj_2
// Return Type  : void
//
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
void RigidBodyTree::addBody(RigidBody *bodyin, const char parentName_data[],
                            const int parentName_size[2], CollisionSet &iobj_0,
                            rigidBodyJoint &iobj_1, RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  CharacterVector obj;
  RigidBody *body;
  rigidBodyJoint *jnt;
  double b_index;
  double pid;
  int obj_size[2];
  int loop_ub;
  char obj_data[200];
  boolean_T b_bool;
  obj = bodyin->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  findBodyIndexByName(obj_data, obj_size);
  pid = findBodyIndexByName(parentName_data, parentName_size);
  jnt = bodyin->JointInternal;
  obj = jnt->NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  findBodyIndexByJointName(obj_data, obj_size);
  b_index = NumBodies + 1.0;
  body = bodyin->copy((&iobj_0)[0], (&iobj_1)[0], iobj_2);
  Bodies[static_cast<int>(b_index) - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  NumBodies++;
  jnt = body->JointInternal;
  obj = jnt->TypeInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (loop_ub == 5) {
    loop_ub = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (loop_ub < 5) {
        if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
          exitg1 = 1;
        } else {
          loop_ub++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  if (!b_bool) {
    NumNonFixedBodies++;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    PositionDoFMap[loop_ub] = PositionNumber + 1.0;
    PositionDoFMap[loop_ub + 11] = PositionNumber + jnt->PositionNumber;
    jnt = body->JointInternal;
    loop_ub = static_cast<int>(body->Index) - 1;
    VelocityDoFMap[loop_ub] = VelocityNumber + 1.0;
    VelocityDoFMap[loop_ub + 11] = VelocityNumber + jnt->VelocityNumber;
  } else {
    loop_ub = static_cast<int>(body->Index);
    PositionDoFMap[loop_ub - 1] = 0.0;
    PositionDoFMap[loop_ub + 10] = -1.0;
    loop_ub = static_cast<int>(body->Index);
    VelocityDoFMap[loop_ub - 1] = 0.0;
    VelocityDoFMap[loop_ub + 10] = -1.0;
  }
  jnt = body->JointInternal;
  PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  VelocityNumber += jnt->VelocityNumber;
}

//
// Arguments    : void
// Return Type  : b_RigidBodyTree
//
b_RigidBodyTree::b_RigidBodyTree()
{
  matlabCodegenIsDeleted = true;
}

//
// Arguments    : void
// Return Type  : void
//
b_RigidBodyTree::~b_RigidBodyTree()
{
  matlabCodegenDestructor();
}

//
// Arguments    : const array<double, 1U> &qv
//                double bid1
//                double bid2
//                double T_data[]
//                int T_size[2]
//                array<double, 2U> &Jac
// Return Type  : void
//
void RigidBodyTree::efficientFKAndJacobianForIK(const array<double, 1U> &qv,
                                                double bid1, double bid2,
                                                double T_data[], int T_size[2],
                                                array<double, 2U> &Jac)
{
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  RigidBody *body1;
  RigidBody *body2;
  rigidBodyJoint *joint;
  array<double, 2U> B;
  array<double, 2U> kinematicPathIndices;
  array<double, 1U> b_qv;
  double b_data[36];
  double y_data[36];
  double T1[16];
  double T1j[16];
  double Tj1[16];
  double b[16];
  double R[9];
  double tempR[9];
  double b_v[3];
  double v[3];
  if ((bid1 >= 0.0) && (bid2 >= 0.0)) {
    double X[36];
    double s;
    int X_tmp;
    int c_i;
    int i;
    int loop_ub;
    int n_tmp;
    if (bid1 == 0.0) {
      body1 = &Base;
    } else {
      body1 = Bodies[static_cast<int>(bid1) - 1];
    }
    if (bid2 == 0.0) {
      body2 = &Base;
    } else {
      body2 = Bodies[static_cast<int>(bid2) - 1];
    }
    kinematicPathInternal(body1, body2, kinematicPathIndices);
    std::memset(&T1[0], 0, 16U * sizeof(double));
    T1[0] = 1.0;
    T1[5] = 1.0;
    T1[10] = 1.0;
    T1[15] = 1.0;
    Jac.set_size(6, static_cast<int>(PositionNumber));
    loop_ub = 6 * static_cast<int>(PositionNumber);
    for (i = 0; i < loop_ub; i++) {
      Jac[i] = 0.0;
    }
    i = kinematicPathIndices.size(1);
    for (int b_i{0}; b_i <= i - 2; b_i++) {
      __m128d r;
      __m128d r1;
      CharacterVector obj;
      double Tc2p[16];
      double b_tempR_tmp;
      double c_tempR_tmp;
      double tempR_tmp;
      int exitg1;
      int jointSign;
      boolean_T b_bool;
      boolean_T nextBodyIsParent;
      s = kinematicPathIndices[b_i];
      if (s != 0.0) {
        body1 = Bodies[static_cast<int>(s) - 1];
      } else {
        body1 = &Base;
      }
      s = kinematicPathIndices[b_i + 1];
      if (s != 0.0) {
        body2 = Bodies[static_cast<int>(s) - 1];
      } else {
        body2 = &Base;
      }
      nextBodyIsParent = (body2->Index == body1->ParentIndex);
      if (nextBodyIsParent) {
        body2 = body1;
        jointSign = 1;
      } else {
        jointSign = -1;
      }
      joint = body2->JointInternal;
      obj = joint->TypeInternal;
      if (obj.Length < 1.0) {
        c_i = 0;
      } else {
        c_i = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (c_i == 5) {
        loop_ub = 0;
        do {
          exitg1 = 0;
          if (loop_ub < 5) {
            if (obj.Vector[loop_ub] != b_cv[loop_ub]) {
              exitg1 = 1;
            } else {
              loop_ub++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }
      if (b_bool) {
        double Tj[16];
        for (c_i = 0; c_i < 16; c_i++) {
          Tj1[c_i] = joint->JointToParentTransform[c_i];
        }
        obj = joint->TypeInternal;
        if (obj.Length < 1.0) {
          c_i = 0;
        } else {
          c_i = static_cast<int>(obj.Length);
        }
        b_bool = false;
        if (c_i == 8) {
          loop_ub = 0;
          do {
            exitg1 = 0;
            if (loop_ub < 8) {
              if (b_cv1[loop_ub] != obj.Vector[loop_ub]) {
                exitg1 = 1;
              } else {
                loop_ub++;
              }
            } else {
              b_bool = true;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }
        if (b_bool) {
          loop_ub = 0;
        } else {
          b_bool = false;
          if (c_i == 9) {
            loop_ub = 0;
            do {
              exitg1 = 0;
              if (loop_ub < 9) {
                if (cv1[loop_ub] != obj.Vector[loop_ub]) {
                  exitg1 = 1;
                } else {
                  loop_ub++;
                }
              } else {
                b_bool = true;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }
          if (b_bool) {
            loop_ub = 1;
          } else {
            b_bool = false;
            if (c_i == 8) {
              loop_ub = 0;
              do {
                exitg1 = 0;
                if (loop_ub < 8) {
                  if (b_cv2[loop_ub] != obj.Vector[loop_ub]) {
                    exitg1 = 1;
                  } else {
                    loop_ub++;
                  }
                } else {
                  b_bool = true;
                  exitg1 = 1;
                }
              } while (exitg1 == 0);
            }
            if (b_bool) {
              loop_ub = 2;
            } else {
              loop_ub = -1;
            }
          }
        }
        switch (loop_ub) {
        case 0: {
          double d_tempR_tmp;
          double e_tempR_tmp;
          double f_tempR_tmp;
          double qidx_idx_0;
          double qidx_idx_1;
          joint->get_JointAxis(v);
          r = _mm_loadu_pd(&v[0]);
          _mm_storeu_pd(&b_v[0], _mm_mul_pd(r, r));
          s = 1.0 / std::sqrt((b_v[0] + b_v[1]) + v[2] * v[2]);
          r = _mm_loadu_pd(&v[0]);
          _mm_storeu_pd(&b_v[0], _mm_mul_pd(r, _mm_set1_pd(s)));
          b_v[2] = v[2] * s;
          s = b_v[0] * b_v[0] * 0.0 + 1.0;
          tempR[0] = s;
          tempR_tmp = b_v[0] * b_v[1] * 0.0;
          b_tempR_tmp = tempR_tmp - b_v[2] * 0.0;
          tempR[1] = b_tempR_tmp;
          c_tempR_tmp = b_v[0] * b_v[2] * 0.0;
          qidx_idx_0 = c_tempR_tmp + b_v[1] * 0.0;
          tempR[2] = qidx_idx_0;
          tempR_tmp += b_v[2] * 0.0;
          tempR[3] = tempR_tmp;
          qidx_idx_1 = b_v[1] * b_v[1] * 0.0 + 1.0;
          tempR[4] = qidx_idx_1;
          d_tempR_tmp = b_v[1] * b_v[2] * 0.0;
          e_tempR_tmp = d_tempR_tmp - b_v[0] * 0.0;
          tempR[5] = e_tempR_tmp;
          c_tempR_tmp -= b_v[1] * 0.0;
          tempR[6] = c_tempR_tmp;
          d_tempR_tmp += b_v[0] * 0.0;
          tempR[7] = d_tempR_tmp;
          f_tempR_tmp = b_v[2] * b_v[2] * 0.0 + 1.0;
          tempR[8] = f_tempR_tmp;
          R[0] = s;
          R[1] = b_tempR_tmp;
          R[2] = qidx_idx_0;
          R[3] = tempR_tmp;
          R[4] = qidx_idx_1;
          R[5] = e_tempR_tmp;
          R[6] = c_tempR_tmp;
          R[7] = d_tempR_tmp;
          R[8] = f_tempR_tmp;
          for (int k{0}; k < 3; k++) {
            R[k] = tempR[3 * k];
            R[k + 3] = tempR[3 * k + 1];
            R[k + 6] = tempR[3 * k + 2];
          }
          std::memset(&b[0], 0, 16U * sizeof(double));
          for (c_i = 0; c_i < 3; c_i++) {
            loop_ub = c_i << 2;
            b[loop_ub] = R[3 * c_i];
            b[loop_ub + 1] = R[3 * c_i + 1];
            b[loop_ub + 2] = R[3 * c_i + 2];
          }
          b[15] = 1.0;
        } break;
        case 1:
          joint->get_JointAxis(v);
          std::memset(&R[0], 0, 9U * sizeof(double));
          R[0] = 1.0;
          R[4] = 1.0;
          R[8] = 1.0;
          for (c_i = 0; c_i < 3; c_i++) {
            loop_ub = c_i << 2;
            b[loop_ub] = R[3 * c_i];
            b[loop_ub + 1] = R[3 * c_i + 1];
            b[loop_ub + 2] = R[3 * c_i + 2];
            b[c_i + 12] = v[c_i] * 0.0;
          }
          b[3] = 0.0;
          b[7] = 0.0;
          b[11] = 0.0;
          b[15] = 1.0;
          break;
        case 2:
          // A check that is always false is detected at compile-time.
          // Eliminating code that follows.
          break;
        default:
          std::memset(&b[0], 0, 16U * sizeof(double));
          b[0] = 1.0;
          b[5] = 1.0;
          b[10] = 1.0;
          b[15] = 1.0;
          break;
        }
        for (c_i = 0; c_i < 16; c_i++) {
          Tj[c_i] = joint->ChildToJointTransform[c_i];
        }
        for (c_i = 0; c_i < 4; c_i++) {
          s = Tj1[c_i];
          tempR_tmp = Tj1[c_i + 4];
          b_tempR_tmp = Tj1[c_i + 8];
          c_tempR_tmp = Tj1[c_i + 12];
          for (n_tmp = 0; n_tmp < 4; n_tmp++) {
            X_tmp = n_tmp << 2;
            T1j[c_i + X_tmp] = ((s * b[X_tmp] + tempR_tmp * b[X_tmp + 1]) +
                                b_tempR_tmp * b[X_tmp + 2]) +
                               c_tempR_tmp * b[X_tmp + 3];
          }
          s = T1j[c_i];
          tempR_tmp = T1j[c_i + 4];
          b_tempR_tmp = T1j[c_i + 8];
          c_tempR_tmp = T1j[c_i + 12];
          for (n_tmp = 0; n_tmp < 4; n_tmp++) {
            X_tmp = n_tmp << 2;
            Tc2p[c_i + X_tmp] = ((s * Tj[X_tmp] + tempR_tmp * Tj[X_tmp + 1]) +
                                 b_tempR_tmp * Tj[X_tmp + 2]) +
                                c_tempR_tmp * Tj[X_tmp + 3];
          }
        }
      } else {
        double Tj[16];
        double qidx_idx_0;
        double qidx_idx_1;
        loop_ub = static_cast<int>(body2->Index);
        qidx_idx_0 = PositionDoFMap[loop_ub - 1];
        qidx_idx_1 = PositionDoFMap[loop_ub + 10];
        if (qidx_idx_0 > qidx_idx_1) {
          c_i = 0;
          n_tmp = 0;
        } else {
          c_i = static_cast<int>(qidx_idx_0) - 1;
          n_tmp = static_cast<int>(qidx_idx_1);
        }
        loop_ub = n_tmp - c_i;
        b_qv.set_size(loop_ub);
        for (n_tmp = 0; n_tmp < loop_ub; n_tmp++) {
          b_qv[n_tmp] = qv[c_i + n_tmp];
        }
        joint->transformBodyToParent(b_qv, Tc2p);
        loop_ub = static_cast<int>(body2->Index);
        qidx_idx_0 = VelocityDoFMap[loop_ub - 1];
        qidx_idx_1 = VelocityDoFMap[loop_ub + 10];
        if (nextBodyIsParent) {
          for (c_i = 0; c_i < 16; c_i++) {
            Tj[c_i] = joint->ChildToJointTransform[c_i];
          }
        } else {
          for (c_i = 0; c_i < 16; c_i++) {
            Tj1[c_i] = joint->JointToParentTransform[c_i];
          }
          for (c_i = 0; c_i < 3; c_i++) {
            R[3 * c_i] = Tj1[c_i];
            R[3 * c_i + 1] = Tj1[c_i + 4];
            R[3 * c_i + 2] = Tj1[c_i + 8];
          }
          r = _mm_loadu_pd(&R[0]);
          r1 = _mm_set1_pd(-1.0);
          _mm_storeu_pd(&tempR[0], _mm_mul_pd(r, r1));
          r = _mm_loadu_pd(&R[2]);
          _mm_storeu_pd(&tempR[2], _mm_mul_pd(r, r1));
          r = _mm_loadu_pd(&R[4]);
          _mm_storeu_pd(&tempR[4], _mm_mul_pd(r, r1));
          r = _mm_loadu_pd(&R[6]);
          _mm_storeu_pd(&tempR[6], _mm_mul_pd(r, r1));
          tempR[8] = -R[8];
          s = Tj1[12];
          tempR_tmp = Tj1[13];
          b_tempR_tmp = Tj1[14];
          for (c_i = 0; c_i < 3; c_i++) {
            loop_ub = c_i << 2;
            Tj[loop_ub] = R[3 * c_i];
            Tj[loop_ub + 1] = R[3 * c_i + 1];
            Tj[loop_ub + 2] = R[3 * c_i + 2];
            Tj[c_i + 12] = (tempR[c_i] * s + tempR[c_i + 3] * tempR_tmp) +
                           tempR[c_i + 6] * b_tempR_tmp;
          }
          Tj[3] = 0.0;
          Tj[7] = 0.0;
          Tj[11] = 0.0;
          Tj[15] = 1.0;
        }
        for (c_i = 0; c_i < 4; c_i++) {
          s = Tj[c_i];
          tempR_tmp = Tj[c_i + 4];
          b_tempR_tmp = Tj[c_i + 8];
          c_tempR_tmp = Tj[c_i + 12];
          for (n_tmp = 0; n_tmp < 4; n_tmp++) {
            X_tmp = n_tmp << 2;
            T1j[c_i + X_tmp] = ((s * T1[X_tmp] + tempR_tmp * T1[X_tmp + 1]) +
                                b_tempR_tmp * T1[X_tmp + 2]) +
                               c_tempR_tmp * T1[X_tmp + 3];
          }
        }
        for (c_i = 0; c_i < 3; c_i++) {
          R[3 * c_i] = T1j[c_i];
          R[3 * c_i + 1] = T1j[c_i + 4];
          R[3 * c_i + 2] = T1j[c_i + 8];
        }
        r = _mm_loadu_pd(&R[0]);
        r1 = _mm_set1_pd(-1.0);
        _mm_storeu_pd(&tempR[0], _mm_mul_pd(r, r1));
        r = _mm_loadu_pd(&R[2]);
        _mm_storeu_pd(&tempR[2], _mm_mul_pd(r, r1));
        r = _mm_loadu_pd(&R[4]);
        _mm_storeu_pd(&tempR[4], _mm_mul_pd(r, r1));
        r = _mm_loadu_pd(&R[6]);
        _mm_storeu_pd(&tempR[6], _mm_mul_pd(r, r1));
        tempR[8] = -R[8];
        s = T1j[12];
        tempR_tmp = T1j[13];
        b_tempR_tmp = T1j[14];
        for (c_i = 0; c_i < 3; c_i++) {
          loop_ub = c_i << 2;
          Tj1[loop_ub] = R[3 * c_i];
          Tj1[loop_ub + 1] = R[3 * c_i + 1];
          Tj1[loop_ub + 2] = R[3 * c_i + 2];
          Tj1[c_i + 12] = (tempR[c_i] * s + tempR[c_i + 3] * tempR_tmp) +
                          tempR[c_i + 6] * b_tempR_tmp;
        }
        Tj1[3] = 0.0;
        Tj1[7] = 0.0;
        Tj1[11] = 0.0;
        Tj1[15] = 1.0;
        R[0] = 0.0;
        R[3] = -Tj1[14];
        R[6] = Tj1[13];
        R[1] = Tj1[14];
        R[4] = 0.0;
        R[7] = -Tj1[12];
        R[2] = -Tj1[13];
        R[5] = Tj1[12];
        R[8] = 0.0;
        for (c_i = 0; c_i < 3; c_i++) {
          s = R[c_i];
          tempR_tmp = R[c_i + 3];
          b_tempR_tmp = R[c_i + 6];
          for (n_tmp = 0; n_tmp < 3; n_tmp++) {
            X_tmp = n_tmp << 2;
            tempR[c_i + 3 * n_tmp] =
                (s * Tj1[X_tmp] + tempR_tmp * Tj1[X_tmp + 1]) +
                b_tempR_tmp * Tj1[X_tmp + 2];
            X[n_tmp + 6 * c_i] = Tj1[n_tmp + (c_i << 2)];
            X[n_tmp + 6 * (c_i + 3)] = 0.0;
          }
        }
        for (c_i = 0; c_i < 3; c_i++) {
          X[6 * c_i + 3] = tempR[3 * c_i];
          loop_ub = c_i << 2;
          X_tmp = 6 * (c_i + 3);
          X[X_tmp + 3] = Tj1[loop_ub];
          X[6 * c_i + 4] = tempR[3 * c_i + 1];
          X[X_tmp + 4] = Tj1[loop_ub + 1];
          X[6 * c_i + 5] = tempR[3 * c_i + 2];
          X[X_tmp + 5] = Tj1[loop_ub + 2];
        }
        int b_size[2];
        joint->get_MotionSubspace(b_data, b_size);
        n_tmp = b_size[1];
        for (loop_ub = 0; loop_ub < n_tmp; loop_ub++) {
          X_tmp = loop_ub * 6;
          for (c_i = 0; c_i < 6; c_i++) {
            s = 0.0;
            for (int k{0}; k < 6; k++) {
              s += X[k * 6 + c_i] * b_data[X_tmp + k];
            }
            y_data[X_tmp + c_i] = s;
          }
        }
        if (qidx_idx_0 > qidx_idx_1) {
          c_i = 0;
          n_tmp = 0;
        } else {
          c_i = static_cast<int>(qidx_idx_0) - 1;
          n_tmp = static_cast<int>(qidx_idx_1);
        }
        loop_ub = n_tmp - c_i;
        for (n_tmp = 0; n_tmp < loop_ub; n_tmp++) {
          r = _mm_loadu_pd(&y_data[6 * n_tmp]);
          X_tmp = c_i + n_tmp;
          r1 = _mm_set1_pd(static_cast<double>(jointSign));
          _mm_storeu_pd(&Jac[6 * X_tmp], _mm_mul_pd(r, r1));
          r = _mm_loadu_pd(&y_data[6 * n_tmp + 2]);
          _mm_storeu_pd(&Jac[6 * X_tmp + 2], _mm_mul_pd(r, r1));
          r = _mm_loadu_pd(&y_data[6 * n_tmp + 4]);
          _mm_storeu_pd(&Jac[6 * X_tmp + 4], _mm_mul_pd(r, r1));
        }
      }
      if (nextBodyIsParent) {
        for (c_i = 0; c_i < 4; c_i++) {
          s = Tc2p[c_i];
          tempR_tmp = Tc2p[c_i + 4];
          b_tempR_tmp = Tc2p[c_i + 8];
          c_tempR_tmp = Tc2p[c_i + 12];
          for (n_tmp = 0; n_tmp < 4; n_tmp++) {
            X_tmp = n_tmp << 2;
            T1j[c_i + X_tmp] = ((s * T1[X_tmp] + tempR_tmp * T1[X_tmp + 1]) +
                                b_tempR_tmp * T1[X_tmp + 2]) +
                               c_tempR_tmp * T1[X_tmp + 3];
          }
        }
        ::std::copy(&T1j[0], &T1j[16], &T1[0]);
      } else {
        for (c_i = 0; c_i < 3; c_i++) {
          R[3 * c_i] = Tc2p[c_i];
          R[3 * c_i + 1] = Tc2p[c_i + 4];
          R[3 * c_i + 2] = Tc2p[c_i + 8];
        }
        r = _mm_loadu_pd(&R[0]);
        r1 = _mm_set1_pd(-1.0);
        _mm_storeu_pd(&tempR[0], _mm_mul_pd(r, r1));
        r = _mm_loadu_pd(&R[2]);
        _mm_storeu_pd(&tempR[2], _mm_mul_pd(r, r1));
        r = _mm_loadu_pd(&R[4]);
        _mm_storeu_pd(&tempR[4], _mm_mul_pd(r, r1));
        r = _mm_loadu_pd(&R[6]);
        _mm_storeu_pd(&tempR[6], _mm_mul_pd(r, r1));
        tempR[8] = -R[8];
        s = Tc2p[12];
        tempR_tmp = Tc2p[13];
        b_tempR_tmp = Tc2p[14];
        for (c_i = 0; c_i < 3; c_i++) {
          loop_ub = c_i << 2;
          T1j[loop_ub] = R[3 * c_i];
          T1j[loop_ub + 1] = R[3 * c_i + 1];
          T1j[loop_ub + 2] = R[3 * c_i + 2];
          T1j[c_i + 12] = (tempR[c_i] * s + tempR[c_i + 3] * tempR_tmp) +
                          tempR[c_i + 6] * b_tempR_tmp;
        }
        T1j[3] = 0.0;
        T1j[7] = 0.0;
        T1j[11] = 0.0;
        T1j[15] = 1.0;
        for (c_i = 0; c_i < 4; c_i++) {
          s = T1j[c_i];
          tempR_tmp = T1j[c_i + 4];
          b_tempR_tmp = T1j[c_i + 8];
          c_tempR_tmp = T1j[c_i + 12];
          for (n_tmp = 0; n_tmp < 4; n_tmp++) {
            X_tmp = n_tmp << 2;
            Tj1[c_i + X_tmp] = ((s * T1[X_tmp] + tempR_tmp * T1[X_tmp + 1]) +
                                b_tempR_tmp * T1[X_tmp + 2]) +
                               c_tempR_tmp * T1[X_tmp + 3];
          }
        }
        ::std::copy(&Tj1[0], &Tj1[16], &T1[0]);
      }
    }
    for (i = 0; i < 3; i++) {
      c_i = i << 2;
      s = T1[c_i];
      X[6 * i] = s;
      loop_ub = 6 * (i + 3);
      X[loop_ub] = 0.0;
      X[6 * i + 3] = 0.0;
      X[loop_ub + 3] = s;
      s = T1[c_i + 1];
      X[6 * i + 1] = s;
      X[loop_ub + 1] = 0.0;
      X[6 * i + 4] = 0.0;
      X[loop_ub + 4] = s;
      s = T1[c_i + 2];
      X[6 * i + 2] = s;
      X[loop_ub + 2] = 0.0;
      X[6 * i + 5] = 0.0;
      X[loop_ub + 5] = s;
    }
    n_tmp = Jac.size(1);
    B.set_size(6, Jac.size(1));
    loop_ub = 6 * Jac.size(1);
    for (i = 0; i < loop_ub; i++) {
      B[i] = Jac[i];
    }
    Jac.set_size(6, n_tmp);
    for (loop_ub = 0; loop_ub < n_tmp; loop_ub++) {
      X_tmp = loop_ub * 6;
      for (int b_i{0}; b_i < 6; b_i++) {
        s = 0.0;
        for (int k{0}; k < 6; k++) {
          s += X[k * 6 + b_i] * B[X_tmp + k];
        }
        Jac[X_tmp + b_i] = s;
      }
    }
    T_size[0] = 4;
    T_size[1] = 4;
    ::std::copy(&T1[0], &T1[16], &T_data[0]);
  } else {
    T_size[0] = 0;
    T_size[1] = 0;
    Jac.set_size(6, 0);
  }
}

//
// Arguments    : const char jointname_data[]
//                const int jointname_size[2]
// Return Type  : double
//
double RigidBodyTree::findBodyIndexByJointName(const char jointname_data[],
                                               const int jointname_size[2])
{
  CharacterVector b_obj;
  RigidBody *obj;
  rigidBodyJoint *jnt;
  double bid;
  double d;
  int obj_size[2];
  int i;
  char obj_data[200];
  boolean_T exitg1;
  bid = -1.0;
  d = NumBodies;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= static_cast<int>(d) - 1)) {
    int loop_ub;
    obj = Bodies[i];
    jnt = obj->JointInternal;
    b_obj = jnt->NameInternal;
    if (b_obj.Length < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(b_obj.Length);
    }
    obj_size[0] = 1;
    obj_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &obj_data[0]);
    }
    if (::coder::internal::b_strcmp(obj_data, obj_size, jointname_data,
                                    jointname_size)) {
      bid = static_cast<double>(i) + 1.0;
      exitg1 = true;
    } else {
      i++;
    }
  }
  return bid;
}

//
// Arguments    : const char bodyname_data[]
//                const int bodyname_size[2]
// Return Type  : double
//
double RigidBodyTree::findBodyIndexByName(const char bodyname_data[],
                                          const int bodyname_size[2])
{
  CharacterVector obj;
  RigidBody *b_obj;
  double bid;
  int obj_size[2];
  int loop_ub;
  char obj_data[200];
  bid = -1.0;
  obj = Base.NameInternal;
  if (obj.Length < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(obj.Length);
  }
  obj_size[0] = 1;
  obj_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  if (::coder::internal::b_strcmp(obj_data, obj_size, bodyname_data,
                                  bodyname_size)) {
    bid = 0.0;
  } else {
    double d;
    int i;
    boolean_T exitg1;
    d = NumBodies;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= static_cast<int>(d) - 1)) {
      b_obj = Bodies[i];
      obj = b_obj->NameInternal;
      if (obj.Length < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = static_cast<int>(obj.Length);
      }
      obj_size[0] = 1;
      obj_size[1] = loop_ub;
      if (loop_ub - 1 >= 0) {
        ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
      }
      if (::coder::internal::b_strcmp(obj_data, obj_size, bodyname_data,
                                      bodyname_size)) {
        bid = static_cast<double>(i) + 1.0;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }
  return bid;
}

//
// Arguments    : array<double, 2U> &limits
// Return Type  : void
//
void RigidBodyTree::get_JointPositionLimits(array<double, 2U> &limits)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  RigidBody *body;
  rigidBodyJoint *obj;
  double d;
  double k;
  int i;
  int loop_ub;
  limits.set_size(static_cast<int>(PositionNumber), 2);
  loop_ub = static_cast<int>(PositionNumber) << 1;
  for (i = 0; i < loop_ub; i++) {
    limits[i] = 0.0;
  }
  k = 1.0;
  d = NumBodies;
  i = static_cast<int>(d);
  for (int b_i{0}; b_i < i; b_i++) {
    CharacterVector b_obj;
    int exitg1;
    int i1;
    int kstr;
    boolean_T b_bool;
    body = Bodies[b_i];
    obj = body->JointInternal;
    b_obj = obj->TypeInternal;
    if (b_obj.Length < 1.0) {
      i1 = 0;
    } else {
      i1 = static_cast<int>(b_obj.Length);
    }
    b_bool = false;
    if (i1 == 5) {
      kstr = 0;
      do {
        exitg1 = 0;
        if (kstr < 5) {
          if (b_obj.Vector[kstr] != b_cv[kstr]) {
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
      double tmp_data[14];
      double pnum;
      int i2;
      int i3;
      pnum = body->JointInternal->PositionNumber;
      d = k + pnum;
      if (k > d - 1.0) {
        i1 = 0;
        i2 = 0;
      } else {
        i1 = static_cast<int>(k) - 1;
        i2 = static_cast<int>(d - 1.0);
      }
      obj = body->JointInternal;
      b_obj = obj->TypeInternal;
      if (b_obj.Length < 1.0) {
        i3 = 0;
      } else {
        i3 = static_cast<int>(b_obj.Length);
      }
      b_bool = false;
      if (i3 == 5) {
        kstr = 0;
        do {
          exitg1 = 0;
          if (kstr < 5) {
            if (b_obj.Vector[kstr] != b_cv[kstr]) {
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
        pnum = obj->PositionNumber;
        if (pnum < 1.0) {
          loop_ub = 0;
        } else {
          loop_ub = static_cast<int>(pnum);
        }
        for (i3 = 0; i3 < 2; i3++) {
          for (kstr = 0; kstr < loop_ub; kstr++) {
            tmp_data[kstr + loop_ub * i3] =
                obj->PositionLimitsInternal[kstr + 7 * i3];
          }
        }
      } else {
        loop_ub = 1;
        tmp_data[0] = 0.0;
        tmp_data[1] = 0.0;
      }
      kstr = i2 - i1;
      for (i2 = 0; i2 < 2; i2++) {
        for (i3 = 0; i3 < kstr; i3++) {
          limits[(i1 + i3) + limits.size(0) * i2] = tmp_data[i3 + loop_ub * i2];
        }
      }
      k = d;
    }
  }
}

//
// Arguments    : void
// Return Type  : b_RigidBodyTree *
//
b_RigidBodyTree *b_RigidBodyTree::init()
{
  static const char jname[14]{'d', 'u', 'm', 'm', 'y', 'b', 'o',
                              'd', 'y', '1', '_', 'j', 'n', 't'};
  CharacterVector s;
  b_RigidBodyTree *obj;
  double unusedExpr[5];
  obj = this;
  b_rand(unusedExpr);
  s.Length = 200.0;
  for (int i{0}; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  obj->Base.NameInternal = s;
  s = obj->Base.NameInternal;
  s.Length = 4.0;
  s.Vector[0] = 'b';
  s.Vector[1] = 'a';
  s.Vector[2] = 's';
  s.Vector[3] = 'e';
  obj->Base.NameInternal = s;
  obj->Base.JointInternal.init();
  obj->Base.CollisionsInternal.init(static_cast<double>(0.0));
  obj->Base.matlabCodegenIsDeleted = false;
  s.Length = 200.0;
  for (int i{0}; i < 200; i++) {
    s.Vector[i] = ' ';
  }
  obj->_pobj0.NameInternal = s;
  s = obj->_pobj0.NameInternal;
  s.Length = 10.0;
  for (int i{0}; i < 10; i++) {
    s.Vector[i] = cv[i];
  }
  obj->_pobj0.NameInternal = s;
  obj->_pobj0.JointInternal.init(jname);
  obj->_pobj0.CollisionsInternal.init(static_cast<double>(0.0));
  obj->_pobj0.matlabCodegenIsDeleted = false;
  obj->Bodies[0] = &obj->_pobj0;
  b_rand(unusedExpr);
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : RigidBodyTree *
//
RigidBodyTree *RigidBodyTree::init()
{
  static const signed char b_iv[22]{0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                                    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
  static const char b_cv8[11]{'d', 'u', 'm', 'm', 'y', 'b',
                              'o', 'd', 'y', '1', '0'};
  static const char b_cv9[11]{'d', 'u', 'm', 'm', 'y', 'b',
                              'o', 'd', 'y', '1', '1'};
  static const char b_cv[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '2'};
  static const char b_cv1[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '3'};
  static const char b_cv2[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '4'};
  static const char b_cv3[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '5'};
  static const char b_cv4[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '6'};
  static const char b_cv5[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '7'};
  static const char b_cv6[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '8'};
  static const char b_cv7[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '9'};
  CharacterVector s;
  RigidBodyTree *obj;
  double unusedExpr[5];
  signed char c_I[36];
  signed char b_I[9];
  obj = this;
  b_rand(unusedExpr);
  s.Length = 200.0;
  for (int k{0}; k < 200; k++) {
    s.Vector[k] = ' ';
  }
  obj->Base.NameInternal = s;
  s = obj->Base.NameInternal;
  s.Length = 4.0;
  s.Vector[0] = 'b';
  s.Vector[1] = 'a';
  s.Vector[2] = 's';
  s.Vector[3] = 'e';
  obj->Base.NameInternal = s;
  obj->Base.JointInternal = obj->Base._pobj1.init();
  obj->Base.Index = -1.0;
  obj->Base.ParentIndex = -1.0;
  obj->Base.MassInternal = 1.0;
  obj->Base.CenterOfMassInternal[0] = 0.0;
  obj->Base.CenterOfMassInternal[1] = 0.0;
  obj->Base.CenterOfMassInternal[2] = 0.0;
  for (int k{0}; k < 9; k++) {
    b_I[k] = 0;
  }
  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (int k{0}; k < 9; k++) {
    obj->Base.InertiaInternal[k] = b_I[k];
  }
  for (int k{0}; k < 36; k++) {
    c_I[k] = 0;
  }
  for (int k{0}; k < 6; k++) {
    c_I[k + 6 * k] = 1;
  }
  for (int k{0}; k < 36; k++) {
    obj->Base.SpatialInertia[k] = c_I[k];
  }
  obj->Base.CollisionsInternal =
      obj->Base._pobj0.init(static_cast<double>(0.0));
  obj->Base.matlabCodegenIsDeleted = false;
  obj->Base.Index = 0.0;
  obj->Gravity[0] = 0.0;
  obj->Gravity[1] = 0.0;
  obj->Gravity[2] = 0.0;
  obj->Bodies[0] = obj->_pobj2[0].init(cv);
  obj->Bodies[1] = obj->_pobj2[1].init(b_cv);
  obj->Bodies[2] = obj->_pobj2[2].init(b_cv1);
  obj->Bodies[3] = obj->_pobj2[3].init(b_cv2);
  obj->Bodies[4] = obj->_pobj2[4].init(b_cv3);
  obj->Bodies[5] = obj->_pobj2[5].init(b_cv4);
  obj->Bodies[6] = obj->_pobj2[6].init(b_cv5);
  obj->Bodies[7] = obj->_pobj2[7].init(b_cv6);
  obj->Bodies[8] = obj->_pobj2[8].init(b_cv7);
  obj->Bodies[9] = obj->_pobj2[9].init(b_cv8, obj->_pobj0[0], obj->_pobj1[0]);
  obj->Bodies[10] = obj->_pobj2[10].init(b_cv9, obj->_pobj0[1], obj->_pobj1[1]);
  obj->NumBodies = 0.0;
  obj->NumNonFixedBodies = 0.0;
  obj->PositionNumber = 0.0;
  obj->VelocityNumber = 0.0;
  b_rand(unusedExpr);
  for (int k{0}; k < 22; k++) {
    obj->PositionDoFMap[k] = b_iv[k];
  }
  for (int k{0}; k < 22; k++) {
    obj->VelocityDoFMap[k] = b_iv[k];
  }
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : const char body1Name_data[]
//                const int body1Name_size[2]
//                const char body2Name_data[]
//                const int body2Name_size[2]
//                array<double, 2U> &indices
// Return Type  : void
//
void RigidBodyTree::kinematicPath(const char body1Name_data[],
                                  const int body1Name_size[2],
                                  const char body2Name_data[],
                                  const int body2Name_size[2],
                                  array<double, 2U> &indices)
{
  RigidBody *body1;
  RigidBody *body2;
  double bid1;
  double bid2;
  bid1 = findBodyIndexByName(body1Name_data, body1Name_size);
  bid2 = findBodyIndexByName(body2Name_data, body2Name_size);
  if (bid1 == 0.0) {
    body1 = &Base;
  } else {
    body1 = Bodies[static_cast<int>(bid1) - 1];
  }
  if (bid2 == 0.0) {
    body2 = &Base;
  } else {
    body2 = Bodies[static_cast<int>(bid2) - 1];
  }
  kinematicPathInternal(body1, body2, indices);
}

//
// Arguments    : void
// Return Type  : void
//
void b_RigidBodyTree::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

//
// Arguments    : double Q[9]
// Return Type  : void
//
void RigidBodyTree::validateConfigurationWithLimits(double Q[9])
{
  array<double, 2U> limits;
  int indicesUpperBoundViolation_data[9];
  int k;
  boolean_T lbOK[9];
  boolean_T ubOK[9];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T y;
  get_JointPositionLimits(limits);
  if (limits.size(0) == 9) {
    for (int i{0}; i < 9; i++) {
      ubOK[i] = (Q[i] <= limits[i + limits.size(0)] + 4.4408920985006262E-16);
    }
  } else {
    binary_expand_op_30(ubOK, Q, limits);
  }
  if (limits.size(0) == 9) {
    for (int i{0}; i < 9; i++) {
      lbOK[i] = (Q[i] >= limits[i] - 4.4408920985006262E-16);
    }
  } else {
    binary_expand_op_29(lbOK, Q, limits);
  }
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 8)) {
    if (!ubOK[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  guard1 = false;
  if (y) {
    y = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= 8)) {
      if (!lbOK[k]) {
        y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (!y) {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    int i1;
    for (int i{0}; i < 9; i++) {
      ubOK[i] = !ubOK[i];
    }
    k = eml_find(ubOK, indicesUpperBoundViolation_data);
    for (int i{0}; i < k; i++) {
      i1 = indicesUpperBoundViolation_data[i];
      Q[i1 - 1] = limits[(i1 + limits.size(0)) - 1];
    }
    for (int i{0}; i < 9; i++) {
      lbOK[i] = !lbOK[i];
    }
    k = eml_find(lbOK, indicesUpperBoundViolation_data);
    for (int i{0}; i < k; i++) {
      i1 = indicesUpperBoundViolation_data[i];
      Q[i1 - 1] = limits[i1 - 1];
    }
  }
}

//
// Arguments    : const char bodyname[17]
// Return Type  : double
//
double RigidBodyTree::validateInputBodyName(const char bodyname[17])
{
  CharacterVector obj;
  RigidBody *b_obj;
  double bid;
  int exitg1;
  int kstr;
  boolean_T b_bool;
  bid = -1.0;
  obj = Base.NameInternal;
  if (obj.Length < 1.0) {
    kstr = 0;
  } else {
    kstr = static_cast<int>(obj.Length);
  }
  b_bool = false;
  if (kstr == 17) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 17) {
        if (obj.Vector[kstr] != bodyname[kstr]) {
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
    bid = 0.0;
  } else {
    double d;
    int i;
    boolean_T exitg2;
    d = NumBodies;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i <= static_cast<int>(d) - 1)) {
      b_obj = Bodies[i];
      obj = b_obj->NameInternal;
      if (obj.Length < 1.0) {
        kstr = 0;
      } else {
        kstr = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (kstr == 17) {
        kstr = 0;
        do {
          exitg1 = 0;
          if (kstr < 17) {
            if (obj.Vector[kstr] != bodyname[kstr]) {
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
        bid = static_cast<double>(i) + 1.0;
        exitg2 = true;
      } else {
        i++;
      }
    }
  }
  return bid;
}

} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder

//
// File trailer for RigidBodyTree.cpp
//
// [EOF]
//
