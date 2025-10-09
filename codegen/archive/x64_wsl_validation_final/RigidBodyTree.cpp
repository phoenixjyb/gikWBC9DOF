//
// File: RigidBodyTree.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 10:12:29
//

// Include Files
#include "RigidBodyTree.h"
#include "CharacterVector.h"
#include "CollisionSet.h"
#include "GIKSolver.h"
#include "RigidBody.h"
#include "rand.h"
#include "randn.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWrapper_data.h"
#include "strcmp.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Variable Definitions
namespace gik9dof {
static const char cv5[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '1'};

}

// Function Declarations
namespace gik9dof {
static int div_s32(int numerator, int denominator);

}

// Function Definitions
//
// Arguments    : RigidBody *body
//                ::coder::array<double, 2U> &indices
// Return Type  : void
//
namespace gik9dof {
namespace coder {
namespace robotics {
namespace manip {
namespace internal {
void b_RigidBodyTree::ancestorIndices(RigidBody *body,
                                      ::coder::array<double, 2U> &indices)
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
//                ::coder::array<double, 2U> &indices
// Return Type  : void
//
void b_RigidBodyTree::kinematicPathInternal(RigidBody *body1, RigidBody *body2,
                                            ::coder::array<double, 2U> &indices)
{
  ::coder::array<double, 2U> ancestorIndices1;
  ::coder::array<double, 2U> ancestorIndices2;
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
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
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
    unsigned int tempAbsQuotient;
    unsigned int u;
    if (numerator < 0) {
      tempAbsQuotient = ~static_cast<unsigned int>(numerator) + 1U;
    } else {
      tempAbsQuotient = static_cast<unsigned int>(numerator);
    }
    if (denominator < 0) {
      u = ~static_cast<unsigned int>(denominator) + 1U;
    } else {
      u = static_cast<unsigned int>(denominator);
    }
    tempAbsQuotient /= u;
    if ((numerator < 0) != (denominator < 0)) {
      quotient = -static_cast<int>(tempAbsQuotient);
    } else {
      quotient = static_cast<int>(tempAbsQuotient);
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
void b_RigidBodyTree::addBody(RigidBody *bodyin, const char parentName_data[],
                              const int parentName_size[2],
                              CollisionSet &iobj_0, rigidBodyJoint &iobj_1,
                              RigidBody &iobj_2)
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  CharacterVector obj;
  RigidBody *body;
  rigidBodyJoint *jnt;
  ::coder::array<char, 2U> b_obj_data;
  ::coder::array<char, 2U> b_parentName_data;
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
  if (loop_ub - 1 >= 0) {
    ::std::copy(&obj.Vector[0], &obj.Vector[loop_ub], &obj_data[0]);
  }
  b_obj_data.set(&obj_data[0], 1, loop_ub);
  findBodyIndexByName(b_obj_data);
  b_parentName_data.set((char *)&parentName_data[0], parentName_size[0],
                        parentName_size[1]);
  pid = findBodyIndexByName(b_parentName_data);
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
// Return Type  : void
//
b_RigidBodyTree::~b_RigidBodyTree() = default;

//
// Arguments    : void
// Return Type  : b_RigidBodyTree
//
b_RigidBodyTree::b_RigidBodyTree() = default;

//
// Arguments    : void
// Return Type  : RigidBodyTree
//
RigidBodyTree::RigidBodyTree()
{
  matlabCodegenIsDeleted = true;
}

//
// Arguments    : void
// Return Type  : void
//
RigidBodyTree::~RigidBodyTree()
{
  matlabCodegenDestructor();
}

//
// Arguments    : const ::coder::array<double, 1U> &qv
//                double bid1
//                double bid2
//                double T_data[]
//                int T_size[2]
//                ::coder::array<double, 2U> &Jac
// Return Type  : void
//
void b_RigidBodyTree::efficientFKAndJacobianForIK(
    const ::coder::array<double, 1U> &qv, double bid1, double bid2,
    double T_data[], int T_size[2], ::coder::array<double, 2U> &Jac)
{
  static const char b_cv1[8]{'r', 'e', 'v', 'o', 'l', 'u', 't', 'e'};
  static const char b_cv2[8]{'f', 'l', 'o', 'a', 't', 'i', 'n', 'g'};
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  RigidBody *body1;
  RigidBody *body2;
  rigidBodyJoint *joint;
  ::coder::array<double, 2U> C;
  ::coder::array<double, 2U> b;
  ::coder::array<double, 2U> kinematicPathIndices;
  ::coder::array<double, 1U> b_qv;
  double T1[16];
  double T1j[16];
  double Tj1[16];
  double b_b[16];
  double R[9];
  double tempR[9];
  double b_v[3];
  double v[3];
  if ((bid1 >= 0.0) && (bid2 >= 0.0)) {
    double A[36];
    double s;
    int A_tmp;
    int coffset_tmp;
    int i;
    int i1;
    int loop_ub;
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
      int i2;
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
        i1 = 0;
      } else {
        i1 = static_cast<int>(obj.Length);
      }
      b_bool = false;
      if (i1 == 5) {
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
        for (i1 = 0; i1 < 16; i1++) {
          Tj1[i1] = joint->JointToParentTransform[i1];
        }
        obj = joint->TypeInternal;
        if (obj.Length < 1.0) {
          i1 = 0;
        } else {
          i1 = static_cast<int>(obj.Length);
        }
        b_bool = false;
        if (i1 == 8) {
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
          if (i1 == 9) {
            loop_ub = 0;
            do {
              exitg1 = 0;
              if (loop_ub < 9) {
                if (cv6[loop_ub] != obj.Vector[loop_ub]) {
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
            if (i1 == 8) {
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
          qidx_idx_0 = b_v[0] * b_v[1] * 0.0;
          qidx_idx_1 = qidx_idx_0 - b_v[2] * 0.0;
          tempR[1] = qidx_idx_1;
          tempR_tmp = b_v[0] * b_v[2] * 0.0;
          b_tempR_tmp = tempR_tmp + b_v[1] * 0.0;
          tempR[2] = b_tempR_tmp;
          qidx_idx_0 += b_v[2] * 0.0;
          tempR[3] = qidx_idx_0;
          c_tempR_tmp = b_v[1] * b_v[1] * 0.0 + 1.0;
          tempR[4] = c_tempR_tmp;
          d_tempR_tmp = b_v[1] * b_v[2] * 0.0;
          e_tempR_tmp = d_tempR_tmp - b_v[0] * 0.0;
          tempR[5] = e_tempR_tmp;
          tempR_tmp -= b_v[1] * 0.0;
          tempR[6] = tempR_tmp;
          d_tempR_tmp += b_v[0] * 0.0;
          tempR[7] = d_tempR_tmp;
          f_tempR_tmp = b_v[2] * b_v[2] * 0.0 + 1.0;
          tempR[8] = f_tempR_tmp;
          R[0] = s;
          R[1] = qidx_idx_1;
          R[2] = b_tempR_tmp;
          R[3] = qidx_idx_0;
          R[4] = c_tempR_tmp;
          R[5] = e_tempR_tmp;
          R[6] = tempR_tmp;
          R[7] = d_tempR_tmp;
          R[8] = f_tempR_tmp;
          for (int k{0}; k < 3; k++) {
            R[k] = tempR[3 * k];
            R[k + 3] = tempR[3 * k + 1];
            R[k + 6] = tempR[3 * k + 2];
          }
          std::memset(&b_b[0], 0, 16U * sizeof(double));
          for (i1 = 0; i1 < 3; i1++) {
            loop_ub = i1 << 2;
            b_b[loop_ub] = R[3 * i1];
            b_b[loop_ub + 1] = R[3 * i1 + 1];
            b_b[loop_ub + 2] = R[3 * i1 + 2];
          }
          b_b[15] = 1.0;
        } break;
        case 1:
          joint->get_JointAxis(v);
          std::memset(&R[0], 0, 9U * sizeof(double));
          R[0] = 1.0;
          R[4] = 1.0;
          R[8] = 1.0;
          for (i1 = 0; i1 < 3; i1++) {
            loop_ub = i1 << 2;
            b_b[loop_ub] = R[3 * i1];
            b_b[loop_ub + 1] = R[3 * i1 + 1];
            b_b[loop_ub + 2] = R[3 * i1 + 2];
            b_b[i1 + 12] = v[i1] * 0.0;
          }
          b_b[3] = 0.0;
          b_b[7] = 0.0;
          b_b[11] = 0.0;
          b_b[15] = 1.0;
          break;
        case 2:
          // A check that is always false is detected at compile-time.
          // Eliminating code that follows.
          break;
        default:
          std::memset(&b_b[0], 0, 16U * sizeof(double));
          b_b[0] = 1.0;
          b_b[5] = 1.0;
          b_b[10] = 1.0;
          b_b[15] = 1.0;
          break;
        }
        for (i1 = 0; i1 < 16; i1++) {
          Tj[i1] = joint->ChildToJointTransform[i1];
        }
        for (i1 = 0; i1 < 4; i1++) {
          s = Tj1[i1];
          tempR_tmp = Tj1[i1 + 4];
          b_tempR_tmp = Tj1[i1 + 8];
          c_tempR_tmp = Tj1[i1 + 12];
          for (i2 = 0; i2 < 4; i2++) {
            coffset_tmp = i2 << 2;
            T1j[i1 + coffset_tmp] =
                ((s * b_b[coffset_tmp] + tempR_tmp * b_b[coffset_tmp + 1]) +
                 b_tempR_tmp * b_b[coffset_tmp + 2]) +
                c_tempR_tmp * b_b[coffset_tmp + 3];
          }
          s = T1j[i1];
          tempR_tmp = T1j[i1 + 4];
          b_tempR_tmp = T1j[i1 + 8];
          c_tempR_tmp = T1j[i1 + 12];
          for (i2 = 0; i2 < 4; i2++) {
            coffset_tmp = i2 << 2;
            Tc2p[i1 + coffset_tmp] =
                ((s * Tj[coffset_tmp] + tempR_tmp * Tj[coffset_tmp + 1]) +
                 b_tempR_tmp * Tj[coffset_tmp + 2]) +
                c_tempR_tmp * Tj[coffset_tmp + 3];
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
          i1 = 0;
          i2 = 0;
        } else {
          i1 = static_cast<int>(qidx_idx_0) - 1;
          i2 = static_cast<int>(qidx_idx_1);
        }
        loop_ub = i2 - i1;
        b_qv.set_size(loop_ub);
        for (i2 = 0; i2 < loop_ub; i2++) {
          b_qv[i2] = qv[i1 + i2];
        }
        joint->transformBodyToParent(b_qv, Tc2p);
        loop_ub = static_cast<int>(body2->Index);
        qidx_idx_0 = VelocityDoFMap[loop_ub - 1];
        qidx_idx_1 = VelocityDoFMap[loop_ub + 10];
        if (nextBodyIsParent) {
          for (i1 = 0; i1 < 16; i1++) {
            Tj[i1] = joint->ChildToJointTransform[i1];
          }
        } else {
          for (i1 = 0; i1 < 16; i1++) {
            Tj1[i1] = joint->JointToParentTransform[i1];
          }
          for (i1 = 0; i1 < 3; i1++) {
            R[3 * i1] = Tj1[i1];
            R[3 * i1 + 1] = Tj1[i1 + 4];
            R[3 * i1 + 2] = Tj1[i1 + 8];
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
          for (i1 = 0; i1 < 3; i1++) {
            loop_ub = i1 << 2;
            Tj[loop_ub] = R[3 * i1];
            Tj[loop_ub + 1] = R[3 * i1 + 1];
            Tj[loop_ub + 2] = R[3 * i1 + 2];
            Tj[i1 + 12] = (tempR[i1] * s + tempR[i1 + 3] * tempR_tmp) +
                          tempR[i1 + 6] * b_tempR_tmp;
          }
          Tj[3] = 0.0;
          Tj[7] = 0.0;
          Tj[11] = 0.0;
          Tj[15] = 1.0;
        }
        for (i1 = 0; i1 < 4; i1++) {
          s = Tj[i1];
          tempR_tmp = Tj[i1 + 4];
          b_tempR_tmp = Tj[i1 + 8];
          c_tempR_tmp = Tj[i1 + 12];
          for (i2 = 0; i2 < 4; i2++) {
            coffset_tmp = i2 << 2;
            T1j[i1 + coffset_tmp] =
                ((s * T1[coffset_tmp] + tempR_tmp * T1[coffset_tmp + 1]) +
                 b_tempR_tmp * T1[coffset_tmp + 2]) +
                c_tempR_tmp * T1[coffset_tmp + 3];
          }
        }
        for (i1 = 0; i1 < 3; i1++) {
          R[3 * i1] = T1j[i1];
          R[3 * i1 + 1] = T1j[i1 + 4];
          R[3 * i1 + 2] = T1j[i1 + 8];
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
        for (i1 = 0; i1 < 3; i1++) {
          loop_ub = i1 << 2;
          Tj1[loop_ub] = R[3 * i1];
          Tj1[loop_ub + 1] = R[3 * i1 + 1];
          Tj1[loop_ub + 2] = R[3 * i1 + 2];
          Tj1[i1 + 12] = (tempR[i1] * s + tempR[i1 + 3] * tempR_tmp) +
                         tempR[i1 + 6] * b_tempR_tmp;
        }
        int b_iv[2];
        Tj1[3] = 0.0;
        Tj1[7] = 0.0;
        Tj1[11] = 0.0;
        Tj1[15] = 1.0;
        b.reserve(36);
        joint->get_MotionSubspace((double *)b.data(), b_iv);
        (*(int(*)[2])b.size())[0] = b_iv[0];
        (*(int(*)[2])b.size())[1] = b_iv[1];
        b.set_size(b.size(0), b.size(1));
        if (qidx_idx_0 > qidx_idx_1) {
          i1 = 0;
          i2 = 0;
        } else {
          i1 = static_cast<int>(qidx_idx_0) - 1;
          i2 = static_cast<int>(qidx_idx_1);
        }
        R[0] = 0.0;
        R[3] = -Tj1[14];
        R[6] = Tj1[13];
        R[1] = Tj1[14];
        R[4] = 0.0;
        R[7] = -Tj1[12];
        R[2] = -Tj1[13];
        R[5] = Tj1[12];
        R[8] = 0.0;
        for (coffset_tmp = 0; coffset_tmp < 3; coffset_tmp++) {
          s = R[coffset_tmp];
          tempR_tmp = R[coffset_tmp + 3];
          b_tempR_tmp = R[coffset_tmp + 6];
          for (A_tmp = 0; A_tmp < 3; A_tmp++) {
            loop_ub = A_tmp << 2;
            tempR[coffset_tmp + 3 * A_tmp] =
                (s * Tj1[loop_ub] + tempR_tmp * Tj1[loop_ub + 1]) +
                b_tempR_tmp * Tj1[loop_ub + 2];
            A[A_tmp + 6 * coffset_tmp] = Tj1[A_tmp + (coffset_tmp << 2)];
            A[A_tmp + 6 * (coffset_tmp + 3)] = 0.0;
          }
        }
        for (coffset_tmp = 0; coffset_tmp < 3; coffset_tmp++) {
          A[6 * coffset_tmp + 3] = tempR[3 * coffset_tmp];
          loop_ub = coffset_tmp << 2;
          A_tmp = 6 * (coffset_tmp + 3);
          A[A_tmp + 3] = Tj1[loop_ub];
          A[6 * coffset_tmp + 4] = tempR[3 * coffset_tmp + 1];
          A[A_tmp + 4] = Tj1[loop_ub + 1];
          A[6 * coffset_tmp + 5] = tempR[3 * coffset_tmp + 2];
          A[A_tmp + 5] = Tj1[loop_ub + 2];
        }
        loop_ub = b.size(1);
        C.set_size(6, b.size(1));
        for (int j{0}; j < loop_ub; j++) {
          coffset_tmp = j * 6;
          for (A_tmp = 0; A_tmp < 6; A_tmp++) {
            s = 0.0;
            for (int k{0}; k < 6; k++) {
              s += A[k * 6 + A_tmp] * b[coffset_tmp + k];
            }
            C[coffset_tmp + A_tmp] = s;
          }
        }
        loop_ub = i2 - i1;
        for (i2 = 0; i2 < loop_ub; i2++) {
          r = _mm_loadu_pd(&C[6 * i2]);
          coffset_tmp = i1 + i2;
          r1 = _mm_set1_pd(static_cast<double>(jointSign));
          _mm_storeu_pd(&Jac[6 * coffset_tmp], _mm_mul_pd(r, r1));
          r = _mm_loadu_pd(&C[6 * i2 + 2]);
          _mm_storeu_pd(&Jac[6 * coffset_tmp + 2], _mm_mul_pd(r, r1));
          r = _mm_loadu_pd(&C[6 * i2 + 4]);
          _mm_storeu_pd(&Jac[6 * coffset_tmp + 4], _mm_mul_pd(r, r1));
        }
      }
      if (nextBodyIsParent) {
        for (i1 = 0; i1 < 4; i1++) {
          s = Tc2p[i1];
          tempR_tmp = Tc2p[i1 + 4];
          b_tempR_tmp = Tc2p[i1 + 8];
          c_tempR_tmp = Tc2p[i1 + 12];
          for (i2 = 0; i2 < 4; i2++) {
            coffset_tmp = i2 << 2;
            T1j[i1 + coffset_tmp] =
                ((s * T1[coffset_tmp] + tempR_tmp * T1[coffset_tmp + 1]) +
                 b_tempR_tmp * T1[coffset_tmp + 2]) +
                c_tempR_tmp * T1[coffset_tmp + 3];
          }
        }
        ::std::copy(&T1j[0], &T1j[16], &T1[0]);
      } else {
        for (i1 = 0; i1 < 3; i1++) {
          R[3 * i1] = Tc2p[i1];
          R[3 * i1 + 1] = Tc2p[i1 + 4];
          R[3 * i1 + 2] = Tc2p[i1 + 8];
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
        for (i1 = 0; i1 < 3; i1++) {
          loop_ub = i1 << 2;
          T1j[loop_ub] = R[3 * i1];
          T1j[loop_ub + 1] = R[3 * i1 + 1];
          T1j[loop_ub + 2] = R[3 * i1 + 2];
          T1j[i1 + 12] = (tempR[i1] * s + tempR[i1 + 3] * tempR_tmp) +
                         tempR[i1 + 6] * b_tempR_tmp;
        }
        T1j[3] = 0.0;
        T1j[7] = 0.0;
        T1j[11] = 0.0;
        T1j[15] = 1.0;
        for (i1 = 0; i1 < 4; i1++) {
          s = T1j[i1];
          tempR_tmp = T1j[i1 + 4];
          b_tempR_tmp = T1j[i1 + 8];
          c_tempR_tmp = T1j[i1 + 12];
          for (i2 = 0; i2 < 4; i2++) {
            coffset_tmp = i2 << 2;
            Tj1[i1 + coffset_tmp] =
                ((s * T1[coffset_tmp] + tempR_tmp * T1[coffset_tmp + 1]) +
                 b_tempR_tmp * T1[coffset_tmp + 2]) +
                c_tempR_tmp * T1[coffset_tmp + 3];
          }
        }
        ::std::copy(&Tj1[0], &Tj1[16], &T1[0]);
      }
    }
    for (i = 0; i < 3; i++) {
      i1 = i << 2;
      s = T1[i1];
      A[6 * i] = s;
      loop_ub = 6 * (i + 3);
      A[loop_ub] = 0.0;
      A[6 * i + 3] = 0.0;
      A[loop_ub + 3] = s;
      s = T1[i1 + 1];
      A[6 * i + 1] = s;
      A[loop_ub + 1] = 0.0;
      A[6 * i + 4] = 0.0;
      A[loop_ub + 4] = s;
      s = T1[i1 + 2];
      A[6 * i + 2] = s;
      A[loop_ub + 2] = 0.0;
      A[6 * i + 5] = 0.0;
      A[loop_ub + 5] = s;
    }
    A_tmp = Jac.size(1);
    b.set_size(6, Jac.size(1));
    loop_ub = 6 * Jac.size(1);
    for (i = 0; i < loop_ub; i++) {
      b[i] = Jac[i];
    }
    Jac.set_size(6, A_tmp);
    for (int j{0}; j < A_tmp; j++) {
      coffset_tmp = j * 6;
      for (int b_i{0}; b_i < 6; b_i++) {
        s = 0.0;
        for (int k{0}; k < 6; k++) {
          s += A[k * 6 + b_i] * b[coffset_tmp + k];
        }
        Jac[coffset_tmp + b_i] = s;
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
double b_RigidBodyTree::findBodyIndexByJointName(const char jointname_data[],
                                                 const int jointname_size[2])
{
  CharacterVector b_obj;
  RigidBody *obj;
  rigidBodyJoint *jnt;
  ::coder::array<char, 2U> b_jointname_data;
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
    b_jointname_data.set((char *)&jointname_data[0], jointname_size[0],
                         jointname_size[1]);
    if (::gik9dof::coder::internal::b_strcmp(obj_data, obj_size,
                                             b_jointname_data)) {
      bid = static_cast<double>(i) + 1.0;
      exitg1 = true;
    } else {
      i++;
    }
  }
  return bid;
}

//
// Arguments    : const ::coder::array<char, 2U> &bodyname
// Return Type  : double
//
double
b_RigidBodyTree::findBodyIndexByName(const ::coder::array<char, 2U> &bodyname)
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
  if (::gik9dof::coder::internal::b_strcmp(obj_data, obj_size, bodyname)) {
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
      if (::gik9dof::coder::internal::b_strcmp(obj_data, obj_size, bodyname)) {
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
// Arguments    : ::coder::array<double, 2U> &limits
// Return Type  : void
//
void b_RigidBodyTree::get_JointPositionLimits(
    ::coder::array<double, 2U> &limits)
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
// Arguments    : GIKSolver *aInstancePtr
// Return Type  : RigidBodyTree *
//
RigidBodyTree *RigidBodyTree::init(GIKSolver *aInstancePtr)
{
  static const char jname[14]{'d', 'u', 'm', 'm', 'y', 'b', 'o',
                              'd', 'y', '1', '_', 'j', 'n', 't'};
  CharacterVector b_obj;
  RigidBodyTree *obj;
  double unusedExpr[5];
  obj = this;
  b_rand(aInstancePtr, unusedExpr);
  for (int i{0}; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->Base.NameInternal = b_obj;
  b_obj = obj->Base.NameInternal;
  b_obj.Length = 4.0;
  b_obj.Vector[0] = 'b';
  b_obj.Vector[1] = 'a';
  b_obj.Vector[2] = 's';
  b_obj.Vector[3] = 'e';
  obj->Base.NameInternal = b_obj;
  obj->Base.JointInternal.init();
  obj->Base.CollisionsInternal.init(static_cast<double>(0.0));
  obj->Base.matlabCodegenIsDeleted = false;
  for (int i{0}; i < 200; i++) {
    b_obj.Vector[i] = ' ';
  }
  b_obj.Length = 200.0;
  obj->_pobj0.NameInternal = b_obj;
  b_obj = obj->_pobj0.NameInternal;
  b_obj.Length = 10.0;
  for (int i{0}; i < 10; i++) {
    b_obj.Vector[i] = cv5[i];
  }
  obj->_pobj0.NameInternal = b_obj;
  obj->_pobj0.JointInternal.init(jname);
  obj->_pobj0.CollisionsInternal.init(static_cast<double>(0.0));
  obj->_pobj0.matlabCodegenIsDeleted = false;
  obj->Bodies[0] = &obj->_pobj0;
  b_rand(aInstancePtr, unusedExpr);
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : GIKSolver *aInstancePtr
// Return Type  : b_RigidBodyTree *
//
b_RigidBodyTree *b_RigidBodyTree::init(GIKSolver *aInstancePtr)
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
  CharacterVector b_obj;
  b_RigidBodyTree *obj;
  double unusedExpr[5];
  signed char c_I[36];
  signed char b_I[9];
  obj = this;
  b_rand(aInstancePtr, unusedExpr);
  for (int k{0}; k < 200; k++) {
    b_obj.Vector[k] = ' ';
  }
  b_obj.Length = 200.0;
  obj->Base.NameInternal = b_obj;
  b_obj = obj->Base.NameInternal;
  b_obj.Length = 4.0;
  b_obj.Vector[0] = 'b';
  b_obj.Vector[1] = 'a';
  b_obj.Vector[2] = 's';
  b_obj.Vector[3] = 'e';
  obj->Base.NameInternal = b_obj;
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
  obj->Bodies[0] = (&(&obj->_pobj2[0])[0])[0].init(cv5);
  obj->Bodies[1] = (&(&obj->_pobj2[0])[0])[1].init(b_cv);
  obj->Bodies[2] = (&(&obj->_pobj2[0])[0])[2].init(b_cv1);
  obj->Bodies[3] = (&(&obj->_pobj2[0])[0])[3].init(b_cv2);
  obj->Bodies[4] = (&(&obj->_pobj2[0])[0])[4].init(b_cv3);
  obj->Bodies[5] = (&(&obj->_pobj2[0])[0])[5].init(b_cv4);
  obj->Bodies[6] = (&(&obj->_pobj2[0])[0])[6].init(b_cv5);
  obj->Bodies[7] = (&(&obj->_pobj2[0])[0])[7].init(b_cv6);
  obj->Bodies[8] = (&(&obj->_pobj2[0])[0])[8].init(b_cv7);
  obj->Bodies[9] = (&(&obj->_pobj2[0])[0])[9].init(
      b_cv8, (&(&obj->_pobj0[0])[0])[0], (&(&obj->_pobj1[0])[0])[0]);
  obj->Bodies[10] = (&(&obj->_pobj2[0])[0])[10].init(
      b_cv9, (&(&obj->_pobj0[0])[0])[1], (&(&obj->_pobj1[0])[0])[1]);
  obj->NumBodies = 0.0;
  obj->NumNonFixedBodies = 0.0;
  obj->PositionNumber = 0.0;
  obj->VelocityNumber = 0.0;
  b_rand(aInstancePtr, unusedExpr);
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
//                ::coder::array<double, 2U> &indices
// Return Type  : void
//
void b_RigidBodyTree::kinematicPath(const char body1Name_data[],
                                    const int body1Name_size[2],
                                    const char body2Name_data[],
                                    const int body2Name_size[2],
                                    ::coder::array<double, 2U> &indices)
{
  RigidBody *body1;
  RigidBody *body2;
  ::coder::array<char, 2U> b_body1Name_data;
  ::coder::array<char, 2U> b_body2Name_data;
  double bid1;
  double bid2;
  b_body1Name_data.set((char *)&body1Name_data[0], body1Name_size[0],
                       body1Name_size[1]);
  bid1 = findBodyIndexByName(b_body1Name_data);
  b_body2Name_data.set((char *)&body2Name_data[0], body2Name_size[0],
                       body2Name_size[1]);
  bid2 = findBodyIndexByName(b_body2Name_data);
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
void RigidBodyTree::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

//
// Arguments    : GIKSolver *aInstancePtr
//                double q_data[]
// Return Type  : int
//
int b_RigidBodyTree::randomJointPositions(GIKSolver *aInstancePtr,
                                          double q_data[])
{
  static const char b_cv[5]{'f', 'i', 'x', 'e', 'd'};
  rigidBodyJoint *obj;
  ::coder::array<double, 1U> b_x;
  ::coder::array<double, 1U> y;
  ::coder::array<boolean_T, 1U> x;
  double qi_data[49];
  double bounds_data[14];
  double translbounds[6];
  double rn[3];
  double dv[2];
  double dv1[2];
  double posnum;
  int bounds_size[2];
  int i;
  int loop_ub_tmp;
  int q_size;
  posnum = PositionNumber;
  loop_ub_tmp = static_cast<int>(posnum);
  q_size = static_cast<int>(posnum);
  if (loop_ub_tmp - 1 >= 0) {
    std::memset(&q_data[0], 0,
                static_cast<unsigned int>(loop_ub_tmp) * sizeof(double));
  }
  posnum = NumBodies;
  i = static_cast<int>(posnum);
  for (int b_i{0}; b_i < i; b_i++) {
    double p_idx_0;
    double p_idx_1;
    p_idx_0 = PositionDoFMap[b_i];
    p_idx_1 = PositionDoFMap[b_i + 11];
    if (p_idx_0 <= p_idx_1) {
      CharacterVector b_obj;
      int i1;
      int i2;
      int kstr;
      signed char unnamed_idx_1;
      boolean_T b_bool;
      obj = Bodies[b_i]->JointInternal;
      b_obj = obj->TypeInternal;
      if (b_obj.Length < 1.0) {
        i1 = 0;
      } else {
        i1 = static_cast<int>(b_obj.Length);
      }
      b_bool = false;
      if (i1 == 5) {
        kstr = 0;
        int exitg1;
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
        posnum = obj->PositionNumber;
        if (posnum < 1.0) {
          unnamed_idx_1 = 0;
        } else {
          unnamed_idx_1 = static_cast<signed char>(static_cast<int>(posnum));
        }
      } else {
        unnamed_idx_1 = 1;
      }
      kstr = unnamed_idx_1;
      if (kstr - 1 >= 0) {
        std::memset(&qi_data[0], 0,
                    static_cast<unsigned int>(kstr) * sizeof(double));
      }
      switch (static_cast<int>(obj->PositionNumber)) {
      case 0:
        qi_data[0] = rtNaN;
        break;
      case 7: {
        __m128d b_r;
        __m128d r1;
        double r[4];
        double c_x[3];
        int k;
        boolean_T b[3];
        boolean_T b_b[3];
        boolean_T bv[3];
        boolean_T exitg2;
        boolean_T guard1;
        boolean_T guard2;
        boolean_T guard3;
        randn(aInstancePtr, r);
        posnum = std::sqrt(((r[0] * r[0] + r[1] * r[1]) + r[2] * r[2]) +
                           r[3] * r[3]);
        qi_data[0] = r[0] / posnum;
        qi_data[1] = r[1] / posnum;
        qi_data[2] = r[2] / posnum;
        qi_data[3] = r[3] / posnum;
        for (i1 = 0; i1 < 2; i1++) {
          translbounds[3 * i1] = obj->PositionLimitsInternal[7 * i1 + 4];
          translbounds[3 * i1 + 1] = obj->PositionLimitsInternal[7 * i1 + 5];
          translbounds[3 * i1 + 2] = obj->PositionLimitsInternal[7 * i1 + 6];
        }
        if (unnamed_idx_1 < 5) {
          i1 = 0;
          i2 = 0;
        } else {
          i1 = 4;
          i2 = unnamed_idx_1;
        }
        bv[0] =
            ((!std::isinf(translbounds[0])) && (!std::isnan(translbounds[0])));
        bv[1] =
            ((!std::isinf(translbounds[1])) && (!std::isnan(translbounds[1])));
        bv[2] =
            ((!std::isinf(translbounds[2])) && (!std::isnan(translbounds[2])));
        b_bool = true;
        k = 0;
        exitg2 = false;
        while ((!exitg2) && (k <= 2)) {
          if (!bv[k]) {
            b_bool = false;
            exitg2 = true;
          } else {
            k++;
          }
        }
        guard1 = false;
        guard2 = false;
        guard3 = false;
        if (b_bool) {
          b[0] = std::isinf(translbounds[3]);
          b_b[0] = std::isnan(translbounds[3]);
          b[1] = std::isinf(translbounds[4]);
          b_b[1] = std::isnan(translbounds[4]);
          b[2] = std::isinf(translbounds[5]);
          b_b[2] = std::isnan(translbounds[5]);
          b_bool = true;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k <= 2)) {
            if (b[k] || b_b[k]) {
              b_bool = false;
              exitg2 = true;
            } else {
              k++;
            }
          }
          if (b_bool) {
            __m128d r2;
            c_rand(aInstancePtr, rn);
            b_r = _mm_loadu_pd(&translbounds[3]);
            r1 = _mm_loadu_pd(&translbounds[0]);
            r2 = _mm_loadu_pd(&rn[0]);
            _mm_storeu_pd(&rn[0],
                          _mm_add_pd(r1, _mm_mul_pd(r2, _mm_sub_pd(b_r, r1))));
            rn[2] =
                translbounds[2] + rn[2] * (translbounds[5] - translbounds[2]);
          } else {
            guard3 = true;
          }
        } else {
          guard3 = true;
        }
        if (guard3) {
          b_bool = true;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k <= 2)) {
            if (!bv[k]) {
              b_bool = false;
              exitg2 = true;
            } else {
              k++;
            }
          }
          if (b_bool) {
            b[0] = (std::isinf(translbounds[3]) || std::isnan(translbounds[3]));
            b[1] = (std::isinf(translbounds[4]) || std::isnan(translbounds[4]));
            b[2] = (std::isinf(translbounds[5]) || std::isnan(translbounds[5]));
            b_bool = false;
            k = 0;
            exitg2 = false;
            while ((!exitg2) && (k <= 2)) {
              if (b[k]) {
                b_bool = true;
                exitg2 = true;
              } else {
                k++;
              }
            }
            if (b_bool) {
              b_randn(aInstancePtr, c_x);
              dv[0] = std::abs(c_x[0]);
              dv[1] = std::abs(c_x[1]);
              b_r = _mm_loadu_pd(&translbounds[0]);
              r1 = _mm_loadu_pd(&dv[0]);
              _mm_storeu_pd(&rn[0], _mm_add_pd(b_r, r1));
              rn[2] = translbounds[2] + std::abs(c_x[2]);
            } else {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }
        }
        if (guard2) {
          b_bool = false;
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k <= 2)) {
            if (!bv[k]) {
              b_bool = true;
              exitg2 = true;
            } else {
              k++;
            }
          }
          if (b_bool) {
            b[0] = std::isinf(translbounds[3]);
            b_b[0] = std::isnan(translbounds[3]);
            b[1] = std::isinf(translbounds[4]);
            b_b[1] = std::isnan(translbounds[4]);
            b[2] = std::isinf(translbounds[5]);
            b_b[2] = std::isnan(translbounds[5]);
            b_bool = true;
            k = 0;
            exitg2 = false;
            while ((!exitg2) && (k <= 2)) {
              if (b[k] || b_b[k]) {
                b_bool = false;
                exitg2 = true;
              } else {
                k++;
              }
            }
            if (b_bool) {
              b_randn(aInstancePtr, c_x);
              dv1[0] = std::abs(c_x[0]);
              dv1[1] = std::abs(c_x[1]);
              b_r = _mm_loadu_pd(&translbounds[3]);
              r1 = _mm_loadu_pd(&dv1[0]);
              _mm_storeu_pd(&rn[0], _mm_sub_pd(b_r, r1));
              rn[2] = translbounds[5] - std::abs(c_x[2]);
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        }
        if (guard1) {
          b_randn(aInstancePtr, rn);
        }
        kstr = i2 - i1;
        for (i2 = 0; i2 < kstr; i2++) {
          qi_data[i1 + i2] = rn[i2];
        }
      } break;
      default: {
        __m128d b_r;
        __m128d r1;
        double bounds[2];
        int k;
        boolean_T x_data[7];
        boolean_T exitg2;
        boolean_T guard1;
        boolean_T guard2;
        boolean_T guard3;
        posnum = obj->PositionNumber;
        if (posnum < 1.0) {
          loop_ub_tmp = 0;
        } else {
          loop_ub_tmp = static_cast<int>(posnum);
        }
        bounds_size[0] = loop_ub_tmp;
        bounds_size[1] = 2;
        for (i1 = 0; i1 < 2; i1++) {
          for (i2 = 0; i2 < loop_ub_tmp; i2++) {
            bounds_data[i2 + loop_ub_tmp * i1] =
                obj->PositionLimitsInternal[i2 + 7 * i1];
          }
        }
        x.set_size(loop_ub_tmp);
        for (i1 = 0; i1 < loop_ub_tmp; i1++) {
          posnum = bounds_data[i1];
          x[i1] = ((!std::isinf(posnum)) && (!std::isnan(posnum)));
        }
        b_bool = true;
        kstr = 1;
        exitg2 = false;
        while ((!exitg2) && (kstr <= x.size(0))) {
          if (!x[kstr - 1]) {
            b_bool = false;
            exitg2 = true;
          } else {
            kstr++;
          }
        }
        guard1 = false;
        guard2 = false;
        guard3 = false;
        if (b_bool) {
          x.set_size(loop_ub_tmp);
          for (i1 = 0; i1 < loop_ub_tmp; i1++) {
            posnum = bounds_data[i1 + loop_ub_tmp];
            x[i1] = ((!std::isinf(posnum)) && (!std::isnan(posnum)));
          }
          b_bool = true;
          kstr = 1;
          exitg2 = false;
          while ((!exitg2) && (kstr <= x.size(0))) {
            if (!x[kstr - 1]) {
              b_bool = false;
              exitg2 = true;
            } else {
              kstr++;
            }
          }
          if (b_bool) {
            b_x.reserve(7);
            i1 = b_rand(aInstancePtr, static_cast<double>(loop_ub_tmp),
                        (double *)b_x.data());
            (*(int(*)[1])b_x.size())[0] = i1;
            b_x.set_size(b_x.size(0));
            if (b_x.size(0) == 1) {
              i1 = loop_ub_tmp;
            } else {
              i1 = b_x.size(0);
            }
            if ((b_x.size(0) == loop_ub_tmp) && (loop_ub_tmp == i1)) {
              b_x.set_size(loop_ub_tmp);
              for (i1 = 0; i1 < loop_ub_tmp; i1++) {
                posnum = bounds_data[i1];
                b_x[i1] =
                    posnum + b_x[i1] * (bounds_data[i1 + loop_ub_tmp] - posnum);
              }
            } else {
              binary_expand_op_5(b_x, bounds_data, bounds_size);
            }
          } else {
            guard3 = true;
          }
        } else {
          guard3 = true;
        }
        if (guard3) {
          x.set_size(loop_ub_tmp);
          for (i1 = 0; i1 < loop_ub_tmp; i1++) {
            posnum = bounds_data[i1];
            x[i1] = ((!std::isinf(posnum)) && (!std::isnan(posnum)));
          }
          b_bool = true;
          kstr = 1;
          exitg2 = false;
          while ((!exitg2) && (kstr <= x.size(0))) {
            if (!x[kstr - 1]) {
              b_bool = false;
              exitg2 = true;
            } else {
              kstr++;
            }
          }
          if (b_bool) {
            for (i1 = 0; i1 < loop_ub_tmp; i1++) {
              posnum = bounds_data[i1 + loop_ub_tmp];
              x_data[i1] = (std::isinf(posnum) || std::isnan(posnum));
            }
            b_bool = false;
            kstr = 1;
            exitg2 = false;
            while ((!exitg2) && (kstr <= loop_ub_tmp)) {
              if (x_data[kstr - 1]) {
                b_bool = true;
                exitg2 = true;
              } else {
                kstr++;
              }
            }
            if (b_bool) {
              bounds[0] = loop_ub_tmp;
              bounds[1] = 1.0;
              b_x.reserve(7);
              i1 = randn(aInstancePtr, bounds, (double *)b_x.data());
              (*(int(*)[1])b_x.size())[0] = i1;
              b_x.set_size(b_x.size(0));
              kstr = b_x.size(0);
              y.set_size(b_x.size(0));
              for (k = 0; k < kstr; k++) {
                y[k] = std::abs(b_x[k]);
              }
              if (loop_ub_tmp == y.size(0)) {
                b_x.set_size(loop_ub_tmp);
                kstr = (loop_ub_tmp / 2) << 1;
                k = kstr - 2;
                for (i1 = 0; i1 <= k; i1 += 2) {
                  b_r = _mm_loadu_pd(&bounds_data[i1]);
                  r1 = _mm_loadu_pd(&y[i1]);
                  _mm_storeu_pd(&b_x[i1], _mm_add_pd(b_r, r1));
                }
                for (i1 = kstr; i1 < loop_ub_tmp; i1++) {
                  b_x[i1] = bounds_data[i1] + y[i1];
                }
              } else {
                binary_expand_op_6(b_x, bounds_data, bounds_size, y);
              }
            } else {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }
        }
        if (guard2) {
          for (i1 = 0; i1 < loop_ub_tmp; i1++) {
            posnum = bounds_data[i1];
            x_data[i1] = (std::isinf(posnum) || std::isnan(posnum));
          }
          b_bool = false;
          kstr = 1;
          exitg2 = false;
          while ((!exitg2) && (kstr <= loop_ub_tmp)) {
            if (x_data[kstr - 1]) {
              b_bool = true;
              exitg2 = true;
            } else {
              kstr++;
            }
          }
          if (b_bool) {
            x.set_size(loop_ub_tmp);
            for (i1 = 0; i1 < loop_ub_tmp; i1++) {
              posnum = bounds_data[i1 + loop_ub_tmp];
              x[i1] = ((!std::isinf(posnum)) && (!std::isnan(posnum)));
            }
            b_bool = true;
            kstr = 1;
            exitg2 = false;
            while ((!exitg2) && (kstr <= x.size(0))) {
              if (!x[kstr - 1]) {
                b_bool = false;
                exitg2 = true;
              } else {
                kstr++;
              }
            }
            if (b_bool) {
              bounds[0] = loop_ub_tmp;
              bounds[1] = 1.0;
              b_x.reserve(7);
              i1 = randn(aInstancePtr, bounds, (double *)b_x.data());
              (*(int(*)[1])b_x.size())[0] = i1;
              b_x.set_size(b_x.size(0));
              kstr = b_x.size(0);
              y.set_size(b_x.size(0));
              for (k = 0; k < kstr; k++) {
                y[k] = std::abs(b_x[k]);
              }
              if (loop_ub_tmp == y.size(0)) {
                b_x.set_size(loop_ub_tmp);
                kstr = (loop_ub_tmp / 2) << 1;
                k = kstr - 2;
                for (i1 = 0; i1 <= k; i1 += 2) {
                  b_r = _mm_loadu_pd(&bounds_data[i1 + loop_ub_tmp]);
                  r1 = _mm_loadu_pd(&y[i1]);
                  _mm_storeu_pd(&b_x[i1], _mm_sub_pd(b_r, r1));
                }
                for (i1 = kstr; i1 < loop_ub_tmp; i1++) {
                  b_x[i1] = bounds_data[i1 + loop_ub_tmp] - y[i1];
                }
              } else {
                binary_expand_op_7(b_x, bounds_data, bounds_size, y);
              }
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
        }
        if (guard1) {
          bounds[0] = loop_ub_tmp;
          bounds[1] = 1.0;
          b_x.reserve(7);
          i1 = randn(aInstancePtr, bounds, (double *)b_x.data());
          (*(int(*)[1])b_x.size())[0] = i1;
          b_x.set_size(b_x.size(0));
        }
        kstr = b_x.size(0);
        for (i1 = 0; i1 < kstr; i1++) {
          qi_data[i1] = b_x[i1];
        }
      } break;
      }
      if (p_idx_0 > p_idx_1) {
        i1 = 0;
        i2 = 0;
      } else {
        i1 = static_cast<int>(p_idx_0) - 1;
        i2 = static_cast<int>(p_idx_1);
      }
      kstr = i2 - i1;
      for (i2 = 0; i2 < kstr; i2++) {
        q_data[i1 + i2] = qi_data[i2];
      }
    }
  }
  return q_size;
}

//
// Arguments    : boolean_T in1[9]
//                const double in2[9]
//                const ::coder::array<double, 2U> &in3
// Return Type  : void
//
} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
void binary_expand_op_29(boolean_T in1[9], const double in2[9],
                         const ::coder::array<double, 2U> &in3)
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
//                const ::coder::array<double, 2U> &in3
// Return Type  : void
//
void binary_expand_op_30(boolean_T in1[9], const double in2[9],
                         const ::coder::array<double, 2U> &in3)
{
  int stride_0_0;
  stride_0_0 = (in3.size(0) != 1);
  for (int i{0}; i < 9; i++) {
    in1[i] =
        (in2[i] <= in3[i * stride_0_0 + in3.size(0)] + 4.4408920985006262E-16);
  }
}

} // namespace gik9dof

//
// File trailer for RigidBodyTree.cpp
//
// [EOF]
//
