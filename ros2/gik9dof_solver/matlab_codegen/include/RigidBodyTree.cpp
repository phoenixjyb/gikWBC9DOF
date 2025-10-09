//
// File: RigidBodyTree.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

// Include Files
#include "RigidBodyTree.h"
#include "CharacterVector.h"
#include "CollisionSet.h"
#include "GIKSolver.h"
#include "RigidBody.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "rand.h"
#include "rigidBodyJoint.h"
#include "rt_nonfinite.h"
#include "strcmp.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Variable Definitions
namespace gik9dof {
static const char cv2[10]{'d', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y', '1'};

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
  bool exitg1;
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
  bool b_bool;
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
// Return Type  : b_RigidBodyTree
//
b_RigidBodyTree::b_RigidBodyTree() = default;

//
// Arguments    : void
// Return Type  : void
//
b_RigidBodyTree::~b_RigidBodyTree() = default;

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
  ::coder::array<double, 2U> B;
  ::coder::array<double, 2U> kinematicPathIndices;
  ::coder::array<double, 1U> b_qv;
  double b_data[36];
  double T1[16];
  double T1j[16];
  double Tj1[16];
  double b[16];
  double R[9];
  if ((bid1 >= 0.0) && (bid2 >= 0.0)) {
    double X[36];
    double s;
    int c_i;
    int i;
    int j;
    int loop_ub;
    int n;
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
    n = static_cast<int>(PositionNumber);
    Jac.set_size(6, n);
    loop_ub = 6 * static_cast<int>(PositionNumber);
    for (i = 0; i < loop_ub; i++) {
      Jac[i] = 0.0;
    }
    i = kinematicPathIndices.size(1);
    for (int b_i{0}; b_i <= i - 2; b_i++) {
      CharacterVector obj;
      double Tc2p[16];
      double tempR[9];
      double tempR_tmp;
      double v_idx_0;
      double v_idx_1;
      int X_tmp;
      int exitg1;
      int jointSign;
      bool b_bool;
      bool nextBodyIsParent;
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
                if (cv3[loop_ub] != obj.Vector[loop_ub]) {
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
          double v[3];
          double b_tempR_tmp;
          double c_tempR_tmp;
          double d_tempR_tmp;
          double e_tempR_tmp;
          double f_tempR_tmp;
          double qidx_idx_0;
          double qidx_idx_1;
          joint->get_JointAxis(v);
          s = 1.0 / std::sqrt((v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]);
          v_idx_0 = v[0] * s;
          v_idx_1 = v[1] * s;
          s *= v[2];
          tempR_tmp = v_idx_0 * v_idx_0 * 0.0 + 1.0;
          tempR[0] = tempR_tmp;
          qidx_idx_0 = v_idx_0 * v_idx_1 * 0.0;
          qidx_idx_1 = qidx_idx_0 - s * 0.0;
          tempR[1] = qidx_idx_1;
          b_tempR_tmp = v_idx_0 * s * 0.0;
          c_tempR_tmp = b_tempR_tmp + v_idx_1 * 0.0;
          tempR[2] = c_tempR_tmp;
          qidx_idx_0 += s * 0.0;
          tempR[3] = qidx_idx_0;
          d_tempR_tmp = v_idx_1 * v_idx_1 * 0.0 + 1.0;
          tempR[4] = d_tempR_tmp;
          e_tempR_tmp = v_idx_1 * s * 0.0;
          f_tempR_tmp = e_tempR_tmp - v_idx_0 * 0.0;
          tempR[5] = f_tempR_tmp;
          b_tempR_tmp -= v_idx_1 * 0.0;
          tempR[6] = b_tempR_tmp;
          e_tempR_tmp += v_idx_0 * 0.0;
          tempR[7] = e_tempR_tmp;
          s = s * s * 0.0 + 1.0;
          tempR[8] = s;
          R[0] = tempR_tmp;
          R[1] = qidx_idx_1;
          R[2] = c_tempR_tmp;
          R[3] = qidx_idx_0;
          R[4] = d_tempR_tmp;
          R[5] = f_tempR_tmp;
          R[6] = b_tempR_tmp;
          R[7] = e_tempR_tmp;
          R[8] = s;
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
        case 1: {
          double v[3];
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
        } break;
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
          v_idx_0 = Tj1[c_i + 4];
          v_idx_1 = Tj1[c_i + 8];
          tempR_tmp = Tj1[c_i + 12];
          for (X_tmp = 0; X_tmp < 4; X_tmp++) {
            j = X_tmp << 2;
            T1j[c_i + j] =
                ((s * b[j] + v_idx_0 * b[j + 1]) + v_idx_1 * b[j + 2]) +
                tempR_tmp * b[j + 3];
          }
          s = T1j[c_i];
          v_idx_0 = T1j[c_i + 4];
          v_idx_1 = T1j[c_i + 8];
          tempR_tmp = T1j[c_i + 12];
          for (X_tmp = 0; X_tmp < 4; X_tmp++) {
            j = X_tmp << 2;
            Tc2p[c_i + j] =
                ((s * Tj[j] + v_idx_0 * Tj[j + 1]) + v_idx_1 * Tj[j + 2]) +
                tempR_tmp * Tj[j + 3];
          }
        }
      } else {
        double y_data[36];
        double Tj[16];
        double qidx_idx_0;
        double qidx_idx_1;
        loop_ub = static_cast<int>(body2->Index);
        qidx_idx_0 = PositionDoFMap[loop_ub - 1];
        qidx_idx_1 = PositionDoFMap[loop_ub + 10];
        if (qidx_idx_0 > qidx_idx_1) {
          c_i = 0;
          X_tmp = 0;
        } else {
          c_i = static_cast<int>(qidx_idx_0) - 1;
          X_tmp = static_cast<int>(qidx_idx_1);
        }
        loop_ub = X_tmp - c_i;
        b_qv.set_size(loop_ub);
        for (X_tmp = 0; X_tmp < loop_ub; X_tmp++) {
          b_qv[X_tmp] = qv[c_i + X_tmp];
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
          for (c_i = 0; c_i < 9; c_i++) {
            tempR[c_i] = -R[c_i];
          }
          s = Tj1[12];
          v_idx_0 = Tj1[13];
          v_idx_1 = Tj1[14];
          for (c_i = 0; c_i < 3; c_i++) {
            loop_ub = c_i << 2;
            Tj[loop_ub] = R[3 * c_i];
            Tj[loop_ub + 1] = R[3 * c_i + 1];
            Tj[loop_ub + 2] = R[3 * c_i + 2];
            Tj[c_i + 12] = (tempR[c_i] * s + tempR[c_i + 3] * v_idx_0) +
                           tempR[c_i + 6] * v_idx_1;
          }
          Tj[3] = 0.0;
          Tj[7] = 0.0;
          Tj[11] = 0.0;
          Tj[15] = 1.0;
        }
        for (c_i = 0; c_i < 4; c_i++) {
          s = Tj[c_i];
          v_idx_0 = Tj[c_i + 4];
          v_idx_1 = Tj[c_i + 8];
          tempR_tmp = Tj[c_i + 12];
          for (X_tmp = 0; X_tmp < 4; X_tmp++) {
            j = X_tmp << 2;
            T1j[c_i + j] =
                ((s * T1[j] + v_idx_0 * T1[j + 1]) + v_idx_1 * T1[j + 2]) +
                tempR_tmp * T1[j + 3];
          }
        }
        for (c_i = 0; c_i < 3; c_i++) {
          R[3 * c_i] = T1j[c_i];
          R[3 * c_i + 1] = T1j[c_i + 4];
          R[3 * c_i + 2] = T1j[c_i + 8];
        }
        for (c_i = 0; c_i < 9; c_i++) {
          tempR[c_i] = -R[c_i];
        }
        s = T1j[12];
        v_idx_0 = T1j[13];
        v_idx_1 = T1j[14];
        for (c_i = 0; c_i < 3; c_i++) {
          loop_ub = c_i << 2;
          Tj1[loop_ub] = R[3 * c_i];
          Tj1[loop_ub + 1] = R[3 * c_i + 1];
          Tj1[loop_ub + 2] = R[3 * c_i + 2];
          Tj1[c_i + 12] = (tempR[c_i] * s + tempR[c_i + 3] * v_idx_0) +
                          tempR[c_i + 6] * v_idx_1;
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
          v_idx_0 = R[c_i + 3];
          v_idx_1 = R[c_i + 6];
          for (X_tmp = 0; X_tmp < 3; X_tmp++) {
            j = X_tmp << 2;
            tempR[c_i + 3 * X_tmp] =
                (s * Tj1[j] + v_idx_0 * Tj1[j + 1]) + v_idx_1 * Tj1[j + 2];
            X[X_tmp + 6 * c_i] = Tj1[X_tmp + (c_i << 2)];
            X[X_tmp + 6 * (c_i + 3)] = 0.0;
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
        X_tmp = b_size[1];
        for (j = 0; j < X_tmp; j++) {
          loop_ub = j * 6;
          for (c_i = 0; c_i < 6; c_i++) {
            s = 0.0;
            for (int k{0}; k < 6; k++) {
              s += X[k * 6 + c_i] * b_data[loop_ub + k];
            }
            y_data[loop_ub + c_i] = s;
          }
        }
        if (qidx_idx_0 > qidx_idx_1) {
          c_i = 0;
          X_tmp = 0;
        } else {
          c_i = static_cast<int>(qidx_idx_0) - 1;
          X_tmp = static_cast<int>(qidx_idx_1);
        }
        loop_ub = X_tmp - c_i;
        for (X_tmp = 0; X_tmp < loop_ub; X_tmp++) {
          for (j = 0; j < 6; j++) {
            Jac[j + 6 * (c_i + X_tmp)] =
                y_data[j + 6 * X_tmp] * static_cast<double>(jointSign);
          }
        }
      }
      if (nextBodyIsParent) {
        for (c_i = 0; c_i < 4; c_i++) {
          s = Tc2p[c_i];
          v_idx_0 = Tc2p[c_i + 4];
          v_idx_1 = Tc2p[c_i + 8];
          tempR_tmp = Tc2p[c_i + 12];
          for (X_tmp = 0; X_tmp < 4; X_tmp++) {
            j = X_tmp << 2;
            T1j[c_i + j] =
                ((s * T1[j] + v_idx_0 * T1[j + 1]) + v_idx_1 * T1[j + 2]) +
                tempR_tmp * T1[j + 3];
          }
        }
        ::std::copy(&T1j[0], &T1j[16], &T1[0]);
      } else {
        for (c_i = 0; c_i < 3; c_i++) {
          R[3 * c_i] = Tc2p[c_i];
          R[3 * c_i + 1] = Tc2p[c_i + 4];
          R[3 * c_i + 2] = Tc2p[c_i + 8];
        }
        for (c_i = 0; c_i < 9; c_i++) {
          tempR[c_i] = -R[c_i];
        }
        s = Tc2p[12];
        v_idx_0 = Tc2p[13];
        v_idx_1 = Tc2p[14];
        for (c_i = 0; c_i < 3; c_i++) {
          loop_ub = c_i << 2;
          T1j[loop_ub] = R[3 * c_i];
          T1j[loop_ub + 1] = R[3 * c_i + 1];
          T1j[loop_ub + 2] = R[3 * c_i + 2];
          T1j[c_i + 12] = (tempR[c_i] * s + tempR[c_i + 3] * v_idx_0) +
                          tempR[c_i + 6] * v_idx_1;
        }
        T1j[3] = 0.0;
        T1j[7] = 0.0;
        T1j[11] = 0.0;
        T1j[15] = 1.0;
        for (c_i = 0; c_i < 4; c_i++) {
          s = T1j[c_i];
          v_idx_0 = T1j[c_i + 4];
          v_idx_1 = T1j[c_i + 8];
          tempR_tmp = T1j[c_i + 12];
          for (X_tmp = 0; X_tmp < 4; X_tmp++) {
            j = X_tmp << 2;
            Tj1[c_i + j] =
                ((s * T1[j] + v_idx_0 * T1[j + 1]) + v_idx_1 * T1[j + 2]) +
                tempR_tmp * T1[j + 3];
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
    B.set_size(6, n);
    loop_ub = 6 * Jac.size(1);
    for (i = 0; i < loop_ub; i++) {
      B[i] = Jac[i];
    }
    Jac.set_size(6, n);
    for (j = 0; j < n; j++) {
      loop_ub = j * 6;
      for (int b_i{0}; b_i < 6; b_i++) {
        s = 0.0;
        for (int k{0}; k < 6; k++) {
          s += X[k * 6 + b_i] * B[loop_ub + k];
        }
        Jac[loop_ub + b_i] = s;
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
  bool exitg1;
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
    bool exitg1;
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
    bool b_bool;
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
  CharacterVector s;
  RigidBodyTree *obj;
  double unusedExpr[5];
  obj = this;
  b_rand(aInstancePtr, unusedExpr);
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
    s.Vector[i] = cv2[i];
  }
  obj->_pobj0.NameInternal = s;
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
  CharacterVector s;
  b_RigidBodyTree *obj;
  double unusedExpr[5];
  signed char c_I[36];
  signed char b_I[9];
  obj = this;
  b_rand(aInstancePtr, unusedExpr);
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
  obj->Bodies[0] = obj->_pobj2[0].init(cv2);
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
// Arguments    : bool in1[9]
//                const double in2[9]
//                const ::coder::array<double, 2U> &in3
// Return Type  : void
//
} // namespace internal
} // namespace manip
} // namespace robotics
} // namespace coder
void binary_expand_op_29(bool in1[9], const double in2[9],
                         const ::coder::array<double, 2U> &in3)
{
  int stride_0_0;
  stride_0_0 = (in3.size(0) != 1);
  for (int i{0}; i < 9; i++) {
    in1[i] = (in2[i] >= in3[i * stride_0_0] - 4.4408920985006262E-16);
  }
}

//
// Arguments    : bool in1[9]
//                const double in2[9]
//                const ::coder::array<double, 2U> &in3
// Return Type  : void
//
void binary_expand_op_30(bool in1[9], const double in2[9],
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
