//
// File: generalizedInverseKinematics.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
//

// Include Files
#include "generalizedInverseKinematics.h"
#include "CharacterVector.h"
#include "CollisionGeometry.h"
#include "CollisionSet.h"
#include "DistanceBoundsConstraint.h"
#include "ErrorDampedLevenbergMarquardt.h"
#include "GIKProblem.h"
#include "GIKSolver.h"
#include "JointPositionBounds.h"
#include "KinematicConstraint.h"
#include "PoseTarget.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "SystemTimeProvider.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "find.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types1.h"
#include "ixfun.h"
#include "rigidBodyJoint.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "structConstructorHelper.h"
#include "tic.h"
#include "toc.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#include "coder_posix_time.h"
#include "collisioncodegen_api.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Variable Definitions
namespace gik9dof {
static const char cv13[18]{'L', 'e', 'v', 'e', 'n', 'b', 'e', 'r', 'g',
                           'M', 'a', 'r', 'q', 'u', 'a', 'r', 'd', 't'};

}

// Function Definitions
//
// Arguments    : GIKSolver *aInstancePtr
//                double initialGuess[9]
//                const constraintPoseTarget &varargin_1
//                const constraintJointBounds &varargin_2
//                const constraintDistanceBounds &varargin_3
//                struct1_T solutionInfo_ConstraintViolations[3]
//                char solutionInfo_Status_data[]
//                int solutionInfo_Status_size[2]
//                double &solutionInfo_NumRandomRestarts
//                double &solutionInfo_ExitFlag
// Return Type  : double
//
namespace gik9dof {
namespace coder {
double generalizedInverseKinematics::stepImpl(
    GIKSolver *aInstancePtr, double initialGuess[9],
    const constraintPoseTarget &varargin_1,
    const constraintJointBounds &varargin_2,
    const constraintDistanceBounds &varargin_3,
    struct1_T solutionInfo_ConstraintViolations[3],
    char solutionInfo_Status_data[], int solutionInfo_Status_size[2],
    double &solutionInfo_NumRandomRestarts, double &solutionInfo_ExitFlag)
{
  static const char b_cv[14]{'b', 'e', 's', 't', ' ', 'a', 'v',
                             'a', 'i', 'l', 'a', 'b', 'l', 'e'};
  static const char b_cv3[8]{'d', 'i', 's', 't', 'a', 'n', 'c', 'e'};
  static const char b_cv1[7]{'s', 'u', 'c', 'c', 'e', 's', 's'};
  static const char b_cv2[5]{'j', 'o', 'i', 'n', 't'};
  __m128d r1;
  __m128d r2;
  robotics::core::internal::ErrorDampedLevenbergMarquardt *b_obj;
  robotics::manip::internal::DistanceBoundsConstraint *e_obj;
  robotics::manip::internal::JointPositionBounds *d_obj;
  robotics::manip::internal::PoseTarget *c_obj;
  robotics::manip::internal::RigidBodyTree *obj;
  ::coder::array<double, 2U> Jrobot;
  ::coder::array<double, 2U> b_y;
  ::coder::array<double, 2U> limits;
  ::coder::array<double, 2U> positionIndices;
  ::coder::array<double, 1U> newseed;
  ::coder::array<double, 1U> q;
  ::coder::array<double, 1U> r;
  ::coder::array<double, 1U> r3;
  ::coder::array<double, 1U> xSolPrev;
  ::coder::array<int, 2U> r4;
  double positionMap_data[72];
  double kinematicPath_data[36];
  double T_data[16];
  double cv_max[2];
  double cv_min[2];
  double g[2];
  double err;
  double errPrev;
  double iter;
  double solutionInfo_Iterations;
  double tol;
  int indicesUpperBoundViolation_data[9];
  int T_size[2];
  int i;
  int i1;
  int indicesUpperBoundViolation_size;
  int k;
  int nm1d2;
  int trueCount;
  signed char tmp_data[36];
  bool lbOK[9];
  bool ubOK[9];
  bool exitg1;
  bool guard1;
  bool y;
  robotics::core::internal::NLPSolverExitFlags exitFlag;
  robotics::core::internal::NLPSolverExitFlags exitFlagPrev;
  obj = Tree;
  obj->get_JointPositionLimits(limits);
  if (limits.size(0) == 9) {
    for (i = 0; i < 9; i++) {
      ubOK[i] = (initialGuess[i] <=
                 limits[i + limits.size(0)] + 4.4408920985006262E-16);
    }
  } else {
    binary_expand_op_11(ubOK, initialGuess, limits);
  }
  if (limits.size(0) == 9) {
    for (i = 0; i < 9; i++) {
      lbOK[i] = (initialGuess[i] >= limits[i] - 4.4408920985006262E-16);
    }
  } else {
    binary_expand_op_10(lbOK, initialGuess, limits);
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
    for (i = 0; i < 9; i++) {
      ubOK[i] = !ubOK[i];
    }
    indicesUpperBoundViolation_size =
        eml_find(ubOK, indicesUpperBoundViolation_data);
    for (i = 0; i < indicesUpperBoundViolation_size; i++) {
      i1 = indicesUpperBoundViolation_data[i];
      initialGuess[i1 - 1] = limits[(i1 + limits.size(0)) - 1];
    }
    for (i = 0; i < 9; i++) {
      lbOK[i] = !lbOK[i];
    }
    indicesUpperBoundViolation_size =
        eml_find(lbOK, indicesUpperBoundViolation_data);
    for (i = 0; i < indicesUpperBoundViolation_size; i++) {
      i1 = indicesUpperBoundViolation_data[i];
      initialGuess[i1 - 1] = limits[i1 - 1];
    }
  }
  Problem.update(varargin_1, varargin_2, varargin_3);
  Problem.set_EnforceJointLimits(EnforceJointLimits);
  limits.set_size(Problem.DesignVariableBoundsInternal.size(0), 2);
  k = Problem.DesignVariableBoundsInternal.size(0) << 1;
  for (i = 0; i < k; i++) {
    limits[i] = Problem.DesignVariableBoundsInternal[i];
  }
  err = Problem.NumPositions + 1.0;
  if (err > limits.size(0)) {
    i = 0;
    i1 = -1;
  } else {
    i = static_cast<int>(err) - 1;
    i1 = limits.size(0) - 1;
  }
  indicesUpperBoundViolation_size = static_cast<int>(Problem.NumSlacks);
  xSolPrev.set_size(indicesUpperBoundViolation_size + 9);
  for (nm1d2 = 0; nm1d2 < 9; nm1d2++) {
    xSolPrev[nm1d2] = initialGuess[nm1d2];
  }
  for (nm1d2 = 0; nm1d2 < indicesUpperBoundViolation_size; nm1d2++) {
    xSolPrev[nm1d2 + 9] = 0.0;
  }
  Problem.residuals(xSolPrev, newseed);
  k = i1 - i;
  if (k + 1 == newseed.size(0)) {
    q.set_size(k + 1);
    for (i1 = 0; i1 <= k; i1++) {
      err = limits[i + i1];
      iter = newseed[i1];
      q[i1] = std::fmax(err, iter);
    }
  } else {
    xSolPrev.set_size(k + 1);
    for (i1 = 0; i1 <= k; i1++) {
      xSolPrev[i1] = limits[i + i1];
    }
    internal::expand_max(xSolPrev, newseed, q);
  }
  if (k + 1 == q.size(0)) {
    newseed.set_size(k + 1);
    for (i1 = 0; i1 <= k; i1++) {
      err = limits[(i + i1) + limits.size(0)];
      iter = q[i1];
      newseed[i1] = std::fmin(err, iter);
    }
  } else {
    xSolPrev.set_size(k + 1);
    for (i1 = 0; i1 <= k; i1++) {
      xSolPrev[i1] = limits[(i + i1) + limits.size(0)];
    }
    internal::expand_min(xSolPrev, q, newseed);
  }
  b_obj = Solver;
  b_obj->MaxNumIterationInternal = b_obj->MaxNumIteration;
  b_obj->MaxTimeInternal = b_obj->MaxTime;
  b_obj->SeedInternal.set_size(newseed.size(0) + 9);
  for (i = 0; i < 9; i++) {
    b_obj->SeedInternal[i] = initialGuess[i];
  }
  k = newseed.size(0);
  for (i = 0; i < k; i++) {
    b_obj->SeedInternal[i + 9] = newseed[i];
  }
  tol = b_obj->SolutionTolerance;
  b_obj->TimeObj.StartTime.tv_sec =
      tic(aInstancePtr, b_obj->TimeObj.StartTime.tv_nsec);
  exitFlag = b_obj->solveInternal(aInstancePtr, xSolPrev, err, iter);
  solutionInfo_NumRandomRestarts = 0.0;
  solutionInfo_Iterations = iter;
  errPrev = err;
  exitFlagPrev = exitFlag;
  exitg1 = false;
  while ((!exitg1) && (b_obj->RandomRestart && (err > tol))) {
    b_obj->MaxNumIterationInternal -= iter;
    err = toc(aInstancePtr, b_obj->TimeObj.StartTime.tv_sec,
              b_obj->TimeObj.StartTime.tv_nsec);
    b_obj->MaxTimeInternal = b_obj->MaxTime - err;
    if (b_obj->MaxNumIterationInternal <= 0.0) {
      exitFlag =
          robotics::core::internal::NLPSolverExitFlags::IterationLimitExceeded;
    }
    if ((exitFlag == robotics::core::internal::NLPSolverExitFlags::
                         IterationLimitExceeded) ||
        (exitFlag ==
         robotics::core::internal::NLPSolverExitFlags::TimeLimitExceeded)) {
      exitFlagPrev = exitFlag;
      exitg1 = true;
    } else {
      rigidBodyJoint::randomConfig(aInstancePtr, b_obj->ExtraArgs, newseed);
      k = newseed.size(0);
      b_obj->SeedInternal.set_size(newseed.size(0));
      for (i = 0; i < k; i++) {
        b_obj->SeedInternal[i] = newseed[i];
      }
      exitFlag = b_obj->solveInternal(aInstancePtr, q, err, iter);
      if (err < errPrev) {
        k = q.size(0);
        xSolPrev.set_size(q.size(0));
        for (i = 0; i < k; i++) {
          xSolPrev[i] = q[i];
        }
        errPrev = err;
        exitFlagPrev = exitFlag;
      }
      solutionInfo_NumRandomRestarts++;
      solutionInfo_Iterations += iter;
    }
  }
  if (errPrev < tol) {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 7;
    for (i = 0; i < 7; i++) {
      solutionInfo_Status_data[i] = b_cv1[i];
    }
  } else {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 14;
    for (i = 0; i < 14; i++) {
      solutionInfo_Status_data[i] = b_cv[i];
    }
  }
  err = Problem.NumPositions;
  if (err < 1.0) {
    k = 0;
  } else {
    k = static_cast<int>(err);
  }
  q.set_size(k);
  for (i = 0; i < k; i++) {
    q[i] = xSolPrev[i];
  }
  double JTwist[12];
  c_obj = Problem.Constraints.f1;
  c_obj->Tree->efficientFKAndJacobianForIK(q, c_obj->EndEffectorIndex,
                                           c_obj->ReferenceBodyIndex, T_data,
                                           T_size, Jrobot);
  c_obj->evaluateFromTransform(T_data, T_size, g, JTwist);
  r.set_size(c_obj->BoundsInternal.size(0));
  indicesUpperBoundViolation_size = c_obj->BoundsInternal.size(0);
  for (i = 0; i < indicesUpperBoundViolation_size; i++) {
    r[i] = c_obj->BoundsInternal[i];
  }
  if (r.size(0) == 2) {
    r1 = _mm_loadu_pd(&g[0]);
    r2 = _mm_loadu_pd(&(r.data())[0]);
    _mm_storeu_pd(&cv_max[0], _mm_sub_pd(r1, r2));
  } else {
    minus(cv_max, g, r);
  }
  cv_min[0] = std::fmin(0.0, cv_max[0]);
  cv_min[1] = std::fmin(0.0, cv_max[1]);
  r.set_size(c_obj->BoundsInternal.size(0));
  indicesUpperBoundViolation_size = c_obj->BoundsInternal.size(0);
  for (i = 0; i < indicesUpperBoundViolation_size; i++) {
    r[i] = c_obj->BoundsInternal[i + c_obj->BoundsInternal.size(0)];
  }
  if (r.size(0) == 2) {
    r1 = _mm_loadu_pd(&g[0]);
    r2 = _mm_loadu_pd(&(r.data())[0]);
    _mm_storeu_pd(&g[0], _mm_sub_pd(r1, r2));
  } else {
    minus(g, r);
  }
  cv_max[0] = std::fmax(0.0, g[0]);
  cv_max[1] = std::fmax(0.0, g[1]);
  d_obj = Problem.Constraints.f2;
  r.set_size(d_obj->BoundsInternal.size(0));
  indicesUpperBoundViolation_size = d_obj->BoundsInternal.size(0);
  for (i = 0; i < indicesUpperBoundViolation_size; i++) {
    r[i] = d_obj->BoundsInternal[i];
  }
  r3.set_size(d_obj->BoundsInternal.size(0));
  indicesUpperBoundViolation_size = d_obj->BoundsInternal.size(0);
  for (i = 0; i < indicesUpperBoundViolation_size; i++) {
    r3[i] = d_obj->BoundsInternal[i + d_obj->BoundsInternal.size(0)];
  }
  e_obj = Problem.Constraints.f3;
  e_obj->Tree->efficientFKAndJacobianForIK(q, e_obj->EndEffectorIndex,
                                           e_obj->ReferenceBodyIndex, T_data,
                                           T_size, Jrobot);
  i = T_size[0] * (T_size[1] - 1);
  err = T_data[i];
  iter = err * err;
  err = T_data[i + 1];
  iter += err * err;
  err = T_data[i + 2];
  iter += err * err;
  err = std::sqrt(iter + 2.2204460492503131E-16);
  indicesUpperBoundViolation_size = e_obj->BoundsInternal.size(0);
  newseed.set_size(indicesUpperBoundViolation_size);
  nm1d2 = e_obj->BoundsInternal.size(0);
  for (i = 0; i < nm1d2; i++) {
    newseed[i] = err - e_obj->BoundsInternal[i];
  }
  q.set_size(e_obj->BoundsInternal.size(0));
  nm1d2 = e_obj->BoundsInternal.size(0);
  for (i = 0; i < nm1d2; i++) {
    q[i] = err - e_obj->BoundsInternal[i + e_obj->BoundsInternal.size(0)];
  }
  solutionInfo_ConstraintViolations[0].Type.size[0] = 1;
  solutionInfo_ConstraintViolations[0].Type.size[1] = 4;
  solutionInfo_ConstraintViolations[0].Type.data[0] = 'p';
  solutionInfo_ConstraintViolations[0].Type.data[1] = 'o';
  solutionInfo_ConstraintViolations[0].Type.data[2] = 's';
  solutionInfo_ConstraintViolations[0].Type.data[3] = 'e';
  solutionInfo_ConstraintViolations[1].Type.size[0] = 1;
  solutionInfo_ConstraintViolations[1].Type.size[1] = 5;
  for (i = 0; i < 5; i++) {
    solutionInfo_ConstraintViolations[1].Type.data[i] = b_cv2[i];
  }
  solutionInfo_ConstraintViolations[2].Type.size[0] = 1;
  solutionInfo_ConstraintViolations[2].Type.size[1] = 8;
  for (i = 0; i < 8; i++) {
    solutionInfo_ConstraintViolations[2].Type.data[i] = b_cv3[i];
  }
  solutionInfo_ConstraintViolations[0].Violation.set_size(1, 2);
  r1 = _mm_loadu_pd(&cv_min[0]);
  r2 = _mm_loadu_pd(&cv_max[0]);
  _mm_storeu_pd(&solutionInfo_ConstraintViolations[0].Violation[0],
                _mm_add_pd(r1, r2));
  if (k == 1) {
    i = r.size(0);
    i1 = r3.size(0);
  } else {
    i = k;
    i1 = k;
  }
  if ((k == r.size(0)) && (k == r3.size(0)) && (i == i1)) {
    solutionInfo_ConstraintViolations[1].Violation.set_size(1, k);
    for (i = 0; i < k; i++) {
      iter = xSolPrev[i] - r[i];
      err = xSolPrev[i] - r3[i];
      solutionInfo_ConstraintViolations[1].Violation[i] =
          std::fmin(0.0, iter) + std::fmax(0.0, err);
    }
  } else {
    binary_expand_op_9(solutionInfo_ConstraintViolations, xSolPrev, k - 1, r,
                       r3);
  }
  if (newseed.size(0) == q.size(0)) {
    solutionInfo_ConstraintViolations[2].Violation.set_size(
        1, indicesUpperBoundViolation_size);
    for (i = 0; i < indicesUpperBoundViolation_size; i++) {
      iter = newseed[i];
      err = q[i];
      solutionInfo_ConstraintViolations[2].Violation[i] =
          std::fmin(0.0, iter) + std::fmax(0.0, err);
    }
  } else {
    binary_expand_op_8(solutionInfo_ConstraintViolations, newseed, q);
  }
  Problem.get_KinematicPath(kinematicPath_data, T_size);
  obj = Tree;
  indicesUpperBoundViolation_size = T_size[1];
  trueCount = 0;
  nm1d2 = 0;
  for (i1 = 0; i1 < indicesUpperBoundViolation_size; i1++) {
    if (kinematicPath_data[i1] != 0.0) {
      trueCount++;
      tmp_data[nm1d2] = static_cast<signed char>(i1);
      nm1d2++;
    }
  }
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < trueCount; i1++) {
      positionMap_data[i1 + trueCount * i] =
          obj->PositionDoFMap
              [(static_cast<int>(kinematicPath_data[tmp_data[i1]]) + 11 * i) -
               1];
    }
  }
  positionIndices.set_size(1, static_cast<int>(obj->PositionNumber));
  k = static_cast<int>(obj->PositionNumber);
  for (i = 0; i < k; i++) {
    positionIndices[i] = 0.0;
  }
  err = 0.0;
  for (i1 = 0; i1 < trueCount; i1++) {
    iter = positionMap_data[i1 + trueCount];
    tol = iter - positionMap_data[i1];
    if (tol + 1.0 > 0.0) {
      if (tol + 1.0 < 1.0) {
        b_y.set_size(1, 0);
      } else {
        b_y.set_size(1, static_cast<int>((tol + 1.0) - 1.0) + 1);
        k = static_cast<int>((tol + 1.0) - 1.0);
        for (i = 0; i <= k; i++) {
          b_y[i] = static_cast<double>(i) + 1.0;
        }
      }
      k = b_y.size(1);
      r4.set_size(1, b_y.size(1));
      for (i = 0; i < k; i++) {
        r4[i] = static_cast<int>(err + b_y[i]);
      }
      errPrev = positionMap_data[i1];
      if (std::isnan(errPrev) || std::isnan(iter)) {
        b_y.set_size(1, 1);
        b_y[0] = rtNaN;
      } else if (iter < errPrev) {
        b_y.set_size(1, 0);
      } else if ((std::isinf(errPrev) || std::isinf(iter)) &&
                 (errPrev == iter)) {
        b_y.set_size(1, 1);
        b_y[0] = rtNaN;
      } else if (std::floor(errPrev) == errPrev) {
        indicesUpperBoundViolation_size = static_cast<int>(tol);
        b_y.set_size(1, static_cast<int>(tol) + 1);
        for (i = 0; i <= indicesUpperBoundViolation_size; i++) {
          b_y[i] = errPrev + static_cast<double>(i);
        }
      } else {
        double apnd;
        double cdiff;
        double ndbl;
        ndbl = std::floor(tol + 0.5);
        apnd = errPrev + ndbl;
        cdiff = apnd - iter;
        if (std::abs(cdiff) <
            4.4408920985006262E-16 *
                std::fmax(std::abs(errPrev), std::abs(iter))) {
          ndbl++;
          apnd = iter;
        } else if (cdiff > 0.0) {
          apnd = errPrev + (ndbl - 1.0);
        } else {
          ndbl++;
        }
        if (ndbl >= 0.0) {
          indicesUpperBoundViolation_size = static_cast<int>(ndbl);
        } else {
          indicesUpperBoundViolation_size = 0;
        }
        b_y.set_size(1, indicesUpperBoundViolation_size);
        if (indicesUpperBoundViolation_size > 0) {
          b_y[0] = errPrev;
          if (indicesUpperBoundViolation_size > 1) {
            b_y[indicesUpperBoundViolation_size - 1] = apnd;
            nm1d2 = (indicesUpperBoundViolation_size - 1) / 2;
            for (k = 0; k <= nm1d2 - 2; k++) {
              b_y[k + 1] = errPrev + (static_cast<double>(k) + 1.0);
              b_y[(indicesUpperBoundViolation_size - k) - 2] =
                  apnd - (static_cast<double>(k) + 1.0);
            }
            if (nm1d2 << 1 == indicesUpperBoundViolation_size - 1) {
              b_y[nm1d2] = (positionMap_data[i1] + apnd) / 2.0;
            } else {
              b_y[nm1d2] = positionMap_data[i1] + static_cast<double>(nm1d2);
              b_y[nm1d2 + 1] = apnd - static_cast<double>(nm1d2);
            }
          }
        }
      }
      k = b_y.size(1) - 1;
      for (i = 0; i <= k; i++) {
        positionIndices[r4[i] - 1] = b_y[i];
      }
      err += tol + 1.0;
    }
  }
  if (err < 1.0) {
    k = 0;
  } else {
    k = static_cast<int>(err);
  }
  positionIndices.set_size(positionIndices.size(0), k);
  for (i = 0; i < k; i++) {
    err = positionIndices[i];
    initialGuess[static_cast<int>(err) - 1] =
        xSolPrev[static_cast<int>(err) - 1];
  }
  solutionInfo_ExitFlag = static_cast<int>(exitFlagPrev);
  return solutionInfo_Iterations;
}

//
// Arguments    : void
// Return Type  : generalizedInverseKinematics
//
generalizedInverseKinematics::generalizedInverseKinematics()
{
  matlabCodegenIsDeleted = true;
}

//
// Arguments    : void
// Return Type  : void
//
generalizedInverseKinematics::~generalizedInverseKinematics()
{
  matlabCodegenDestructor();
}

//
// Arguments    : GIKSolver *aInstancePtr
//                rigidBodyTree &varargin_2
// Return Type  : generalizedInverseKinematics *
//
generalizedInverseKinematics *
generalizedInverseKinematics::init(GIKSolver *aInstancePtr,
                                   rigidBodyTree &varargin_2)
{
  void *copyGeometryInternal;
  generalizedInverseKinematics *obj;
  robotics::manip::internal::CharacterVector b_obj;
  robotics::manip::internal::CharacterVector c_obj;
  robotics::manip::internal::CollisionGeometry b_newObj;
  robotics::manip::internal::CollisionGeometry e_obj;
  robotics::manip::internal::CollisionSet *d_obj;
  robotics::manip::internal::CollisionSet *newObj;
  robotics::manip::internal::RigidBody *body;
  robotics::manip::internal::RigidBody *parent;
  robotics::manip::internal::RigidBodyTree *newrobot;
  double bid;
  int obj_size[2];
  int loop_ub;
  char obj_data[200];
  obj = this;
  obj->EnforceJointLimits = true;
  obj->isInitialized = 0;
  newrobot = obj->_pobj3.init(aInstancePtr);
  b_obj = varargin_2.TreeInternal.Base.NameInternal;
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
  bid = newrobot->findBodyIndexByName(obj_data, obj_size);
  if ((!(bid == 0.0)) && (bid < 0.0)) {
    c_obj = newrobot->Base.NameInternal;
    c_obj.Length = loop_ub;
    if (loop_ub < 1) {
      loop_ub = 0;
    }
    if (loop_ub - 1 >= 0) {
      ::std::copy(&b_obj.Vector[0], &b_obj.Vector[loop_ub], &c_obj.Vector[0]);
    }
    newrobot->Base.NameInternal = c_obj;
  }
  d_obj = varargin_2.TreeInternal.Base.CollisionsInternal;
  newObj = obj->_pobj2[0].init(d_obj->MaxElements);
  newObj->Size = d_obj->Size;
  bid = d_obj->Size;
  loop_ub = static_cast<int>(bid);
  for (int i{0}; i < loop_ub; i++) {
    e_obj = d_obj->CollisionGeometries[i];
    copyGeometryInternal =
        collisioncodegen_copyGeometry(e_obj.CollisionPrimitive);
    b_newObj.CollisionPrimitive = copyGeometryInternal;
    ::std::copy(&e_obj.LocalPose[0], &e_obj.LocalPose[16],
                &b_newObj.LocalPose[0]);
    b_newObj.MeshScale[0] = e_obj.MeshScale[0];
    b_newObj.MeshScale[1] = e_obj.MeshScale[1];
    b_newObj.MeshScale[2] = e_obj.MeshScale[2];
    ::std::copy(&e_obj.WorldPose[0], &e_obj.WorldPose[16],
                &b_newObj.WorldPose[0]);
    newObj->CollisionGeometries[i] = b_newObj;
  }
  double g_idx_1;
  double g_idx_2;
  newrobot->Base.CollisionsInternal = newObj;
  bid = varargin_2.TreeInternal.Gravity[0];
  g_idx_1 = varargin_2.TreeInternal.Gravity[1];
  g_idx_2 = varargin_2.TreeInternal.Gravity[2];
  newrobot->Gravity[0] = bid;
  newrobot->Gravity[1] = g_idx_1;
  newrobot->Gravity[2] = g_idx_2;
  if (varargin_2.TreeInternal.NumBodies >= 1.0) {
    body = varargin_2.TreeInternal.Bodies[0];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[1], obj->_pobj0[0],
                      obj->_pobj1[0]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 2.0) {
    body = varargin_2.TreeInternal.Bodies[1];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[3], obj->_pobj0[2],
                      obj->_pobj1[1]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 3.0) {
    body = varargin_2.TreeInternal.Bodies[2];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[5], obj->_pobj0[4],
                      obj->_pobj1[2]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 4.0) {
    body = varargin_2.TreeInternal.Bodies[3];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[7], obj->_pobj0[6],
                      obj->_pobj1[3]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 5.0) {
    body = varargin_2.TreeInternal.Bodies[4];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[9], obj->_pobj0[8],
                      obj->_pobj1[4]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 6.0) {
    body = varargin_2.TreeInternal.Bodies[5];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[11],
                      obj->_pobj0[10], obj->_pobj1[5]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 7.0) {
    body = varargin_2.TreeInternal.Bodies[6];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[13],
                      obj->_pobj0[12], obj->_pobj1[6]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 8.0) {
    body = varargin_2.TreeInternal.Bodies[7];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[15],
                      obj->_pobj0[14], obj->_pobj1[7]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 9.0) {
    body = varargin_2.TreeInternal.Bodies[8];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[17],
                      obj->_pobj0[16], obj->_pobj1[8]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 10.0) {
    body = varargin_2.TreeInternal.Bodies[9];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[19],
                      obj->_pobj0[18], obj->_pobj1[9]);
  }
  if (varargin_2.TreeInternal.NumBodies >= 11.0) {
    body = varargin_2.TreeInternal.Bodies[10];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = varargin_2.TreeInternal.Bodies[static_cast<int>(bid) - 1];
    } else {
      parent = &varargin_2.TreeInternal.Base;
    }
    b_obj = parent->NameInternal;
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
    newrobot->addBody(body, obj_data, obj_size, obj->_pobj2[21],
                      obj->_pobj0[20], obj->_pobj1[10]);
  }
  obj->Tree = newrobot;
  obj->RigidBodyTreeHasBeenSet = true;
  obj->_pobj4.MaxNumIteration = 1500.0;
  obj->_pobj4.MaxTime = 10.0;
  obj->_pobj4.SolutionTolerance = 1.0E-6;
  obj->_pobj4.ConstraintsOn = true;
  obj->_pobj4.RandomRestart = true;
  obj->_pobj4.StepTolerance = 1.0E-12;
  obj->_pobj4.GradientTolerance = 5.0E-9;
  obj->_pobj4.ErrorChangeTolerance = 1.0E-12;
  obj->_pobj4.DampingBias = 0.0025;
  obj->_pobj4.UseErrorDamping = true;
  for (loop_ub = 0; loop_ub < 18; loop_ub++) {
    obj->_pobj4.Name[loop_ub] = cv13[loop_ub];
  }
  obj->_pobj4.TimeObj.StartTime.tv_sec = 0.0;
  obj->_pobj4.TimeObj.StartTime.tv_nsec = 0.0;
  obj->_pobj4.TimeObjInternal.StartTime.tv_sec = 0.0;
  obj->_pobj4.TimeObjInternal.StartTime.tv_nsec = 0.0;
  obj->_pobj4.matlabCodegenIsDeleted = false;
  obj->Solver = &obj->_pobj4;
  obj->matlabCodegenIsDeleted = false;
  return obj;
}

//
// Arguments    : void
// Return Type  : void
//
void generalizedInverseKinematics::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
    if (isInitialized == 1) {
      isInitialized = 2;
    }
  }
}

//
// Arguments    : double solverparams_MaxIterations
//                double solverparams_MaxTime
//                double solverparams_GradientTolerance
//                double solverparams_SolutionTolerance
//                bool solverparams_EnforceJointLimits
//                bool solverparams_AllowRandomRestart
//                double solverparams_StepTolerance
//                double solverparams_ErrorChangeTolerance
//                double solverparams_DampingBias
//                bool solverparams_UseErrorDamping
// Return Type  : void
//
void generalizedInverseKinematics::set_SolverParameters(
    double solverparams_MaxIterations, double solverparams_MaxTime,
    double solverparams_GradientTolerance,
    double solverparams_SolutionTolerance, bool solverparams_EnforceJointLimits,
    bool solverparams_AllowRandomRestart, double solverparams_StepTolerance,
    double solverparams_ErrorChangeTolerance, double solverparams_DampingBias,
    bool solverparams_UseErrorDamping)
{
  robotics::core::internal::ErrorDampedLevenbergMarquardt *obj;
  double b_expl_temp;
  double c_expl_temp;
  double d_expl_temp;
  double g_expl_temp;
  double params_DampingBias;
  double params_ErrorChangeTolerance;
  int ret;
  char expl_temp[18];
  bool e_expl_temp;
  bool f_expl_temp;
  bool params_UseErrorDamping;
  Solver->getSolverParams(expl_temp, b_expl_temp, c_expl_temp, d_expl_temp,
                          e_expl_temp, f_expl_temp, g_expl_temp,
                          params_ErrorChangeTolerance, params_DampingBias,
                          params_UseErrorDamping);
  EnforceJointLimits = solverparams_EnforceJointLimits;
  for (ret = 0; ret < 18; ret++) {
    expl_temp[ret] = Solver->Name[ret];
  }
  ret = std::memcmp(&cv13[0], &expl_temp[0], 18);
  if (ret == 0) {
    ret = 1;
  } else {
    ret = -1;
  }
  switch (ret) {
  case 0:
    break;
  case 1:
    params_ErrorChangeTolerance = solverparams_ErrorChangeTolerance;
    params_DampingBias = solverparams_DampingBias;
    params_UseErrorDamping = solverparams_UseErrorDamping;
    break;
  }
  obj = Solver;
  obj->MaxNumIteration = solverparams_MaxIterations;
  obj->MaxTime = solverparams_MaxTime;
  obj->GradientTolerance = solverparams_GradientTolerance;
  obj->SolutionTolerance = solverparams_SolutionTolerance;
  obj->ConstraintsOn = true;
  obj->RandomRestart = solverparams_AllowRandomRestart;
  obj->StepTolerance = solverparams_StepTolerance;
  obj->ErrorChangeTolerance = params_ErrorChangeTolerance;
  obj->DampingBias = params_DampingBias;
  obj->UseErrorDamping = params_UseErrorDamping;
}

//
// Arguments    : GIKSolver *aInstancePtr
//                const double varargin_1[9]
//                constraintPoseTarget &varargin_2
//                constraintJointBounds &varargin_3
//                constraintDistanceBounds &varargin_4
//                double varargout_1[9]
//                struct1_T varargout_2_ConstraintViolations[3]
//                char varargout_2_Status_data[]
//                int varargout_2_Status_size[2]
//                double &varargout_2_NumRandomRestarts
//                double &varargout_2_ExitFlag
// Return Type  : double
//
double generalizedInverseKinematics::step(
    GIKSolver *aInstancePtr, const double varargin_1[9],
    constraintPoseTarget &varargin_2, constraintJointBounds &varargin_3,
    constraintDistanceBounds &varargin_4, double varargout_1[9],
    struct1_T varargout_2_ConstraintViolations[3],
    char varargout_2_Status_data[], int varargout_2_Status_size[2],
    double &varargout_2_NumRandomRestarts, double &varargout_2_ExitFlag)
{
  robotics::manip::internal::RigidBodyTree *tree;
  ::coder::array<double, 2U> A;
  ::coder::array<double, 2U> b_value;
  ::coder::array<double, 2U> r1;
  ::coder::array<double, 2U> y;
  ::coder::array<double, 1U> b;
  ::coder::array<double, 1U> obj;
  cell_wrap_7 residualIndices[3];
  cell_wrap_7 slackIndices[3];
  cell_wrap_8 equalityFlags[3];
  double b_expl_temp;
  double e_expl_temp;
  double f_expl_temp;
  double g_expl_temp;
  double varargout_2_Iterations;
  bool c_expl_temp;
  bool d_expl_temp;
  bool h_expl_temp;
  if (isInitialized != 1) {
    __m128d r;
    double numResidualsTotal;
    int loop_ub;
    int obj_idx_0;
    int vectorUB;
    isSetupComplete = false;
    isInitialized = 1;
    tree = Tree;
    Problem.Tree = tree;
    Problem.NumPositions = Problem.Tree->PositionNumber;
    tree = Problem.Tree;
    for (int i{0}; i < 16; i++) {
      Problem._pobj2.TargetTransform[i] = iv[i];
    }
    Problem._pobj2.Tree = tree;
    Problem._pobj2.NumElements = 2.0;
    obj_idx_0 = static_cast<int>(Problem._pobj2.NumElements);
    Problem._pobj2.BoundsInternal.set_size(obj_idx_0, 2);
    loop_ub = obj_idx_0 << 1;
    for (int i{0}; i < loop_ub; i++) {
      Problem._pobj2.BoundsInternal[i] = 0.0;
    }
    obj_idx_0 = static_cast<int>(Problem._pobj2.NumElements);
    Problem._pobj2.Weights.set_size(1, obj_idx_0);
    for (int i{0}; i < obj_idx_0; i++) {
      Problem._pobj2.Weights[i] = 1.0;
    }
    Problem._pobj2.EndEffectorIndex = 1.0;
    Problem._pobj2.ReferenceBodyIndex = 0.0;
    Problem._pobj2.matlabCodegenIsDeleted = false;
    Problem.Constraints.f1 = &Problem._pobj2;
    varargout_2_Iterations = Problem.Constraints.f1->NumElements;
    if (std::isnan(varargout_2_Iterations)) {
      residualIndices[0].f1.set_size(1, 1);
      residualIndices[0].f1[0] = rtNaN;
    } else if (varargout_2_Iterations < 1.0) {
      residualIndices[0].f1.set_size(1, 0);
    } else {
      residualIndices[0].f1.set_size(
          1, static_cast<int>(varargout_2_Iterations - 1.0) + 1);
      loop_ub = static_cast<int>(varargout_2_Iterations - 1.0);
      for (int i{0}; i <= loop_ub; i++) {
        residualIndices[0].f1[i] = static_cast<double>(i) + 1.0;
      }
    }
    loop_ub = residualIndices[0].f1.size(1);
    slackIndices[0].f1.set_size(1, residualIndices[0].f1.size(1));
    for (int i{0}; i < loop_ub; i++) {
      slackIndices[0].f1[i] = Problem.NumPositions + residualIndices[0].f1[i];
    }
    obj_idx_0 = static_cast<int>(varargout_2_Iterations);
    equalityFlags[0].f1.set_size(1, obj_idx_0);
    for (int i{0}; i < obj_idx_0; i++) {
      equalityFlags[0].f1[i] = false;
    }
    numResidualsTotal = varargout_2_Iterations;
    tree = Problem.Tree;
    varargout_2_Iterations = tree->PositionNumber;
    Problem._pobj1.Tree = tree;
    Problem._pobj1.NumElements = varargout_2_Iterations;
    obj_idx_0 = static_cast<int>(Problem._pobj1.NumElements);
    Problem._pobj1.BoundsInternal.set_size(obj_idx_0, 2);
    loop_ub = obj_idx_0 << 1;
    for (int i{0}; i < loop_ub; i++) {
      Problem._pobj1.BoundsInternal[i] = 0.0;
    }
    obj_idx_0 = static_cast<int>(Problem._pobj1.NumElements);
    Problem._pobj1.Weights.set_size(1, obj_idx_0);
    for (int i{0}; i < obj_idx_0; i++) {
      Problem._pobj1.Weights[i] = 1.0;
    }
    Problem._pobj1.Tree->get_JointPositionLimits(Problem._pobj1.BoundsInternal);
    obj_idx_0 = static_cast<int>(Problem._pobj1.NumElements);
    Problem._pobj1.Weights.set_size(1, obj_idx_0);
    for (int i{0}; i < obj_idx_0; i++) {
      Problem._pobj1.Weights[i] = 1.0;
    }
    Problem._pobj1.matlabCodegenIsDeleted = false;
    Problem.Constraints.f2 = &Problem._pobj1;
    varargout_2_Iterations = Problem.Constraints.f2->NumElements;
    if (std::isnan(varargout_2_Iterations)) {
      y.set_size(1, 1);
      y[0] = rtNaN;
    } else if (varargout_2_Iterations < 1.0) {
      y.set_size(1, 0);
    } else {
      y.set_size(1, static_cast<int>(varargout_2_Iterations - 1.0) + 1);
      loop_ub = static_cast<int>(varargout_2_Iterations - 1.0);
      for (int i{0}; i <= loop_ub; i++) {
        y[i] = static_cast<double>(i) + 1.0;
      }
    }
    loop_ub = y.size(1);
    residualIndices[1].f1.set_size(1, y.size(1));
    obj_idx_0 = (y.size(1) / 2) << 1;
    vectorUB = obj_idx_0 - 2;
    for (int i{0}; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&y[i]);
      _mm_storeu_pd(&residualIndices[1].f1[i],
                    _mm_add_pd(_mm_set1_pd(numResidualsTotal), r));
    }
    for (int i{obj_idx_0}; i < loop_ub; i++) {
      residualIndices[1].f1[i] = numResidualsTotal + y[i];
    }
    loop_ub = residualIndices[1].f1.size(1);
    slackIndices[1].f1.set_size(1, residualIndices[1].f1.size(1));
    for (int i{0}; i < loop_ub; i++) {
      slackIndices[1].f1[i] = Problem.NumPositions + residualIndices[1].f1[i];
    }
    obj_idx_0 = static_cast<int>(varargout_2_Iterations);
    equalityFlags[1].f1.set_size(1, obj_idx_0);
    for (int i{0}; i < obj_idx_0; i++) {
      equalityFlags[1].f1[i] = false;
    }
    numResidualsTotal += varargout_2_Iterations;
    tree = Problem.Tree;
    Problem._pobj0.Tree = tree;
    Problem._pobj0.NumElements = 1.0;
    obj_idx_0 = static_cast<int>(Problem._pobj0.NumElements);
    Problem._pobj0.BoundsInternal.set_size(obj_idx_0, 2);
    loop_ub = obj_idx_0 << 1;
    for (int i{0}; i < loop_ub; i++) {
      Problem._pobj0.BoundsInternal[i] = 0.0;
    }
    obj_idx_0 = static_cast<int>(Problem._pobj0.NumElements);
    Problem._pobj0.Weights.set_size(1, obj_idx_0);
    for (int i{0}; i < obj_idx_0; i++) {
      Problem._pobj0.Weights[i] = 1.0;
    }
    Problem._pobj0.EndEffectorIndex = 1.0;
    Problem._pobj0.ReferenceBodyIndex = 0.0;
    Problem._pobj0.matlabCodegenIsDeleted = false;
    Problem.Constraints.f3 = &Problem._pobj0;
    varargout_2_Iterations = Problem.Constraints.f3->NumElements;
    if (std::isnan(varargout_2_Iterations)) {
      y.set_size(1, 1);
      y[0] = rtNaN;
    } else if (varargout_2_Iterations < 1.0) {
      y.set_size(1, 0);
    } else {
      y.set_size(1, static_cast<int>(varargout_2_Iterations - 1.0) + 1);
      loop_ub = static_cast<int>(varargout_2_Iterations - 1.0);
      for (int i{0}; i <= loop_ub; i++) {
        y[i] = static_cast<double>(i) + 1.0;
      }
    }
    loop_ub = y.size(1);
    residualIndices[2].f1.set_size(1, y.size(1));
    obj_idx_0 = (y.size(1) / 2) << 1;
    vectorUB = obj_idx_0 - 2;
    for (int i{0}; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&y[i]);
      _mm_storeu_pd(&residualIndices[2].f1[i],
                    _mm_add_pd(_mm_set1_pd(numResidualsTotal), r));
    }
    for (int i{obj_idx_0}; i < loop_ub; i++) {
      residualIndices[2].f1[i] = numResidualsTotal + y[i];
    }
    loop_ub = residualIndices[2].f1.size(1);
    slackIndices[2].f1.set_size(1, residualIndices[2].f1.size(1));
    for (int i{0}; i < loop_ub; i++) {
      slackIndices[2].f1[i] = Problem.NumPositions + residualIndices[2].f1[i];
    }
    obj_idx_0 = static_cast<int>(varargout_2_Iterations);
    equalityFlags[2].f1.set_size(1, obj_idx_0);
    for (int i{0}; i < obj_idx_0; i++) {
      equalityFlags[2].f1[i] = false;
    }
    numResidualsTotal += varargout_2_Iterations;
    Problem.ResidualIndices[0] = residualIndices[0];
    Problem.ResidualIndices[1] = residualIndices[1];
    Problem.ResidualIndices[2] = residualIndices[2];
    Problem.SlackIndices[0] = slackIndices[0];
    Problem.SlackIndices[1] = slackIndices[1];
    Problem.SlackIndices[2] = slackIndices[2];
    Problem.EqualityFlags[0] = equalityFlags[0];
    Problem.EqualityFlags[1] = equalityFlags[1];
    Problem.EqualityFlags[2] = equalityFlags[2];
    Problem.NumResiduals = numResidualsTotal;
    Problem.NumSlacks = numResidualsTotal;
    Problem.NumVariables = Problem.NumPositions + Problem.NumSlacks;
    loop_ub = static_cast<int>(Problem.NumVariables);
    vectorUB = static_cast<int>(Problem.NumVariables);
    b_value.set_size(loop_ub, 2);
    for (int i{0}; i < loop_ub; i++) {
      b_value[i] = rtMinusInf;
    }
    for (int i{0}; i < vectorUB; i++) {
      b_value[i + b_value.size(0)] = rtInf;
    }
    Problem.DesignVariableBoundsInternal.set_size(loop_ub, 2);
    obj_idx_0 = b_value.size(0) << 1;
    for (int i{0}; i < obj_idx_0; i++) {
      Problem.DesignVariableBoundsInternal[i] = b_value[i];
    }
    vectorUB = 2 * b_value.size(0);
    A.set_size(vectorUB, loop_ub);
    obj_idx_0 = vectorUB * b_value.size(0);
    for (int i{0}; i < obj_idx_0; i++) {
      A[i] = 0.0;
    }
    b.set_size(vectorUB);
    for (int i{0}; i < vectorUB; i++) {
      b[i] = 0.0;
    }
    for (int i{0}; i < loop_ub; i++) {
      obj_idx_0 = static_cast<int>(static_cast<unsigned int>(i + 1) << 1);
      A[(obj_idx_0 + A.size(0) * i) - 2] = -1.0;
      A[(obj_idx_0 + A.size(0) * i) - 1] = 1.0;
      b[obj_idx_0 - 2] = -b_value[i];
      b[obj_idx_0 - 1] = b_value[i + b_value.size(0)];
    }
    Problem.ConstraintMatrixInternal.set_size(loop_ub, vectorUB);
    for (int i{0}; i < vectorUB; i++) {
      for (obj_idx_0 = 0; obj_idx_0 < loop_ub; obj_idx_0++) {
        Problem.ConstraintMatrixInternal
            [obj_idx_0 + Problem.ConstraintMatrixInternal.size(0) * i] =
            A[i + A.size(0) * obj_idx_0];
      }
    }
    Problem.ConstraintBoundInternal.set_size(vectorUB);
    for (int i{0}; i < vectorUB; i++) {
      Problem.ConstraintBoundInternal[i] = b[i];
    }
    Problem.EnforceJointLimitsInternal = true;
    if (Problem.EnforceJointLimitsInternal) {
      loop_ub = Problem.DesignVariableBoundsInternal.size(0);
      b_value.set_size(loop_ub, 2);
      vectorUB = Problem.DesignVariableBoundsInternal.size(0) << 1;
      for (int i{0}; i < vectorUB; i++) {
        b_value[i] = Problem.DesignVariableBoundsInternal[i];
      }
      varargout_2_Iterations = Problem.NumPositions;
      Problem.Tree->get_JointPositionLimits(r1);
      if (varargout_2_Iterations < 1.0) {
        vectorUB = 0;
      } else {
        vectorUB = static_cast<int>(varargout_2_Iterations);
      }
      for (int i{0}; i < 2; i++) {
        for (obj_idx_0 = 0; obj_idx_0 < vectorUB; obj_idx_0++) {
          b_value[obj_idx_0 + b_value.size(0) * i] =
              r1[obj_idx_0 + r1.size(0) * i];
        }
      }
      Problem.DesignVariableBoundsInternal.set_size(loop_ub, 2);
      obj_idx_0 = b_value.size(0) << 1;
      for (int i{0}; i < obj_idx_0; i++) {
        Problem.DesignVariableBoundsInternal[i] = b_value[i];
      }
      vectorUB = 2 * b_value.size(0);
      A.set_size(vectorUB, loop_ub);
      obj_idx_0 = vectorUB * b_value.size(0);
      for (int i{0}; i < obj_idx_0; i++) {
        A[i] = 0.0;
      }
      b.set_size(vectorUB);
      for (int i{0}; i < vectorUB; i++) {
        b[i] = 0.0;
      }
      for (int i{0}; i < loop_ub; i++) {
        obj_idx_0 = static_cast<int>(static_cast<unsigned int>(i + 1) << 1);
        A[(obj_idx_0 + A.size(0) * i) - 2] = -1.0;
        A[(obj_idx_0 + A.size(0) * i) - 1] = 1.0;
        b[obj_idx_0 - 2] = -b_value[i];
        b[obj_idx_0 - 1] = b_value[i + b_value.size(0)];
      }
      Problem.ConstraintMatrixInternal.set_size(loop_ub, vectorUB);
      for (int i{0}; i < vectorUB; i++) {
        for (obj_idx_0 = 0; obj_idx_0 < loop_ub; obj_idx_0++) {
          Problem.ConstraintMatrixInternal
              [obj_idx_0 + Problem.ConstraintMatrixInternal.size(0) * i] =
              A[i + A.size(0) * obj_idx_0];
        }
      }
      Problem.ConstraintBoundInternal.set_size(vectorUB);
      for (int i{0}; i < vectorUB; i++) {
        Problem.ConstraintBoundInternal[i] = b[i];
      }
    } else {
      loop_ub = Problem.DesignVariableBoundsInternal.size(0);
      b_value.set_size(loop_ub, 2);
      vectorUB = Problem.DesignVariableBoundsInternal.size(0) << 1;
      for (int i{0}; i < vectorUB; i++) {
        b_value[i] = Problem.DesignVariableBoundsInternal[i];
      }
      vectorUB = static_cast<int>(Problem.NumPositions);
      obj_idx_0 = static_cast<int>(Problem.NumPositions);
      for (int i{0}; i < vectorUB; i++) {
        b_value[i] = rtMinusInf;
      }
      for (int i{0}; i < obj_idx_0; i++) {
        b_value[i + b_value.size(0)] = rtInf;
      }
      Problem.DesignVariableBoundsInternal.set_size(loop_ub, 2);
      obj_idx_0 = b_value.size(0) << 1;
      for (int i{0}; i < obj_idx_0; i++) {
        Problem.DesignVariableBoundsInternal[i] = b_value[i];
      }
      vectorUB = 2 * b_value.size(0);
      A.set_size(vectorUB, loop_ub);
      obj_idx_0 = vectorUB * b_value.size(0);
      for (int i{0}; i < obj_idx_0; i++) {
        A[i] = 0.0;
      }
      b.set_size(vectorUB);
      for (int i{0}; i < vectorUB; i++) {
        b[i] = 0.0;
      }
      for (int i{0}; i < loop_ub; i++) {
        obj_idx_0 = static_cast<int>(static_cast<unsigned int>(i + 1) << 1);
        A[(obj_idx_0 + A.size(0) * i) - 2] = -1.0;
        A[(obj_idx_0 + A.size(0) * i) - 1] = 1.0;
        b[obj_idx_0 - 2] = -b_value[i];
        b[obj_idx_0 - 1] = b_value[i + b_value.size(0)];
      }
      Problem.ConstraintMatrixInternal.set_size(loop_ub, vectorUB);
      for (int i{0}; i < vectorUB; i++) {
        for (obj_idx_0 = 0; obj_idx_0 < loop_ub; obj_idx_0++) {
          Problem.ConstraintMatrixInternal
              [obj_idx_0 + Problem.ConstraintMatrixInternal.size(0) * i] =
              A[i + A.size(0) * obj_idx_0];
        }
      }
      Problem.ConstraintBoundInternal.set_size(vectorUB);
      for (int i{0}; i < vectorUB; i++) {
        Problem.ConstraintBoundInternal[i] = b[i];
      }
    }
    Problem.updateDesignVariableBounds();
    obj_idx_0 = static_cast<int>(Problem.NumVariables);
    Problem.LastX.set_size(obj_idx_0);
    for (int i{0}; i < obj_idx_0; i++) {
      Problem.LastX[i] = 0.0;
    }
    obj.set_size(Problem.LastX.size(0));
    loop_ub = Problem.LastX.size(0) - 1;
    for (int i{0}; i <= loop_ub; i++) {
      obj[i] = Problem.LastX[i];
    }
    Problem.residualsInternal(obj, b, A);
    loop_ub = b.size(0);
    Problem.LastF.set_size(b.size(0));
    for (int i{0}; i < loop_ub; i++) {
      Problem.LastF[i] = b[i];
    }
    Problem.LastJ.set_size(A.size(0), A.size(1));
    obj_idx_0 = A.size(0) * A.size(1);
    for (int i{0}; i < obj_idx_0; i++) {
      Problem.LastJ[i] = A[i];
    }
    char expl_temp[18];
    Problem.matlabCodegenIsDeleted = false;
    Solver->getSolverParams(expl_temp, numResidualsTotal,
                            varargout_2_Iterations, b_expl_temp, c_expl_temp,
                            d_expl_temp, e_expl_temp, f_expl_temp, g_expl_temp,
                            h_expl_temp);
    c_expl_temp = EnforceJointLimits;
    Problem.set_EnforceJointLimits(c_expl_temp);
    Solver->ExtraArgs = &Problem;
    isSetupComplete = true;
  }
  ::std::copy(&varargin_1[0], &varargin_1[9], &varargout_1[0]);
  return stepImpl(aInstancePtr, varargout_1, varargin_2, varargin_3, varargin_4,
                  varargout_2_ConstraintViolations, varargout_2_Status_data,
                  varargout_2_Status_size, varargout_2_NumRandomRestarts,
                  varargout_2_ExitFlag);
}

} // namespace coder
} // namespace gik9dof

//
// File trailer for generalizedInverseKinematics.cpp
//
// [EOF]
//
