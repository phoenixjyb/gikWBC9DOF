//
// gik9dof_codegen_stagedFollowTrajectory.cpp
//
// Code generation for function 'gik9dof_codegen_stagedFollowTrajectory'
//

// Include files
#include "gik9dof_codegen_stagedFollowTrajectory.h"
#include "CollisionSet.h"
#include "RigidBody.h"
#include "RigidBodyTree.h"
#include "atan2.h"
#include "constraintDistanceBounds.h"
#include "constraintJointBounds.h"
#include "constraintPoseTarget.h"
#include "dot.h"
#include "generalizedInverseKinematics.h"
#include "gik9dof_codegen_stagedFollowTrajectory_data.h"
#include "gik9dof_codegen_stagedFollowTrajectory_initialize.h"
#include "gik9dof_codegen_stagedFollowTrajectory_internal_types.h"
#include "gik9dof_codegen_stagedFollowTrajectory_types.h"
#include "loadRobotForCodegen.h"
#include "rigidBodyTree1.h"
#include "rt_nonfinite.h"
#include "solveGIKStepWithLock.h"
#include "stageBPlanPath.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Variable Definitions
static bool robot_not_empty;

static char eeName[17];

// Function Declarations
namespace gik9dof {
namespace codegen {
static void rotmToQuat(const double R[9], double q[4]);

}
} // namespace gik9dof

// Function Definitions
namespace gik9dof {
namespace codegen {
static void rotmToQuat(const double R[9], double q[4])
{
  double normVal;
  double tr;
  double w;
  double x;
  double y;
  tr = (R[0] + R[4]) + R[8];
  if (tr > 0.0) {
    tr = std::sqrt(tr + 1.0) * 2.0;
    w = 0.25 * tr;
    x = (R[5] - R[7]) / tr;
    y = (R[6] - R[2]) / tr;
    tr = (R[1] - R[3]) / tr;
  } else if ((R[0] > R[4]) && (R[0] > R[8])) {
    tr = std::sqrt(((R[0] + 1.0) - R[4]) - R[8]) * 2.0;
    w = (R[5] - R[7]) / tr;
    x = 0.25 * tr;
    y = (R[1] + R[3]) / tr;
    tr = (R[2] + R[6]) / tr;
  } else if (R[4] > R[8]) {
    tr = std::sqrt(((R[4] + 1.0) - R[0]) - R[8]) * 2.0;
    w = (R[6] - R[2]) / tr;
    x = (R[1] + R[3]) / tr;
    y = 0.25 * tr;
    tr = (R[5] + R[7]) / tr;
  } else {
    tr = std::sqrt(((R[8] + 1.0) - R[0]) - R[4]) * 2.0;
    w = (R[1] - R[3]) / tr;
    x = (R[2] + R[6]) / tr;
    y = (R[5] + R[7]) / tr;
    tr *= 0.25;
  }
  normVal = std::sqrt(((w * w + x * x) + y * y) + tr * tr);
  if (normVal > 0.0) {
    q[0] = w / normVal;
    q[1] = x / normVal;
    q[2] = y / normVal;
    q[3] = tr / normVal;
  } else {
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
  }
}

void stagedFollowTrajectory(const double qInit[9], const double poses_data[],
                            const int poses_size[3], double distanceLower,
                            double distanceWeight, const struct0_T *params,
                            const double floorCenters_data[],
                            const int floorCenters_size[2],
                            const double floorRadii_data[], const int[1],
                            const double floorMargins_data[], const int[1],
                            double floorCount, double qOut[4320], int *qCount,
                            int stageCounts[3], double baseStates[600],
                            int *baseCount, double baseCommands[600],
                            int *baseCmdCount, double *sampleTime)
{
  b_struct_T expl_temp;
  double qCurrent[9];
  double qGoalBase[9];
  double hybridMinTurningRadius;
  double hybridMotionPrimitiveLength;
  double maxLinearSpeed;
  double maxPathLengthParam;
  double maxYawRate;
  double rateHz;
  if (!isInitialized_gik9dof_codegen_stagedFollowTrajectory) {
    gik9dof_codegen_stagedFollowTrajectory_initialize();
  }
  // STAGEDFOLLOWTRAJECTORY Execute staged pipeline (Stages A/B/C) for codegen.
  std::memset(&qOut[0], 0, 4320U * sizeof(double));
  std::memset(&baseStates[0], 0, 600U * sizeof(double));
  std::memset(&baseCommands[0], 0, 600U * sizeof(double));
  *qCount = 0;
  *baseCount = 0;
  *baseCmdCount = 0;
  stageCounts[0] = 0;
  stageCounts[1] = 0;
  stageCounts[2] = 0;
  if (!robot_not_empty) {
    loadRobotForCodegen();
    robot_not_empty = true;
  }
  std::copy(&qInit[0], &qInit[9], &qCurrent[0]);
  rateHz = params->rateHz;
  if (std::isinf(params->rateHz) || std::isnan(params->rateHz) ||
      (!(params->rateHz > 0.0))) {
    rateHz = 100.0;
  }
  *sampleTime = 1.0 / rateHz;
  maxLinearSpeed = params->maxLinearSpeed;
  if (std::isinf(params->maxLinearSpeed) ||
      std::isnan(params->maxLinearSpeed) || (!(params->maxLinearSpeed > 0.0))) {
    maxLinearSpeed = 1.5;
  }
  maxYawRate = params->maxYawRate;
  if (std::isinf(params->maxYawRate) || std::isnan(params->maxYawRate) ||
      (!(params->maxYawRate > 0.0))) {
    maxYawRate = 3.0;
  }
  maxPathLengthParam = params->maxPathLength;
  if (std::isinf(params->maxPathLength) || std::isnan(params->maxPathLength) ||
      (!(params->maxPathLength > 0.0))) {
    maxPathLengthParam = 25.0;
  }
  hybridMinTurningRadius = params->hybridMinTurningRadius;
  if (std::isinf(params->hybridMinTurningRadius) ||
      std::isnan(params->hybridMinTurningRadius) ||
      (!(params->hybridMinTurningRadius > 0.05))) {
    hybridMinTurningRadius = 0.5;
  }
  hybridMotionPrimitiveLength = params->hybridMotionPrimitiveLength;
  if (std::isinf(params->hybridMotionPrimitiveLength) ||
      std::isnan(params->hybridMotionPrimitiveLength) ||
      (!(params->hybridMotionPrimitiveLength > 0.05))) {
    hybridMotionPrimitiveLength = 0.5;
  }
  if (poses_size[2] == 0) {
    std::copy(&qInit[0], &qInit[9], &qOut[0]);
    *qCount = 1;
  } else {
    double TStart[16];
    double cosTheta;
    double normVal;
    double qGoalBase_tmp;
    int b_i;
    int i;
    int stageASteps;
    int stageCsteps;
    bool lockMask[9];
    robot.getTransform(qInit, eeName, TStart);
    //  Stage A: arm ramp with base locked
    stageASteps = params->stageA_samples;
    if (params->stageA_samples < 0) {
      stageASteps = 0;
    } else if (params->stageA_samples > 80) {
      stageASteps = 80;
    }
    if (stageASteps <= 1) {
      stageCounts[0] = stageASteps;
    } else {
      for (i = 0; i < 9; i++) {
        lockMask[i] = false;
      }
      lockMask[0] = true;
      lockMask[1] = true;
      lockMask[2] = true;
      for (stageCsteps = 0; stageCsteps < stageASteps; stageCsteps++) {
        double T[16];
        double quatGoal[4];
        double quatStart[4];
        double a;
        double alpha;
        double y_idx_1;
        double y_idx_2;
        alpha = static_cast<double>(stageCsteps) /
                (static_cast<double>(stageASteps) - 1.0);
        for (b_i = 0; b_i < 3; b_i++) {
          i = b_i << 2;
          qGoalBase[3 * b_i] = TStart[i];
          qGoalBase[3 * b_i + 1] = TStart[i + 1];
          qGoalBase[3 * b_i + 2] = TStart[i + 2];
        }
        rotmToQuat(qGoalBase, quatStart);
        for (b_i = 0; b_i < 3; b_i++) {
          qGoalBase[3 * b_i] = poses_data[4 * b_i];
          qGoalBase[3 * b_i + 1] = poses_data[4 * b_i + 1];
          qGoalBase[3 * b_i + 2] = poses_data[4 * b_i + 2];
        }
        rotmToQuat(qGoalBase, quatGoal);
        normVal = std::sqrt(
            ((quatStart[0] * quatStart[0] + quatStart[1] * quatStart[1]) +
             quatStart[2] * quatStart[2]) +
            quatStart[3] * quatStart[3]);
        if (normVal > 0.0) {
          quatStart[0] /= normVal;
          quatStart[1] /= normVal;
          quatStart[2] /= normVal;
          quatStart[3] /= normVal;
        } else {
          quatStart[0] = 1.0;
          quatStart[1] = 0.0;
          quatStart[2] = 0.0;
          quatStart[3] = 0.0;
        }
        normVal =
            std::sqrt(((quatGoal[0] * quatGoal[0] + quatGoal[1] * quatGoal[1]) +
                       quatGoal[2] * quatGoal[2]) +
                      quatGoal[3] * quatGoal[3]);
        if (normVal > 0.0) {
          quatGoal[0] /= normVal;
          quatGoal[1] /= normVal;
          quatGoal[2] /= normVal;
          quatGoal[3] /= normVal;
        } else {
          quatGoal[0] = 1.0;
          quatGoal[1] = 0.0;
          quatGoal[2] = 0.0;
          quatGoal[3] = 0.0;
        }
        if (coder::dot(quatStart, quatGoal) < 0.0) {
          quatGoal[0] = -quatGoal[0];
          quatGoal[1] = -quatGoal[1];
          quatGoal[2] = -quatGoal[2];
          quatGoal[3] = -quatGoal[3];
        }
        cosTheta = coder::dot(quatStart, quatGoal);
        if (cosTheta > 0.9995) {
          qGoalBase_tmp = (1.0 - alpha) * quatStart[0] + alpha * quatGoal[0];
          quatStart[0] = qGoalBase_tmp;
          normVal = qGoalBase_tmp * qGoalBase_tmp;
          qGoalBase_tmp = (1.0 - alpha) * quatStart[1] + alpha * quatGoal[1];
          quatStart[1] = qGoalBase_tmp;
          y_idx_1 = qGoalBase_tmp * qGoalBase_tmp;
          qGoalBase_tmp = (1.0 - alpha) * quatStart[2] + alpha * quatGoal[2];
          quatStart[2] = qGoalBase_tmp;
          y_idx_2 = qGoalBase_tmp * qGoalBase_tmp;
          qGoalBase_tmp = (1.0 - alpha) * quatStart[3] + alpha * quatGoal[3];
          normVal = std::sqrt(((normVal + y_idx_1) + y_idx_2) +
                              qGoalBase_tmp * qGoalBase_tmp);
          if (normVal > 0.0) {
            quatStart[0] /= normVal;
            quatStart[1] /= normVal;
            quatStart[2] /= normVal;
            quatStart[3] = qGoalBase_tmp / normVal;
          } else {
            quatStart[0] = 1.0;
            quatStart[1] = 0.0;
            quatStart[2] = 0.0;
            quatStart[3] = 0.0;
          }
        } else {
          cosTheta = std::acos(std::fmax(std::fmin(cosTheta, 1.0), -1.0));
          normVal = std::sin(cosTheta);
          a = std::sin((1.0 - alpha) * cosTheta) / normVal;
          cosTheta = std::sin(alpha * cosTheta) / normVal;
          qGoalBase_tmp = a * quatStart[0] + cosTheta * quatGoal[0];
          quatStart[0] = qGoalBase_tmp;
          normVal = qGoalBase_tmp * qGoalBase_tmp;
          qGoalBase_tmp = a * quatStart[1] + cosTheta * quatGoal[1];
          quatStart[1] = qGoalBase_tmp;
          y_idx_1 = qGoalBase_tmp * qGoalBase_tmp;
          qGoalBase_tmp = a * quatStart[2] + cosTheta * quatGoal[2];
          quatStart[2] = qGoalBase_tmp;
          y_idx_2 = qGoalBase_tmp * qGoalBase_tmp;
          qGoalBase_tmp = a * quatStart[3] + cosTheta * quatGoal[3];
          normVal = std::sqrt(((normVal + y_idx_1) + y_idx_2) +
                              qGoalBase_tmp * qGoalBase_tmp);
          if (normVal > 0.0) {
            quatStart[0] /= normVal;
            quatStart[1] /= normVal;
            quatStart[2] /= normVal;
            quatStart[3] = qGoalBase_tmp / normVal;
          } else {
            quatStart[0] = 1.0;
            quatStart[1] = 0.0;
            quatStart[2] = 0.0;
            quatStart[3] = 0.0;
          }
        }
        for (b_i = 0; b_i < 16; b_i++) {
          T[b_i] = iv[b_i];
        }
        normVal = std::sqrt(
            ((quatStart[0] * quatStart[0] + quatStart[1] * quatStart[1]) +
             quatStart[2] * quatStart[2]) +
            quatStart[3] * quatStart[3]);
        if (normVal > 0.0) {
          quatStart[0] /= normVal;
          quatStart[1] /= normVal;
          quatStart[2] /= normVal;
          quatStart[3] /= normVal;
        } else {
          quatStart[0] = 1.0;
          quatStart[1] = 0.0;
          quatStart[2] = 0.0;
          quatStart[3] = 0.0;
        }
        cosTheta = quatStart[3] * quatStart[3];
        normVal = quatStart[2] * quatStart[2];
        qGoalBase[0] = 1.0 - 2.0 * (normVal + cosTheta);
        y_idx_1 = quatStart[1] * quatStart[2];
        y_idx_2 = quatStart[0] * quatStart[3];
        qGoalBase[3] = 2.0 * (y_idx_1 - y_idx_2);
        qGoalBase_tmp = quatStart[1] * quatStart[3];
        a = quatStart[0] * quatStart[2];
        qGoalBase[6] = 2.0 * (qGoalBase_tmp + a);
        qGoalBase[1] = 2.0 * (y_idx_1 + y_idx_2);
        y_idx_1 = quatStart[1] * quatStart[1];
        qGoalBase[4] = 1.0 - 2.0 * (y_idx_1 + cosTheta);
        cosTheta = quatStart[2] * quatStart[3];
        y_idx_2 = quatStart[0] * quatStart[1];
        qGoalBase[7] = 2.0 * (cosTheta - y_idx_2);
        qGoalBase[2] = 2.0 * (qGoalBase_tmp - a);
        qGoalBase[5] = 2.0 * (cosTheta + y_idx_2);
        qGoalBase[8] = 1.0 - 2.0 * (y_idx_1 + normVal);
        for (b_i = 0; b_i < 3; b_i++) {
          i = b_i << 2;
          T[i] = qGoalBase[3 * b_i];
          T[i + 1] = qGoalBase[3 * b_i + 1];
          T[i + 2] = qGoalBase[3 * b_i + 2];
        }
        T[12] = TStart[12];
        T[13] = TStart[13];
        T[14] = TStart[14] * (1.0 - alpha) + poses_data[14] * alpha;
        std::copy(&qCurrent[0], &qCurrent[9], &qGoalBase[0]);
        solveGIKStepWithLock(qGoalBase, T, distanceLower, distanceWeight,
                             lockMask, qCurrent);
        if (*qCount > 2147483646) {
          *qCount = MAX_int32_T;
        } else {
          (*qCount)++;
        }
        std::copy(&qCurrent[0], &qCurrent[9], &qOut[*qCount * 9 + -9]);
      }
      stageCounts[0] = stageASteps;
    }
    //  Stage B: plan base path with arm frozen
    for (i = 0; i < 9; i++) {
      lockMask[i] = false;
    }
    for (i = 0; i < 6; i++) {
      lockMask[i + 3] = true;
    }
    solveGIKStepWithLock(qCurrent, &poses_data[0], distanceLower,
                         distanceWeight, lockMask, qGoalBase);
    expl_temp.maxYawRate = maxYawRate;
    expl_temp.maxLinearSpeed = maxLinearSpeed;
    expl_temp.rateHz = rateHz;
    expl_temp.sampleTime = *sampleTime;
    expl_temp.rotationStep = params->hybridRotationStep;
    expl_temp.headingBins = params->hybridHeadingBins;
    expl_temp.motionPrimitiveLength = hybridMotionPrimitiveLength;
    expl_temp.minTurningRadius = hybridMinTurningRadius;
    expl_temp.maxPathLength = maxPathLengthParam;
    expl_temp.safetyMargin = params->hybridSafetyMargin;
    expl_temp.resolution = params->hybridResolution;
    *baseCount = stageBPlanPath(&qCurrent[0], &qGoalBase[0], floorCenters_data,
                                floorCenters_size, floorRadii_data,
                                floorMargins_data, floorCount, expl_temp,
                                baseStates, baseCommands, baseCmdCount);
    if (*baseCount < 2) {
      baseStates[0] = qCurrent[0];
      baseStates[3] = qGoalBase[0];
      baseStates[1] = qCurrent[1];
      baseStates[4] = qGoalBase[1];
      baseStates[2] = qCurrent[2];
      baseStates[5] = qGoalBase[2];
      *baseCount = 2;
      *baseCmdCount = 0;
    }
    if (*baseCmdCount > 0) {
      baseCommands[0] = 0.0;
      baseCommands[1] = 0.0;
      baseCommands[2] = 0.0;
      if (*baseCmdCount > *baseCount) {
        *baseCmdCount = *baseCount;
      }
    } else {
      std::memset(&baseCommands[0], 0, 600U * sizeof(double));
      baseCommands[0] = 0.0;
      baseCommands[1] = 0.0;
      baseCommands[2] = 0.0;
      for (stageASteps = 0; stageASteps <= *baseCount - 2; stageASteps++) {
        qGoalBase_tmp = baseStates[3 * stageASteps + 2];
        i = 3 * (stageASteps + 1);
        baseCommands[i] = std::fmax(
            -maxLinearSpeed,
            std::fmin(
                maxLinearSpeed,
                std::cos(qGoalBase_tmp) *
                        ((baseStates[i] - baseStates[3 * stageASteps]) /
                         *sampleTime) +
                    std::sin(qGoalBase_tmp) *
                        ((baseStates[i + 1] - baseStates[3 * stageASteps + 1]) /
                         *sampleTime)));
        cosTheta = baseStates[i + 2] - qGoalBase_tmp;
        baseCommands[i + 1] = std::fmax(
            -maxYawRate,
            std::fmin(maxYawRate, coder::internal::scalar::b_atan2(
                                      std::sin(cosTheta), std::cos(cosTheta)) /
                                      *sampleTime));
        baseCommands[i + 2] =
            ((static_cast<double>(stageASteps) + 2.0) - 1.0) * *sampleTime;
      }
      *baseCmdCount = *baseCount;
    }
    for (stageASteps = 0; stageASteps <= *baseCount - 2; stageASteps++) {
      i = 3 * (stageASteps + 1);
      qCurrent[0] = baseStates[i];
      qCurrent[1] = baseStates[i + 1];
      qCurrent[2] = baseStates[i + 2];
      if (*qCount > 2147483646) {
        *qCount = MAX_int32_T;
      } else {
        (*qCount)++;
      }
      std::copy(&qCurrent[0], &qCurrent[9], &qOut[*qCount * 9 + -9]);
    }
    if (*baseCount < -2147483647) {
      i = MIN_int32_T;
    } else {
      i = *baseCount - 1;
    }
    stageCounts[1] = i;
    //  Stage C: full-body tracking for remaining poses
    if (poses_size[2] > 1) {
      stageCsteps = 0;
      b_i = poses_size[2];
      if (distanceLower <= 0.0) {
        cosTheta = 0.0;
      } else {
        cosTheta = distanceLower;
      }
      if (distanceWeight < 0.0) {
        normVal = 0.0;
      } else {
        normVal = distanceWeight;
      }
      for (int poseIdx{0}; poseIdx <= b_i - 2; poseIdx++) {
        // SOLVEGIKSTEPWITHLOCK Generalized IK step with optional joint locks.
        if (!solver_not_empty) {
          int b_iv[2];
          b_robot = loadRobotForCodegen();
          solver.init(b_robot);
          solver_not_empty = true;
          poseConstraint.init();
          jointConstraint.init(b_robot);
          distanceConstraint.init();
          distanceConstraint.ReferenceBody.reserve(200);
          b_robot->get_BaseName((char *)distanceConstraint.ReferenceBody.data(),
                                b_iv);
          (*(int(*)[2])distanceConstraint.ReferenceBody.size())[0] = b_iv[0];
          (*(int(*)[2])distanceConstraint.ReferenceBody.size())[1] = b_iv[1];
          distanceConstraint.ReferenceBody.set_size(
              distanceConstraint.ReferenceBody.size(0),
              distanceConstraint.ReferenceBody.size(1));
          jointBoundsDefault.set_size(jointConstraint.BoundsInternal.size(0),
                                      2);
          stageASteps = jointConstraint.BoundsInternal.size(0) << 1;
          for (int i1{0}; i1 < stageASteps; i1++) {
            jointBoundsDefault[i1] = jointConstraint.BoundsInternal[i1];
          }
        }
        for (int i1{0}; i1 < 4; i1++) {
          stageASteps = i1 << 2;
          i = 4 * i1 + 16 * (poseIdx + 1);
          poseConstraint.TargetTransform[stageASteps] = poses_data[i];
          poseConstraint.TargetTransform[stageASteps + 1] = poses_data[i + 1];
          poseConstraint.TargetTransform[stageASteps + 2] = poses_data[i + 2];
          poseConstraint.TargetTransform[stageASteps + 3] = poses_data[i + 3];
        }
        distanceConstraint.Bounds[0] = cosTheta;
        distanceConstraint.Bounds[1] = rtInf;
        distanceConstraint.Weights = normVal;
        //  Restore default joint bounds then apply lock mask.
        jointConstraint.BoundsInternal.set_size(jointBoundsDefault.size(0), 2);
        stageASteps = jointBoundsDefault.size(0) << 1;
        for (int i1{0}; i1 < stageASteps; i1++) {
          jointConstraint.BoundsInternal[i1] = jointBoundsDefault[i1];
        }
        std::copy(&qCurrent[0], &qCurrent[9], &qGoalBase[0]);
        solver.step(qGoalBase, poseConstraint, jointConstraint,
                    distanceConstraint, qCurrent);
        if (*qCount > 2147483646) {
          *qCount = MAX_int32_T;
        } else {
          (*qCount)++;
        }
        std::copy(&qCurrent[0], &qCurrent[9], &qOut[*qCount * 9 + -9]);
        stageCsteps++;
      }
      stageCounts[2] = stageCsteps;
    } else {
      stageCounts[2] = 0;
    }
    //  Ensure at least one sample stored
    if (*qCount == 0) {
      *qCount = 1;
      std::copy(&qCurrent[0], &qCurrent[9], &qOut[0]);
    }
  }
}

} // namespace codegen
} // namespace gik9dof
void stagedFollowTrajectory_delete()
{
  coder::robotics::manip::internal::RigidBody *obj;
  if (!robot.matlabCodegenIsDeleted) {
    robot.matlabCodegenIsDeleted = true;
  }
  if (!robot.TreeInternal.matlabCodegenIsDeleted) {
    robot.TreeInternal.matlabCodegenIsDeleted = true;
  }
  if (!robot.TreeInternal.Base.matlabCodegenIsDeleted) {
    robot.TreeInternal.Base.matlabCodegenIsDeleted = true;
  }
  for (int i{0}; i < 12; i++) {
    obj = &robot.TreeInternal._pobj2[i];
    if (!obj->matlabCodegenIsDeleted) {
      obj->matlabCodegenIsDeleted = true;
    }
  }
  robot.TreeInternal.Base._pobj0.matlabCodegenDestructor();
  robot.TreeInternal._pobj0[0].matlabCodegenDestructor();
  robot.TreeInternal._pobj0[1].matlabCodegenDestructor();
  robot.TreeInternal._pobj0[2].matlabCodegenDestructor();
  for (int i{0}; i < 12; i++) {
    robot.TreeInternal._pobj2[i]._pobj0.matlabCodegenDestructor();
  }
  robot._pobj0.matlabCodegenDestructor();
}

void stagedFollowTrajectory_init()
{
  for (int i{0}; i < 17; i++) {
    eeName[i] = cv12[i];
  }
  robot_not_empty = false;
}

void stagedFollowTrajectory_new()
{
  robot._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 12; i++) {
    robot.TreeInternal._pobj2[i]._pobj0.matlabCodegenIsDeleted = true;
  }
  robot.TreeInternal._pobj0[0].matlabCodegenIsDeleted = true;
  robot.TreeInternal._pobj0[1].matlabCodegenIsDeleted = true;
  robot.TreeInternal._pobj0[2].matlabCodegenIsDeleted = true;
  robot.TreeInternal.Base._pobj0.matlabCodegenIsDeleted = true;
  for (int i{0}; i < 12; i++) {
    robot.TreeInternal._pobj2[i].matlabCodegenIsDeleted = true;
  }
  robot.TreeInternal.Base.matlabCodegenIsDeleted = true;
  robot.TreeInternal.matlabCodegenIsDeleted = true;
  robot.matlabCodegenIsDeleted = true;
}

// End of code generation (gik9dof_codegen_stagedFollowTrajectory.cpp)
