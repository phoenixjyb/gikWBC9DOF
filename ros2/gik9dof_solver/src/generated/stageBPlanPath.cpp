//
// stageBPlanPath.cpp
//
// Code generation for function 'stageBPlanPath'
//

// Include files
#include "stageBPlanPath.h"
#include "gik9dof_codegen_stagedFollowTrajectory_internal_types.h"
#include "hypot.h"
#include "minOrMax.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Declarations
namespace gik9dof {
namespace codegen {
static bool collisionAtPoint(double x, double y,
                             const double floorCenters_data[],
                             const int floorCenters_size[2],
                             const double floorRadii_data[],
                             const double floorMargins_data[], int numDiscs,
                             double safetyMargin);

static int computePurePursuitCommands(const double states[600], int stateCount,
                                      double sampleTime, double desiredSpeed,
                                      double maxYawRate, double cmds[600]);

static double propagatePrimitive(
    double x, double y, double yaw, double deltaS, double curvature,
    double deltaYaw, const double minXY[2], const double maxXY[2],
    double resolution, double sampleStep, const double floorCenters_data[],
    const int floorCenters_size[2], const double floorRadii_data[],
    const double floorMargins_data[], int numDiscs, double safetyMargin,
    double minTurningRadius, double &yNext, double &yawNext, double &travel,
    bool &validMove);

static bool reachedGoal(double x, double y, double yaw,
                        const double goalState[3], double posTol,
                        double yawTol);

static int selectBestOpen(const bool openSet[12000], const double fCost[12000],
                          int nodeCount);

static int worldToGrid(double x, double y, const double minXY[2],
                       double resolution, int ny, int nx, int &col,
                       bool &valid);

} // namespace codegen
} // namespace gik9dof

// Function Definitions
namespace gik9dof {
namespace codegen {
static bool collisionAtPoint(double x, double y,
                             const double floorCenters_data[],
                             const int floorCenters_size[2],
                             const double floorRadii_data[],
                             const double floorMargins_data[], int numDiscs,
                             double safetyMargin)
{
  int idx;
  bool exitg1;
  bool hit;
  hit = false;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= numDiscs - 1)) {
    double dx;
    double dy;
    double rr;
    dx = x - floorCenters_data[idx];
    dy = y - floorCenters_data[idx + floorCenters_size[0]];
    rr = (floorRadii_data[idx] + floorMargins_data[idx]) + safetyMargin;
    if (dx * dx + dy * dy <= rr * rr) {
      hit = true;
      exitg1 = true;
    } else {
      idx++;
    }
  }
  return hit;
}

static int computePurePursuitCommands(const double states[600], int stateCount,
                                      double sampleTime, double desiredSpeed,
                                      double maxYawRate, double cmds[600])
{
  double path_data[600];
  double cumDist_data[200];
  int cmdCount;
  int i;
  int startIdx;
  std::memset(&cmds[0], 0, 600U * sizeof(double));
  for (i = 0; i < 3; i++) {
    for (startIdx = 0; startIdx < stateCount; startIdx++) {
      path_data[startIdx + stateCount * i] = states[i + 3 * startIdx];
    }
  }
  if (stateCount - 1 >= 0) {
    std::memset(&cumDist_data[0], 0,
                static_cast<unsigned int>(stateCount) * sizeof(double));
  }
  for (int b_i{2}; b_i <= stateCount; b_i++) {
    startIdx = b_i + stateCount;
    cumDist_data[b_i - 1] =
        cumDist_data[b_i - 2] +
        coder::b_hypot(path_data[b_i - 1] - path_data[b_i - 2],
                       path_data[startIdx - 1] - path_data[startIdx - 2]);
  }
  cmdCount = 1;
  i = static_cast<unsigned char>(stateCount);
  for (int b_i{0}; b_i < i; b_i++) {
    double d;
    double nearestDist;
    int idx;
    int nearestIdx;
    nearestIdx = 1;
    nearestDist = rtInf;
    startIdx =
        coder::internal::minimum2(static_cast<double>(stateCount), cmdCount);
    if (startIdx < 1) {
      startIdx = 1;
    }
    for (idx = startIdx; idx <= stateCount; idx++) {
      d = coder::b_hypot(path_data[idx - 1] - states[3 * b_i],
                         path_data[(idx + stateCount) - 1] -
                             states[3 * b_i + 1]);
      if (d < nearestDist) {
        nearestDist = d;
        nearestIdx = idx;
      }
    }
    cmdCount = nearestIdx;
    while ((cmdCount < stateCount) &&
           (cumDist_data[cmdCount] - cumDist_data[nearestIdx - 1] < 0.6)) {
      cmdCount++;
    }
    cmds[3 * b_i] = desiredSpeed;
    startIdx = 3 * b_i + 2;
    d = states[startIdx];
    idx = 3 * b_i + 1;
    d = 2.0 *
        (-std::sin(d) * (path_data[cmdCount - 1] - states[3 * b_i]) +
         std::cos(d) * (path_data[(cmdCount + stateCount) - 1] - states[idx])) /
        0.36 * desiredSpeed;
    if ((nearestDist <= 0.15) && (cmdCount == stateCount)) {
      cmds[3 * b_i] = 0.0;
      d = 0.0;
    }
    cmds[idx] = std::fmax(-maxYawRate, std::fmin(maxYawRate, d));
    cmds[startIdx] = ((static_cast<double>(b_i) + 1.0) - 1.0) * sampleTime;
  }
  return stateCount;
}

static double propagatePrimitive(
    double x, double y, double yaw, double deltaS, double curvature,
    double deltaYaw, const double minXY[2], const double maxXY[2],
    double resolution, double sampleStep, const double floorCenters_data[],
    const int floorCenters_size[2], const double floorRadii_data[],
    const double floorMargins_data[], int numDiscs, double safetyMargin,
    double minTurningRadius, double &yNext, double &yawNext, double &travel,
    bool &validMove)
{
  double d;
  double radius;
  double rr;
  double xNext;
  double yawPrev;
  int idx;
  bool exitg1;
  bool guard1;
  bool hit;
  validMove = true;
  yawNext = coder::b_mod(yaw + 3.1415926535897931) - 3.1415926535897931;
  xNext = x;
  yNext = y;
  travel = 0.0;
  d = std::abs(deltaS);
  guard1 = false;
  if (d > 1.0E-9) {
    double sStep;
    double xCurr;
    double yCurr;
    double yawCurr;
    double yawStep;
    int i;
    int k;
    yawPrev =
        std::fmax(1.0, std::ceil(d / std::fmax(sampleStep, resolution * 0.5)));
    if (yawPrev < 2.147483648E+9) {
      i = static_cast<int>(yawPrev);
    } else {
      i = MAX_int32_T;
    }
    sStep = deltaS / static_cast<double>(i);
    yawStep = curvature * sStep;
    xCurr = x;
    yCurr = y;
    yawCurr = yaw;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= i - 1)) {
      yawPrev = yawCurr;
      yawCurr += yawStep;
      yawCurr = coder::b_mod(yawCurr + 3.1415926535897931);
      yawCurr -= 3.1415926535897931;
      if (std::abs(curvature) < 1.0E-6) {
        xCurr += sStep * std::cos(yawPrev);
        yCurr += sStep * std::sin(yawPrev);
      } else {
        radius = 1.0 / curvature;
        xCurr += radius * (std::sin(yawCurr) - std::sin(yawPrev));
        yCurr -= radius * (std::cos(yawCurr) - std::cos(yawPrev));
      }
      if ((xCurr < minXY[0]) || (xCurr > maxXY[0]) || (yCurr < minXY[1]) ||
          (yCurr > maxXY[1])) {
        validMove = false;
        exitg1 = true;
      } else {
        bool exitg2;
        hit = false;
        idx = 0;
        exitg2 = false;
        while ((!exitg2) && (idx <= numDiscs - 1)) {
          yawPrev = xCurr - floorCenters_data[idx];
          radius = yCurr - floorCenters_data[idx + floorCenters_size[0]];
          rr = (floorRadii_data[idx] + floorMargins_data[idx]) + safetyMargin;
          if (yawPrev * yawPrev + radius * radius <= rr * rr) {
            hit = true;
            exitg2 = true;
          } else {
            idx++;
          }
        }
        if (hit) {
          validMove = false;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (validMove) {
      xNext = xCurr;
      yNext = yCurr;
      yawNext = coder::b_mod(yawCurr + 3.1415926535897931) - 3.1415926535897931;
      travel = d;
      guard1 = true;
    }
  } else {
    d = std::abs(deltaYaw);
    if (d > 1.0E-9) {
      double yawCurr;
      double yawStep;
      int i;
      yawPrev = std::ceil(d / 0.2);
      if (yawPrev < 2.147483648E+9) {
        i = static_cast<int>(yawPrev);
      } else {
        i = MAX_int32_T;
      }
      yawStep = deltaYaw / static_cast<double>(i);
      yawCurr = yaw;
      for (int k{0}; k < i; k++) {
        yawCurr += yawStep;
        yawCurr = coder::b_mod(yawCurr + 3.1415926535897931);
        yawCurr -= 3.1415926535897931;
      }
      yawNext = yawCurr;
      travel = d * minTurningRadius;
    }
    guard1 = true;
  }
  if (guard1) {
    hit = false;
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx <= numDiscs - 1)) {
      yawPrev = xNext - floorCenters_data[idx];
      radius = yNext - floorCenters_data[idx + floorCenters_size[0]];
      rr = (floorRadii_data[idx] + floorMargins_data[idx]) + safetyMargin;
      if (yawPrev * yawPrev + radius * radius <= rr * rr) {
        hit = true;
        exitg1 = true;
      } else {
        idx++;
      }
    }
    validMove = !hit;
  }
  return xNext;
}

static bool reachedGoal(double x, double y, double yaw,
                        const double goalState[3], double posTol, double yawTol)
{
  bool reached;
  if (coder::b_hypot(goalState[0] - x, goalState[1] - y) > posTol) {
    reached = false;
  } else {
    reached =
        (std::abs(coder::b_mod((goalState[2] - yaw) + 3.1415926535897931) -
                  3.1415926535897931) <= yawTol);
  }
  return reached;
}

static int selectBestOpen(const bool openSet[12000], const double fCost[12000],
                          int nodeCount)
{
  double bestCost;
  int nodeIdx;
  nodeIdx = 0;
  bestCost = rtInf;
  for (int idx{0}; idx < nodeCount; idx++) {
    if (openSet[idx] && (fCost[idx] < bestCost)) {
      bestCost = fCost[idx];
      if (static_cast<unsigned int>(idx) + 1U < 2147483648U) {
        nodeIdx = idx + 1;
      } else {
        nodeIdx = MAX_int32_T;
      }
    }
  }
  return nodeIdx;
}

static int worldToGrid(double x, double y, const double minXY[2],
                       double resolution, int ny, int nx, int &col, bool &valid)
{
  double d;
  int row;
  d = std::floor((x - minXY[0]) / resolution) + 1.0;
  if (d < 2.147483648E+9) {
    if (d >= -2.147483648E+9) {
      col = static_cast<int>(d);
    } else {
      col = MIN_int32_T;
    }
  } else if (d >= 2.147483648E+9) {
    col = MAX_int32_T;
  } else {
    col = 0;
  }
  d = std::floor((y - minXY[1]) / resolution) + 1.0;
  if (d < 2.147483648E+9) {
    if (d >= -2.147483648E+9) {
      row = static_cast<int>(d);
    } else {
      row = MIN_int32_T;
    }
  } else if (d >= 2.147483648E+9) {
    row = MAX_int32_T;
  } else {
    row = 0;
  }
  if ((row >= 1) && (row <= ny) && (col >= 1) && (col <= nx)) {
    valid = true;
  } else {
    valid = false;
  }
  if (!valid) {
    row = 1;
    col = 1;
  }
  return row;
}

int stageBPlanPath(const double startState[3], const double goalState[3],
                   const double floorCenters_data[],
                   const int floorCenters_size[2],
                   const double floorRadii_data[],
                   const double floorMargins_data[], double floorCount,
                   const b_struct_T &params, double states[600],
                   double commands[600], int *commandCount)
{
  static double fCost[12000];
  static double gCost[12000];
  static double nodeX[12000];
  static double nodeY[12000];
  static double nodeYaw[12000];
  static int gridToNode[4718592];
  double maxXY[2];
  double minXY[2];
  double cy;
  double d;
  double dy;
  double maxLinearSpeed;
  double maxPathLength;
  double maxYawRate;
  double minTurningRadius;
  double motionPrimitiveLength;
  double rateHz;
  double resolution;
  double rotationStep;
  double safetyMargin;
  double sampleTime;
  double yawDiff;
  int count;
  int k;
  int nodeCount;
  int numDiscs;
  int nx;
  int ny;
  int thetaBins;
  bool validGoal;
  bool validStart;
  // STAGEBPLANPATH Plan base-only trajectory via Hybrid A* primitives and
  // derive commands.
  std::memset(&states[0], 0, 600U * sizeof(double));
  std::memset(&commands[0], 0, 600U * sizeof(double));
  count = 0;
  *commandCount = 0;
  resolution = params.resolution;
  if (std::isinf(params.resolution) || std::isnan(params.resolution) ||
      (!(params.resolution > 0.0))) {
    resolution = 0.1;
  }
  safetyMargin = params.safetyMargin;
  if (std::isinf(params.safetyMargin) || std::isnan(params.safetyMargin) ||
      (!(params.safetyMargin >= 0.0))) {
    safetyMargin = 0.15;
  }
  minTurningRadius = params.minTurningRadius;
  if (std::isinf(params.minTurningRadius) ||
      std::isnan(params.minTurningRadius) ||
      (!(params.minTurningRadius > 0.05))) {
    minTurningRadius = 0.5;
  }
  motionPrimitiveLength = params.motionPrimitiveLength;
  if (std::isinf(params.motionPrimitiveLength) ||
      std::isnan(params.motionPrimitiveLength) ||
      (!(params.motionPrimitiveLength > 0.05))) {
    motionPrimitiveLength = 0.5;
  }
  rotationStep = params.rotationStep;
  if (std::isinf(params.rotationStep) || std::isnan(params.rotationStep) ||
      (!(params.rotationStep > 0.05))) {
    rotationStep = 0.35;
  }
  maxPathLength = params.maxPathLength;
  if (std::isinf(params.maxPathLength) || std::isnan(params.maxPathLength) ||
      (!(params.maxPathLength > 0.0))) {
    maxPathLength = 25.0;
  }
  rateHz = params.rateHz;
  if (std::isinf(params.rateHz) || std::isnan(params.rateHz) ||
      (!(params.rateHz > 0.0))) {
    rateHz = 100.0;
  }
  sampleTime = params.sampleTime;
  if (std::isinf(params.sampleTime) || std::isnan(params.sampleTime) ||
      (!(params.sampleTime > 0.0))) {
    sampleTime = 1.0 / rateHz;
  }
  maxLinearSpeed = params.maxLinearSpeed;
  if (std::isinf(params.maxLinearSpeed) || std::isnan(params.maxLinearSpeed) ||
      (!(params.maxLinearSpeed > 0.0))) {
    maxLinearSpeed = 1.5;
  }
  maxYawRate = params.maxYawRate;
  if (std::isinf(params.maxYawRate) || std::isnan(params.maxYawRate) ||
      (!(params.maxYawRate > 0.0))) {
    maxYawRate = 3.0;
  }
  d = std::round(params.headingBins);
  if (d < 2.147483648E+9) {
    if (d >= -2.147483648E+9) {
      thetaBins = static_cast<int>(d);
    } else {
      thetaBins = MIN_int32_T;
    }
  } else if (d >= 2.147483648E+9) {
    thetaBins = MAX_int32_T;
  } else {
    thetaBins = 0;
  }
  if (thetaBins < 8) {
    thetaBins = 8;
  } else if (thetaBins > 72) {
    thetaBins = 72;
  }
  minXY[0] = startState[0];
  minXY[1] = startState[1];
  maxXY[0] = startState[0];
  maxXY[1] = startState[1];
  if (goalState[0] < startState[0]) {
    minXY[0] = goalState[0];
  } else if (goalState[0] > startState[0]) {
    maxXY[0] = goalState[0];
  }
  if (goalState[1] < startState[1]) {
    minXY[1] = goalState[1];
  } else if (goalState[1] > startState[1]) {
    maxXY[1] = goalState[1];
  }
  numDiscs = static_cast<int>(std::round(std::fmin(
      static_cast<double>(floorCenters_size[0]), std::fmax(0.0, floorCount))));
  for (k = 0; k < numDiscs; k++) {
    rateHz = floorCenters_data[k];
    cy = floorCenters_data[k + floorCenters_size[0]];
    if (rateHz < minXY[0]) {
      minXY[0] = rateHz;
    } else if (rateHz > maxXY[0]) {
      maxXY[0] = rateHz;
    }
    if (cy < minXY[1]) {
      minXY[1] = cy;
    } else if (cy > maxXY[1]) {
      maxXY[1] = cy;
    }
  }
  minXY[0] -= safetyMargin + 2.0;
  maxXY[0] += safetyMargin + 2.0;
  minXY[1] -= safetyMargin + 2.0;
  maxXY[1] += safetyMargin + 2.0;
  rateHz = maxXY[0] - minXY[0];
  cy = maxXY[1] - minXY[1];
  if (rateHz < resolution) {
    rateHz = resolution;
  }
  if (cy < resolution) {
    cy = resolution;
  }
  nx = coder::internal::minimum2(
      std::fmax(2.0, std::ceil(rateHz / resolution) + 6.0));
  ny = coder::internal::minimum2(
      std::fmax(2.0, std::ceil(cy / resolution) + 6.0));
  maxXY[0] = minXY[0] + static_cast<double>(nx) * resolution;
  maxXY[1] = minXY[1] + static_cast<double>(ny) * resolution;
  if ((!collisionAtPoint(startState[0], startState[1], floorCenters_data,
                         floorCenters_size, floorRadii_data, floorMargins_data,
                         numDiscs, safetyMargin)) &&
      (!collisionAtPoint(goalState[0], goalState[1], floorCenters_data,
                         floorCenters_size, floorRadii_data, floorMargins_data,
                         numDiscs, safetyMargin))) {
    int goalRow;
    int i;
    i = worldToGrid(startState[0], startState[1], minXY, resolution, ny, nx,
                    nodeCount, validStart);
    goalRow = worldToGrid(goalState[0], goalState[1], minXY, resolution, ny, nx,
                          k, validGoal);
    if (validStart && validGoal) {
      double d1;
      long b_i;
      int qY;
      int thetaIdx;
      d = coder::b_mod(startState[2] + 3.1415926535897931) - 3.1415926535897931;
      d1 = std::floor((d + 3.1415926535897931) / 6.2831853071795862 *
                      static_cast<double>(thetaBins)) +
           1.0;
      if (d1 < 2.147483648E+9) {
        if (d1 >= -2.147483648E+9) {
          thetaIdx = static_cast<int>(d1);
        } else {
          thetaIdx = MIN_int32_T;
        }
      } else if (d1 >= 2.147483648E+9) {
        thetaIdx = MAX_int32_T;
      } else {
        thetaIdx = 0;
      }
      if (thetaIdx < 1) {
        thetaIdx = 1;
      } else if (thetaIdx > thetaBins) {
        thetaIdx = thetaBins;
      }
      b_i = static_cast<long>(thetaIdx - 1) * ny;
      if (b_i > 2147483647L) {
        b_i = 2147483647L;
      } else if (b_i < -2147483648L) {
        b_i = -2147483648L;
      }
      if (i < -2147483647) {
        qY = MIN_int32_T;
      } else {
        qY = i - 1;
      }
      if ((static_cast<int>(b_i) < 0) &&
          (qY < MIN_int32_T - static_cast<int>(b_i))) {
        qY = MIN_int32_T;
      } else if ((static_cast<int>(b_i) > 0) &&
                 (qY > MAX_int32_T - static_cast<int>(b_i))) {
        qY = MAX_int32_T;
      } else {
        qY += static_cast<int>(b_i);
      }
      b_i = static_cast<long>(qY) * nx;
      if (b_i > 2147483647L) {
        b_i = 2147483647L;
      } else if (b_i < -2147483648L) {
        b_i = -2147483648L;
      }
      if ((static_cast<int>(b_i) < 0) &&
          (nodeCount < MIN_int32_T - static_cast<int>(b_i))) {
        qY = MIN_int32_T;
      } else if ((static_cast<int>(b_i) > 0) &&
                 (nodeCount > MAX_int32_T - static_cast<int>(b_i))) {
        qY = MAX_int32_T;
      } else {
        qY = static_cast<int>(b_i) + nodeCount;
      }
      d1 = coder::b_mod(goalState[2] + 3.1415926535897931) - 3.1415926535897931;
      rateHz = std::floor((d1 + 3.1415926535897931) / 6.2831853071795862 *
                          static_cast<double>(thetaBins)) +
               1.0;
      if (rateHz < 2.147483648E+9) {
        if (rateHz >= -2.147483648E+9) {
          thetaIdx = static_cast<int>(rateHz);
        } else {
          thetaIdx = MIN_int32_T;
        }
      } else if (rateHz >= 2.147483648E+9) {
        thetaIdx = MAX_int32_T;
      } else {
        thetaIdx = 0;
      }
      if (thetaIdx < 1) {
        thetaIdx = 1;
      } else if (thetaIdx > thetaBins) {
        thetaIdx = thetaBins;
      }
      b_i = static_cast<long>(thetaIdx - 1) * ny;
      if (b_i > 2147483647L) {
        b_i = 2147483647L;
      } else if (b_i < -2147483648L) {
        b_i = -2147483648L;
      }
      if (goalRow < -2147483647) {
        i = MIN_int32_T;
      } else {
        i = goalRow - 1;
      }
      if ((static_cast<int>(b_i) < 0) &&
          (i < MIN_int32_T - static_cast<int>(b_i))) {
        i = MIN_int32_T;
      } else if ((static_cast<int>(b_i) > 0) &&
                 (i > MAX_int32_T - static_cast<int>(b_i))) {
        i = MAX_int32_T;
      } else {
        i += static_cast<int>(b_i);
      }
      b_i = static_cast<long>(i) * nx;
      if (b_i > 2147483647L) {
        b_i = 2147483647L;
      } else if (b_i < -2147483648L) {
        b_i = -2147483648L;
      }
      if ((static_cast<int>(b_i) < 0) &&
          (k < MIN_int32_T - static_cast<int>(b_i))) {
        i = MIN_int32_T;
      } else if ((static_cast<int>(b_i) > 0) &&
                 (k > MAX_int32_T - static_cast<int>(b_i))) {
        i = MAX_int32_T;
      } else {
        i = static_cast<int>(b_i) + k;
      }
      if ((qY >= 1) && (qY <= 4718592) && (i >= 1) && (i <= 4718592)) {
        double motionPrims[24];
        double gCurr;
        double goalPosTol;
        double goalYawTol;
        double xCurr;
        double xNext;
        double yCurr;
        double yawCurr;
        int parent[12000];
        int pathNodes[12000];
        int currentNode;
        int goalNode;
        bool closedSet[12000];
        bool openSet[12000];
        bool exitg1;
        std::memset(&gridToNode[0], 0, 4718592U * sizeof(int));
        for (i = 0; i < 12000; i++) {
          nodeX[i] = 0.0;
          nodeY[i] = 0.0;
          nodeYaw[i] = 0.0;
          gCost[i] = rtInf;
          fCost[i] = rtInf;
          pathNodes[i] = 0;
          parent[i] = 0;
          openSet[i] = false;
          closedSet[i] = false;
        }
        nodeCount = 1;
        nodeX[0] = startState[0];
        nodeY[0] = startState[1];
        nodeYaw[0] = d;
        gCost[0] = 0.0;
        fCost[0] = coder::b_hypot(goalState[0] - startState[0],
                                  goalState[1] - startState[1]) +
                   minTurningRadius *
                       std::abs(coder::b_mod((goalState[2] - startState[2]) +
                                             3.1415926535897931) -
                                3.1415926535897931);
        parent[0] = 0;
        gridToNode[qY - 1] = 1;
        openSet[0] = true;
        rateHz = 1.0 / std::fmax(minTurningRadius, 0.001);
        motionPrims[0] = motionPrimitiveLength;
        motionPrims[8] = 0.0;
        motionPrims[16] = 0.0;
        motionPrims[1] = -motionPrimitiveLength;
        motionPrims[9] = 0.0;
        motionPrims[17] = 0.0;
        motionPrims[2] = motionPrimitiveLength;
        motionPrims[10] = rateHz;
        motionPrims[18] = 0.0;
        motionPrims[3] = motionPrimitiveLength;
        motionPrims[11] = -rateHz;
        motionPrims[19] = 0.0;
        motionPrims[4] = -motionPrimitiveLength;
        motionPrims[12] = rateHz;
        motionPrims[20] = 0.0;
        motionPrims[5] = -motionPrimitiveLength;
        motionPrims[13] = -rateHz;
        motionPrims[21] = 0.0;
        motionPrims[6] = 0.0;
        motionPrims[14] = 0.0;
        motionPrims[22] = rotationStep;
        motionPrims[7] = 0.0;
        motionPrims[15] = 0.0;
        motionPrims[23] = -rotationStep;
        cy = std::fmin(0.5 * motionPrimitiveLength,
                       std::fmax(0.05, 0.25 * resolution));
        goalPosTol = std::fmax(resolution * 1.5, 0.1);
        goalYawTol = std::fmax(
            3.1415926535897931 / static_cast<double>(thetaBins), 0.15);
        goalNode = 0;
        goalRow = 0;
        exitg1 = false;
        while ((!exitg1) && (goalRow < 12000)) {
          currentNode = selectBestOpen(openSet, fCost, nodeCount) - 1;
          if (currentNode + 1 == 0) {
            exitg1 = true;
          } else if (reachedGoal(nodeX[currentNode], nodeY[currentNode],
                                 nodeYaw[currentNode], goalState, goalPosTol,
                                 goalYawTol)) {
            goalNode = currentNode + 1;
            exitg1 = true;
          } else {
            openSet[currentNode] = false;
            closedSet[currentNode] = true;
            xCurr = nodeX[currentNode];
            yCurr = nodeY[currentNode];
            yawCurr = nodeYaw[currentNode];
            gCurr = gCost[currentNode];
            for (int primIdx{0}; primIdx < 8; primIdx++) {
              xNext = propagatePrimitive(
                  xCurr, yCurr, yawCurr, motionPrims[primIdx],
                  motionPrims[primIdx + 8], motionPrims[primIdx + 16], minXY,
                  maxXY, resolution, cy, floorCenters_data, floorCenters_size,
                  floorRadii_data, floorMargins_data, numDiscs, safetyMargin,
                  minTurningRadius, dy, yawDiff, rateHz, validStart);
              if (validStart) {
                rateHz += gCurr;
                if (!(rateHz > maxPathLength + 1.0E-6)) {
                  i = worldToGrid(xNext, dy, minXY, resolution, ny, nx, k,
                                  validStart);
                  if (validStart) {
                    d = std::floor(
                            ((coder::b_mod(yawDiff + 3.1415926535897931) -
                              3.1415926535897931) +
                             3.1415926535897931) /
                            6.2831853071795862 *
                            static_cast<double>(thetaBins)) +
                        1.0;
                    if (d < 2.147483648E+9) {
                      if (d >= -2.147483648E+9) {
                        thetaIdx = static_cast<int>(d);
                      } else {
                        thetaIdx = MIN_int32_T;
                      }
                    } else if (d >= 2.147483648E+9) {
                      thetaIdx = MAX_int32_T;
                    } else {
                      thetaIdx = 0;
                    }
                    if (thetaIdx < 1) {
                      thetaIdx = 1;
                    } else if (thetaIdx > thetaBins) {
                      thetaIdx = thetaBins;
                    }
                    b_i = static_cast<long>(thetaIdx - 1) * ny;
                    if (b_i > 2147483647L) {
                      b_i = 2147483647L;
                    } else if (b_i < -2147483648L) {
                      b_i = -2147483648L;
                    }
                    if (i < -2147483647) {
                      qY = MIN_int32_T;
                    } else {
                      qY = i - 1;
                    }
                    if ((static_cast<int>(b_i) < 0) &&
                        (qY < MIN_int32_T - static_cast<int>(b_i))) {
                      qY = MIN_int32_T;
                    } else if ((static_cast<int>(b_i) > 0) &&
                               (qY > MAX_int32_T - static_cast<int>(b_i))) {
                      qY = MAX_int32_T;
                    } else {
                      qY += static_cast<int>(b_i);
                    }
                    b_i = static_cast<long>(qY) * nx;
                    if (b_i > 2147483647L) {
                      b_i = 2147483647L;
                    } else if (b_i < -2147483648L) {
                      b_i = -2147483648L;
                    }
                    if ((static_cast<int>(b_i) < 0) &&
                        (k < MIN_int32_T - static_cast<int>(b_i))) {
                      qY = MIN_int32_T;
                    } else if ((static_cast<int>(b_i) > 0) &&
                               (k > MAX_int32_T - static_cast<int>(b_i))) {
                      qY = MAX_int32_T;
                    } else {
                      qY = static_cast<int>(b_i) + k;
                    }
                    if ((qY >= 1) && (qY <= 4718592)) {
                      bool guard1;
                      thetaIdx = gridToNode[qY - 1];
                      guard1 = false;
                      if (thetaIdx == 0) {
                        if (nodeCount < 12000) {
                          nodeCount++;
                          i = nodeCount - 1;
                          gridToNode[qY - 1] = nodeCount;
                          guard1 = true;
                        }
                      } else {
                        i = thetaIdx - 1;
                        if ((!closedSet[thetaIdx - 1]) ||
                            (!(rateHz >= gCost[thetaIdx - 1] - 1.0E-9))) {
                          guard1 = true;
                        }
                      }
                      if (guard1 && ((!openSet[i]) || (rateHz < gCost[i]))) {
                        nodeX[i] = xNext;
                        nodeY[i] = dy;
                        nodeYaw[i] = yawDiff;
                        gCost[i] = rateHz;
                        fCost[i] = rateHz +
                                   (coder::b_hypot(goalState[0] - xNext,
                                                   goalState[1] - dy) +
                                    minTurningRadius *
                                        std::abs(coder::b_mod(
                                                     (goalState[2] - yawDiff) +
                                                     3.1415926535897931) -
                                                 3.1415926535897931));
                        parent[i] = currentNode + 1;
                        openSet[i] = true;
                        closedSet[i] = false;
                      }
                    }
                  }
                }
              }
            }
            goalRow++;
          }
        }
        if (goalNode != 0) {
          currentNode = -1;
          while ((goalNode != 0) && (currentNode + 1 < 12000)) {
            currentNode++;
            pathNodes[currentNode] = goalNode;
            goalNode = parent[goalNode - 1];
          }
          if (goalNode == 0) {
            thetaIdx = static_cast<int>(
                std::floor(static_cast<double>(currentNode + 1) / 2.0));
            for (k = 0; k < thetaIdx; k++) {
              i = pathNodes[k];
              nodeCount = currentNode - k;
              pathNodes[k] = pathNodes[nodeCount];
              pathNodes[nodeCount] = i;
            }
            resolution = std::fmin(0.5 * motionPrimitiveLength, 0.25);
            cy = std::fmin(rotationStep, 0.5);
            i = 0;
            goalPosTol = 0.0;
            goalYawTol = 0.0;
            xCurr = 0.0;
            nodeCount = 0;
            exitg1 = false;
            while ((!exitg1) && (nodeCount <= currentNode)) {
              yCurr = coder::b_mod(nodeYaw[pathNodes[nodeCount] - 1] +
                                   3.1415926535897931) -
                      3.1415926535897931;
              if (i == 0) {
                i = 1;
                goalPosTol = nodeX[pathNodes[nodeCount] - 1];
                goalYawTol = nodeY[pathNodes[nodeCount] - 1];
                xCurr = yCurr;
                states[0] = goalPosTol;
                states[1] = goalYawTol;
                states[2] = yCurr;
                nodeCount++;
              } else {
                yawCurr = nodeX[pathNodes[nodeCount] - 1];
                gCurr = yawCurr - goalPosTol;
                xNext = nodeY[pathNodes[nodeCount] - 1];
                dy = xNext - goalYawTol;
                yawDiff = coder::b_mod((yCurr - xCurr) + 3.1415926535897931) -
                          3.1415926535897931;
                d = std::fmax(
                    1.0, std::fmax(std::ceil(coder::b_hypot(gCurr, dy) /
                                             std::fmax(resolution, 1.0E-6)),
                                   std::ceil(std::abs(yawDiff) /
                                             std::fmax(cy, 1.0E-6))));
                if (d < 2.147483648E+9) {
                  thetaIdx = static_cast<int>(d);
                } else {
                  thetaIdx = MAX_int32_T;
                }
                k = 0;
                while ((k <= thetaIdx - 2) && (i < 200)) {
                  rateHz = (static_cast<double>(k) + 1.0) /
                           static_cast<double>(thetaIdx);
                  i++;
                  goalRow = 3 * (i - 1);
                  states[goalRow] = goalPosTol + gCurr * rateHz;
                  states[goalRow + 1] = goalYawTol + dy * rateHz;
                  states[goalRow + 2] =
                      coder::b_mod((xCurr + yawDiff * rateHz) +
                                   3.1415926535897931) -
                      3.1415926535897931;
                  k++;
                }
                if (i >= 200) {
                  exitg1 = true;
                } else {
                  i++;
                  goalPosTol = yawCurr;
                  goalYawTol = xNext;
                  xCurr = yCurr;
                  goalRow = 3 * (i - 1);
                  states[goalRow] = yawCurr;
                  states[goalRow + 1] = xNext;
                  states[goalRow + 2] = yCurr;
                  nodeCount++;
                }
              }
            }
            if (i != 0) {
              states[3 * (i - 1) + 2] = d1;
              count = i;
              *commandCount = computePurePursuitCommands(
                  states, i, sampleTime, maxLinearSpeed, maxYawRate, commands);
            }
          }
        }
      }
    }
  }
  return count;
}

} // namespace codegen
} // namespace gik9dof

// End of code generation (stageBPlanPath.cpp)
