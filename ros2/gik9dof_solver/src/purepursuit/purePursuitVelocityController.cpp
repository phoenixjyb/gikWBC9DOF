//
// File: purePursuitVelocityController.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 12:15:43
//

// Include Files
#include "purePursuitVelocityController.h"
#include "purePursuitVelocityController_types.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// PUREPURSUITVELOCITYCONTROLLER Pure Pursuit path following controller
//
//  Inputs:
//    refX, refY, refTheta, refTime - Current position reference
//    estX, estY, estYaw - Current robot pose estimate
//    params - Controller parameters struct
//    stateIn - Controller state struct (optional, has default)
//
//  Outputs:
//    vx - Forward velocity command (m/s)
//    wz - Angular velocity command (rad/s)
//    stateOut - Updated controller state
//
//  Design:
//    - Path buffer: 30 waypoints max
//    - Update rate: 100 Hz (dt = 0.01s)
//    - Max speed: 1.5 m/s
//    - Adaptive lookahead: L = L_base + k_v * vx + k_t * dt_since_ref
//    - Interpolation: Linear between waypoints
//    - Continuous reference acceptance (no goal stop)
//
// Arguments    : double refX
//                double refY
//                double refTheta
//                double refTime
//                double estX
//                double estY
//                double estYaw
//                const struct0_T *params
//                const struct1_T *stateIn
//                double *vx
//                double *wz
//                struct1_T *stateOut
// Return Type  : void
//
namespace gik9dof_purepursuit {
void purePursuitVelocityController(double refX, double refY, double refTheta,
                                   double refTime, double estX, double estY,
                                   double estYaw, const struct0_T *params,
                                   const struct1_T *stateIn, double *vx,
                                   double *wz, struct1_T *stateOut)
{
  double interpX[200];
  double interpY[200];
  double dx;
  double dy;
  double idx;
  double newNum;
  int b_i;
  int closestIdx;
  int i;
  bool addNewWaypoint;
  //  Copyright 2025, Mobile Manipulator Team
  //  Default state initialization
  *stateOut = *stateIn;
  //  Extract parameters
  //  Base lookahead distance (m)
  //  Velocity-dependent gain
  //  Time-dependent gain
  //  Nominal forward speed (m/s)
  //  Max forward speed (m/s)
  //  Max angular rate (rad/s)
  //  Wheel track width (m)
  //  Max wheel speed (m/s)
  //  Min spacing between waypoints (m)
  //  Max waypoints to keep
  //  Distance to consider waypoint reached (m)
  //  Interpolation spacing (m)
  //  Update path buffer with new reference
  //  Check if new waypoint is sufficiently different from last stored point
  if (stateOut->numWaypoints == 0.0) {
    //  First waypoint
    addNewWaypoint = true;
  } else {
    //  Check distance from last waypoint
    dx = refX - stateOut->pathX[static_cast<int>(stateOut->numWaypoints) - 1];
    dy = refY - stateOut->pathY[static_cast<int>(stateOut->numWaypoints) - 1];
    addNewWaypoint = (std::sqrt(dx * dx + dy * dy) >= params->waypointSpacing);
  }
  if (addNewWaypoint) {
    if (stateOut->numWaypoints < params->pathBufferSize) {
      //  Add to buffer
      stateOut->numWaypoints++;
      idx = stateOut->numWaypoints;
    } else {
      //  Buffer full, shift left and add at end
      i = static_cast<int>(params->pathBufferSize - 1.0);
      for (b_i = 0; b_i < i; b_i++) {
        stateOut->pathX[b_i] = stateOut->pathX[b_i + 1];
        stateOut->pathY[b_i] = stateOut->pathY[b_i + 1];
        stateOut->pathTheta[b_i] = stateOut->pathTheta[b_i + 1];
        stateOut->pathTime[b_i] = stateOut->pathTime[b_i + 1];
      }
      idx = params->pathBufferSize;
    }
    stateOut->pathX[static_cast<int>(idx) - 1] = refX;
    stateOut->pathY[static_cast<int>(idx) - 1] = refY;
    stateOut->pathTheta[static_cast<int>(idx) - 1] = refTheta;
    stateOut->pathTime[static_cast<int>(idx) - 1] = refTime;
    stateOut->lastRefTime = refTime;
  }
  //  Remove passed waypoints
  //  Remove waypoints that are behind the robot
  if (stateOut->numWaypoints > 1.0) {
    idx = 0.0;
    b_i = 0;
    while ((b_i <= static_cast<int>(stateOut->numWaypoints) - 1) &&
           ((stateOut->pathX[b_i] - estX) * std::cos(-estYaw) -
                (stateOut->pathY[b_i] - estY) * std::sin(-estYaw) <
            -params->goalTolerance)) {
      //  Transform waypoint to robot frame
      //  Rotate to robot frame
      //  If waypoint is behind robot (x < -goalTolerance), mark for removal
      idx = static_cast<double>(b_i) + 1.0;
      b_i++;
    }
    //  Shift path buffer to remove passed waypoints
    if (idx > 0.0) {
      newNum = stateOut->numWaypoints - idx;
      i = static_cast<int>(newNum);
      for (b_i = 0; b_i < i; b_i++) {
        closestIdx = b_i + static_cast<int>(static_cast<unsigned int>(idx));
        stateOut->pathX[b_i] = stateOut->pathX[closestIdx];
        stateOut->pathY[b_i] = stateOut->pathY[closestIdx];
        stateOut->pathTheta[b_i] = stateOut->pathTheta[closestIdx];
        stateOut->pathTime[b_i] = stateOut->pathTime[closestIdx];
      }
      stateOut->numWaypoints = newNum;
    }
  }
  //  Handle insufficient waypoints
  if (stateOut->numWaypoints < 1.0) {
    //  No path, stop
    *vx = 0.0;
    *wz = 0.0;
  } else {
    double dxSeg;
    double lookaheadDist;
    double lookaheadY;
    int j;
    int numInterpPoints;
    bool exitg1;
    //  Calculate adaptive lookahead distance
    //  Time since last reference update
    //  Current forward velocity estimate (use previous command as estimate)
    //  Adaptive lookahead: L = L_base + k_v * vx + k_t * dt
    //  Clamp lookahead
    if (stateOut->lastRefTime > 0.0) {
      idx = refTime - stateOut->lastRefTime;
    } else {
      idx = 0.0;
    }
    lookaheadDist = std::fmax(
        0.3,
        std::fmin(2.5, (params->lookaheadBase +
                        params->lookaheadVelGain * std::abs(stateOut->prevVx)) +
                           params->lookaheadTimeGain * idx));
    //  Interpolate path for smooth following
    //  Create interpolated points along the path
    //  Pre-allocate max interpolated points
    std::memset(&interpX[0], 0, 200U * sizeof(double));
    std::memset(&interpY[0], 0, 200U * sizeof(double));
    numInterpPoints = -1;
    if (stateOut->numWaypoints == 1.0) {
      //  Single waypoint, just use it
      numInterpPoints = 0;
      interpX[0] = stateOut->pathX[0];
      interpY[0] = stateOut->pathY[0];
    } else {
      //  Interpolate between waypoints
      i = static_cast<int>(stateOut->numWaypoints - 1.0);
      for (b_i = 0; b_i < i; b_i++) {
        dxSeg = stateOut->pathX[b_i];
        dy = stateOut->pathY[b_i];
        lookaheadY = stateOut->pathX[b_i + 1] - dxSeg;
        dx = stateOut->pathY[b_i + 1] - dy;
        idx = std::sqrt(lookaheadY * lookaheadY + dx * dx);
        if (idx > 1.0E-6) {
          idx = std::fmax(2.0, std::ceil(idx / params->interpSpacing));
          closestIdx = static_cast<int>((idx - 1.0) + 1.0);
          for (j = 0; j < closestIdx; j++) {
            if (numInterpPoints + 1 < 200) {
              newNum = static_cast<double>(j) / (idx - 1.0);
              numInterpPoints++;
              interpX[numInterpPoints] = dxSeg + newNum * lookaheadY;
              interpY[numInterpPoints] = dy + newNum * dx;
            }
          }
        }
      }
      //  Add last waypoint if space available
      if (numInterpPoints + 1 < 200) {
        numInterpPoints++;
        interpX[numInterpPoints] =
            stateOut->pathX[static_cast<int>(stateOut->numWaypoints) - 1];
        interpY[numInterpPoints] =
            stateOut->pathY[static_cast<int>(stateOut->numWaypoints) - 1];
      }
    }
    //  Find lookahead point
    //  Find closest point on path to robot
    newNum = rtInf;
    closestIdx = 0;
    for (b_i = 0; b_i <= numInterpPoints; b_i++) {
      dx = interpX[b_i] - estX;
      dy = interpY[b_i] - estY;
      idx = std::sqrt(dx * dx + dy * dy);
      if (idx < newNum) {
        newNum = idx;
        closestIdx = b_i;
      }
    }
    //  Search forward from closest point for lookahead distance along path
    idx = interpX[numInterpPoints];
    lookaheadY = interpY[numInterpPoints];
    newNum = 0.0;
    i = numInterpPoints - closestIdx;
    b_i = 0;
    exitg1 = false;
    while ((!exitg1) && (b_i <= i)) {
      j = closestIdx + b_i;
      if (j + 1 > closestIdx + 1) {
        //  Accumulate distance along path segments
        dxSeg = interpX[j] - interpX[j - 1];
        dy = interpY[j] - interpY[j - 1];
        newNum += std::sqrt(dxSeg * dxSeg + dy * dy);
      }
      //  Check if accumulated distance along path >= lookahead distance
      if (newNum >= lookaheadDist) {
        idx = interpX[j];
        lookaheadY = interpY[j];
        exitg1 = true;
      } else {
        b_i++;
      }
    }
    //  If we didn't find a point at lookahead distance, use the farthest point
    if (newNum < lookaheadDist) {
      idx = interpX[numInterpPoints];
      lookaheadY = interpY[numInterpPoints];
    }
    //  Transform lookahead point to robot frame
    dx = idx - estX;
    dy = lookaheadY - estY;
    //  Rotate to robot frame
    idx = std::sin(-estYaw);
    newNum = std::cos(-estYaw);
    dxSeg = dx * newNum - dy * idx;
    newNum = dx * idx + dy * newNum;
    //  Pure Pursuit curvature calculation
    //  Curvature: κ = 2 * y / L²
    //  Where y is lateral distance to lookahead point
    //  L is lookahead distance
    idx = std::sqrt(dxSeg * dxSeg + newNum * newNum);
    if (idx < 0.05) {
      //  Too close to lookahead point
      dxSeg = 0.0;
    } else {
      dxSeg = 2.0 * newNum / (idx * idx);
    }
    //  Calculate velocities
    //  Forward velocity: nominal speed
    idx = params->vxNominal;
    //  Reduce speed in sharp turns
    newNum = std::abs(dxSeg);
    if (newNum > 0.5) {
      //  Sharp turn, reduce speed
      idx = params->vxNominal * 0.5;
    } else if (newNum > 0.2) {
      //  Moderate turn
      idx = params->vxNominal * 0.7;
    }
    //  Clamp forward velocity
    *vx = std::fmax(0.0, std::fmin(params->vxMax, idx));
    //  Angular velocity from curvature
    //  Clamp angular velocity
    *wz = std::fmax(-params->wzMax, std::fmin(params->wzMax, *vx * dxSeg));
    //  Apply wheel speed limits
    //  Differential drive kinematics:
    //  v_left = vx - (track/2) * wz
    //  v_right = vx + (track/2) * wz
    idx = params->track / 2.0 * *wz;
    newNum = *vx + idx;
    //  Check wheel limits
    idx = std::abs(*vx - idx);
    if ((idx > params->vwheelMax) || (std::abs(newNum) > params->vwheelMax)) {
      //  Scale down to respect limits
      idx = params->vwheelMax / std::fmax(idx, std::abs(newNum));
      *vx *= idx;
      *wz *= idx;
    }
    //  Update state
    stateOut->prevVx = *vx;
    stateOut->prevWz = *wz;
    stateOut->prevPoseX = estX;
    stateOut->prevPoseY = estY;
    stateOut->prevPoseYaw = estYaw;
  }
}

} // namespace gik9dof_purepursuit

//
// File trailer for purePursuitVelocityController.cpp
//
// [EOF]
//
