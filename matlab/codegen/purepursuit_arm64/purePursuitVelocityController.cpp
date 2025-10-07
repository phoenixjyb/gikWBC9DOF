//
// File: purePursuitVelocityController.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 03:43:41
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
//    vx - Forward velocity command (m/s, positive=forward, negative=reverse)
//    wz - Angular velocity command (rad/s)
//    stateOut - Updated controller state
//
//  Design:
//    - Path buffer: 30 waypoints max
//    - Update rate: 100 Hz (dt = 0.01s)
//    - Max speed: 1.5 m/s forward, -1.0 m/s reverse
//    - Adaptive lookahead: L = L_base + k_v * vx + k_t * dt_since_ref
//    - Interpolation: Linear between waypoints
//    - Continuous reference acceptance (no goal stop)
//    - BIDIRECTIONAL: Automatically detects forward/reverse based on waypoint
//    position
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
  double segmentDist;
  int b_i;
  int c_i;
  int i;
  unsigned int idx;
  int numToRemove;
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
  //  Max reverse speed (m/s, negative)
  //  Max angular rate (rad/s)
  //  Wheel track width (m)
  //  Max wheel speed (m/s)
  //  Min spacing between waypoints (m)
  //  Max waypoints to keep
  //  Distance to consider waypoint reached (m)
  //  Interpolation spacing (m)
  //  Update path buffer with new reference
  //  Check if new waypoint is sufficiently different from last stored point
  if (stateOut->numWaypoints == 0U) {
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
      idx = stateOut->numWaypoints + 1U;
      if (stateOut->numWaypoints + 1U < stateOut->numWaypoints) {
        idx = MAX_uint32_T;
      }
      stateOut->numWaypoints = idx;
    } else {
      //  Buffer full, shift left and add at end
      segmentDist = std::round(params->pathBufferSize - 1.0);
      if (segmentDist < 4.294967296E+9) {
        if (segmentDist >= 0.0) {
          idx = static_cast<unsigned int>(segmentDist);
        } else {
          idx = 0U;
        }
      } else {
        idx = 0U;
      }
      b_i = static_cast<int>(idx);
      for (i = 0; i < b_i; i++) {
        stateOut->pathX[i] = stateOut->pathX[i + 1];
        stateOut->pathY[i] = stateOut->pathY[i + 1];
        stateOut->pathTheta[i] = stateOut->pathTheta[i + 1];
        stateOut->pathTime[i] = stateOut->pathTime[i + 1];
      }
      segmentDist = std::round(params->pathBufferSize);
      if (segmentDist < 4.294967296E+9) {
        if (segmentDist >= 0.0) {
          idx = static_cast<unsigned int>(segmentDist);
        } else {
          idx = 0U;
        }
      } else {
        idx = 0U;
      }
    }
    stateOut->pathX[static_cast<int>(idx) - 1] = refX;
    stateOut->pathY[static_cast<int>(idx) - 1] = refY;
    stateOut->pathTheta[static_cast<int>(idx) - 1] = refTheta;
    stateOut->pathTime[static_cast<int>(idx) - 1] = refTime;
    stateOut->lastRefTime = refTime;
  }
  //  Remove passed waypoints
  //  Remove waypoints that are behind the robot
  if (stateOut->numWaypoints > 1U) {
    numToRemove = 0;
    i = 0;
    while ((i <= static_cast<int>(stateOut->numWaypoints) - 1) &&
           ((stateOut->pathX[i] - estX) * std::cos(-estYaw) -
                (stateOut->pathY[i] - estY) * std::sin(-estYaw) <
            -params->goalTolerance)) {
      //  Transform waypoint to robot frame
      //  Rotate to robot frame
      //  If waypoint is behind robot (x < -goalTolerance), mark for removal
      numToRemove = static_cast<int>(static_cast<unsigned int>(i) + 1U);
      i++;
    }
    //  Shift path buffer to remove passed waypoints
    if (numToRemove > 0) {
      idx = stateOut->numWaypoints - static_cast<unsigned int>(numToRemove);
      if (idx > stateOut->numWaypoints) {
        idx = 0U;
      }
      b_i = static_cast<int>(idx);
      for (i = 0; i < b_i; i++) {
        c_i = static_cast<int>(static_cast<unsigned int>(i) +
                               static_cast<unsigned int>(numToRemove));
        stateOut->pathX[i] = stateOut->pathX[c_i];
        stateOut->pathY[i] = stateOut->pathY[c_i];
        stateOut->pathTheta[i] = stateOut->pathTheta[c_i];
        stateOut->pathTime[i] = stateOut->pathTime[c_i];
      }
      stateOut->numWaypoints = idx;
    }
  }
  //  Handle insufficient waypoints
  if (stateOut->numWaypoints < 1U) {
    //  No path, stop
    *vx = 0.0;
    *wz = 0.0;
  } else {
    double lookaheadDist;
    double lookaheadY;
    double numSegPoints;
    int numInterpPoints;
    bool exitg1;
    //  Calculate adaptive lookahead distance
    //  Time since last reference update
    //  Current forward velocity estimate (use previous command as estimate)
    //  Adaptive lookahead: L = L_base + k_v * vx + k_t * dt
    //  Clamp lookahead
    if (stateOut->lastRefTime > 0.0) {
      lookaheadDist = refTime - stateOut->lastRefTime;
    } else {
      lookaheadDist = 0.0;
    }
    lookaheadDist = std::fmax(
        0.3,
        std::fmin(2.5, (params->lookaheadBase +
                        params->lookaheadVelGain * std::abs(stateOut->prevVx)) +
                           params->lookaheadTimeGain * lookaheadDist));
    //  Interpolate path for smooth following
    //  Create interpolated points along the path
    //  Pre-allocate max interpolated points
    std::memset(&interpX[0], 0, 200U * sizeof(double));
    std::memset(&interpY[0], 0, 200U * sizeof(double));
    numInterpPoints = -1;
    if (stateOut->numWaypoints == 1U) {
      //  Single waypoint, just use it
      numInterpPoints = 0;
      interpX[0] = stateOut->pathX[0];
      interpY[0] = stateOut->pathY[0];
    } else {
      //  Interpolate between waypoints
      b_i = static_cast<int>(stateOut->numWaypoints) - 2;
      for (i = 0; i <= b_i; i++) {
        double a_tmp;
        dy = stateOut->pathX[i];
        lookaheadY = stateOut->pathY[i];
        dx = stateOut
                 ->pathX[static_cast<int>(static_cast<unsigned int>(i) + 1U)] -
             dy;
        a_tmp =
            stateOut
                ->pathY[static_cast<int>(static_cast<unsigned int>(i) + 1U)] -
            lookaheadY;
        segmentDist = std::sqrt(dx * dx + a_tmp * a_tmp);
        if (segmentDist > 1.0E-6) {
          numSegPoints =
              std::fmax(2.0, std::ceil(segmentDist / params->interpSpacing));
          c_i = static_cast<int>((numSegPoints - 1.0) + 1.0);
          for (numToRemove = 0; numToRemove < c_i; numToRemove++) {
            if (numInterpPoints + 1 < 200) {
              segmentDist =
                  static_cast<double>(numToRemove) / (numSegPoints - 1.0);
              numInterpPoints++;
              interpX[numInterpPoints] = dy + segmentDist * dx;
              interpY[numInterpPoints] = lookaheadY + segmentDist * a_tmp;
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
    numSegPoints = rtInf;
    numToRemove = 0;
    for (i = 0; i <= numInterpPoints; i++) {
      dx = interpX[i] - estX;
      dy = interpY[i] - estY;
      segmentDist = std::sqrt(dx * dx + dy * dy);
      if (segmentDist < numSegPoints) {
        numSegPoints = segmentDist;
        numToRemove = i;
      }
    }
    //  Search forward from closest point for lookahead distance along path
    numSegPoints = interpX[numInterpPoints];
    lookaheadY = interpY[numInterpPoints];
    segmentDist = 0.0;
    b_i = numInterpPoints - numToRemove;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= b_i)) {
      c_i = numToRemove + i;
      if (c_i + 1 > numToRemove + 1) {
        //  Accumulate distance along path segments
        dy = interpX[c_i] - interpX[c_i - 1];
        dx = interpY[c_i] - interpY[c_i - 1];
        segmentDist += std::sqrt(dy * dy + dx * dx);
      }
      //  Check if accumulated distance along path >= lookahead distance
      if (segmentDist >= lookaheadDist) {
        numSegPoints = interpX[c_i];
        lookaheadY = interpY[c_i];
        exitg1 = true;
      } else {
        i++;
      }
    }
    //  If we didn't find a point at lookahead distance, use the farthest point
    if (segmentDist < lookaheadDist) {
      numSegPoints = interpX[numInterpPoints];
      lookaheadY = interpY[numInterpPoints];
    }
    //  Transform lookahead point to robot frame
    dx = numSegPoints - estX;
    dy = lookaheadY - estY;
    //  Rotate to robot frame
    segmentDist = std::sin(-estYaw);
    numSegPoints = std::cos(-estYaw);
    lookaheadY = dx * numSegPoints - dy * segmentDist;
    segmentDist = dx * segmentDist + dy * numSegPoints;
    //  Pure Pursuit curvature calculation
    //  Curvature: κ = 2 * y / L²
    //  Where y is lateral distance to lookahead point
    //  L is lookahead distance
    numSegPoints =
        std::sqrt(lookaheadY * lookaheadY + segmentDist * segmentDist);
    if (numSegPoints < 0.05) {
      //  Too close to lookahead point
      numSegPoints = 0.0;
    } else {
      numSegPoints = 2.0 * segmentDist / (numSegPoints * numSegPoints);
    }
    //  BIDIRECTIONAL SUPPORT: Determine if we should move forward or reverse
    //  Check if lookahead point is primarily behind the robot
    numToRemove = 1;
    //  Default: forward motion
    if (lookaheadY < -0.3) {
      //  Lookahead point is significantly behind robot (x < -0.3m in robot
      //  frame) Use reverse motion
      numToRemove = -1;
      //  When reversing, invert the steering (lookahead point interpretation)
      numSegPoints = -numSegPoints;
    }
    //  Calculate velocities
    //  Forward velocity: nominal speed (with direction for bidirectional)
    *vx = params->vxNominal * static_cast<double>(numToRemove);
    //  Reduce speed in sharp turns
    segmentDist = std::abs(numSegPoints);
    if (segmentDist > 0.5) {
      //  Sharp turn, reduce speed
      *vx = params->vxNominal * 0.5 * static_cast<double>(numToRemove);
    } else if (segmentDist > 0.2) {
      //  Moderate turn
      *vx = params->vxNominal * 0.7 * static_cast<double>(numToRemove);
    }
    //  Clamp velocity (BIDIRECTIONAL: allow negative for reverse)
    if (numToRemove > 0) {
      //  Forward motion: clamp to [0, vxMax]
      *vx = std::fmax(0.0, std::fmin(params->vxMax, *vx));
    } else {
      //  Reverse motion: clamp to [vxMin, 0] (vxMin should be negative)
      *vx = std::fmax(params->vxMin, std::fmin(0.0, *vx));
    }
    //  Angular velocity from curvature
    //  Clamp angular velocity
    *wz =
        std::fmax(-params->wzMax, std::fmin(params->wzMax, *vx * numSegPoints));
    //  Apply wheel speed limits
    //  Differential drive kinematics:
    //  v_left = vx - (track/2) * wz
    //  v_right = vx + (track/2) * wz
    segmentDist = params->track / 2.0 * *wz;
    numSegPoints = *vx + segmentDist;
    //  Check wheel limits
    segmentDist = std::abs(*vx - segmentDist);
    if ((segmentDist > params->vwheelMax) ||
        (std::abs(numSegPoints) > params->vwheelMax)) {
      //  Scale down to respect limits
      segmentDist =
          params->vwheelMax / std::fmax(segmentDist, std::abs(numSegPoints));
      *vx *= segmentDist;
      *wz *= segmentDist;
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
