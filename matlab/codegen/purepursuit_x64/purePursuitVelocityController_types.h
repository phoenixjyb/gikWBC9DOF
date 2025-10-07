//
// File: purePursuitVelocityController_types.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 12:15:43
//

#ifndef PUREPURSUITVELOCITYCONTROLLER_TYPES_H
#define PUREPURSUITVELOCITYCONTROLLER_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
namespace gik9dof_purepursuit {
struct struct0_T {
  double lookaheadBase;
  double lookaheadVelGain;
  double lookaheadTimeGain;
  double vxNominal;
  double vxMax;
  double wzMax;
  double track;
  double vwheelMax;
  double waypointSpacing;
  double pathBufferSize;
  double goalTolerance;
  double interpSpacing;
};

struct struct1_T {
  double pathX[30];
  double pathY[30];
  double pathTheta[30];
  double pathTime[30];
  double numWaypoints;
  double prevVx;
  double prevWz;
  double prevPoseX;
  double prevPoseY;
  double prevPoseYaw;
  double lastRefTime;
};

} // namespace gik9dof_purepursuit

#endif
//
// File trailer for purePursuitVelocityController_types.h
//
// [EOF]
//
