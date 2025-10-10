//
// File: chassisPathFollowerCodegen_types.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 11-Oct-2025 00:19:03
//

#ifndef CHASSISPATHFOLLOWERCODEGEN_TYPES_H
#define CHASSISPATHFOLLOWERCODEGEN_TYPES_H

// Include Files
#include "rtwtypes.h"
#include "coder_bounded_array.h"

// Type Definitions
namespace gik9dof {
struct struct2_T {
  double track;
  double wheel_speed_max;
  double vx_max;
  double vx_min;
  double wz_max;
  double accel_limit;
  double decel_limit;
  double jerk_limit;
  double wheel_base;
  bool reverse_enabled;
};

struct struct0_T {
  double PathNumPoints;
  double CurrentIndex;
  double LastVelocity;
  double LastAcceleration;
  double LastHeadingError;
  double IntegralHeadingError;
  double PreviousPose[3];
  double DistanceTraveled;
};

struct struct1_T {
  double ControllerMode;
  bool ReverseEnabled;
  double LookaheadBase;
  double LookaheadVelGain;
  double LookaheadAccelGain;
  double GoalTolerance;
  double HeadingKp;
  double HeadingKi;
  double HeadingKd;
  double FeedforwardGain;
  double KappaThreshold;
  double VxReduction;
  struct2_T Chassis;
  ::coder::bounded_array<double, 1500U, 2U> PathInfo_States;
  ::coder::bounded_array<double, 500U, 1U> PathInfo_Curvature;
  ::coder::bounded_array<double, 500U, 1U> PathInfo_ArcLength;
  ::coder::bounded_array<double, 500U, 1U> PathInfo_DistanceRemaining;
};

struct struct3_T {
  bool isFinished;
  double distanceRemaining;
  double crossTrackError;
  double headingError;
  ::coder::bounded_array<double, 1U, 1U> curvature;
  double lookaheadDistance;
  double currentMode;
  double currentIndex;
};

} // namespace gik9dof

#endif
//
// File trailer for chassisPathFollowerCodegen_types.h
//
// [EOF]
//
