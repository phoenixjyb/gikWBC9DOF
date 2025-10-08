//
// File: KinematicConstraint.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 12:14:03
//

// Include Files
#include "KinematicConstraint.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : double in1[2]
//                const double in2[2]
//                const coder::array<double, 1U> &in3
// Return Type  : void
//
void minus(double in1[2], const double in2[2],
           const coder::array<double, 1U> &in3)
{
  in1[0] = in2[0] - in3[0];
  in1[1] = in2[1] - in3[static_cast<int>(in3.size(0) != 1)];
}

//
// Arguments    : double in1[2]
//                const coder::array<double, 1U> &in2
// Return Type  : void
//
void minus(double in1[2], const coder::array<double, 1U> &in2)
{
  in1[0] -= in2[0];
  in1[1] -= in2[static_cast<int>(in2.size(0) != 1)];
}

//
// File trailer for KinematicConstraint.cpp
//
// [EOF]
//
