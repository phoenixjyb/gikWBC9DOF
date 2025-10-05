//
// KinematicConstraint.cpp
//
// Code generation for function 'KinematicConstraint'
//

// Include files
#include "KinematicConstraint.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
void minus(double in1[2], const double in2[2],
           const coder::array<double, 1U> &in3)
{
  in1[0] = in2[0] - in3[0];
  in1[1] = in2[1] - in3[static_cast<int>(in3.size(0) != 1)];
}

void minus(double in1[2], const coder::array<double, 1U> &in2)
{
  in1[0] -= in2[0];
  in1[1] -= in2[static_cast<int>(in2.size(0) != 1)];
}

// End of code generation (KinematicConstraint.cpp)
