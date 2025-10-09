//
// File: rand.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 18:19:21
//

#ifndef RAND_H
#define RAND_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
class GIKSolver;

}

// Function Declarations
namespace gik9dof {
namespace coder {
void b_rand(GIKSolver *aInstancePtr, double r[5]);

int b_rand(GIKSolver *aInstancePtr, double varargin_1, double r_data[]);

void c_rand(GIKSolver *aInstancePtr, double r[3]);

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for rand.h
//
// [EOF]
//
