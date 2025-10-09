//
// File: randn.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 09-Oct-2025 12:02:50
//

#ifndef RANDN_H
#define RANDN_H

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
void b_randn(GIKSolver *aInstancePtr, double r[3]);

void randn(GIKSolver *aInstancePtr, double r[4]);

int randn(GIKSolver *aInstancePtr, const double varargin_1[2], double r_data[]);

} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for randn.h
//
// [EOF]
//
