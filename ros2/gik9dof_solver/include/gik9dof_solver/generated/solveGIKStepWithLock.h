//
// solveGIKStepWithLock.h
//
// Code generation for function 'solveGIKStepWithLock'
//

#ifndef SOLVEGIKSTEPWITHLOCK_H
#define SOLVEGIKSTEPWITHLOCK_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof {
namespace codegen {
void solveGIKStepWithLock(const double qCurrent[9], const double targetPose[16],
                          double distanceLower, double distanceWeight,
                          const bool lockMask[9], double qNext[9]);

}
} // namespace gik9dof
void solveGIKStepWithLock_delete();

void solveGIKStepWithLock_init();

void solveGIKStepWithLock_new();

#endif
// End of code generation (solveGIKStepWithLock.h)
