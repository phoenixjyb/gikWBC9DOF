//
// gik9dof_codegen_followTrajectory.h
//
// Code generation for function 'gik9dof_codegen_followTrajectory'
//

#ifndef GIK9DOF_CODEGEN_FOLLOWTRAJECTORY_H
#define GIK9DOF_CODEGEN_FOLLOWTRAJECTORY_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof {
namespace codegen {
extern void followTrajectory(const double q0[9], const double poses_data[],
                             const int poses_size[3], double distanceLower,
                             double distanceWeight, double qOut[9]);

}
} // namespace gik9dof

#endif
// End of code generation (gik9dof_codegen_followTrajectory.h)
