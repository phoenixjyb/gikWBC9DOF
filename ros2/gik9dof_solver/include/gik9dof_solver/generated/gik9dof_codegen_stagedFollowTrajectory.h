//
// gik9dof_codegen_stagedFollowTrajectory.h
//
// Code generation for function 'gik9dof_codegen_stagedFollowTrajectory'
//

#ifndef GIK9DOF_CODEGEN_STAGEDFOLLOWTRAJECTORY_H
#define GIK9DOF_CODEGEN_STAGEDFOLLOWTRAJECTORY_H

// Include files
#include "gik9dof_codegen_stagedFollowTrajectory_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof {
namespace codegen {
extern void stagedFollowTrajectory(
    const double qInit[9], const double poses_data[], const int poses_size[3],
    double distanceLower, double distanceWeight, const struct0_T *params,
    const double floorCenters_data[], const int floorCenters_size[2],
    const double floorRadii_data[], const int floorRadii_size[1],
    const double floorMargins_data[], const int floorMargins_size[1],
    double floorCount, double qOut[4320], int *qCount, int stageCounts[3],
    double baseStates[600], int *baseCount, double baseCommands[600],
    int *baseCmdCount, double *sampleTime);

}
} // namespace gik9dof
void stagedFollowTrajectory_delete();

void stagedFollowTrajectory_init();

void stagedFollowTrajectory_new();

#endif
// End of code generation (gik9dof_codegen_stagedFollowTrajectory.h)
