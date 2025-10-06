//
// stageBPlanPath.h
//
// Code generation for function 'stageBPlanPath'
//

#ifndef STAGEBPLANPATH_H
#define STAGEBPLANPATH_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct b_struct_T;

// Function Declarations
namespace gik9dof {
namespace codegen {
int stageBPlanPath(const double startState[3], const double goalState[3],
                   const double floorCenters_data[],
                   const int floorCenters_size[2],
                   const double floorRadii_data[],
                   const double floorMargins_data[], double floorCount,
                   const b_struct_T &params, double states[600],
                   double commands[600], int *commandCount);

}
} // namespace gik9dof

#endif
// End of code generation (stageBPlanPath.h)
