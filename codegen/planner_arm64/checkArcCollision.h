//
// File: checkArcCollision.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 14:15:48
//

#ifndef CHECKARCCOLLISION_H
#define CHECKARCCOLLISION_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace gik9dof {
class OccupancyGrid2D;

}

// Function Declarations
namespace gik9dof {
bool checkArcCollision(double x_start, double y_start, double theta_start,
                       double Vx, double Wz, double dt,
                       const OccupancyGrid2D *grid);

}

#endif
//
// File trailer for checkArcCollision.h
//
// [EOF]
//
