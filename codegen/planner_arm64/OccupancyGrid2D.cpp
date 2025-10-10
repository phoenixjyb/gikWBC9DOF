//
// File: OccupancyGrid2D.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 10-Oct-2025 14:15:48
//

// Include Files
#include "OccupancyGrid2D.h"
#include "rt_nonfinite.h"
#include <algorithm>

// Function Definitions
//
// Arguments    : void
// Return Type  : OccupancyGrid2D
//
namespace gik9dof {
OccupancyGrid2D::OccupancyGrid2D() = default;

//
// Arguments    : void
// Return Type  : void
//
OccupancyGrid2D::~OccupancyGrid2D() = default;

//
// Arguments    : const bool b_data[40000]
//                double b_resolution
//                double b_origin_x
//                double b_origin_y
//                int b_size_x
//                int b_size_y
// Return Type  : void
//
void OccupancyGrid2D::init(const bool b_data[40000], double b_resolution,
                           double b_origin_x, double b_origin_y, int b_size_x,
                           int b_size_y)
{
  std::copy(&b_data[0], &b_data[40000], &data[0]);
  resolution = b_resolution;
  origin_x = b_origin_x;
  origin_y = b_origin_y;
  size_x = b_size_x;
  size_y = b_size_y;
}

} // namespace gik9dof

//
// File trailer for OccupancyGrid2D.cpp
//
// [EOF]
//
