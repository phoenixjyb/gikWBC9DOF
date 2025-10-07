//
// File: OccupancyGrid2D.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

#ifndef OCCUPANCYGRID2D_H
#define OCCUPANCYGRID2D_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
class OccupancyGrid2D {
public:
  void init(const boolean_T b_data[40000], double b_resolution,
            double b_origin_x, double b_origin_y, int b_size_x, int b_size_y);
  OccupancyGrid2D();
  ~OccupancyGrid2D();
  boolean_T data[40000];
  double resolution;
  double origin_x;
  double origin_y;
  int size_x;
  int size_y;
};

} // namespace gik9dof

#endif
//
// File trailer for OccupancyGrid2D.h
//
// [EOF]
//
