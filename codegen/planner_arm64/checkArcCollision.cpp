//
// File: checkArcCollision.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 13:46:29
//

// Include Files
#include "checkArcCollision.h"
#include "OccupancyGrid2D.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// CHECKARCCOLLISION Check if motion primitive arc collides with obstacles
//    has_collision = checkArcCollision(x_start, y_start, theta_start, Vx, Wz,
//    dt, grid, params)
//
//    Samples points along the arc and checks robot footprint against occupancy
//    grid. Uses conservative circle-based collision model for efficiency.
//
//    INPUTS:
//        x_start, y_start - Starting position [m] in world frame
//        theta_start      - Starting heading [rad]
//        Vx               - Forward velocity [m/s]
//        Wz               - Yaw rate [rad/s]
//        dt               - Duration [s]
//        grid             - OccupancyGrid2D object (from
//        gik9dof.OccupancyGrid2D) params           - Chassis params (must have
//        .robot_radius)
//
//    OUTPUTS:
//        has_collision - Boolean: true if arc collides, false if clear
//
//    Algorithm:
//        1. Sample arc at intervals (every 0.1m or finer)
//        2. For each sample point:
//           a. Check if robot center is in bounds
//           b. Check if robot footprint (circle of radius robot_radius)
//           overlaps obstacles
//        3. Return true at first collision, false if all samples clear
//
//    Robot model: Conservative bounding circle (radius = 0.46m)
//    Grid assumes obstacles are already inflated by robot_radius + margin
//
//    See also computeMotionPrimitive, OccupancyGrid2D, inflateObstacles
//
// Arguments    : double x_start
//                double y_start
//                double theta_start
//                double Vx
//                double Wz
//                double dt
//                const OccupancyGrid2D *grid
// Return Type  : bool
//
namespace gik9dof {
bool checkArcCollision(double x_start, double y_start, double theta_start,
                       double Vx, double Wz, double dt,
                       const OccupancyGrid2D *grid)
{
  double t_samples_data[13];
  double delta1;
  int k;
  int t_samples_size_idx_1;
  bool has_collision;
  //  Validate inputs
  //  Sampling resolution: check every 0.1m along arc
  //  [m] Check every 10cm
  //  At least start + end
  //  Time samples
  t_samples_size_idx_1 =
      static_cast<int>(std::fmax(2.0, std::ceil(std::abs(Vx) * dt / 0.1)));
  t_samples_data[t_samples_size_idx_1 - 1] = dt;
  t_samples_data[0] = 0.0;
  if (t_samples_size_idx_1 >= 3) {
    delta1 = dt / (static_cast<double>(t_samples_size_idx_1) - 1.0);
    for (k = 0; k <= t_samples_size_idx_1 - 3; k++) {
      t_samples_data[k + 1] = (static_cast<double>(k) + 1.0) * delta1;
    }
  }
  //  Check each sample point along arc
  k = 0;
  int exitg1;
  do {
    exitg1 = 0;
    if (k <= t_samples_size_idx_1 - 1) {
      double x;
      int grid_x;
      int grid_y;
      //  Compute position at this point along arc
      if (std::abs(Wz) < 0.0001) {
        //  Straight line
        delta1 = Vx * t_samples_data[k];
        x = x_start + delta1 * std::cos(theta_start);
        delta1 = y_start + delta1 * std::sin(theta_start);
      } else {
        double theta_t;
        //  Circular arc
        delta1 = Vx / Wz;
        theta_t = theta_start + Wz * t_samples_data[k];
        x = (x_start - delta1 * std::sin(theta_start)) +
            delta1 * std::sin(theta_t);
        delta1 = (y_start + delta1 * std::cos(theta_start)) -
                 delta1 * std::cos(theta_t);
      }
      //  Convert world position to grid indices
      // WORLDTOGRIDX Convert world x coordinate to grid index
      grid_x = static_cast<int>(
                   std::floor((x - grid->origin_x) / grid->resolution)) +
               1;
      // WORLDTOGRADY Convert world y coordinate to grid index
      grid_y = static_cast<int>(
                   std::floor((delta1 - grid->origin_y) / grid->resolution)) +
               1;
      //  Check if position is within grid bounds
      if ((grid_x < 1) || (grid_x > grid->size_x) || (grid_y < 1) ||
          (grid_y > grid->size_y)) {
        has_collision = true;
        //  Out of bounds = collision
        exitg1 = 1;

        //  Check occupancy at robot center
        //  Assumes grid is already inflated by robot_radius, so just check
        //  center point
      } else if (static_cast<double>(
                     grid->data[(grid_y + 200 * (grid_x - 1)) - 1]) > 0.5) {
        //  Occupied threshold
        has_collision = true;
        exitg1 = 1;
      } else {
        k++;
      }
    } else {
      //  All samples clear
      has_collision = false;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  return has_collision;
}

} // namespace gik9dof

//
// File trailer for checkArcCollision.cpp
//
// [EOF]
//
