//
// File: HybridAStarPlanner.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 13:46:29
//

// Include Files
#include "HybridAStarPlanner.h"
#include "CoderTimeAPI.h"
#include "OccupancyGrid2D.h"
#include "angdiff.h"
#include "checkArcCollision.h"
#include "generateMotionPrimitives.h"
#include "getChassisParams.h"
#include "mod.h"
#include "planHybridAStarCodegen_internal_types.h"
#include "planHybridAStarCodegen_types.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "timeKeeper.h"
#include "toc.h"
#include "coder_array.h"
#include "coder_posix_time.h"
#include "rt_defines.h"
#include <cmath>
#include <cstring>

// Function Declarations
namespace gik9dof {
static void reconstructPathFixed(int final_idx, const int parent_indices[50000],
                                 const struct0_T state_list[50000],
                                 struct1_T path[500]);

static double rt_atan2d_snf(double u0, double u1);

} // namespace gik9dof

// Function Definitions
//
// RECONSTRUCTPATHFIXED Reconstruct path with fixed-size output
//
// Arguments    : int final_idx
//                const int parent_indices[50000]
//                const struct0_T state_list[50000]
//                struct1_T path[500]
// Return Type  : void
//
namespace gik9dof {
static void reconstructPathFixed(int final_idx, const int parent_indices[50000],
                                 const struct0_T state_list[50000],
                                 struct1_T path[500])
{
  int temp_indices[500];
  int current_idx;
  int path_count;
  //  Path Reconstruction
  //  Create empty path
  //  Trace back from goal to start
  std::memset(&path[0], 0, 500U * sizeof(struct1_T));
  std::memset(&temp_indices[0], 0, 500U * sizeof(int));
  path_count = -1;
  current_idx = final_idx;
  while ((current_idx > 0) && (path_count + 1 < 500)) {
    path_count++;
    temp_indices[path_count] = current_idx;
    current_idx = parent_indices[current_idx - 1];
  }
  //  Reverse to get start-to-goal order
  for (current_idx = 0; current_idx <= path_count; current_idx++) {
    int idx;
    idx = temp_indices[path_count - current_idx] - 1;
    path[current_idx].x = state_list[idx].x;
    path[current_idx].y = state_list[idx].y;
    path[current_idx].theta = state_list[idx].theta;
    path[current_idx].Vx = state_list[idx].Vx;
    path[current_idx].Wz = state_list[idx].Wz;
    path[current_idx].dt = state_list[idx].dt;
  }
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = std::atan2(static_cast<double>(i), static_cast<double>(i1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

//
// Arguments    : void
// Return Type  : void
//
HybridAStarPlanner::HybridAStarPlanner()
{
  SD_.pd = &pd_;
  CoderTimeAPI::callCoderClockGettime_init(this);
  timeKeeper_init(this);
}

//
// Arguments    : void
// Return Type  : void
//
HybridAStarPlanner::~HybridAStarPlanner() = default;

//
// PLANHYBRIDASTAR_CODEGEN SE(2) Hybrid A* path planning (MATLAB Coder
// compatible)
//
//    Code-generation compatible version with fixed-size arrays
//
//    Inputs:
//        start_state: HybridState struct (x, y, theta)
//        goal_state: HybridState struct (x, y, theta)
//        occupancy_grid: OccupancyGrid2D object (must be already inflated)
//        options: struct with fields (all optional):
//            max_iterations: Maximum search iterations (default: 10000)
//            goal_tolerance_xy: XY goal tolerance in meters (default: 0.2)
//            goal_tolerance_theta: Heading tolerance in radians (default: 0.3)
//            use_dubins_heuristic: true=Dubins, false=Euclidean (default: true)
//            timeout_sec: Maximum planning time in seconds (default: 5.0)
//            max_states: Maximum states to explore (default: 50000)
//            max_path_length: Maximum path waypoints (default: 500)
//
//    Outputs:
//        path: Struct array with fields (x, y, theta, Vx, Wz, dt)
//            Fixed-size array [max_path_length × 1]
//            Actual length in search_stats.path_length
//        search_stats: Struct with fields:
//            success: true if path found
//            iterations: Number of iterations executed
//            nodes_expanded: Number of states expanded
//            planning_time_sec: Total planning time
//            path_cost: Total path cost (g-value at goal)
//            path_length: Number of waypoints in path
//
// Arguments    : struct0_T *start_state
//                const struct0_T *goal_state
//                const OccupancyGrid2D *occupancy_grid
//                struct1_T path[500]
//                struct2_T *search_stats
// Return Type  : void
//
void HybridAStarPlanner::b_planHybridAStarCodegen(
    struct0_T *start_state, const struct0_T *goal_state,
    const OccupancyGrid2D *occupancy_grid, struct1_T path[500],
    struct2_T *search_stats)
{
  ::coder::array<bool, 3U> visited;
  coderTimespec savedTime;
  struct_T primitives_all[30];
  double d_euclidean;
  double dx;
  double dy;
  double h_tmp;
  double theta_goal_tmp;
  double theta_start;
  int grid_x;
  int grid_y;
  bool goal_reached;
  //  Parse options with defaults
  //  Fixed-size limits for code generation
  //  Get chassis parameters
  getChassisParams();
  //  Generate motion primitives (returns fixed-size array)
  generateMotionPrimitives(primitives_all);
  //  Filter valid primitives (dt > 0)
  //  For codegen: Keep fixed-size array, track valid count
  //  Use full array
  //  Angular discretization (matches design: 16 bins)
  //  Grid size
  //  Discretize start and goal states
  // WORLDTOGRIDX Convert world x coordinate to grid index
  start_state->grid_x =
      static_cast<int>(std::floor((start_state->x - occupancy_grid->origin_x) /
                                  occupancy_grid->resolution)) +
      1;
  // WORLDTOGRADY Convert world y coordinate to grid index
  start_state->grid_y =
      static_cast<int>(std::floor((start_state->y - occupancy_grid->origin_y) /
                                  occupancy_grid->resolution)) +
      1;
  //  Normalize theta to [0, 2*pi)
  //  Convert to bin index [1, num_bins]
  grid_x = static_cast<int>(std::floor(coder::b_mod(start_state->theta) /
                                       0.39269908169872414)) +
           1;
  if (grid_x >= 16) {
    grid_x = 16;
  }
  if (grid_x <= 1) {
    start_state->theta_bin = 1;
  } else {
    start_state->theta_bin = grid_x;
  }
  start_state->g = 0.0;
  // COMPUTEDUBINSHEURISTIC Compute admissible heuristic for non-holonomic
  // planning
  //    h_cost = computeDubinsHeuristic(x_start, y_start, theta_start, x_goal,
  //    y_goal, theta_goal, R_min)
  //
  //    Computes approximate Dubins path length between two SE(2) poses.
  //    Dubins paths are shortest paths for vehicles with minimum turning
  //    radius.
  //
  //    INPUTS:
  //        x_start, y_start   - Start position [m]
  //        theta_start        - Start heading [rad]
  //        x_goal, y_goal     - Goal position [m]
  //        theta_goal         - Goal heading [rad]
  //        R_min              - Minimum turning radius [m]
  //
  //    OUTPUTS:
  //        h_cost - Estimated path length [m] (admissible: never overestimates)
  //
  //    Method:
  //        Uses simplified Dubins path calculation (not full 6 path types)
  //        1. If aligned and goal ahead: Use Euclidean distance (straight line)
  //        2. Else: Estimate using arc + straight + arc approximation
  //
  //    Dubins path types (full):
  //        LSL, RSR, LSR, RSL, RLR, LRL (L=left, R=right, S=straight)
  //
  //    Simplified approximation (for speed):
  //        - Compute straight-line distance d
  //        - Estimate heading alignment cost ~ R_min * |delta_theta|
  //        - Total ≈ d + heading_alignment_cost
  //
  //    Properties:
  //        - Admissible: Always underestimates (guarantees optimal A*)
  //        - Consistent: Satisfies triangle inequality
  //        - Fast: ~0.01 ms/call (suitable for real-time)
  //
  //    See also planHybridAStar, HybridState, computeMotionPrimitive
  //
  //    References:
  //        Dubins, L.E. (1957). "On Curves of Minimal Length with a Constraint
  //        on Average Curvature, and with Prescribed Initial and Terminal
  //        Positions and Tangents"
  //  Normalize angles to [-pi, pi]
  theta_start =
      rt_atan2d_snf(std::sin(start_state->theta), std::cos(start_state->theta));
  theta_goal_tmp =
      rt_atan2d_snf(std::sin(goal_state->theta), std::cos(goal_state->theta));
  //  Straight-line distance (Euclidean)
  dx = goal_state->x - start_state->x;
  dy = goal_state->y - start_state->y;
  d_euclidean = std::sqrt(dx * dx + dy * dy);
  //  Handle trivial case (already at goal)
  if (d_euclidean < 0.001) {
    //  Only heading difference matters
    h_tmp = theta_goal_tmp - theta_start;
    dy = 0.34379999999999994 *
         std::abs(rt_atan2d_snf(std::sin(h_tmp), std::cos(h_tmp)));
    //  Arc length to align heading
  } else {
    //  Heading to goal (straight-line direction)
    dx = rt_atan2d_snf(dy, dx);
    //  Heading differences
    //  Simplified Dubins approximation:
    //  Path ≈ (turn to align) + (straight segment) + (turn to goal heading)
    //
    //  For admissibility, we underestimate:
    //  - Assume we can turn optimally (shortest arc)
    //  - Use Euclidean distance for straight segment
    //  - Don't account for overshoot (actual Dubins may be longer)
    //  Alignment cost (convert heading differences to arc lengths)
    //  Use smaller factor to ensure admissibility
    //  Conservative (ensures h <= actual cost)
    //  Total cost estimate
    //  Ensure non-negative
    h_tmp = dx - theta_start;
    dy = theta_goal_tmp - dx;
    dy = std::fmax(
        0.0, (d_euclidean +
              0.17189999999999997 *
                  std::abs(rt_atan2d_snf(std::sin(h_tmp), std::cos(h_tmp)))) +
                 0.17189999999999997 *
                     std::abs(rt_atan2d_snf(std::sin(dy), std::cos(dy))));
  }
  start_state->h = dy;
  start_state->f = dy;
  // WORLDTOGRIDX Convert world x coordinate to grid index
  // WORLDTOGRADY Convert world y coordinate to grid index
  //  Normalize theta to [0, 2*pi)
  //  Convert to bin index [1, num_bins]
  //  Check start validity
  // CHECKFOOTPRINTCOLLISION Check if robot footprint at pose collides with
  // obstacles
  //    has_collision = checkFootprintCollision(x, y, theta, grid, params)
  //
  //    Checks if the robot's circular footprint at the given pose overlaps
  //    with any obstacles in the occupancy grid.
  //
  //    INPUTS:
  //        x, y     - Robot position [m] in world frame
  //        theta    - Robot heading [rad] (not used for circular model)
  //        grid     - OccupancyGrid2D object
  //        params   - Chassis params with .robot_radius [m]
  //
  //    OUTPUTS:
  //        has_collision - Boolean: true if footprint collides
  //
  //    Method:
  //        Assumes grid is already inflated by robot_radius + margin
  //        So we only need to check the center point
  //
  //    See also checkArcCollision, OccupancyGrid2D
  //  Use circular footprint (simple: check center point since grid is inflated)
  //  Convert to grid coordinates
  // WORLDTOGRIDX Convert world x coordinate to grid index
  grid_x =
      static_cast<int>(std::floor((start_state->x - occupancy_grid->origin_x) /
                                  occupancy_grid->resolution)) +
      1;
  // WORLDTOGRADY Convert world y coordinate to grid index
  grid_y =
      static_cast<int>(std::floor((start_state->y - occupancy_grid->origin_y) /
                                  occupancy_grid->resolution)) +
      1;
  //  Bounds check
  if ((grid_x < 1) || (grid_x > occupancy_grid->size_x) || (grid_y < 1) ||
      (grid_y > occupancy_grid->size_y)) {
    goal_reached = true;
  } else {
    //  Occupancy check (grid is already inflated, so check center)
    goal_reached =
        (static_cast<double>(
             occupancy_grid->data[(grid_y + 200 * (grid_x - 1)) - 1]) > 0.5);
  }
  if (goal_reached) {
    std::memset(&path[0], 0, 500U * sizeof(struct1_T));
    search_stats->success = false;
    search_stats->iterations = 0.0;
    search_stats->nodes_expanded = 0.0;
    search_stats->planning_time_sec = 0.0;
    search_stats->path_cost = rtInf;
    search_stats->path_length = 0.0;
  } else {
    int final_state_idx;
    int heap_size;
    int i;
    int iterations;
    int next_grid_y;
    int nodes_expanded;
    int state_count;
    //  Initialize fixed-size data structures
    //  State list: Pre-allocated array of states
    //  Add start state
    state_count = 1;
    //  Open list: Binary heap (min-heap by f-score)
    //  heap_indices: Indices into state_list
    //  heap_size: Current heap size
    for (i = 0; i < 50000; i++) {
      SD_.f0.state_list[i].x = 0.0;
      SD_.f0.state_list[i].y = 0.0;
      SD_.f0.state_list[i].theta = 0.0;
      SD_.f0.state_list[i].grid_x = 0;
      SD_.f0.state_list[i].grid_y = 0;
      SD_.f0.state_list[i].theta_bin = 0;
      SD_.f0.state_list[i].Vx = 0.0;
      SD_.f0.state_list[i].Wz = 0.0;
      SD_.f0.state_list[i].dt = 0.0;
      SD_.f0.state_list[i].g = 0.0;
      SD_.f0.state_list[i].h = 0.0;
      SD_.f0.state_list[i].f = 0.0;
      SD_.f0.state_list[i].parent_idx = 0;
      SD_.f0.state_list[i].is_valid = true;
      SD_.f0.heap_indices[i] = 0;
    }
    SD_.f0.state_list[0] = *start_state;
    //  Insert start state into heap
    heap_size = 0;
    SD_.f0.heap_indices[0] = 1;
    //  Closed set: 3D visited array (grid_x, grid_y, theta_bin)
    visited.set_size(occupancy_grid->size_x, occupancy_grid->size_y, 16);
    grid_x = (occupancy_grid->size_x * occupancy_grid->size_y) << 4;
    for (next_grid_y = 0; next_grid_y < grid_x; next_grid_y++) {
      visited[next_grid_y] = false;
    }
    //  Parent map: Fixed-size array indexed by state index
    std::memset(&SD_.f0.parent_indices[0], 0, 50000U * sizeof(int));
    //  Search loop
    coder::tic(this, savedTime);
    iterations = 0;
    nodes_expanded = 0;
    goal_reached = false;
    final_state_idx = -1;
    int exitg3;
    do {
      exitg3 = 0;
      if ((heap_size + 1 > 0) && (iterations < 10000)) {
        iterations++;
        //  Check timeout
        if (coder::toc(this, savedTime) > 5.0) {
          exitg3 = 1;
        } else {
          int current_idx;
          int idx;
          //  Pop state with lowest f-score from heap
          // HEAPPOP Pop minimum f-score state from heap
          //  Binary Heap Operations (Min-heap by f-score)
          //  Return root (minimum)
          current_idx = SD_.f0.heap_indices[0] - 1;
          //  Move last element to root and bubble down
          SD_.f0.heap_indices[0] = SD_.f0.heap_indices[heap_size];
          // HEAPBUBBLEDOWN Bubble down root to maintain heap property
          idx = 0;
          int exitg4;
          do {
            exitg4 = 0;
            grid_x = (idx + 1) << 1;
            grid_y = idx;
            //  Find smallest among parent, left, right
            if ((grid_x <= heap_size) &&
                (SD_.f0.state_list[SD_.f0.heap_indices[grid_x - 1] - 1].f <
                 SD_.f0.state_list[SD_.f0.heap_indices[idx] - 1].f)) {
              grid_y = grid_x - 1;
            }
            if ((grid_x + 1 <= heap_size) &&
                (SD_.f0.state_list[SD_.f0.heap_indices[grid_x] - 1].f <
                 SD_.f0.state_list[SD_.f0.heap_indices[grid_y] - 1].f)) {
              grid_y = grid_x;
            }
            if (grid_y + 1 != idx + 1) {
              //  Swap
              next_grid_y = SD_.f0.heap_indices[idx];
              SD_.f0.heap_indices[idx] = SD_.f0.heap_indices[grid_y];
              SD_.f0.heap_indices[grid_y] = next_grid_y;
              idx = grid_y;
            } else {
              exitg4 = 1;
            }
          } while (exitg4 == 0);
          heap_size--;
          if (current_idx + 1 <= 0) {
            exitg3 = 1;
          } else {
            double current_state_g;
            double current_state_theta;
            double current_state_x_tmp;
            double current_state_y_tmp;
            current_state_x_tmp = SD_.f0.state_list[current_idx].x;
            current_state_y_tmp = SD_.f0.state_list[current_idx].y;
            current_state_theta = SD_.f0.state_list[current_idx].theta;
            current_state_g = SD_.f0.state_list[current_idx].g;
            //  Check if already visited
            next_grid_y = SD_.f0.state_list[current_idx].grid_x - 1;
            grid_x = SD_.f0.state_list[current_idx].grid_y - 1;
            grid_y = SD_.f0.state_list[current_idx].theta_bin - 1;
            if (!visited[(next_grid_y + visited.size(0) * grid_x) +
                         visited.size(0) * visited.size(1) * grid_y]) {
              //  Mark as visited
              visited[(next_grid_y + visited.size(0) * grid_x) +
                      visited.size(0) * visited.size(1) * grid_y] = true;
              nodes_expanded++;
              //  Check goal condition
              dx = current_state_x_tmp - goal_state->x;
              dy = current_state_y_tmp - goal_state->y;
              if ((std::sqrt(dx * dx + dy * dy) <= 0.2) &&
                  (std::abs(coder::angdiff(SD_.f0.state_list[current_idx].theta,
                                           goal_state->theta)) <= 0.3)) {
                goal_reached = true;
                final_state_idx = current_idx;
                exitg3 = 1;
              } else {
                bool exitg2;
                //  Expand motion primitives
                i = 0;
                exitg2 = false;
                while ((!exitg2) && (i < 30)) {
                  //  Skip invalid primitives (dt=0 for unused slots)
                  if (primitives_all[i].dt <= 0.0) {
                    i++;

                    //  Check state limit
                  } else if (state_count >= 49999) {
                    exitg2 = true;
                  } else {
                    //  Apply primitive to current state
                    // COMPUTEMOTIONPRIMITIVE Compute endpoint of motion
                    // primitive for front-diff + passive-rear robot
                    //    [x_end, y_end, theta_end] =
                    //    computeMotionPrimitive(x_start, y_start, theta_start,
                    //    Vx, Wz, dt, params)
                    //
                    //    Kinematic model: Simplified Ackermann (front
                    //    differential + passive rear)
                    //    - Front axle: Powered differential (control v_L, v_R)
                    //    - Rear axle: Passive omniwheels (allow lateral slip)
                    //    - Constraint: Minimum turning radius R_min ~ 0.34 m
                    //
                    //    INPUTS:
                    //        x_start, y_start - Starting position [m]
                    //        theta_start      - Starting heading [rad], 0=+X,
                    //        CCW+ Vx               - Forward velocity command
                    //        [m/s] (positive = forward) Wz               - Yaw
                    //        rate command [rad/s] (positive = CCW) dt -
                    //        Duration [s] params           - Chassis params
                    //        from getChassisParams()
                    //                           Must include: .wheelbase,
                    //                           .track, .Vwheel_max,
                    //                           .min_turning_radius
                    //
                    //    OUTPUTS:
                    //        x_end, y_end     - Ending position [m]
                    //        theta_end        - Ending heading [rad]
                    //
                    //    Kinematic equations (SE(2) dynamics):
                    //        dx/dt = Vx * cos(theta)
                    //        dy/dt = Vx * sin(theta)
                    //        dtheta/dt = Wz
                    //
                    //    Arc motion (constant Vx, Wz):
                    //        - If |Wz| < threshold: straight line
                    //        - Else: circular arc with radius R = Vx / Wz
                    //
                    //    Constraints enforced:
                    //        1. Wheel speed limits: |Vx ± (track/2)*Wz| <=
                    //        Vwheel_max
                    //        2. Minimum radius: R = |Vx/Wz| >=
                    //        min_turning_radius
                    //        3. Forward/backward speed: |Vx| <= Vx_max
                    //        4. Yaw rate: |Wz| <= Wz_max
                    //
                    //    See also HybridState, generateMotionPrimitives,
                    //    getChassisParams
                    //  Validate inputs
                    //  Check wheel speed limits
                    //  Check minimum turning radius (if turning)
                    //  [rad/s] Treat as straight line below this
                    //  Compute motion
                    if (std::abs(primitives_all[i].Wz) < 0.0001) {
                      //  Straight line motion
                      dy = primitives_all[i].Vx * primitives_all[i].dt;
                      d_euclidean = current_state_x_tmp +
                                    dy * std::cos(current_state_theta);
                      h_tmp = current_state_y_tmp +
                              dy * std::sin(current_state_theta);
                      dy = current_state_theta;
                    } else {
                      //  Circular arc motion
                      dy = primitives_all[i].Wz;
                      dx = primitives_all[i].Vx / dy;
                      //  Turning radius (signed)
                      //  Total heading change [rad]
                      //  Center of circular arc (perpendicular to heading)
                      //  New heading
                      dy = current_state_theta + dy * primitives_all[i].dt;
                      //  New position (rotate around center)
                      d_euclidean = (current_state_x_tmp -
                                     dx * std::sin(current_state_theta)) +
                                    dx * std::sin(dy);
                      h_tmp = (current_state_y_tmp +
                               dx * std::cos(current_state_theta)) -
                              dx * std::cos(dy);
                    }
                    //  Normalize theta to [-pi, pi]
                    dy = rt_atan2d_snf(std::sin(dy), std::cos(dy));
                    //  Discretize next state
                    // WORLDTOGRIDX Convert world x coordinate to grid index
                    grid_y = static_cast<int>(std::floor(
                                 (d_euclidean - occupancy_grid->origin_x) /
                                 occupancy_grid->resolution)) +
                             1;
                    // WORLDTOGRADY Convert world y coordinate to grid index
                    next_grid_y = static_cast<int>(std::floor(
                                      (h_tmp - occupancy_grid->origin_y) /
                                      occupancy_grid->resolution)) +
                                  1;
                    //  Normalize theta to [0, 2*pi)
                    //  Convert to bin index [1, num_bins]
                    grid_x = static_cast<int>(std::floor(coder::b_mod(dy) /
                                                         0.39269908169872414)) +
                             1;
                    if (grid_x >= 16) {
                      grid_x = 16;
                    }
                    if (grid_x <= 1) {
                      grid_x = 1;
                    }
                    //  Check bounds
                    if ((grid_y >= 1) && (grid_y <= occupancy_grid->size_x) &&
                        (next_grid_y >= 1) &&
                        (next_grid_y <= occupancy_grid->size_y) &&
                        (!visited[((grid_y +
                                    visited.size(0) * (next_grid_y - 1)) +
                                   visited.size(0) * visited.size(1) *
                                       (grid_x - 1)) -
                                  1]) &&
                        (!checkArcCollision(
                            current_state_x_tmp, current_state_y_tmp,
                            current_state_theta, primitives_all[i].Vx,
                            primitives_all[i].Wz, primitives_all[i].dt,
                            occupancy_grid))) {
                      struct0_T next_state;
                      double b_next_state_tmp_tmp;
                      double next_state_tmp;
                      double next_state_tmp_tmp;
                      bool exitg1;
                      //  Skip if already visited
                      //  Collision check along arc
                      //  Compute cost (arc length)
                      //  Create next state
                      next_state.parent_idx = 0;
                      next_state.is_valid = true;
                      next_state.x = d_euclidean;
                      next_state.y = h_tmp;
                      next_state.theta = dy;
                      next_state.grid_x = grid_y;
                      next_state.grid_y = next_grid_y;
                      next_state.theta_bin = grid_x;
                      next_state_tmp_tmp = primitives_all[i].Vx;
                      b_next_state_tmp_tmp = primitives_all[i].dt;
                      next_state_tmp =
                          current_state_g +
                          std::abs(next_state_tmp_tmp) * b_next_state_tmp_tmp;
                      next_state.g = next_state_tmp;
                      // COMPUTEDUBINSHEURISTIC Compute admissible heuristic for
                      // non-holonomic planning
                      //    h_cost = computeDubinsHeuristic(x_start, y_start,
                      //    theta_start, x_goal, y_goal, theta_goal, R_min)
                      //
                      //    Computes approximate Dubins path length between two
                      //    SE(2) poses. Dubins paths are shortest paths for
                      //    vehicles with minimum turning radius.
                      //
                      //    INPUTS:
                      //        x_start, y_start   - Start position [m]
                      //        theta_start        - Start heading [rad]
                      //        x_goal, y_goal     - Goal position [m]
                      //        theta_goal         - Goal heading [rad]
                      //        R_min              - Minimum turning radius [m]
                      //
                      //    OUTPUTS:
                      //        h_cost - Estimated path length [m] (admissible:
                      //        never overestimates)
                      //
                      //    Method:
                      //        Uses simplified Dubins path calculation (not
                      //        full 6 path types)
                      //        1. If aligned and goal ahead: Use Euclidean
                      //        distance (straight line)
                      //        2. Else: Estimate using arc + straight + arc
                      //        approximation
                      //
                      //    Dubins path types (full):
                      //        LSL, RSR, LSR, RSL, RLR, LRL (L=left, R=right,
                      //        S=straight)
                      //
                      //    Simplified approximation (for speed):
                      //        - Compute straight-line distance d
                      //        - Estimate heading alignment cost ~ R_min *
                      //        |delta_theta|
                      //        - Total ≈ d + heading_alignment_cost
                      //
                      //    Properties:
                      //        - Admissible: Always underestimates (guarantees
                      //        optimal A*)
                      //        - Consistent: Satisfies triangle inequality
                      //        - Fast: ~0.01 ms/call (suitable for real-time)
                      //
                      //    See also planHybridAStar, HybridState,
                      //    computeMotionPrimitive
                      //
                      //    References:
                      //        Dubins, L.E. (1957). "On Curves of Minimal
                      //        Length with a Constraint on Average Curvature,
                      //        and with Prescribed Initial and Terminal
                      //        Positions and Tangents"
                      //  Normalize angles to [-pi, pi]
                      theta_start = rt_atan2d_snf(std::sin(dy), std::cos(dy));
                      //  Straight-line distance (Euclidean)
                      dx = goal_state->x - d_euclidean;
                      dy = goal_state->y - h_tmp;
                      d_euclidean = std::sqrt(dx * dx + dy * dy);
                      //  Handle trivial case (already at goal)
                      if (d_euclidean < 0.001) {
                        //  Only heading difference matters
                        h_tmp = theta_goal_tmp - theta_start;
                        dy = 0.34379999999999994 *
                             std::abs(rt_atan2d_snf(std::sin(h_tmp),
                                                    std::cos(h_tmp)));
                        //  Arc length to align heading
                      } else {
                        //  Heading to goal (straight-line direction)
                        dx = rt_atan2d_snf(dy, dx);
                        //  Heading differences
                        //  Simplified Dubins approximation:
                        //  Path ≈ (turn to align) + (straight segment) + (turn
                        //  to goal heading)
                        //
                        //  For admissibility, we underestimate:
                        //  - Assume we can turn optimally (shortest arc)
                        //  - Use Euclidean distance for straight segment
                        //  - Don't account for overshoot (actual Dubins may be
                        //  longer) Alignment cost (convert heading differences
                        //  to arc lengths) Use smaller factor to ensure
                        //  admissibility Conservative (ensures h <= actual
                        //  cost) Total cost estimate Ensure non-negative
                        h_tmp = dx - theta_start;
                        dy = theta_goal_tmp - dx;
                        dy = std::fmax(
                            0.0, (d_euclidean +
                                  0.17189999999999997 *
                                      std::abs(rt_atan2d_snf(
                                          std::sin(h_tmp), std::cos(h_tmp)))) +
                                     0.17189999999999997 *
                                         std::abs(rt_atan2d_snf(std::sin(dy),
                                                                std::cos(dy))));
                      }
                      next_state.h = dy;
                      next_state.f = next_state_tmp + dy;
                      next_state.Vx = next_state_tmp_tmp;
                      next_state.Wz = primitives_all[i].Wz;
                      next_state.dt = b_next_state_tmp_tmp;
                      //  Add to state list
                      state_count++;
                      SD_.f0.state_list[state_count - 1] = next_state;
                      //  Store parent
                      SD_.f0.parent_indices[state_count - 1] = current_idx + 1;
                      //  Add to heap
                      heap_size++;
                      SD_.f0.heap_indices[heap_size] = state_count;
                      idx = heap_size;
                      // HEAPBUBBLEUP Bubble up last element to maintain heap
                      // property
                      exitg1 = false;
                      while ((!exitg1) && (idx + 1 > 1)) {
                        grid_x = static_cast<int>(std::floor(
                                     static_cast<double>(idx + 1) / 2.0)) -
                                 1;
                        //  Compare f-scores
                        if (SD_.f0.state_list[SD_.f0.heap_indices[idx] - 1].f <
                            SD_.f0.state_list[SD_.f0.heap_indices[grid_x] - 1]
                                .f) {
                          //  Swap
                          next_grid_y = SD_.f0.heap_indices[idx];
                          SD_.f0.heap_indices[idx] =
                              SD_.f0.heap_indices[grid_x];
                          SD_.f0.heap_indices[grid_x] = next_grid_y;
                          idx = grid_x;
                        } else {
                          exitg1 = true;
                        }
                      }
                    }
                    i++;
                  }
                }
              }
            }
          }
        }
      } else {
        exitg3 = 1;
      }
    } while (exitg3 == 0);
    dy = coder::toc(this, savedTime);
    //  Reconstruct path
    if (goal_reached) {
      reconstructPathFixed(final_state_idx + 1, SD_.f0.parent_indices,
                           SD_.f0.state_list, path);
      //  Count actual path length
      grid_x = 0;
      i = 0;
      while ((i < 500) && ((path[i].dt > 0.0) || (i + 1 == 1))) {
        grid_x++;
        i++;
      }
      search_stats->planning_time_sec = dy;
      search_stats->path_cost = SD_.f0.state_list[final_state_idx].g;
      search_stats->success = true;
      search_stats->iterations = iterations;
      search_stats->nodes_expanded = nodes_expanded;
      search_stats->path_length = grid_x;
    } else {
      std::memset(&path[0], 0, 500U * sizeof(struct1_T));
      search_stats->planning_time_sec = dy;
      search_stats->success = false;
      search_stats->iterations = iterations;
      search_stats->nodes_expanded = nodes_expanded;
      search_stats->path_cost = rtInf;
      search_stats->path_length = 0.0;
    }
  }
}

//
// Arguments    : void
// Return Type  : planHybridAStarCodegenStackData *
//
planHybridAStarCodegenStackData *HybridAStarPlanner::getStackData()
{
  return &SD_;
}

} // namespace gik9dof

//
// File trailer for HybridAStarPlanner.cpp
//
// [EOF]
//
