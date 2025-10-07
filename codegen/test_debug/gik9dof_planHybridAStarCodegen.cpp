//
// File: gik9dof_planHybridAStarCodegen.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 19:25:30
//

// Include Files
#include "gik9dof_planHybridAStarCodegen.h"
#include "OccupancyGrid2D.h"
#include "angdiff.h"
#include "atan2.h"
#include "checkArcCollision.h"
#include "generateMotionPrimitives.h"
#include "gik9dof_planHybridAStarCodegen_data.h"
#include "gik9dof_planHybridAStarCodegen_initialize.h"
#include "gik9dof_planHybridAStarCodegen_internal_types.h"
#include "gik9dof_planHybridAStarCodegen_types.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include "coder_array.h"
#include "coder_posix_time.h"
#include <cmath>
#include <cstdio>
#include <cstring>

// Function Declarations
namespace gik9dof {
static void reconstructPathFixed(int final_idx, const int parent_indices[50000],
                                 const struct0_T state_list[50000],
                                 struct1_T path[500]);

}

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
void planHybridAStarCodegen(struct0_T *start_state, const struct0_T *goal_state,
                            const OccupancyGrid2D *occupancy_grid,
                            struct1_T path[500], struct2_T *search_stats)
{
  static struct0_T state_list[50000];
  static int heap_indices[50000];
  static int parent_indices[50000];
  coder::array<boolean_T, 3U> visited;
  coderTimespec savedTime;
  struct_T primitives_all[30];
  double d_euclidean;
  double dx;
  double dy;
  double theta_goal_tmp;
  double theta_start;
  int grid_size_x;
  int grid_size_y;
  int next_theta_bin;
  int parent_idx;
  int qY;
  boolean_T goal_reached;
  if (!isInitialized_gik9dof_planHybridAStarCodegen) {
    gik9dof_planHybridAStarCodegen_initialize();
  }
  //  Parse options with defaults
  //  Fixed-size limits for code generation
  //  Get chassis parameters
  // GETCHASSISPARAMS Return WHEELTEC chassis parameters for path planning
  //    params = getChassisParams() returns default "fourwheel" platform
  //    params = getChassisParams('compact') - Top diff chassis (0.329m track)
  //    params = getChassisParams('fourwheel') - Four-wheel platform (0.573m
  //    track) ← DEFAULT
  //
  //    YOUR PLATFORM: WHEELTEC Four-Wheel Drive with Passive Rear
  //    - Front axle: Differential drive (two powered wheels)
  //    - Rear axle: Passive omni-wheels (lateral rolling rollers)
  //    - Wheelbase: 0.36 m (front-to-rear distance)
  //    - Track width: 0.573 m (left-to-right at front wheels)
  //    - Wheel radius: 0.1075 m (107.5mm, 215mm diameter drive wheels)
  //    - Kinematics: Simplified Ackermann (front-wheel steer via differential)
  //
  //    This is a HYBRID kinematic model:
  //    - Driven like differential drive (front axle, left/right speeds)
  //    - Constrained like Ackermann (passive rear creates ICR)
  //    - Minimum turning radius exists (NOT zero radius!)
  //
  //    Based on firmware analysis in docs/chassis-summary.txt
  //
  //    Returns struct with fields:
  //        .chassis_type    - 'front_diff_rear_passive' (unique hybrid)
  //        .wheelbase       - Front-to-rear distance [m] ← KEY PARAMETER
  //        .track           - Front wheel track width [m]
  //        .wheel_radius    - Drive wheel radius [m]
  //        .robot_length    - Front-to-back dimension [m]
  //        .robot_width     - Side-to-side dimension [m]
  //        .robot_radius    - Conservative bounding circle [m]
  //        .Vwheel_max      - Max wheel speed [m/s]
  //        .Vx_max          - Max forward speed [m/s]
  //        .Wz_max          - Max yaw rate [rad/s]
  //        .min_turning_radius - Minimum turn radius [m] ← IMPORTANT
  //        .accel_max       - Max acceleration [m/s^2]
  //
  //    See also gik9dof.control.defaultUnifiedParams
  //  Pre-allocate ALL struct fields for MATLAB Coder compatibility
  //  Four-wheel platform: Front diff + Rear passive omni
  //  [m] Front-to-rear axle distance ← YOUR SPEC
  //  [m] Front track width
  //  [m] 215mm diameter drive wheels
  //  [m] Including bumpers (wheelbase + clearance)
  //  [m] Track + side clearance
  //  Conservative bounding circle (for collision checking)
  //  Velocity limits (from firmware + existing controller)
  //  [m/s] Per-wheel limit (verified in defaultUnifiedParams)
  //  [m/s] Conservative forward speed (existing)
  //  KINEMATICS: Front diff + passive rear creates Instantaneous Center of
  //  Rotation (ICR) The passive rear wheels with lateral rollers allow the
  //  robot to pivot, but the ICR is constrained by geometry (NOT zero radius
  //  like pure diff-drive) Yaw rate limits (differential constraints at front
  //  axle) Front axle differential: |Wz| <= 2 * (Vwheel_max - |Vx|) / track
  //  [rad/s] Pure spin
  //  Conservative yaw limit for mixed motion (Vx = Vx_max)
  //  Use firmware limit (2.5 rad/s) as ultimate cap
  //  [rad/s]
  //  Acceleration limit (from firmware ramp: 0.02 m/s per 10ms = 2 m/s^2)
  //  [m/s^2]
  //  MINIMUM TURNING RADIUS (KEY for Hybrid A*)
  //  With passive rear, the robot pivots around a point related to wheelbase
  //  Approximation: R_min ≈ wheelbase / tan(max_virtual_steer_angle)
  //
  //  For front diff + passive rear, the effective steering angle is:
  //    tan(delta) = (v_R - v_L) / (Vx * track) * wheelbase
  //
  //  At max differential (v_R - v_L = 2*Vwheel_max), assuming Vx ≈ Vwheel_max:
  //    tan(delta_max) ≈ 2 * Vwheel_max / (Vwheel_max * track) * wheelbase
  //                   = 2 * wheelbase / track
  //
  //  Minimum radius: R_min = wheelbase / tan(delta_max)
  //                        = wheelbase / (2 * wheelbase / track)
  //                        = track / 2
  //
  //  This is a SIMPLIFIED estimate. Actual min radius depends on tire slip,
  //  omni-wheel rolling resistance, and speed-dependent effects.
  //  [m] Geometric minimum
  //  Conservative estimate (add 20% margin for real-world effects)
  //  [m]
  //  Alternative calculation based on velocity limits
  //  [m]
  //  Use the larger (more conservative) of the two estimates
  //  Safety margins
  //  [m] Extra clearance (5cm)
  //  Planning parameters
  //  Use conservative forward speed
  //  [m/s] Typical cruising speed
  std::printf("WHEELTEC Chassis Parameters (%s variant):\n", "fourwheel");
  std::fflush(stdout);
  std::printf("  Chassis type:       %s\n", "front_diff_rear_passive");
  std::fflush(stdout);
  std::printf("  Wheelbase:          %.3f m (front-to-rear)\n", 0.36);
  std::fflush(stdout);
  std::printf("  Track width:        %.3f m (front wheels)\n", 0.573);
  std::fflush(stdout);
  std::printf("  Wheel radius:       %.3f m\n", 0.1075);
  std::fflush(stdout);
  std::printf("  Robot radius:       %.3f m (bounding circle)\n",
              0.46097722286464432);
  std::fflush(stdout);
  std::printf("  Inflation radius:   %.3f m (with safety margin)\n",
              0.51097722286464431);
  std::fflush(stdout);
  std::printf("  Max forward speed:  %.2f m/s\n", 0.8);
  std::fflush(stdout);
  std::printf("  Max yaw rate:       %.2f rad/s (%.1f deg/s)\n", 2.5,
              143.23944878270581);
  std::fflush(stdout);
  std::printf("  Yaw (at max speed): %.2f rad/s (%.1f deg/s)\n",
              2.4432809773123911, 139.98968816459904);
  std::fflush(stdout);
  std::printf("  Min turn radius:    %.3f m (passive rear constraint!)\n",
              0.34379999999999994);
  std::fflush(stdout);
  std::printf("  Max acceleration:   %.2f m/s^2\n", 2.0);
  std::fflush(stdout);
  //  Generate motion primitives (returns fixed-size array)
  generateMotionPrimitives(primitives_all);
  //  Filter valid primitives (dt > 0)
  //  For codegen: Keep fixed-size array, track valid count
  //  Use full array
  //  Angular discretization (matches design: 16 bins)
  //  Grid size
  grid_size_x = occupancy_grid->size_x;
  grid_size_y = occupancy_grid->size_y;
  //  Discretize start and goal states
  // WORLDTOGRIDX Convert world x coordinate to grid index
  dy = std::floor((start_state->x - occupancy_grid->origin_x) /
                  occupancy_grid->resolution);
  if (dy < 2.147483648E+9) {
    if (dy >= -2.147483648E+9) {
      next_theta_bin = static_cast<int>(dy);
    } else {
      next_theta_bin = MIN_int32_T;
    }
  } else if (dy >= 2.147483648E+9) {
    next_theta_bin = MAX_int32_T;
  } else {
    next_theta_bin = 0;
  }
  if (next_theta_bin > 2147483646) {
    qY = MAX_int32_T;
  } else {
    qY = next_theta_bin + 1;
  }
  start_state->grid_x = qY;
  // WORLDTOGRADY Convert world y coordinate to grid index
  dy = std::floor((start_state->y - occupancy_grid->origin_y) /
                  occupancy_grid->resolution);
  if (dy < 2.147483648E+9) {
    if (dy >= -2.147483648E+9) {
      next_theta_bin = static_cast<int>(dy);
    } else {
      next_theta_bin = MIN_int32_T;
    }
  } else if (dy >= 2.147483648E+9) {
    next_theta_bin = MAX_int32_T;
  } else {
    next_theta_bin = 0;
  }
  if (next_theta_bin > 2147483646) {
    qY = MAX_int32_T;
  } else {
    qY = next_theta_bin + 1;
  }
  start_state->grid_y = qY;
  //  Normalize theta to [0, 2*pi)
  //  Convert to bin index [1, num_bins]
  dy = std::floor(coder::b_mod(start_state->theta) / 0.39269908169872414);
  if (dy < 2.147483648E+9) {
    if (dy >= -2.147483648E+9) {
      next_theta_bin = static_cast<int>(dy);
    } else {
      next_theta_bin = MIN_int32_T;
    }
  } else if (dy >= 2.147483648E+9) {
    next_theta_bin = MAX_int32_T;
  } else {
    next_theta_bin = 0;
  }
  if (next_theta_bin > 2147483646) {
    qY = MAX_int32_T;
  } else {
    qY = next_theta_bin + 1;
  }
  if (qY >= 16) {
    start_state->theta_bin = 16;
  } else {
    start_state->theta_bin = qY;
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
  theta_start = coder::b_atan2(std::sin(start_state->theta),
                               std::cos(start_state->theta));
  theta_goal_tmp =
      coder::b_atan2(std::sin(goal_state->theta), std::cos(goal_state->theta));
  //  Straight-line distance (Euclidean)
  dx = goal_state->x - start_state->x;
  dy = goal_state->y - start_state->y;
  d_euclidean = std::sqrt(dx * dx + dy * dy);
  //  Handle trivial case (already at goal)
  if (d_euclidean < 0.001) {
    //  Only heading difference matters
    theta_start = theta_goal_tmp - theta_start;
    dy = 0.34379999999999994 *
         std::abs(coder::b_atan2(std::sin(theta_start), std::cos(theta_start)));
    //  Arc length to align heading
  } else {
    //  Heading to goal (straight-line direction)
    dx = coder::b_atan2(dy, dx);
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
    theta_start = dx - theta_start;
    dy = theta_goal_tmp - dx;
    dy = std::fmax(
        0.0, (d_euclidean + 0.17189999999999997 * std::abs(coder::b_atan2(
                                                      std::sin(theta_start),
                                                      std::cos(theta_start)))) +
                 0.17189999999999997 *
                     std::abs(coder::b_atan2(std::sin(dy), std::cos(dy))));
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
  dy = std::floor((start_state->x - occupancy_grid->origin_x) /
                  occupancy_grid->resolution);
  if (dy < 2.147483648E+9) {
    if (dy >= -2.147483648E+9) {
      next_theta_bin = static_cast<int>(dy);
    } else {
      next_theta_bin = MIN_int32_T;
    }
  } else if (dy >= 2.147483648E+9) {
    next_theta_bin = MAX_int32_T;
  } else {
    next_theta_bin = 0;
  }
  if (next_theta_bin > 2147483646) {
    qY = MAX_int32_T;
  } else {
    qY = next_theta_bin + 1;
  }
  // WORLDTOGRADY Convert world y coordinate to grid index
  dy = std::floor((start_state->y - occupancy_grid->origin_y) /
                  occupancy_grid->resolution);
  if (dy < 2.147483648E+9) {
    if (dy >= -2.147483648E+9) {
      next_theta_bin = static_cast<int>(dy);
    } else {
      next_theta_bin = MIN_int32_T;
    }
  } else if (dy >= 2.147483648E+9) {
    next_theta_bin = MAX_int32_T;
  } else {
    next_theta_bin = 0;
  }
  if (next_theta_bin > 2147483646) {
    parent_idx = MAX_int32_T;
  } else {
    parent_idx = next_theta_bin + 1;
  }
  //  Bounds check
  if ((qY < 1) || (qY > occupancy_grid->size_x) || (parent_idx < 1) ||
      (parent_idx > occupancy_grid->size_y)) {
    goal_reached = true;
  } else {
    //  Occupancy check (grid is already inflated, so check center)
    goal_reached =
        (static_cast<double>(
             occupancy_grid->data[(parent_idx + 200 * (qY - 1)) - 1]) > 0.5);
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
      state_list[i].x = 0.0;
      state_list[i].y = 0.0;
      state_list[i].theta = 0.0;
      state_list[i].grid_x = 0;
      state_list[i].grid_y = 0;
      state_list[i].theta_bin = 0;
      state_list[i].Vx = 0.0;
      state_list[i].Wz = 0.0;
      state_list[i].dt = 0.0;
      state_list[i].g = 0.0;
      state_list[i].h = 0.0;
      state_list[i].f = 0.0;
      state_list[i].parent_idx = 0;
      state_list[i].is_valid = true;
      parent_indices[i] = 0;
      heap_indices[i] = 0;
    }
    state_list[0] = *start_state;
    //  Insert start state into heap
    heap_size = 0;
    heap_indices[0] = 1;
    //  Closed set: 3D visited array (grid_x, grid_y, theta_bin)
    visited.set_size(occupancy_grid->size_x, occupancy_grid->size_y, 16);
    next_theta_bin = (occupancy_grid->size_x * occupancy_grid->size_y) << 4;
    for (i = 0; i < next_theta_bin; i++) {
      visited[i] = false;
    }
    //  Parent map: Fixed-size array indexed by state index
    //  Search loop
    coder::tic(savedTime);
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
        if (coder::toc(savedTime) > 5.0) {
          exitg3 = 1;
        } else {
          int current_idx;
          int idx;
          //  Pop state with lowest f-score from heap
          // HEAPPOP Pop minimum f-score state from heap
          //  Binary Heap Operations (Min-heap by f-score)
          //  Return root (minimum)
          current_idx = heap_indices[0] - 1;
          //  Move last element to root and bubble down
          heap_indices[0] = heap_indices[heap_size];
          // HEAPBUBBLEDOWN Bubble down root to maintain heap property
          idx = 1;
          int exitg4;
          do {
            exitg4 = 0;
            if (idx > 1073741823) {
              next_theta_bin = MAX_int32_T;
              qY = MAX_int32_T;
            } else {
              next_theta_bin = idx << 1;
              qY = next_theta_bin + 1;
            }
            i = idx;
            //  Find smallest among parent, left, right
            if ((next_theta_bin <= heap_size) &&
                (state_list[heap_indices[next_theta_bin - 1] - 1].f <
                 state_list[heap_indices[idx - 1] - 1].f)) {
              i = next_theta_bin;
            }
            if ((qY <= heap_size) && (state_list[heap_indices[qY - 1] - 1].f <
                                      state_list[heap_indices[i - 1] - 1].f)) {
              i = qY;
            }
            if (i != idx) {
              //  Swap
              next_theta_bin = heap_indices[idx - 1];
              heap_indices[idx - 1] = heap_indices[i - 1];
              heap_indices[i - 1] = next_theta_bin;
              idx = i;
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
            current_state_x_tmp = state_list[current_idx].x;
            current_state_y_tmp = state_list[current_idx].y;
            current_state_theta = state_list[current_idx].theta;
            current_state_g = state_list[current_idx].g;
            //  Check if already visited
            i = state_list[current_idx].grid_x - 1;
            next_theta_bin = state_list[current_idx].grid_y - 1;
            parent_idx = state_list[current_idx].theta_bin - 1;
            if (!visited[(i + visited.size(0) * next_theta_bin) +
                         visited.size(0) * visited.size(1) * parent_idx]) {
              //  Mark as visited
              visited[(i + visited.size(0) * next_theta_bin) +
                      visited.size(0) * visited.size(1) * parent_idx] = true;
              if (nodes_expanded > 2147483646) {
                nodes_expanded = MAX_int32_T;
              } else {
                nodes_expanded++;
              }
              //  Check goal condition
              dx = current_state_x_tmp - goal_state->x;
              dy = current_state_y_tmp - goal_state->y;
              if ((std::sqrt(dx * dx + dy * dy) <= 0.2) &&
                  (std::abs(coder::angdiff(state_list[current_idx].theta,
                                           goal_state->theta)) <= 0.3)) {
                goal_reached = true;
                final_state_idx = current_idx;
                exitg3 = 1;
              } else {
                boolean_T exitg2;
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
                    double next_x;
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
                      next_x = current_state_x_tmp +
                               dy * std::cos(current_state_theta);
                      d_euclidean = current_state_y_tmp +
                                    dy * std::sin(current_state_theta);
                      theta_start = current_state_theta;
                    } else {
                      //  Circular arc motion
                      dy = primitives_all[i].Wz;
                      dx = primitives_all[i].Vx / dy;
                      //  Turning radius (signed)
                      //  Total heading change [rad]
                      //  Center of circular arc (perpendicular to heading)
                      //  New heading
                      theta_start =
                          current_state_theta + dy * primitives_all[i].dt;
                      //  New position (rotate around center)
                      next_x = (current_state_x_tmp -
                                dx * std::sin(current_state_theta)) +
                               dx * std::sin(theta_start);
                      d_euclidean = (current_state_y_tmp +
                                     dx * std::cos(current_state_theta)) -
                                    dx * std::cos(theta_start);
                    }
                    //  Normalize theta to [-pi, pi]
                    theta_start = coder::b_atan2(std::sin(theta_start),
                                                 std::cos(theta_start));
                    //  Discretize next state
                    // WORLDTOGRIDX Convert world x coordinate to grid index
                    dy = std::floor((next_x - occupancy_grid->origin_x) /
                                    occupancy_grid->resolution);
                    if (dy < 2.147483648E+9) {
                      if (dy >= -2.147483648E+9) {
                        next_theta_bin = static_cast<int>(dy);
                      } else {
                        next_theta_bin = MIN_int32_T;
                      }
                    } else if (dy >= 2.147483648E+9) {
                      next_theta_bin = MAX_int32_T;
                    } else {
                      next_theta_bin = 0;
                    }
                    if (next_theta_bin > 2147483646) {
                      qY = MAX_int32_T;
                    } else {
                      qY = next_theta_bin + 1;
                    }
                    // WORLDTOGRADY Convert world y coordinate to grid index
                    dy = std::floor((d_euclidean - occupancy_grid->origin_y) /
                                    occupancy_grid->resolution);
                    if (dy < 2.147483648E+9) {
                      if (dy >= -2.147483648E+9) {
                        next_theta_bin = static_cast<int>(dy);
                      } else {
                        next_theta_bin = MIN_int32_T;
                      }
                    } else if (dy >= 2.147483648E+9) {
                      next_theta_bin = MAX_int32_T;
                    } else {
                      next_theta_bin = 0;
                    }
                    if (next_theta_bin > 2147483646) {
                      parent_idx = MAX_int32_T;
                    } else {
                      parent_idx = next_theta_bin + 1;
                    }
                    //  Normalize theta to [0, 2*pi)
                    //  Convert to bin index [1, num_bins]
                    dy = std::floor(coder::b_mod(theta_start) /
                                    0.39269908169872414);
                    if (dy < 2.147483648E+9) {
                      if (dy >= -2.147483648E+9) {
                        next_theta_bin = static_cast<int>(dy);
                      } else {
                        next_theta_bin = MIN_int32_T;
                      }
                    } else if (dy >= 2.147483648E+9) {
                      next_theta_bin = MAX_int32_T;
                    } else {
                      next_theta_bin = 0;
                    }
                    if (next_theta_bin > 2147483646) {
                      next_theta_bin = MAX_int32_T;
                    } else {
                      next_theta_bin++;
                    }
                    if (next_theta_bin >= 16) {
                      next_theta_bin = 16;
                    }
                    //  Check bounds
                    if ((qY >= 1) && (qY <= grid_size_x) && (parent_idx >= 1) &&
                        (parent_idx <= grid_size_y) &&
                        (!visited[((qY + visited.size(0) * (parent_idx - 1)) +
                                   visited.size(0) * visited.size(1) *
                                       (next_theta_bin - 1)) -
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
                      boolean_T exitg1;
                      //  Skip if already visited
                      //  Collision check along arc
                      //  Compute cost (arc length)
                      //  Create next state
                      next_state.parent_idx = 0;
                      next_state.is_valid = true;
                      next_state.x = next_x;
                      next_state.y = d_euclidean;
                      next_state.theta = theta_start;
                      next_state.grid_x = qY;
                      next_state.grid_y = parent_idx;
                      next_state.theta_bin = next_theta_bin;
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
                      theta_start = coder::b_atan2(std::sin(theta_start),
                                                   std::cos(theta_start));
                      //  Straight-line distance (Euclidean)
                      dx = goal_state->x - next_x;
                      dy = goal_state->y - d_euclidean;
                      d_euclidean = std::sqrt(dx * dx + dy * dy);
                      //  Handle trivial case (already at goal)
                      if (d_euclidean < 0.001) {
                        //  Only heading difference matters
                        theta_start = theta_goal_tmp - theta_start;
                        dy = 0.34379999999999994 *
                             std::abs(coder::b_atan2(std::sin(theta_start),
                                                     std::cos(theta_start)));
                        //  Arc length to align heading
                      } else {
                        //  Heading to goal (straight-line direction)
                        dx = coder::b_atan2(dy, dx);
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
                        theta_start = dx - theta_start;
                        dy = theta_goal_tmp - dx;
                        dy = std::fmax(
                            0.0,
                            (d_euclidean + 0.17189999999999997 *
                                               std::abs(coder::b_atan2(
                                                   std::sin(theta_start),
                                                   std::cos(theta_start)))) +
                                0.17189999999999997 *
                                    std::abs(coder::b_atan2(std::sin(dy),
                                                            std::cos(dy))));
                      }
                      next_state.h = dy;
                      next_state.f = next_state_tmp + dy;
                      next_state.Vx = next_state_tmp_tmp;
                      next_state.Wz = primitives_all[i].Wz;
                      next_state.dt = b_next_state_tmp_tmp;
                      //  Add to state list
                      state_count++;
                      state_list[state_count - 1] = next_state;
                      //  Store parent
                      parent_indices[state_count - 1] = current_idx + 1;
                      //  Add to heap
                      heap_size++;
                      heap_indices[heap_size] = state_count;
                      idx = heap_size;
                      // HEAPBUBBLEUP Bubble up last element to maintain heap
                      // property
                      exitg1 = false;
                      while ((!exitg1) && (idx + 1 > 1)) {
                        parent_idx = static_cast<int>(std::floor(
                                         static_cast<double>(idx + 1) / 2.0)) -
                                     1;
                        //  Compare f-scores
                        if (state_list[heap_indices[idx] - 1].f <
                            state_list[heap_indices[parent_idx] - 1].f) {
                          //  Swap
                          next_theta_bin = heap_indices[idx];
                          heap_indices[idx] = heap_indices[parent_idx];
                          heap_indices[parent_idx] = next_theta_bin;
                          idx = parent_idx;
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
    dy = coder::toc(savedTime);
    //  Reconstruct path
    if (goal_reached) {
      reconstructPathFixed(final_state_idx + 1, parent_indices, state_list,
                           path);
      //  Count actual path length
      next_theta_bin = 0;
      i = 0;
      while ((i < 500) && ((path[i].dt > 0.0) || (i + 1 == 1))) {
        if (next_theta_bin > 2147483646) {
          next_theta_bin = MAX_int32_T;
        } else {
          next_theta_bin++;
        }
        i++;
      }
      search_stats->planning_time_sec = dy;
      search_stats->path_cost = state_list[final_state_idx].g;
      search_stats->success = true;
      search_stats->iterations = iterations;
      search_stats->nodes_expanded = nodes_expanded;
      search_stats->path_length = next_theta_bin;
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

} // namespace gik9dof

//
// File trailer for gik9dof_planHybridAStarCodegen.cpp
//
// [EOF]
//
