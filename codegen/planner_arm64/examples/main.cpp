//
// File: main.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 13:46:29
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "HybridAStarPlanner.h"
#include "OccupancyGrid2D.h"
#include "planHybridAStarCodegen_types.h"
#include "rt_nonfinite.h"

// Function Declarations
static void argInit_200x200_boolean_T(bool result[40000]);

static void argInit_OccupancyGrid2D(gik9dof::OccupancyGrid2D &result);

static bool argInit_boolean_T();

static int argInit_int32_T();

static double argInit_real_T();

static void argInit_struct0_T(gik9dof::struct0_T &result);

// Function Definitions
//
// Arguments    : bool result[40000]
// Return Type  : void
//
static void argInit_200x200_boolean_T(bool result[40000])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 40000; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_boolean_T();
  }
}

//
// Arguments    : gik9dof::OccupancyGrid2D &result
// Return Type  : void
//
static void argInit_OccupancyGrid2D(gik9dof::OccupancyGrid2D &result)
{
  double result_tmp;
  int b_result_tmp;
  bool bv[40000];
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  b_result_tmp = argInit_int32_T();
  argInit_200x200_boolean_T(bv);
  result.init(bv, result_tmp, result_tmp, result_tmp, b_result_tmp,
              b_result_tmp);
}

//
// Arguments    : void
// Return Type  : bool
//
static bool argInit_boolean_T()
{
  return false;
}

//
// Arguments    : void
// Return Type  : int
//
static int argInit_int32_T()
{
  return 0;
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : gik9dof::struct0_T &result
// Return Type  : void
//
static void argInit_struct0_T(gik9dof::struct0_T &result)
{
  double result_tmp;
  int b_result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.y = result_tmp;
  result.theta = result_tmp;
  b_result_tmp = argInit_int32_T();
  result.grid_y = b_result_tmp;
  result.theta_bin = b_result_tmp;
  result.Vx = result_tmp;
  result.Wz = result_tmp;
  result.dt = result_tmp;
  result.g = result_tmp;
  result.h = result_tmp;
  result.f = result_tmp;
  result.parent_idx = b_result_tmp;
  result.x = result_tmp;
  result.grid_x = b_result_tmp;
  result.is_valid = argInit_boolean_T();
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  gik9dof::HybridAStarPlanner *classInstance;
  classInstance = new gik9dof::HybridAStarPlanner;
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_planHybridAStarCodegen(classInstance);
  delete classInstance;
  return 0;
}

//
// Arguments    : gik9dof::HybridAStarPlanner *instancePtr
// Return Type  : void
//
void main_planHybridAStarCodegen(gik9dof::HybridAStarPlanner *instancePtr)
{
  gik9dof::OccupancyGrid2D r;
  gik9dof::struct0_T b_start_state_tmp;
  gik9dof::struct0_T start_state_tmp;
  gik9dof::struct1_T path[500];
  gik9dof::struct2_T search_stats;
  // Initialize function 'planHybridAStarCodegen' input arguments.
  // Initialize function input argument 'start_state'.
  argInit_struct0_T(start_state_tmp);
  // Initialize function input argument 'goal_state'.
  // Initialize function input argument 'occupancy_grid'.
  // Initialize function input argument 'options'.
  // Call the entry-point 'planHybridAStarCodegen'.
  argInit_OccupancyGrid2D(r);
  b_start_state_tmp = start_state_tmp;
  instancePtr->b_planHybridAStarCodegen(&b_start_state_tmp, &start_state_tmp,
                                        &r, path, &search_stats);
}

//
// File trailer for main.cpp
//
// [EOF]
//
