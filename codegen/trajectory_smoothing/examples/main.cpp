//
// File: main.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 09-Oct-2025 23:31:47
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
#include "rt_nonfinite.h"
#include "smoothTrajectoryVelocity.h"
#include "smoothTrajectoryVelocity_types.h"

// Function Declarations
static void argInit_1x10_char_T(char result[10]);

static void argInit_5x1_real_T(double result[5]);

static char argInit_char_T();

static double argInit_real_T();

static struct0_T argInit_struct0_T();

// Function Definitions
//
// Arguments    : char result[10]
// Return Type  : void
//
static void argInit_1x10_char_T(char result[10])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 10; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_char_T();
  }
}

//
// Arguments    : double result[5]
// Return Type  : void
//
static void argInit_5x1_real_T(double result[5])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 5; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : char
//
static char argInit_char_T()
{
  return '?';
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
// Arguments    : void
// Return Type  : struct0_T
//
static struct0_T argInit_struct0_T()
{
  struct0_T result;
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.ax_max = result_tmp;
  result.jx_max = result_tmp;
  result.wz_max = result_tmp;
  result.alpha_max = result_tmp;
  result.jerk_wz_max = result_tmp;
  result.vx_max = result_tmp;
  argInit_1x10_char_T(result.smoothing_method);
  return result;
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_smoothTrajectoryVelocity();
  // Terminate the application.
  // You do not need to do this more than one time.
  smoothTrajectoryVelocity_terminate();
  return 0;
}

//
// Arguments    : void
// Return Type  : void
//
void main_smoothTrajectoryVelocity()
{
  struct0_T r;
  double waypoints_x_tmp[5];
  double alpha_cmd;
  double ax_cmd;
  double jerk_vx_cmd;
  double jerk_wz_cmd;
  double vx_cmd;
  double wz_cmd;
  // Initialize function 'smoothTrajectoryVelocity' input arguments.
  // Initialize function input argument 'waypoints_x'.
  argInit_5x1_real_T(waypoints_x_tmp);
  // Initialize function input argument 'waypoints_y'.
  // Initialize function input argument 'waypoints_theta'.
  // Initialize function input argument 't_waypoints'.
  // Initialize function input argument 'params'.
  // Call the entry-point 'smoothTrajectoryVelocity'.
  r = argInit_struct0_T();
  smoothTrajectoryVelocity(waypoints_x_tmp, waypoints_x_tmp, waypoints_x_tmp,
                           waypoints_x_tmp, argInit_real_T(), &r, &vx_cmd,
                           &wz_cmd, &ax_cmd, &alpha_cmd, &jerk_vx_cmd,
                           &jerk_wz_cmd);
}

//
// File trailer for main.cpp
//
// [EOF]
//
