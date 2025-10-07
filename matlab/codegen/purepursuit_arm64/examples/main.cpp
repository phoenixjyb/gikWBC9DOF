//
// File: main.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 12:15:24
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
#include "purePursuitVelocityController.h"
#include "purePursuitVelocityController_terminate.h"
#include "purePursuitVelocityController_types.h"
#include "rt_nonfinite.h"

// Function Declarations
static void argInit_1x30_real_T(double result[30]);

static double argInit_real_T();

static void argInit_struct0_T(gik9dof_purepursuit::struct0_T &result);

static void argInit_struct1_T(gik9dof_purepursuit::struct1_T &result);

// Function Definitions
//
// Arguments    : double result[30]
// Return Type  : void
//
static void argInit_1x30_real_T(double result[30])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 30; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
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
// Arguments    : gik9dof_purepursuit::struct0_T &result
// Return Type  : void
//
static void argInit_struct0_T(gik9dof_purepursuit::struct0_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.lookaheadBase = result_tmp;
  result.lookaheadVelGain = result_tmp;
  result.lookaheadTimeGain = result_tmp;
  result.vxNominal = result_tmp;
  result.vxMax = result_tmp;
  result.wzMax = result_tmp;
  result.track = result_tmp;
  result.vwheelMax = result_tmp;
  result.waypointSpacing = result_tmp;
  result.pathBufferSize = result_tmp;
  result.goalTolerance = result_tmp;
  result.interpSpacing = result_tmp;
}

//
// Arguments    : gik9dof_purepursuit::struct1_T &result
// Return Type  : void
//
static void argInit_struct1_T(gik9dof_purepursuit::struct1_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_1x30_real_T(result.pathX);
  result_tmp = argInit_real_T();
  result.numWaypoints = result_tmp;
  result.prevVx = result_tmp;
  result.prevWz = result_tmp;
  result.prevPoseX = result_tmp;
  result.prevPoseY = result_tmp;
  result.prevPoseYaw = result_tmp;
  result.lastRefTime = result_tmp;
  for (int i{0}; i < 30; i++) {
    result.pathY[i] = result.pathX[i];
    result.pathTheta[i] = result.pathX[i];
    result.pathTime[i] = result.pathX[i];
  }
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
  main_purePursuitVelocityController();
  // Terminate the application.
  // You do not need to do this more than one time.
  gik9dof_purepursuit::purePursuitVelocityController_terminate();
  return 0;
}

//
// Arguments    : void
// Return Type  : void
//
void main_purePursuitVelocityController()
{
  gik9dof_purepursuit::struct0_T r;
  gik9dof_purepursuit::struct1_T r1;
  gik9dof_purepursuit::struct1_T stateOut;
  double vx;
  double wz;
  // Initialize function 'purePursuitVelocityController' input arguments.
  vx = argInit_real_T();
  // Initialize function input argument 'params'.
  // Initialize function input argument 'stateIn'.
  // Call the entry-point 'purePursuitVelocityController'.
  argInit_struct0_T(r);
  argInit_struct1_T(r1);
  gik9dof_purepursuit::purePursuitVelocityController(
      vx, vx, vx, vx, vx, vx, vx, &r, &r1, &vx, &wz, &stateOut);
}

//
// File trailer for main.cpp
//
// [EOF]
//
