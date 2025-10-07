//
// File: main.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 11:20:37
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
#include "holisticVelocityController.h"
#include "holisticVelocityController_terminate.h"
#include "holisticVelocityController_types.h"

// Function Declarations
static double argInit_real_T();

static gik9dof_velocity::struct0_T argInit_struct0_T();

static gik9dof_velocity::struct1_T argInit_struct1_T();

static gik9dof_velocity::struct2_T argInit_struct2_T();

// Function Definitions
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
// Return Type  : gik9dof_velocity::struct0_T
//
static gik9dof_velocity::struct0_T argInit_struct0_T()
{
  gik9dof_velocity::struct0_T result;
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.track = result_tmp;
  result.Vwheel_max = result_tmp;
  result.Vx_max = result_tmp;
  result.W_max = result_tmp;
  result.yawKp = result_tmp;
  result.yawKff = result_tmp;
  return result;
}

//
// Arguments    : void
// Return Type  : gik9dof_velocity::struct1_T
//
static gik9dof_velocity::struct1_T argInit_struct1_T()
{
  gik9dof_velocity::struct1_T result;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result.prev = argInit_struct2_T();
  return result;
}

//
// Arguments    : void
// Return Type  : gik9dof_velocity::struct2_T
//
static gik9dof_velocity::struct2_T argInit_struct2_T()
{
  gik9dof_velocity::struct2_T result;
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.x = result_tmp;
  result.y = result_tmp;
  result.theta = result_tmp;
  result.t = result_tmp;
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
  main_holisticVelocityController();
  // Terminate the application.
  // You do not need to do this more than one time.
  gik9dof_velocity::holisticVelocityController_terminate();
  return 0;
}

//
// Arguments    : void
// Return Type  : void
//
void main_holisticVelocityController()
{
  gik9dof_velocity::struct0_T r;
  gik9dof_velocity::struct1_T r1;
  gik9dof_velocity::struct1_T stateOut;
  double Vx;
  double Wz;
  // Initialize function 'holisticVelocityController' input arguments.
  Vx = argInit_real_T();
  // Initialize function input argument 'params'.
  // Initialize function input argument 'stateIn'.
  // Call the entry-point 'holisticVelocityController'.
  r = argInit_struct0_T();
  r1 = argInit_struct1_T();
  gik9dof_velocity::holisticVelocityController(Vx, Vx, Vx, Vx, Vx, Vx, Vx, &r,
                                               &r1, &Vx, &Wz, &stateOut);
}

//
// File trailer for main.cpp
//
// [EOF]
//
