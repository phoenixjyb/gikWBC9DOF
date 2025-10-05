//
// main.cpp
//
// Code generation for function 'main'
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

// Include files
#include "main.h"
#include "gik9dof_codegen_followTrajectory.h"
#include "gik9dof_codegen_followTrajectory_terminate.h"
#include "rt_nonfinite.h"

// Function Declarations
static void argInit_4x4xd50_real_T(double result_data[], int result_size[3]);

static void argInit_9x1_real_T(double result[9]);

static double argInit_real_T();

// Function Definitions
static void argInit_4x4xd50_real_T(double result_data[], int result_size[3])
{
  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 4;
  result_size[1] = 4;
  result_size[2] = 2;
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 4; idx0++) {
    for (int idx1{0}; idx1 < 4; idx1++) {
      for (int idx2{0}; idx2 < 2; idx2++) {
        // Set the value of the array element.
        // Change this value to the value that the application requires.
        result_data[(idx0 + 4 * idx1) + 16 * idx2] = argInit_real_T();
      }
    }
  }
}

static void argInit_9x1_real_T(double result[9])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 9; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_gik9dof_codegen_followTrajectory();
  // Terminate the application.
  // You do not need to do this more than one time.
  gik9dof_codegen_followTrajectory_terminate();
  return 0;
}

void main_gik9dof_codegen_followTrajectory()
{
  double poses_data[800];
  double dv[9];
  double qOut[9];
  double distanceLower_tmp;
  int poses_size[3];
  // Initialize function 'followTrajectory' input arguments.
  // Initialize function input argument 'q0'.
  // Initialize function input argument 'poses'.
  argInit_4x4xd50_real_T(poses_data, poses_size);
  distanceLower_tmp = argInit_real_T();
  // Call the entry-point 'followTrajectory'.
  argInit_9x1_real_T(dv);
  gik9dof::codegen::followTrajectory(
      dv, poses_data, poses_size, distanceLower_tmp, distanceLower_tmp, qOut);
}

// End of code generation (main.cpp)
