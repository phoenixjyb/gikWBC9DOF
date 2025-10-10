//
// File: main.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 11-Oct-2025 00:19:03
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
#include "ChassisPathFollower.h"
#include "chassisPathFollowerCodegen_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"

// Function Declarations
static void argInit_1x3_real_T(double result[3]);

static bool argInit_boolean_T();

static int argInit_d500x1_real_T(double result_data[]);

static void argInit_d500x3_real_T(double result_data[], int result_size[2]);

static double argInit_real_T();

static void argInit_struct0_T(gik9dof::struct0_T &result);

static void argInit_struct1_T(gik9dof::struct1_T &result);

static void argInit_struct2_T(gik9dof::struct2_T &result);

// Function Definitions
//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_1x3_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 3; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
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
// Arguments    : double result_data[]
// Return Type  : int
//
static int argInit_d500x1_real_T(double result_data[])
{
  int result_size;
  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size = 2;
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 2; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result_data[idx0] = argInit_real_T();
  }
  return result_size;
}

//
// Arguments    : double result_data[]
//                int result_size[2]
// Return Type  : void
//
static void argInit_d500x3_real_T(double result_data[], int result_size[2])
{
  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 2;
  result_size[1] = 3;
  // Loop over the array to initialize each element.
  for (int i{0}; i < 6; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result_data[i] = argInit_real_T();
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
// Arguments    : gik9dof::struct0_T &result
// Return Type  : void
//
static void argInit_struct0_T(gik9dof::struct0_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.CurrentIndex = result_tmp;
  result.LastVelocity = result_tmp;
  result.LastAcceleration = result_tmp;
  result.LastHeadingError = result_tmp;
  result.IntegralHeadingError = result_tmp;
  result.DistanceTraveled = result_tmp;
  result.PathNumPoints = result_tmp;
  argInit_1x3_real_T(result.PreviousPose);
}

//
// Arguments    : gik9dof::struct1_T &result
// Return Type  : void
//
static void argInit_struct1_T(gik9dof::struct1_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.LookaheadBase = result_tmp;
  result.LookaheadVelGain = result_tmp;
  result.LookaheadAccelGain = result_tmp;
  result.GoalTolerance = result_tmp;
  result.HeadingKp = result_tmp;
  result.HeadingKi = result_tmp;
  result.HeadingKd = result_tmp;
  result.FeedforwardGain = result_tmp;
  result.KappaThreshold = result_tmp;
  result.VxReduction = result_tmp;
  result.ControllerMode = result_tmp;
  result.ReverseEnabled = argInit_boolean_T();
  argInit_struct2_T(result.Chassis);
  argInit_d500x3_real_T(result.PathInfo_States.data,
                        result.PathInfo_States.size);
  result.PathInfo_Curvature.size[0] =
      argInit_d500x1_real_T(result.PathInfo_Curvature.data);
  result.PathInfo_ArcLength.size[0] =
      argInit_d500x1_real_T(result.PathInfo_ArcLength.data);
  result.PathInfo_DistanceRemaining.size[0] =
      argInit_d500x1_real_T(result.PathInfo_DistanceRemaining.data);
}

//
// Arguments    : gik9dof::struct2_T &result
// Return Type  : void
//
static void argInit_struct2_T(gik9dof::struct2_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.wheel_speed_max = result_tmp;
  result.vx_max = result_tmp;
  result.vx_min = result_tmp;
  result.wz_max = result_tmp;
  result.accel_limit = result_tmp;
  result.decel_limit = result_tmp;
  result.jerk_limit = result_tmp;
  result.wheel_base = result_tmp;
  result.track = result_tmp;
  result.reverse_enabled = argInit_boolean_T();
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  gik9dof::ChassisPathFollower *classInstance;
  classInstance = new gik9dof::ChassisPathFollower;
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_chassisPathFollowerCodegen(classInstance);
  delete classInstance;
  return 0;
}

//
// Arguments    : gik9dof::ChassisPathFollower *instancePtr
// Return Type  : void
//
void main_chassisPathFollowerCodegen(gik9dof::ChassisPathFollower *instancePtr)
{
  gik9dof::struct0_T state;
  gik9dof::struct1_T r;
  gik9dof::struct3_T status;
  double dv[3];
  double vx;
  double wz;
  // Initialize function 'chassisPathFollowerCodegen' input arguments.
  // Initialize function input argument 'pose'.
  // Initialize function input argument 'state'.
  // Initialize function input argument 'params'.
  // Call the entry-point 'chassisPathFollowerCodegen'.
  argInit_struct0_T(state);
  argInit_1x3_real_T(dv);
  argInit_struct1_T(r);
  instancePtr->chassisPathFollowerCodegen(dv, argInit_real_T(), &state, &r, &vx,
                                          &wz, &status);
}

//
// File trailer for main.cpp
//
// [EOF]
//
