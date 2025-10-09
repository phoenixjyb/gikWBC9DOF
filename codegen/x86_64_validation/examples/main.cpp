//
// File: main.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 18:33:39
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
#include "GIKSolver.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Declarations
static void argInit_20x1_int32_T(int result[20]);

static void argInit_20x1_real_T(double result[20]);

static void argInit_4x4_real_T(double result[16]);

static void argInit_9x1_real_T(double result[9]);

static int argInit_int32_T();

static double argInit_real_T();

// Function Definitions
//
// Arguments    : int result[20]
// Return Type  : void
//
static void argInit_20x1_int32_T(int result[20])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 20; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_int32_T();
  }
}

//
// Arguments    : double result[20]
// Return Type  : void
//
static void argInit_20x1_real_T(double result[20])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 20; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[16]
// Return Type  : void
//
static void argInit_4x4_real_T(double result[16])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 16; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

//
// Arguments    : double result[9]
// Return Type  : void
//
static void argInit_9x1_real_T(double result[9])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 9; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
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
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  gik9dof::GIKSolver *classInstance;
  classInstance = new gik9dof::GIKSolver;
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_gik9dof_codegen_inuse_solveGIKStepWrapper(classInstance);
  delete classInstance;
  return 0;
}

//
// Arguments    : gik9dof::GIKSolver *instancePtr
// Return Type  : void
//
void main_gik9dof_codegen_inuse_solveGIKStepWrapper(
    gik9dof::GIKSolver *instancePtr)
{
  gik9dof::struct0_T solverInfo;
  double distBoundsLower_tmp[20];
  double dv1[16];
  double dv[9];
  double qNext[9];
  int distBodyIndices_tmp[20];
  // Initialize function 'gik9dof_codegen_inuse_solveGIKStepWrapper' input
  // arguments. Initialize function input argument 'qCurrent'. Initialize
  // function input argument 'targetPose'. Initialize function input argument
  // 'distBodyIndices'.
  argInit_20x1_int32_T(distBodyIndices_tmp);
  // Initialize function input argument 'distRefBodyIndices'.
  // Initialize function input argument 'distBoundsLower'.
  argInit_20x1_real_T(distBoundsLower_tmp);
  // Initialize function input argument 'distBoundsUpper'.
  // Initialize function input argument 'distWeights'.
  // Call the entry-point 'gik9dof_codegen_inuse_solveGIKStepWrapper'.
  argInit_9x1_real_T(dv);
  argInit_4x4_real_T(dv1);
  instancePtr->gik9dof_codegen_inuse_solveGIKStepWrapper(
      dv, dv1, distBodyIndices_tmp, distBodyIndices_tmp, distBoundsLower_tmp,
      distBoundsLower_tmp, distBoundsLower_tmp, qNext, &solverInfo);
}

//
// File trailer for main.cpp
//
// [EOF]
//
