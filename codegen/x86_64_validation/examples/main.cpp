//
// File: main.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:17:44
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
static void argInit_4x4_real_T(double result[16]);

static void argInit_9x1_real_T(double result[9]);

static double argInit_real_T();

// Function Definitions
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
  double dv1[16];
  double dv[9];
  double qNext[9];
  double distanceLower_tmp;
  // Initialize function 'gik9dof_codegen_inuse_solveGIKStepWrapper' input
  // arguments. Initialize function input argument 'qCurrent'. Initialize
  // function input argument 'targetPose'.
  distanceLower_tmp = argInit_real_T();
  // Call the entry-point 'gik9dof_codegen_inuse_solveGIKStepWrapper'.
  argInit_9x1_real_T(dv);
  argInit_4x4_real_T(dv1);
  instancePtr->gik9dof_codegen_inuse_solveGIKStepWrapper(
      dv, dv1, distanceLower_tmp, distanceLower_tmp, qNext, &solverInfo);
}

//
// File trailer for main.cpp
//
// [EOF]
//
