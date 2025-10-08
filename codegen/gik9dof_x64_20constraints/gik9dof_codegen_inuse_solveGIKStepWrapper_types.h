//
// File: gik9dof_codegen_inuse_solveGIKStepWrapper_types.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

#ifndef GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_TYPES_H
#define GIK9DOF_CODEGEN_INUSE_SOLVEGIKSTEPWRAPPER_TYPES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#define MAX_THREADS omp_get_max_threads()

// Type Definitions
struct struct1_T {
  coder::bounded_array<char, 8U, 2U> Type;
  coder::array<double, 2U> Violation;
};

struct struct0_T {
  double Iterations;
  double NumRandomRestarts;
  struct1_T ConstraintViolations[22];
  double ExitFlag;
  coder::bounded_array<char, 14U, 2U> Status;
};

#endif
//
// File trailer for gik9dof_codegen_inuse_solveGIKStepWrapper_types.h
//
// [EOF]
//
