//
// File: randn.cpp
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 13:54:35
//

// Include Files
#include "randn.h"
#include "eml_rand_mt19937ar.h"
#include "gik9dof_codegen_inuse_solveGIKStepWrapper_data.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : double r[3]
// Return Type  : void
//
namespace coder {
void b_randn(double r[3])
{
  r[0] = internal::randfun::b_eml_rand_mt19937ar(state);
  r[1] = internal::randfun::b_eml_rand_mt19937ar(state);
  r[2] = internal::randfun::b_eml_rand_mt19937ar(state);
}

//
// Arguments    : const double varargin_1[2]
//                double r_data[]
// Return Type  : int
//
int randn(const double varargin_1[2], double r_data[])
{
  int i;
  int r_size;
  i = static_cast<int>(varargin_1[0]);
  r_size = static_cast<int>(varargin_1[0]);
  for (int k{0}; k < i; k++) {
    r_data[k] = internal::randfun::b_eml_rand_mt19937ar(state);
  }
  return r_size;
}

//
// Arguments    : double r[4]
// Return Type  : void
//
void randn(double r[4])
{
  r[0] = internal::randfun::b_eml_rand_mt19937ar(state);
  r[1] = internal::randfun::b_eml_rand_mt19937ar(state);
  r[2] = internal::randfun::b_eml_rand_mt19937ar(state);
  r[3] = internal::randfun::b_eml_rand_mt19937ar(state);
}

} // namespace coder

//
// File trailer for randn.cpp
//
// [EOF]
//
