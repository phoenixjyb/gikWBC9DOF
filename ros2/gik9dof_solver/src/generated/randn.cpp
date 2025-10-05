//
// randn.cpp
//
// Code generation for function 'randn'
//

// Include files
#include "randn.h"
#include "eml_rand_mt19937ar.h"
#include "gik9dof_codegen_followTrajectory_data.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
void b_randn(double r[3])
{
  r[0] = internal::randfun::b_eml_rand_mt19937ar(state);
  r[1] = internal::randfun::b_eml_rand_mt19937ar(state);
  r[2] = internal::randfun::b_eml_rand_mt19937ar(state);
}

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

void randn(double r[4])
{
  r[0] = internal::randfun::b_eml_rand_mt19937ar(state);
  r[1] = internal::randfun::b_eml_rand_mt19937ar(state);
  r[2] = internal::randfun::b_eml_rand_mt19937ar(state);
  r[3] = internal::randfun::b_eml_rand_mt19937ar(state);
}

} // namespace coder

// End of code generation (randn.cpp)
