//
// File: eml_rand_mt19937ar.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 07-Oct-2025 08:09:07
//

#ifndef EML_RAND_MT19937AR_H
#define EML_RAND_MT19937AR_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace gik9dof {
namespace coder {
namespace internal {
namespace randfun {
double b_eml_rand_mt19937ar(unsigned int state[625]);

void eml_rand_mt19937ar(unsigned int state[625]);

void genrand_uint32_vector(unsigned int mt[625], unsigned int u[2]);

} // namespace randfun
} // namespace internal
} // namespace coder
} // namespace gik9dof

#endif
//
// File trailer for eml_rand_mt19937ar.h
//
// [EOF]
//
