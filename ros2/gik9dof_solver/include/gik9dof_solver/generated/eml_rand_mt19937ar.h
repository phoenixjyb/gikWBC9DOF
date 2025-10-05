//
// eml_rand_mt19937ar.h
//
// Code generation for function 'eml_rand_mt19937ar'
//

#ifndef EML_RAND_MT19937AR_H
#define EML_RAND_MT19937AR_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace randfun {
double b_eml_rand_mt19937ar(unsigned int b_state[625]);

void eml_rand_mt19937ar(unsigned int b_state[625]);

void genrand_uint32_vector(unsigned int mt[625], unsigned int u[2]);

} // namespace randfun
} // namespace internal
} // namespace coder

#endif
// End of code generation (eml_rand_mt19937ar.h)
