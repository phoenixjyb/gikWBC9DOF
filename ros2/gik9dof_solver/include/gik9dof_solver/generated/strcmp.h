//
// strcmp.h
//
// Code generation for function 'strcmp'
//

#ifndef STRCMP_H
#define STRCMP_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
bool b_strcmp(const char a_data[], const int a_size[2],
              const array<char, 2U> &b);

bool b_strcmp(const char a_data[], const int a_size[2], const char b_data[],
              const int b_size[2]);

} // namespace internal
} // namespace coder

#endif
// End of code generation (strcmp.h)
