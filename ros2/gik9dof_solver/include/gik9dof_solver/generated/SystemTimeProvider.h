//
// SystemTimeProvider.h
//
// Code generation for function 'SystemTimeProvider'
//

#ifndef SYSTEMTIMEPROVIDER_H
#define SYSTEMTIMEPROVIDER_H

// Include files
#include "rtwtypes.h"
#include "coder_posix_time.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace robotics {
namespace core {
namespace internal {
class SystemTimeProvider {
public:
  SystemTimeProvider();
  ~SystemTimeProvider();
  coderTimespec StartTime;
};

} // namespace internal
} // namespace core
} // namespace robotics
} // namespace coder

#endif
// End of code generation (SystemTimeProvider.h)
