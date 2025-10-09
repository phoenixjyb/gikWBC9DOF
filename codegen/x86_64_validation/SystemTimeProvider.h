//
// File: SystemTimeProvider.h
//
// MATLAB Coder version            : 24.2
// C/C++ source code generated on  : 08-Oct-2025 18:33:39
//

#ifndef SYSTEMTIMEPROVIDER_H
#define SYSTEMTIMEPROVIDER_H

// Include Files
#include "rtwtypes.h"
#include "coder_posix_time.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace gik9dof {
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
} // namespace gik9dof

#endif
//
// File trailer for SystemTimeProvider.h
//
// [EOF]
//
