// Stub implementations for MATLAB Coder POSIX time functions
// These provide minimal implementations for ARM64 builds

#include "../matlab_codegen/include/coder_posix_time.h"
#include <time.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

int coderInitTimeFunctions(double* const aFrequency) {
    // On POSIX systems (Linux/ARM64), we use clock_gettime which doesn't need frequency
    // Return frequency as 1.0 (representing nanosecond precision)
    if (aFrequency != nullptr) {
        *aFrequency = 1.0;
    }
    return 0;  // Success
}

int coderTimeClockGettimeMonotonic(coderTimespec* const aCoderTimespec, double aFrequency) {
    (void)aFrequency;  // Unused on POSIX systems
    
    if (aCoderTimespec == nullptr) {
        return EINVAL;
    }
    
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return errno;
    }
    
    // Convert to coderTimespec format
    aCoderTimespec->tv_sec = static_cast<double>(ts.tv_sec);
    aCoderTimespec->tv_nsec = static_cast<double>(ts.tv_nsec);
    
    return 0;  // Success
}

int coderTimeSleep(coderTimespec const * const aCoderTimespec) {
    if (aCoderTimespec == nullptr) {
        return EINVAL;
    }
    
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(aCoderTimespec->tv_sec);
    ts.tv_nsec = static_cast<long>(aCoderTimespec->tv_nsec);
    
    if (nanosleep(&ts, nullptr) != 0) {
        return errno;
    }
    
    return 0;  // Success
}

#ifdef __cplusplus
}
#endif
