// emmintrin.h stub for ARM64 compatibility
// This file provides empty stubs for x86 SSE2 intrinsics on ARM platforms
// The MATLAB generated code will compile but SSE optimizations will be disabled

#ifndef EMMINTRIN_H_ARM64_STUB
#define EMMINTRIN_H_ARM64_STUB

#ifdef __aarch64__

// Define empty SSE2 types as basic types
typedef double __m128d __attribute__((__vector_size__(16), __may_alias__));
typedef long long __m128i __attribute__((__vector_size__(16), __may_alias__));
typedef float __m128 __attribute__((__vector_size__(16), __may_alias__));

// Stub out SSE2 intrinsics - they won't be called on ARM if MATLAB respects arch checks
// Double-precision floating point intrinsics
inline __m128d _mm_setzero_pd() { return __m128d{0.0, 0.0}; }
inline __m128i _mm_setzero_si128() { __m128i r; return r; }
inline __m128d _mm_set1_pd(double a) { return __m128d{a, a}; }
inline __m128d _mm_load_pd(double const* p) { return *(__m128d*)p; }
inline __m128d _mm_loadu_pd(double const* p) { return *(__m128d*)p; }
inline void _mm_store_pd(double* p, __m128d a) { *(__m128d*)p = a; }
inline void _mm_storeu_pd(double* p, __m128d a) { *(__m128d*)p = a; }
inline __m128d _mm_add_pd(__m128d a, __m128d b) { return a + b; }
inline __m128d _mm_sub_pd(__m128d a, __m128d b) { return a - b; }
inline __m128d _mm_mul_pd(__m128d a, __m128d b) { return a * b; }
inline __m128d _mm_div_pd(__m128d a, __m128d b) { return a / b; }
inline __m128d _mm_sqrt_pd(__m128d a) { 
    return __m128d{__builtin_sqrt(a[0]), __builtin_sqrt(a[1])}; 
}

// Integer intrinsics (epi32 = 32-bit integer operations)
inline __m128i _mm_set1_epi32(int a) { 
    __m128i r; 
    int* p = (int*)&r;
    p[0] = p[1] = p[2] = p[3] = a;
    return r; 
}
inline __m128i _mm_loadu_si128(__m128i const* p) { return *p; }
inline void _mm_storeu_si128(__m128i* p, __m128i a) { *p = a; }
inline __m128i _mm_add_epi32(__m128i a, __m128i b) { 
    __m128i r;
    int* pr = (int*)&r;
    int* pa = (int*)&a;
    int* pb = (int*)&b;
    pr[0] = pa[0] + pb[0];
    pr[1] = pa[1] + pb[1];
    pr[2] = pa[2] + pb[2];
    pr[3] = pa[3] + pb[3];
    return r;
}

#else
// On x86, include the real header
#include_next <emmintrin.h>
#endif

#endif // EMMINTRIN_H_ARM64_STUB
