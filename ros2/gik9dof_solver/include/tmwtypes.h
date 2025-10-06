/*
 * File: tmwtypes.h
 * 
 * Minimal type definitions for MATLAB Coder generated code
 * This is a standalone stub that doesn't require MATLAB runtime
 */

#ifndef TMWTYPES_H
#define TMWTYPES_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <float.h>

/*
 * Target hardware information
 */
#ifndef __MW_TARGET_USE_HARDWARE_RESOURCES_H__
#define __MW_TARGET_USE_HARDWARE_RESOURCES_H__

#ifndef MW_MULTI_TASKING_MODE
#define MW_MULTI_TASKING_MODE 1
#endif

#endif

/*
 * Fixed-width integer types
 */
typedef int8_t int8_T;
typedef uint8_t uint8_T;
typedef int16_t int16_T;
typedef uint16_t uint16_T;
typedef int32_t int32_T;
typedef uint32_t uint32_T;
typedef int64_t int64_T;
typedef uint64_t uint64_T;

/*
 * Floating-point types
 */
typedef float real32_T;
typedef double real64_T;
typedef double real_T;
typedef double time_T;

/*
 * Boolean type
 */
#ifndef __cplusplus
typedef bool boolean_T;
#else
typedef bool boolean_T;
#endif

/*
 * Character type
 */
typedef char char_T;
typedef unsigned char uchar_T;
typedef char_T byte_T;

/*
 * Pointer types
 */
#ifdef __cplusplus
#define POINTER_T
#else
typedef void * pointer_T;
#endif

/*
 * Complex types
 */
typedef struct {
  real32_T re;
  real32_T im;
} creal32_T;

typedef struct {
  real64_T re;
  real64_T im;
} creal64_T;

typedef struct {
  real_T re;
  real_T im;
} creal_T;

typedef struct {
  int8_T re;
  int8_T im;
} cint8_T;

typedef struct {
  uint8_T re;
  uint8_T im;
} cuint8_T;

typedef struct {
  int16_T re;
  int16_T im;
} cint16_T;

typedef struct {
  uint16_T re;
  uint16_T im;
} cuint16_T;

typedef struct {
  int32_T re;
  int32_T im;
} cint32_T;

typedef struct {
  uint32_T re;
  uint32_T im;
} cuint32_T;

typedef struct {
  int64_T re;
  int64_T im;
} cint64_T;

typedef struct {
  uint64_T re;
  uint64_T im;
} cuint64_T;

/*
 * Min/Max macros and constants
 */
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/* Min/Max values for integer types */
#define MAX_int8_T ((int8_T)(127))
#define MIN_int8_T ((int8_T)(-128))
#define MAX_uint8_T ((uint8_T)(255U))
#define MAX_int16_T ((int16_T)(32767))
#define MIN_int16_T ((int16_T)(-32768))
#define MAX_uint16_T ((uint16_T)(65535U))
#define MAX_int32_T ((int32_T)(2147483647))
#define MIN_int32_T ((int32_T)(-2147483647-1))
#define MAX_uint32_T ((uint32_T)(0xFFFFFFFFU))
#define MAX_int64_T ((int64_T)(9223372036854775807LL))
#define MIN_int64_T ((int64_T)(-9223372036854775807LL-1LL))
#define MAX_uint64_T ((uint64_T)(0xFFFFFFFFFFFFFFFFULL))

/*
 * Mathematical constants
 * Note: These are NOT defined as macros to avoid conflicts with generated code
 * that may define them as static const variables
 */

/*
 * UNUSED_PARAMETER macro
 */
#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)
#else
#define UNUSED_PARAMETER(x) (void) (x)
#endif
#endif

#endif /* TMWTYPES_H */
