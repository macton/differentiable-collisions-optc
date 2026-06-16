#ifndef CPINT_FLOAT
#define CPINT_FLOAT

#include <stdint.h>

typedef float    f32;
typedef double   f64;
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;
typedef int64_t  s64;

// less typing for casting, Timothy Lottes way
#define f32_(x) ((f32)(x))
#define f64_(x) ((f64)(x))
#define  u8_(x)  ((u8)(x))
#define u16_(x) ((u16)(x))
#define u32_(x) ((u32)(x))
#define u64_(x) ((u64)(x))
#define  s8_(x)  ((s8)(x))
#define s16_(x) ((s16)(x))
#define s32_(x) ((s32)(x))
#define s64_(x) ((s64)(x))

#endif