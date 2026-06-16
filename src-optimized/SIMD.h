

#ifndef SIMD_H
#define SIMD_H

// this is a helper file for working with ARM neon and SSE-SSE4.2 intrinsics, 
// But everytime writing both versions can be pain, so I've used macros for this purpose,.
// another reason why I've used macros is if I use operator overriding or functions for abstracting intrinsics,
// in debug mode, code compiles down to call instruction which is slower than instruction itself
// and macro's allows many more optimizations that compiler can do.
// a bit more explanation here: https://medium.com/@anilcangulkaya7/what-is-simd-and-how-to-use-it-3d1125faac89

#include "IntFloat.h"

#if defined(_MSC_VER)       /* MSVC */
#  define AX_ALIGN(N) __declspec(align(N))
#elif defined(__GNUC__)     /* GCC, Clang */
#  define AX_ALIGN(N) __attribute__((aligned(N)))
#elif defined(__INTEL_COMPILER) /* Intel C Compiler */
#  define AX_ALIGN(N) __attribute__((aligned(N)))
#else                       /* Unknown compiler, no alignment */
#  define AX_ALIGN(N)
#endif

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(__x86_64__) || defined(_M_X64)
    #define AX_X64
#elif defined(__i386) || defined(_M_IX86)
    #define AX_X86
#elif defined(_M_ARM) || defined(_M_ARM64) || defined(_M_HYBRID_X86_ARM64) || defined(_M_ARM64EC) || __arm__ || __aarch64__
    #define AX_ARM
#endif

#if defined( _M_ARM64 ) || defined( __aarch64__ ) || defined( __arm64__ ) || defined(__ARM_NEON__)
    #define AX_SUPPORT_NEON
    #if defined(__ARM_FEATURE_FP16_SCALAR_ARITHMETIC)
        #include <arm_fp16.h> // float16_t scalar ops, armv7 neon does not have this header
    #endif
#endif

#if defined(AX_ARM)
    #if defined(_MSC_VER) && !defined(__clang__) && (defined(_M_ARM64) || defined(_M_HYBRID_X86_ARM64) || defined(_M_ARM64EC) || defined(__aarch64__))
        #include <arm64_neon.h>
    #else
        #include <arm_neon.h>
    #endif
#endif

/* Intrinsics Support */
#if (defined(AX_X64) || defined(AX_X86)) && !defined(AX_ARM)
    #if defined(_MSC_VER) && !defined(__clang__)
        #if _MSC_VER >= 1400   /* 2005 */
            #define AX_SUPPORT_SSE
        #endif
        #if _MSC_VER >= 1700 && !defined(AX_NO_AVX2)   /* 2012 */
            #define AX_SUPPORT_AVX2
        #endif
    #else
        #if defined(__SSE2__) 
            #define AX_SUPPORT_SSE
        #endif
        #if defined(__AVX2__) && !defined(AX_NO_AVX2)
            #define AX_SUPPORT_AVX2
        #endif
    #endif
    
    #include <intrin.h>

    /* If at this point we still haven't determined compiler support for the intrinsics just fall back to __has_include. */
    #if !defined(__GNUC__) && !defined(__clang__) && defined(__has_include)
        #if !defined(AX_SUPPORT_SSE) && __has_include(<emmintrin.h>)
            #define AX_SUPPORT_SSE
        #endif
        #if !defined(AX_SUPPORT_AVX2) && __has_include(<immintrin.h>)
            #define AX_SUPPORT_AVX2
        #endif
    #endif

    #if defined(AX_SUPPORT_AVX2) || defined(AX_SUPPORT_AVX)
        #include <immintrin.h>
    #elif defined(AX_SUPPORT_SSE)
        #include <emmintrin.h>
    #endif
#endif


#ifdef AX_SUPPORT_AVX2
    #define SIMD_NUM_BYTES 32
#elif defined(AX_SUPPORT_SSE) || defined(AX_SUPPORT_NEON)
    #define SIMD_NUM_BYTES 16
#else
    #define SIMD_NUM_BYTES sizeof(long)
#endif


#ifdef _MSC_VER
    #define VCALL __vectorcall
#else
    #define VCALL
#endif

#if defined(_MSC_VER)
    #define forceinline __forceinline
#elif defined(__clang__) || defined(__GNUC__)
    #define forceinline inline __attribute__((always_inline))
#else
    #define forceinline inline
#endif

#if defined(__clang__) || defined(__GNUC__)
    #define purefn static inline __attribute__((pure))
#elif defined(_MSC_VER)
    #define purefn static __forceinline __declspec(noalias)
#else
    #define purefn static inline __attribute__((always_inline))
#endif

#if defined(__GNUC__) || defined(__clang__)
    #define TrailingZeroCount32(x) __builtin_ctz(x)
    #define TrailingZeroCount64(x) __builtin_ctzll(x)
#elif defined(_MSC_VER) && !defined(AX_ARM)
    #define TrailingZeroCount32(x) _tzcnt_u32(x)
    #define TrailingZeroCount64(x) _tzcnt_u64(x)
#else
    // fallback implementation
    static inline uint32_t PopCount32_fallback(uint32_t x) {
        x = x - ((x >> 1) & 0x55555555);
        x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
        x = (x + (x >> 4)) & 0x0F0F0F0F;
        return (x * 0x01010101) >> 24;
    }
    static inline uint64_t PopCount64_fallback(uint64_t x) {
        x -= ((x >> 1) & 0x5555555555555555ull);
        x = (x & 0x3333333333333333ull) + (x >> 2 & 0x3333333333333333ull);
        return ((x + (x >> 4)) & 0xf0f0f0f0f0f0f0full) * 0x101010101010101ull >> 56;
    }
    #define TrailingZeroCount32(x) PopCount32_fallback((x & -(x)) - 1u)
    #define TrailingZeroCount64(x) PopCount64_fallback((x & -(x)) - 1ull)
#endif


#if defined(AX_SUPPORT_SSE) && !defined(AX_ARM)
/*//////////////////////////////////////////////////////////////////////////*/
/*                                 SSE                                      */
/*//////////////////////////////////////////////////////////////////////////*/

typedef __m128  v128f;
typedef __m128  v128i;
typedef __m128i v128u;

#define VecZero()                _mm_setzero_ps()
#define VecNegZero()             _mm_set1_ps(-0.0f)
#define VecOne()                 _mm_set1_ps(1.0f)
#define VecNegativeOne()         _mm_setr_ps( -1.0f, -1.0f, -1.0f, -1.0f)
#define VecSet1(x)               _mm_set1_ps(x)
#define VecSetBytes(x)           _mm_set1_epi8(x)
                                 
#define VecSet(x, y, z, w)       _mm_set_ps(x, y, z, w)  /* -> {w, z, y, x} */
#define VecSetR(x, y, z, w)      _mm_setr_ps(x, y, z, w) /* -> {x, y, z, w} */
#define VecLoad(x)               _mm_loadu_ps(x)
#define VecLoadA(x)              _mm_load_ps(x)
#define VecLoadI(x)              _mm_load_si128(x)
#define VecLoadIU(x)             _mm_loadu_si128(x)
#define VecStoreU(ptr, x)        _mm_storeu_si128((v128u*)ptr, x)
#define VecStoreI(ptr, x)        _mm_store_si128((v128u*)ptr, x)
                                
#define VecStore(ptr, x)         _mm_storeu_ps(ptr, x)
#define VecStoreA(ptr, x)        _mm_store_ps(ptr, x)

#define MakeShuffleMask(x,y,z,w)     (x | (y<<2) | (z<<4) | (w<<6)) /* internal use only */
// Get Set
// _mm_permute_ps is avx only
#define VecSplatX(v)             _mm_permute_ps(v, MakeShuffleMask(0, 0, 0, 0)) /* { v.x, v.x, v.x, v.x} */
#define VecSplatY(v)             _mm_permute_ps(v, MakeShuffleMask(1, 1, 1, 1)) /* { v.y, v.y, v.y, v.y} */
#define VecSplatZ(v)             _mm_permute_ps(v, MakeShuffleMask(2, 2, 2, 2)) /* { v.z, v.z, v.z, v.z} */
#define VecSplatW(v)             _mm_permute_ps(v, MakeShuffleMask(3, 3, 3, 3)) /* { v.w, v.w, v.w, v.w} */
                          
#define VecGetX(v)               _mm_cvtss_f32(v)             /* return v.x */
#define VecGetY(v)               _mm_cvtss_f32(VecSplatY(v))  /* return v.y */
#define VecGetZ(v)               _mm_cvtss_f32(VecSplatZ(v))  /* return v.z */
#define VecGetW(v)               _mm_cvtss_f32(VecSplatW(v))  /* return v.w */

#define VeciGetX(v)    ((u32)_mm_cvtsi128_si32(v))
#define VeciGetY(v)    ((u32)_mm_cvtsi128_si32(_mm_shuffle_epi32((v), _MM_SHUFFLE(1,1,1,1))))
#define VeciGetZ(v)    ((u32)_mm_cvtsi128_si32(_mm_shuffle_epi32((v), _MM_SHUFFLE(2,2,2,2))))
#define VeciGetW(v)    ((u32)_mm_cvtsi128_si32(_mm_shuffle_epi32((v), _MM_SHUFFLE(3,3,3,3))))

// SSE4.1                 
#define VecSetX(v, x)     ((v) = _mm_move_ss  ((v), _mm_set_ss(x)))
#define VecSetY(v, y)     ((v) = _mm_insert_ps((v), _mm_set_ss(y), 0x10))
#define VecSetZ(v, z)     ((v) = _mm_insert_ps((v), _mm_set_ss(z), 0x20))
#define VecSetW(v, w)     ((v) = _mm_insert_ps((v), _mm_set_ss(w), 0x30))

// Arithmetic
#define VecAdd(a, b)             _mm_add_ps(a, b)
#define VecSub(a, b)             _mm_sub_ps(a, b)
#define VecMul(a, b)             _mm_mul_ps(a, b)
#define VecDiv(a, b)             _mm_div_ps(a, b)
                                 
#define VecAddf(a, b)            _mm_add_ps(a, VecSet1(b))
#define VecSubf(a, b)            _mm_sub_ps(a, VecSet1(b))
#define VecMulf(a, b)            _mm_mul_ps(a, VecSet1(b))
#define VecDivf(a, b)            _mm_div_ps(a, VecSet1(b))
#define VecRound(v)              _mm_round_ps((v), _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC)

// a * b[l] + c
#define VecFmaddLane(a, b, c, l) _mm_fmadd_ps(a, _mm_permute_ps(b, MakeShuffleMask(l, l, l, l)), c)
#define VecFmadd(a, b, c)        _mm_fmadd_ps(a, b, c) /* a * b + c */
#define VecFmsub(a, b, c)        _mm_fmsub_ps(a, b, c)
#define VecNegMulSub(a, b, c)    VecFmsub(c, a, b) 
#define VecHadd(a, b)            _mm_hadd_ps(a, b) /* pairwise add (aw+bz, ay+bx, aw+bz, ay+bx) */

#define VecNeg(a)                _mm_sub_ps(_mm_setzero_ps(), a) /* -a */
#define VecRcp(a)                _mm_rcp_ps(a) /* 1.0f / a */
#define VecSqrt(a)               _mm_sqrt_ps(a)
#define VecRSqrt(a)              _mm_rsqrt_ps(a)
                                 
#define VeciNeg(a)               _mm_sub_epi32(_mm_set1_epi32(0), a) /* -a */

// Vector Math
// Dot products avoid _mm_dp_ps (DPPS): it has ~13-cycle latency and serializes
// on dependency chains. A multiply + lane reduction is shorter-latency. The *v
// forms broadcast the sum to all lanes (matching the old DPPS result); the *f
// forms extract a scalar. The 3-component forms sum x,y,z via splats, so the w
// lane is naturally excluded (no mask needed).
purefn v128f VCALL VecDotV(v128f a, v128f b) {
    v128f m = _mm_mul_ps(a, b);
    return _mm_add_ps(_mm_add_ps(VecSplatX(m), VecSplatY(m)),
                      _mm_add_ps(VecSplatZ(m), VecSplatW(m)));
}
purefn v128f VCALL Vec3DotVImpl(v128f a, v128f b) {
    v128f m = _mm_mul_ps(a, b);
    return _mm_add_ps(_mm_add_ps(VecSplatX(m), VecSplatY(m)), VecSplatZ(m));
}
purefn f32 VCALL VecDotfImpl(v128f a, v128f b) {
    v128f m = _mm_mul_ps(a, b);
    return VecGetX(m) + VecGetY(m) + VecGetZ(m) + VecGetW(m);
}
purefn f32 VCALL Vec3DotfImpl(v128f a, v128f b) {
    v128f m = _mm_mul_ps(a, b);
    return VecGetX(m) + VecGetY(m) + VecGetZ(m);
}
#define VecDot(a, b)             VecDotV(a, b)
#define VecDotf(a, b)            VecDotfImpl(a, b)
#define VecNorm(v)               _mm_div_ps(v, _mm_sqrt_ps(VecDotV(v, v)))
#define VecNormEst(v)            _mm_mul_ps(_mm_rsqrt_ps(VecDotV(v, v)), v)
#define VecLenf(v)               _mm_cvtss_f32(_mm_sqrt_ss(VecDotV(v, v)))
#define VecLen(v)                _mm_sqrt_ps(VecDotV(v, v))
#define VecLenSq(v)              VecDotV(v, v)

#define Vec3DotV(a, b)           Vec3DotVImpl(a, b)
#define Vec3DotfV(a, b)          Vec3DotfImpl(a, b)
#define Vec3NormV(v)             _mm_div_ps(v, _mm_sqrt_ps(Vec3DotVImpl(v, v)))
#define Vec3NormEstV(v)          _mm_mul_ps(_mm_rsqrt_ps(Vec3DotVImpl(v, v)), v)
#define Vec3LenfV(v)             _mm_cvtss_f32(_mm_sqrt_ss(Vec3DotVImpl(v, v)))
#define Vec3LenV(v)              _mm_sqrt_ps(Vec3DotVImpl(v, v))

// Swizzling Masking
#define VecSelect1000  _mm_castsi128_ps(_mm_setr_epi32(0xFFFFFFFF, 0x00000000, 0x00000000, 0x00000000))
#define VecSelect1100  _mm_castsi128_ps(_mm_setr_epi32(0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000))
#define VecSelect1110  _mm_castsi128_ps(_mm_setr_epi32(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000))
#define VecSelect1011  _mm_castsi128_ps(_mm_setr_epi32(0xFFFFFFFF, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF))
#define VecSelect1111  _mm_castsi128_ps(_mm_setr_epi32(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF))

#define VecMaskXY      _mm_castsi128_ps(_mm_setr_epi32(0xFFFFFFFF, 0xFFFFFFFF, 0x00000000, 0x00000000))
#define VecMask3       _mm_castsi128_ps(_mm_setr_epi32(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000))
#define VecMaskX       _mm_castsi128_ps(_mm_setr_epi32(0xFFFFFFFF, 0x00000000, 0x00000000, 0x00000000))
#define VecMaskY       _mm_castsi128_ps(_mm_setr_epi32(0x00000000, 0xFFFFFFFF, 0x00000000, 0x00000000))
#define VecMaskZ       _mm_castsi128_ps(_mm_setr_epi32(0x00000000, 0x00000000, 0xFFFFFFFF, 0x00000000))
#define VecMaskW       _mm_castsi128_ps(_mm_setr_epi32(0x00000000, 0x00000000, 0x00000000, 0xFFFFFFFF))

// vec(0, 1, 2, 3) -> (vec[x], vec[y], vec[z], vec[w])
#define VecSwizzleMask(vec, msk)    _mm_shuffle_ps(vec, vec, msk)
#define VecSwizzle(vec, x, y, z, w) VecSwizzleMask(vec, MakeShuffleMask(x,y,z,w))

// return (vec1[x], vec1[y], vec2[z], vec2[w])
#define VecShuffle(vec1, vec2, x, y, z, w)  _mm_shuffle_ps(vec1, vec2, MakeShuffleMask(x,y,z,w))
#define VecShuffleR(vec1, vec2, x, y, z, w) _mm_shuffle_ps(vec1, vec2, MakeShuffleMask(w,z,y,x))

// Special shuffle
#define VecShuffle_0101(vec1, vec2) _mm_movelh_ps(vec1, vec2)
#define VecShuffle_2323(vec1, vec2) _mm_movehl_ps(vec2, vec1)
#define VecRev(v) VecShuffle((v), (v), 3, 2, 1, 0)

// Pairwise swap (0<->1, 2<->3)
#define VecSwapPairs(v)             _mm_shuffle_ps(v,v,_MM_SHUFFLE(2,3,0,1))
#define VecSwapPairsU(a)            _mm_shuffle_epi32((a), _MM_SHUFFLE(2,3,0,1))
// Half swap (0123 -> 2301)
#define VecSwapHalves(v)            _mm_shuffle_ps(v,v,_MM_SHUFFLE(1,0,3,2))
#define VecSwapHalvesU(a)           _mm_shuffle_epi32((a), _MM_SHUFFLE(1,0,3,2))

// Logical
#define VecNot(a)                   _mm_andnot_ps(a, VecSelect1111)
#define VecAnd(a, b)                _mm_and_ps(a, b)
#define VecAndNot(a, b)             _mm_andnot_ps(a, b)
#define VecOr(a, b)                 _mm_or_ps(a, b)
#define VecXor(a, b)                _mm_xor_ps(a, b)
#define VecMask(a, msk)             _mm_and_ps(a, msk)

#define VecMax(a, b)                _mm_max_ps(a, b)
#define VecMin(a, b)                _mm_min_ps(a, b)
#define VecFloor(a)                 _mm_floor_ps(a)
#define VecCeil(v)                  _mm_ceil_ps(v)        // SSE4.1

#define VecCmpGt(a, b)              _mm_cmpgt_ps(a, b) /* greater than */
#define VecCmpGe(a, b)              _mm_cmpge_ps(a, b) /* greater or equal */
#define VecCmpLt(a, b)              _mm_cmplt_ps(a, b) /* less than */
#define VecCmpLe(a, b)              _mm_cmple_ps(a, b) /* less or equal */
#define VecCmpEq(a, b)              _mm_cmpeq_ps(a, b)
#define VecMovemask(a)              _mm_movemask_ps(a) /* */

#define VecSelect(V1, V2, Control)  _mm_blendv_ps(V1, V2, Control)
#define VecBlend(a, b, c)           _mm_blendv_ps(a, b, c)

//------------------------------------------------------------------------
// Veci
#define VeciZero()                  _mm_set1_epi32(0)
#define VeciSet1(x)                 _mm_set1_epi32(x)
#define VeciSet(x, y, z, w)         _mm_set_epi32(x, y, z, w)
#define VeciSetR(x, y, z, w)        _mm_setr_epi32(x, y, z, w)
#define VeciDup64(x)                _mm_set1_epi64x(x)
// _mm_load_epi32/_mm_loadu_epi32 are avx512 instructions, do not use them here
#define VeciLoadA(x)                _mm_load_si128((const v128u*)(x))
#define VeciLoad(x)                 _mm_loadu_si128((const v128u*)(x))
#define VeciLoad64(qword)           _mm_loadu_si64(qword)     /* loads 64bit integer to first 8 bytes of register */
                                    
// SSE4.1                           
#define VeciSetX(v, x)              ((v) = _mm_insert_epi32((v), 0, x))
#define VeciSetY(v, y)              ((v) = _mm_insert_epi32((v), 1, y))
#define VeciSetZ(v, z)              ((v) = _mm_insert_epi32((v), 2, z))
#define VeciSetW(v, w)              ((v) = _mm_insert_epi32((v), 3, w))
                                    
#define VeciSelect1111              _mm_set1_epi32(0xFFFFFFFF)
                                    
#define VecIdentityR0               _mm_setr_ps(1.0f, 0.0f, 0.0f, 0.0f)
#define VecIdentityR1               _mm_setr_ps(0.0f, 1.0f, 0.0f, 0.0f)
#define VecIdentityR2               _mm_setr_ps(0.0f, 0.0f, 1.0f, 0.0f)
#define VecIdentityR3               _mm_setr_ps(0.0f, 0.0f, 0.0f, 1.0f)
                                    
#define VeciAdd(a, b)               _mm_add_epi32(a, b)
#define VeciSub(a, b)               _mm_sub_epi32(a, b)
#define VeciMul(a, b)               _mm_mullo_epi32(a, b)
                                    
#define VeciNot(a)                  _mm_andnot_si128(a, _mm_set1_epi32(0xFFFFFFFF))
#define VeciAnd(a, b)               _mm_and_si128(a, b)
#define VeciOr(a, b)                _mm_or_si128(a, b)
#define VeciXor(a, b)               _mm_xor_si128(a, b)
                                    
#define VeciAndNot(a, b)            _mm_andnot_si128(a, b)  /* ~a  & b */
#define VeciSrl(a, b)               _mm_srlv_epi32(a, b)    /*  a >> b */
#define VeciSll(a, b)               _mm_sllv_epi32(a, b)    /*  a << b */
#define VeciSrl32(a, b)             _mm_srli_epi32(a, b)    /*  a >> b */
#define VeciSll32(a, b)             _mm_slli_epi32(a, b)    /*  a << b */
#define VeciToVecf(a)               _mm_castsi128_ps(a)     /*  a << b */
                                    
// signed compares. le/ge have no sse instruction, they are not(gt)/not(lt)
#define VeciCmpLt(a, b)             _mm_cmplt_epi32(a, b)
#define VeciCmpLe(a, b)             VeciNot(_mm_cmpgt_epi32(a, b))
#define VeciCmpGt(a, b)             _mm_cmpgt_epi32(a, b)
#define VeciCmpGe(a, b)             VeciNot(_mm_cmplt_epi32(a, b))
#define VeciCmpEq(a, b)             _mm_cmpeq_epi32(a, b)
                                    
#define VeciBlend(a, b, c)          _mm_blendv_epi8(a, b, c)
#define VecFabs(x)                  VecAnd(x, VecFromInt1(0x7fffffff))

#define VecFromInt(x, y, z, w)      _mm_castsi128_ps(_mm_setr_epi32(x, y, z, w))
#define VecFromInt1(x)              _mm_castsi128_ps(_mm_set1_epi32(x))
#define VecToInt(x) x               
                                    
#define VecBitcastU32(x)            _mm_castps_si128(x)
#define VeciBitcastF32(x)           _mm_castsi128_ps(x)
                                    
#define VecF32ToI32(x)              _mm_cvtps_epi32(x)                         /* f32[4] -> i32[4] round to nearest */
#define VecF32ToU32(x)              _mm_cvttps_epi32(x)                         /* f32[4] -> i32[4] truncate toward zero */
#define VecI32ToF32(x)              _mm_cvtepi32_ps(x)                         /* i32[4] -> f32[4] */

/* u32[4] -> f32[4]. _mm_cvtepu32_ps is avx512, split halves to stay on sse/avx2 */
purefn __m128 VCALL VecU32ToF32(__m128i v) {
    __m128i lo = _mm_and_si128(v, _mm_set1_epi32(0xFFFF));
    __m128i hi = _mm_srli_epi32(v, 16);
    return _mm_add_ps(_mm_mul_ps(_mm_cvtepi32_ps(hi), _mm_set1_ps(65536.0f)), _mm_cvtepi32_ps(lo));
}
                                                                               
#define VecZipLo32(a, b)            _mm_unpacklo_epi32(a, b)                   /* interleave low i32: [a0 b0 a1 b1] | NEON: vzip1q_s32 */
#define VecZipLo16(a, b)            _mm_unpacklo_epi16(a, b)                   /* interleave low i16  | NEON: vzip1q_s16 */
#define VecZipHi16(a, b)            _mm_unpackhi_epi16(a, b)                   /* interleave high i16 | NEON: vzip2q_s16 */
                                    
#define VecUnpackLo32(x)            _mm_unpacklo_epi16(x, _mm_setzero_si128()) /* zero-extend low 4 i16 -> i32 | NEON: vmovl_s16(vget_low_s16) */
#define VecUnpackHi32(x)            _mm_unpackhi_epi16(x, _mm_setzero_si128()) /* zero-extend high 4 i16 -> i32 | NEON: vmovl_s16(vget_high_s16) */
                                    
#define VecPack16(x)                _mm_packus_epi32((x), _mm_setzero_si128()) /* narrow u32 -> u16 (sat) | NEON: vqmovn_u32 */
#define VecPack16S(x)               _mm_packs_epi32((x), _mm_setzero_si128())  /* narrow i32 -> i16 (signed sat) | NEON: vqmovn_s32 */

/* sign-extend variants, VecUnpackLo32/Hi32 zero-extend and destroy negative i16 values */
#define VecUnpackLo32S(x)           _mm_srai_epi32(_mm_unpacklo_epi16(_mm_setzero_si128(), x), 16)
#define VecUnpackHi32S(x)           _mm_srai_epi32(_mm_unpackhi_epi16(_mm_setzero_si128(), x), 16)
                                    
#define VecStoreLo64(p, v)          _mm_storel_pi((__m64*)(p), _mm_castsi128_ps(v)) /* store lower 64 bits | NEON: vst1 */
#define VecStoreHi64(p, v)          _mm_storeh_pi((__m64*)(p), _mm_castsi128_ps(v)) /* store lower 64 bits | NEON: vst1 */
#define VecLoadLo64(p, v)           _mm_loadl_pi(v, (__m64*)(p))  /* store lower 64 bits | NEON: vst1 */
#define VecLoadHi64(p, v)           _mm_loadh_pi(v, (__m64*)(p))  /* store high 64 bits | NEON: vst1 */

static inline v128f VCALL Vec3Load(void const* x) {
    v128f v = _mm_loadu_ps((float const*)x); 
    VecSetW(v, 0.0); return v;
}

#elif defined(AX_ARM)
/*//////////////////////////////////////////////////////////////////////////*/
/*                                 NEON                                     */
/*//////////////////////////////////////////////////////////////////////////*/

typedef float32x4_t v128f;
typedef uint32x4_t v128i;
typedef uint32x4_t v128u;

#define VecZero()                   vdupq_n_f32( 0.0f)
#define VecNegZero()                vdupq_n_f32(-0.0f)
#define VecOne()                    vdupq_n_f32( 1.0f)
#define VecNegativeOne()            vdupq_n_f32(-1.0f)
#define VecSet1(x)                  vdupq_n_f32(x)
#define VecSet(x, y, z, w)          ARMCreateVec(w, z, y, x) /* -> {w, z, y, x} */
#define VecSetR(x, y, z, w)         ARMCreateVec(x, y, z, w) /* -> {x, y, z, w} */
#define VecLoad(x)                  vld1q_f32(x)
#define VecLoadA(x)                 vld1q_f32(x)
#define VecLoadI(x)                 vld1q_u32((const u32*)x)
#define VecLoadIU(x)                vld1q_u32((const u32*)x)
#define VecStore(ptr, x)            vst1q_f32(ptr, x)
#define VecStoreA(ptr, x)           vst1q_f32(ptr, x)
#define VecStoreI(ptr, x)           vst1q_u32((u32*)ptr, x)
#define VecStoreU(ptr, x)           vst1q_u32((u32*)ptr, x)
#define VecSetBytes(x)              vdupq_n_u8(x)
                                    
#define Vec3Load(x)                 ARMVector3Load(x)
                                    
/* conversions match the documented SSE behavior exactly:
   VecF32ToI32 rounds to nearest (cvtps), VecF32ToU32 truncates as SIGNED (cvttps) */
#define VecF32ToI32(x)              vreinterpretq_u32_s32(vcvtnq_s32_f32(x)) /* f32[4] -> i32[4] round to nearest */
#define VecF32ToU32(x)              vreinterpretq_u32_s32(vcvtq_s32_f32(x))  /* f32[4] -> i32[4] truncate toward zero */
#define VecI32ToF32(x)              vcvtq_f32_s32(vreinterpretq_s32_u32(x))  /* i32[4] -> f32[4] */
#define VecU32ToF32(x)              vcvtq_f32_u32(x)                         /* u32[4] -> f32[4] */

#define VecZipLo32(a, b)            vzip1q_u32(a, b)             /* interleave low i32: [a0 b0 a1 b1] */
#define VecZipLo16(a, b)            vreinterpretq_u32_u16(vzip1q_u16(vreinterpretq_u16_u32(a), vreinterpretq_u16_u32(b)))
#define VecZipHi16(a, b)            vreinterpretq_u32_u16(vzip2q_u16(vreinterpretq_u16_u32(a), vreinterpretq_u16_u32(b)))

/* zero-extend like the sse unpack with zero */
#define VecUnpackLo32(x)            vmovl_u16(vget_low_u16(vreinterpretq_u16_u32(x)))   /* zero-extend low 4 u16 -> u32 */
#define VecUnpackHi32(x)            vmovl_u16(vget_high_u16(vreinterpretq_u16_u32(x)))  /* zero-extend high 4 u16 -> u32 */

/* sse packus: input treated signed, saturated to [0, 65535], high half zeroed */
#define VecPack16(x)                vreinterpretq_u32_u16(vcombine_u16(vqmovun_s32(vreinterpretq_s32_u32(x)), vdup_n_u16(0)))
#define VecPack16S(x)               vreinterpretq_u32_s16(vcombine_s16(vqmovn_s32(vreinterpretq_s32_u32(x)), vdup_n_s16(0)))

/* sign-extend variants */
#define VecUnpackLo32S(x)           vreinterpretq_u32_s32(vmovl_s16(vget_low_s16(vreinterpretq_s16_u32(x))))
#define VecUnpackHi32S(x)           vreinterpretq_u32_s32(vmovl_s16(vget_high_s16(vreinterpretq_s16_u32(x))))
#define VecStoreLo64(p, v)          vst1_u32((u32*)(p), vget_low_u32(v))  /* store lower 64 bits */
#define VecStoreHi64(p, v)          vst1_u32((u32*)(p), vget_high_u32(v)) /* store higher 64 bits */
#define VecLoadLo64(p, v)           vcombine_f32(vld1_f32((float*)(p)), vget_high_f32(v)) /* load lower 64 bits */
#define VecLoadHi64(p, v)           vcombine_f32(vget_low_f32(v), vld1_f32((float*)(p))) /* load higher 64 bits */

// Get Set                          
#define VecSplatX(v)                vdupq_lane_f32(vget_low_f32(v), 0)
#define VecSplatY(v)                vdupq_lane_f32(vget_low_f32(v), 1)
#define VecSplatZ(v)                vdupq_lane_f32(vget_high_f32(v), 0)
#define VecSplatW(v)                vdupq_lane_f32(vget_high_f32(v), 1)
                                    
#define VecGetX(v)                  vgetq_lane_f32(v, 0)
#define VecGetY(v)                  vgetq_lane_f32(v, 1)
#define VecGetZ(v)                  vgetq_lane_f32(v, 2)
#define VecGetW(v)                  vgetq_lane_f32(v, 3)
                                    
#define VeciGetX(v)                 vgetq_lane_u32((v), 0)
#define VeciGetY(v)                 vgetq_lane_u32((v), 1)
#define VeciGetZ(v)                 vgetq_lane_u32((v), 2)
#define VeciGetW(v)                 vgetq_lane_u32((v), 3)

#define VecSetX(v, x)               ((v) = vsetq_lane_f32(x, v, 0))
#define VecSetY(v, y)               ((v) = vsetq_lane_f32(y, v, 1))
#define VecSetZ(v, z)               ((v) = vsetq_lane_f32(z, v, 2))
#define VecSetW(v, w)               ((v) = vsetq_lane_f32(w, v, 3))
                                    
// Arithmetic                       
#define VecAdd(a, b)                vaddq_f32(a, b)
#define VecSub(a, b)                vsubq_f32(a, b)
#define VecMul(a, b)                vmulq_f32(a, b)
#define VecDiv(a, b)                ARMVectorDevide(a, b)
                                              
#define VecAddf(a, b)               vaddq_f32(a, vdupq_n_f32(b))
#define VecSubf(a, b)               vsubq_f32(a, vdupq_n_f32(b))
#define VecMulf(a, b)               vmulq_n_f32(a, b)
#define VecDivf(a, b)               ARMVectorDevide(a, VecSet1(b))
#define VecRound(v)                 vrndnq_f32(v)   // round to nearest int (float output)
                                    
// a * b[l] + c                     
#define VecFmaddLane(a, b, c, l)    vfmaq_laneq_f32(c, a, b, l)
#define VecFmadd(a, b, c)           vfmaq_f32(c, a, b)
#define VecFmsub(a, b, c)           vnegq_f32(vfmsq_f32(c, a, b))
#define VecNegMulSub(a, b, c)       VecFmsub(c, a, b) 
#define VecHadd(a, b)               vpaddq_f32(a, b)
#define VecSqrt(a)                  vsqrtq_f32(a)
#define VecRcp(a)                   vrecpeq_f32(a)
#define VecRSqrt(a)                 vrsqrteq_f32(a)
#define VecNeg(a)                   vnegq_f32(a)
                                    
// Vector Math                      
#define VecDot(a, b)                ARMVectorDot(a, b)
#define VecDotf(a, b)               VecGetX(ARMVectorDot(a, b))
#define VecNorm(v)                  ARMVectorNorm(v)
#define VecNormEst(v)               ARMVectorNormEst(v)
#define VecLenf(v)                  VecGetX(ARMVectorLength(v))
#define VecLen(v)                   ARMVectorLength(v)
#define VecLenSq(v)                 ARMVectorSqrLength(v)
                                    
#define Vec3DotV(a, b)              ARMVector3Dot(a, b)
#define Vec3DotfV(a, b)             VecGetX(ARMVector3Dot(a, b))
#define Vec3NormV(v)                ARMVector3Norm(v)
#define Vec3NormEstV(v)             ARMVector3NormEst(v)
#define Vec3LenfV(v)                VecGetX(ARMVector3Length(v))
#define Vec3LenV(v)                 ARMVector3Length(v)

#define ARMVectorSwizzle(E0, E1, E2, E3, v) \
    VecSetR( \
        vgetq_lane_f32((v), (E0)), \
        vgetq_lane_f32((v), (E1)), \
        vgetq_lane_f32((v), (E2)), \
        vgetq_lane_f32((v), (E3))  \
    )

#define ARMVectorShuffle(E0, E1, E2, E3, v0, v1) \
    VecSetR( \
        vgetq_lane_f32((v0), (E0)), \
        vgetq_lane_f32((v0), (E1)), \
        vgetq_lane_f32((v1), (E2)), \
        vgetq_lane_f32((v1), (E3))  \
    )

#define ARMVectorU32Swizzle(E0, E1, E2, E3, v) \
VecSetR( \
    vgetq_lane_u32((v), (E0)), \
    vgetq_lane_u32((v), (E1)), \
    vgetq_lane_u32((v), (E2)), \
    vgetq_lane_u32((v), (E3))  \
)

#define ARMVectorU32Shuffle(E0, E1, E2, E3, v0, v1) \
    ARMCreateVecI( \
        vgetq_lane_u32((v0), (E0)), \
        vgetq_lane_u32((v0), (E1)), \
        vgetq_lane_u32((v1), (E2)), \
        vgetq_lane_u32((v1), (E3))  \
    )


// vec(0, 1, 2, 3) -> (vec[x], vec[y], vec[z], vec[w])
#define VecSwizzle(vec, x, y, z, w)         ARMVectorSwizzle(x, y, z, w, vec)

#define VecShuffle(vec1, vec2, x, y, z, w)  ARMVectorShuffle(x, y, z, w, vec1, vec2)
#define VecShuffleR(vec1, vec2, x, y, z, w) ARMVectorShuffle(w, z, y, x, vec1, vec2)

// special shuffle
#define VecShuffle_0101(vec1, vec2) vcombine_f32(vget_low_f32(vec1), vget_low_f32(vec2))
#define VecShuffle_2323(vec1, vec2) vcombine_f32(vget_high_f32(vec1), vget_high_f32(vec2))
#define VecRev(v) ARMVectorRev(v)
// Pairwise swap (0<->1, 2<->3)
#define VecSwapPairs(v)             vrev64q_f32(v)
#define VecSwapPairsU(a)            vrev64q_u32(a)
// Half swap (0123 -> 2301)         
#define VecSwapHalves(v)            vextq_f32(v,v,2)
#define VecSwapHalvesU(a)           vcombine_u32(vget_high_u32(a), vget_low_u32(a))

#define VecNot(a)                   vreinterpretq_f32_u32(vmvnq_u32(vreinterpretq_u32_f32(a)))
#define VecAnd(a, b)                vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(b)))
#define VecAndNot(a, b)             vreinterpretq_f32_u32(vbicq_u32(vreinterpretq_u32_f32(b), vreinterpretq_u32_f32(a)))
#define VecOr(a, b)                 vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(b)))
#define VecXor(a, b)                vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(b)))
                                    
#define VecMask(a, msk)             VecSelect(vdupq_n_f32(0.0f), a, msk)
                                    
#define VecMax(a, b)                vmaxq_f32(a, b)
#define VecMin(a, b)                vminq_f32(a, b)
#define VecFloor(a)                 vrndmq_f32(a)
#define VecCeil(v)                  vrndpq_f32(v)         
                                    
/* comparisons return v128f like sse so the results compose with VecAnd/VecOr/
   VecSelect and variable declarations identically on both platforms */
#define VecCmpGt(a, b)              vreinterpretq_f32_u32(vcgtq_f32(a, b)) // greater than
#define VecCmpGe(a, b)              vreinterpretq_f32_u32(vcgeq_f32(a, b)) // greater or equal
#define VecCmpLt(a, b)              vreinterpretq_f32_u32(vcltq_f32(a, b)) // less than
#define VecCmpLe(a, b)              vreinterpretq_f32_u32(vcleq_f32(a, b)) // less or equal
#define VecCmpEq(a, b)              vreinterpretq_f32_u32(vceqq_f32(a, b)) // equal
#define VecMovemask(a)              ARMVecMovemask(a)

/* note: sse blendv selects by the sign bit only, neon bsl selects per bit.
   identical for full lane masks (comparison results), do not pass arbitrary floats */
#define VecSelect(V1, V2, Control)  vbslq_f32(vreinterpretq_u32_f32(Control), V2, V1)
#define VecBlend(a, b, Control)     vbslq_f32(vreinterpretq_u32_f32(Control), b, a)

//------------------------------------------------------------------------
// Veci
#define VeciZero()                  vdupq_n_u32(0)
#define VeciSet1(x)                 vdupq_n_u32(x)
#define VeciSetR(x, y, z, w)        ARMCreateVecI(x, y, z, w)
#define VeciSet(x, y, z, w)         ARMCreateVecI(w, z, y, x)
#define VeciDup64(x)                vreinterpretq_u32_u64(vdupq_n_u64(x))

#define VeciLoadA(x)                vld1q_u32((const u32*)(x))
#define VeciLoad(x)                 vld1q_u32((const u32*)(x))
#define VeciLoad64(qword)           vcombine_u32(vcreate_u32(qword), vcreate_u32(0ull)) /* loads 64bit integer to first 8 bytes of register */

#define VeciSetX(v, x)       ((v) = vsetq_lane_u32(x, v, 0))
#define VeciSetY(v, y)       ((v) = vsetq_lane_u32(y, v, 1))
#define VeciSetZ(v, z)       ((v) = vsetq_lane_u32(z, v, 2))
#define VeciSetW(v, w)       ((v) = vsetq_lane_u32(w, v, 3))

#define VeciAdd(a, b)               vaddq_u32(a, b)
#define VeciSub(a, b)               vsubq_u32(a, b)
#define VeciMul(a, b)               vmulq_u32(a, b)

#define VecBitcastU32(x)            vreinterpretq_u32_f32(x)
#define VeciBitcastF32(x)           vreinterpretq_f32_u32(x)
#define VecFromInt(x, y, z, w)      VeciBitcastF32(ARMCreateVecI(x, y, z, w))
#define VecFromInt1(x)              VeciBitcastF32(vdupq_n_u32(x))

// Swizzling Masking: float typed like the sse _mm_castsi128_ps versions
#define VecSelect1000  VecFromInt(0xFFFFFFFFu, 0x00000000u, 0x00000000u, 0x00000000u)
#define VecSelect1100  VecFromInt(0xFFFFFFFFu, 0xFFFFFFFFu, 0x00000000u, 0x00000000u)
#define VecSelect1110  VecFromInt(0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0x00000000u)
#define VecSelect1011  VecFromInt(0xFFFFFFFFu, 0x00000000u, 0xFFFFFFFFu, 0xFFFFFFFFu)
#define VecSelect1111  VecFromInt(0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu)

#define VeciSelect1111 ARMCreateVecI(0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu)

#define VecIdentityR0  ARMCreateVec(1.0f, 0.0f, 0.0f, 0.0f)
#define VecIdentityR1  ARMCreateVec(0.0f, 1.0f, 0.0f, 0.0f)
#define VecIdentityR2  ARMCreateVec(0.0f, 0.0f, 1.0f, 0.0f)
#define VecIdentityR3  ARMCreateVec(0.0f, 0.0f, 0.0f, 1.0f)

#define VecMaskXY      VecFromInt(0xFFFFFFFFu, 0xFFFFFFFFu, 0x00000000u, 0x00000000u)
#define VecMask3       VecFromInt(0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0x00000000u)
#define VecMaskX       VecFromInt(0xFFFFFFFFu, 0x00000000u, 0x00000000u, 0x00000000u)
#define VecMaskY       VecFromInt(0x00000000u, 0xFFFFFFFFu, 0x00000000u, 0x00000000u)
#define VecMaskZ       VecFromInt(0x00000000u, 0x00000000u, 0xFFFFFFFFu, 0x00000000u)
#define VecMaskW       VecFromInt(0x00000000u, 0x00000000u, 0x00000000u, 0xFFFFFFFFu)

// Logical
#define VeciNot(a)                  vmvnq_u32(a)
#define VeciAnd(a, b)               vandq_u32(a, b)
#define VeciOr(a, b)                vorrq_u32(a, b)
#define VeciXor(a, b)               veorq_u32(a, b)
                                    
#define VeciAndNot(a, b)            vandq_u32(vmvnq_u32(a), b)  /* ~a & b */
#define VeciSrl(a, b)               vshlq_u32(a, vnegq_s32(vreinterpretq_s32_u32(b)))  /* a >> b */
#define VeciSll(a, b)               vshlq_u32(a, vreinterpretq_s32_u32(b))             /* a << b */
#define VeciSrl32(a, b)             vshrq_n_u32(a, b)           /* a >> b */
#define VeciSll32(a, b)             vshlq_n_u32(a, b)           /* a << b */
#define VeciToVecf(a)               vreinterpretq_f32_u32(a)    /* Reinterpret int as float */
#define VeciNeg(a)                  vreinterpretq_u32_s32(vnegq_s32(vreinterpretq_s32_u32(a))) /* -a */

/* signed compares like the sse versions */
#define VeciCmpLt(a, b)             vcltq_s32(vreinterpretq_s32_u32(a), vreinterpretq_s32_u32(b))
#define VeciCmpLe(a, b)             vcleq_s32(vreinterpretq_s32_u32(a), vreinterpretq_s32_u32(b))
#define VeciCmpGt(a, b)             vcgtq_s32(vreinterpretq_s32_u32(a), vreinterpretq_s32_u32(b))
#define VeciCmpGe(a, b)             vcgeq_s32(vreinterpretq_s32_u32(a), vreinterpretq_s32_u32(b))
#define VeciCmpEq(a, b)             vceqq_u32(a, b)

#define VeciBlend(a, b, c)          vbslq_u32(c, b, a)  /* Blend a and b based on mask c */
#define VecFabs(x)                  vabsq_f32(x)

purefn v128f ARMVectorRev(v128f v)
{
    float32x4_t rev64 = vrev64q_f32(v);
    return vextq_f32(rev64, rev64, 2);
}

purefn v128f ARMVector3Load(float* src) {
    return vcombine_f32(vld1_f32(src), vld1_lane_f32(src + 2, vdup_n_f32(0), 0));
}

purefn v128f ARMCreateVec(float x, float y, float z, float w) {
    AX_ALIGN(16) float v[4] = {x, y, z, w};
    return vld1q_f32(v);
}

purefn v128i ARMCreateVecI(u32 x, u32 y, u32 z, u32 w) {
    return vcombine_u32(vcreate_u32(((uint64_t)x) | (((uint64_t)y) << 32)),
                        vcreate_u32(((uint64_t)z) | (((uint64_t)w) << 32)));
}

purefn v128f ARMVector3NormEst(v128f v) {
    float32x4_t vTemp = vmulq_f32(v, v);
    float32x2_t v1 = vget_low_f32(vTemp);
    float32x2_t v2 = vget_high_f32(vTemp);
    v1 = vpadd_f32(v1, v1);
    v2 = vdup_lane_f32(v2, 0);
    v1 = vadd_f32(v1, v2); // Dot3
    v2 = vrsqrte_f32(v1); // Reciprocal sqrt (estimate)
    return vmulq_f32(v, vcombine_f32(v2, v2)); // Normalize
}

static inline v128f ARMVector3Norm(v128f v) {
    // Dot3
    float32x4_t vTemp = vmulq_f32(v, v);
    float32x2_t v1 = vget_low_f32(vTemp);
    float32x2_t v2 = vget_high_f32(vTemp);
    v1 = vpadd_f32(v1, v1);
    v2 = vdup_lane_f32(v2, 0);
    v1 = vadd_f32(v1, v2);
    uint32x2_t VEqualsZero = vceq_f32(v1, vdup_n_f32(0));
    // Reciprocal sqrt (2 iterations of Newton-Raphson)
    float32x2_t S0 = vrsqrte_f32(v1);
    float32x2_t P0 = vmul_f32(v1, S0);
    float32x2_t R0 = vrsqrts_f32(P0, S0);
    float32x2_t S1 = vmul_f32(S0, R0);
    float32x2_t R1 = vrsqrts_f32(vmul_f32(v1, S1), S1);
    v2 = vmul_f32(S1, R1);
    // Normalize
    v128f vResult = vmulq_f32(v, vcombine_f32(v2, v2));
    vResult = vbslq_f32(vcombine_u32(VEqualsZero, VEqualsZero), vdupq_n_f32(0), vResult);
    return vResult;
}

purefn v128f ARMVector3Dot(v128f a, v128f b) {
    float32x4_t vTemp = vmulq_f32(a, b);
    float32x2_t v1 = vget_low_f32(vTemp);
    float32x2_t v2 = vget_high_f32(vTemp);
    v1 = vpadd_f32(v1, v1);
    v2 = vdup_lane_f32(v2, 0);
    v1 = vadd_f32(v1, v2);
    return vcombine_f32(v1, v1);
}

static inline v128f ARMVector3Length(v128f v) {
    float32x4_t vTemp = vmulq_f32(v, v);
    float32x2_t v1 = vget_low_f32(vTemp);
    float32x2_t v2 = vget_high_f32(vTemp);
    v1 = vpadd_f32(v1, v1);
    v2 = vdup_lane_f32(v2, 0);
    v1 = vadd_f32(v1, v2);
    const float32x2_t zero = vdup_n_f32(0);
    uint32x2_t VEqualsZero = vceq_f32(v1, zero);
    float32x2_t Result = vrsqrte_f32(v1);
    Result = vmul_f32(v1, Result);
    Result = vbsl_f32(VEqualsZero, zero, Result);
    return vcombine_f32(Result, Result);
}

static inline v128f ARMVectorLengthEst(v128f v) {
    float32x4_t vTemp = vmulq_f32(v, v);
    float32x2_t v1 = vget_low_f32(vTemp);
    float32x2_t v2 = vget_high_f32(vTemp);
    v1 = vadd_f32(v1, v2);
    v1 = vpadd_f32(v1, v1);
    const float32x2_t zero = vdup_n_f32(0);
    uint32x2_t VEqualsZero = vceq_f32(v1, zero);
    float32x2_t Result = vrsqrte_f32(v1);
    Result = vmul_f32(v1, Result);
    Result = vbsl_f32(VEqualsZero, zero, Result);
    return vcombine_f32(Result, Result);
}

static inline v128f ARMVectorSqrLength(v128f v) {
	// Dot4
    float32x4_t vTemp = vmulq_f32(v, v);
    float32x2_t v1 = vget_low_f32(vTemp);
    float32x2_t v2 = vget_high_f32(vTemp);
    v1 = vadd_f32(v1, v2);
    v1 = vadd_f32(v1, vrev64_f32(v1));
    return vcombine_f32(v1, v1);
}

static inline v128f ARMVectorLength(v128f v) {
	// Dot4
    float32x4_t vTemp = vmulq_f32(v, v);
    float32x2_t v1 = vget_low_f32(vTemp);
    float32x2_t v2 = vget_high_f32(vTemp);
    v1 = vadd_f32(v1, v2);
    v1 = vpadd_f32(v1, v1);
    const float32x2_t zero = vdup_n_f32(0);
    uint32x2_t VEqualsZero = vceq_f32(v1, zero);
    // Sqrt
    float32x2_t S0 = vrsqrte_f32(v1);
    float32x2_t P0 = vmul_f32(v1, S0);
    float32x2_t R0 = vrsqrts_f32(P0, S0);
    float32x2_t S1 = vmul_f32(S0, R0);
    float32x2_t P1 = vmul_f32(v1, S1);
    float32x2_t R1 = vrsqrts_f32(P1, S1);
    float32x2_t Result = vmul_f32(S1, R1);
    Result = vmul_f32(v1, Result);
    Result = vbsl_f32(VEqualsZero, zero, Result);
    return vcombine_f32(Result, Result);
}

purefn v128f ARMVectorDevide(v128f V1, v128f V2) {
    // 2 iterations of Newton-Raphson refinement of reciprocal
    float32x4_t Reciprocal = vrecpeq_f32(V2);
    float32x4_t S = vrecpsq_f32(Reciprocal, V2);
    Reciprocal = vmulq_f32(S, Reciprocal);
    S = vrecpsq_f32(Reciprocal, V2);
    Reciprocal = vmulq_f32(S, Reciprocal);
    return vmulq_f32(V1, Reciprocal);
}

purefn v128f ARMVectorDot(v128f a, v128f b) {
    float32x4_t vTemp = vmulq_f32(a, b);
    float32x2_t v1 = vget_low_f32(vTemp);
    float32x2_t v2 = vget_high_f32(vTemp);
    v1 = vadd_f32(v1, v2);
    v1 = vpadd_f32(v1, v1);
    return vcombine_f32(v1, v1);
}

purefn v128f ARMVectorNormEst(v128f v) {
    float32x4_t vTemp = vmulq_f32(v, v);
    float32x2_t v1 = vget_low_f32(vTemp);
    float32x2_t v2 = vget_high_f32(vTemp);
    v1 = vadd_f32(v1, v2);
    v1 = vpadd_f32(v1, v1);
    v2 = vrsqrte_f32(v1);
    return vmulq_f32(v, vcombine_f32(v2, v2));
}

purefn v128f ARMVectorNorm(v128f v) 
{
    return ARMVectorDevide(v, ARMVectorLength(v));
}

purefn int ARMVecMovemask(v128f v) {
    const int shiftArr[4] = { 0, 1, 2, 3 };
    int32x4_t shift = vld1q_s32(shiftArr);
    return (int)vaddvq_u32(vshlq_u32(vshrq_n_u32(vreinterpretq_u32_f32(v), 31), shift));
}

#endif

#if defined(AX_SUPPORT_AVX2)
typedef __m256  v256f;
typedef __m256i v256i;
typedef __m256i v256u;

// parens around ptr matter: the cast must apply after pointer arithmetic like (u64*)res + 4,
// casting first would scale the offset by sizeof(__m256i) and read/write 128 bytes off
#define VecLoadI256(ptr)     _mm256_stream_load_si256((__m256i const *)(ptr))
#define VecStoreI256(ptr, x) _mm256_stream_si256((__m256i *)(ptr), x)

#define VeciAndNot256(x, y)  _mm256_andnot_si256(x, y)
#define VeciAnd256(x, y)     _mm256_and_si256(x, y)
#define VeciOr256(x, y)      _mm256_or_si256(x, y)
#define VeciXor256(x, y)     _mm256_xor_si256(x, y)

#define VeciSrl256(a, b)     _mm256_srlv_epi32(a, b)    /*  a >> b */
#define VeciSll256(a, b)     _mm256_sllv_epi32(a, b)    /*  a << b */
#define VeciSrl32_256(a, b)  _mm256_srli_epi32(a, b)    /*  a >> b */
#define VeciSll32_256(a, b)  _mm256_slli_epi32(a, b)
#define VeciSet1_256(x)      _mm256_set1_epi32(x)

#define VeciAdd256(a, b)     _mm256_add_epi32(a, b)
#define VeciSub256(a, b)     _mm256_sub_epi32(a, b)
#define VeciMul256(a, b)     _mm256_mullo_epi32(a, b)
#define VeciDiv256(a, b)     _mm256_div_epi32(a, b)

#define VecSetBytes256(x)    _mm256_set1_epi8(x)

#else
typedef struct Vec8x32f_ { v128f lo, hi; } v256f;
typedef struct Vec8x32i_ { v128i lo, hi; } v256i;
typedef struct Vec8x32u_ { v128u lo, hi; } v256u;

#define VeciAndNot256(x, y)   (v256u){ VeciAndNot((x).lo, (y).lo), VeciAndNot((x).hi, (y).hi) }
#define VeciAnd256(x, y)      (v256u){ VeciAnd   ((x).lo, (y).lo), VeciAnd   ((x).hi, (y).hi) }
#define VeciOr256(x, y)       (v256u){ VeciOr    ((x).lo, (y).lo), VeciOr    ((x).hi, (y).hi) }
#define VeciXor256(x, y)      (v256u){ VeciXor   ((x).lo, (y).lo), VeciXor   ((x).hi, (y).hi) }
#define VeciSrl256(a, b)      (v256u){ VeciSrl   ((a).lo, (b).lo), VeciSrl   ((a).hi, (b).hi) }
#define VeciSll256(a, b)      (v256u){ VeciSll   ((a).lo, (b).lo), VeciSll   ((a).hi, (b).hi) }
#define VeciSrl32_256(a, b)   (v256u){ VeciSrl32 ((a).lo, (b).lo), VeciSrl32 ((a).hi, (b).hi) }
#define VeciSll32_256(a, b)   (v256u){ VeciSll32 ((a).lo, (b).lo), VeciSll32 ((a).hi, (b).hi) }
#define VeciAdd256(a, b)      (v256u){ VeciAdd   ((a).lo, (b).lo), VeciAdd   ((a).hi, (b).hi) }
#define VeciSub256(a, b)      (v256u){ VeciSub   ((a).lo, (b).lo), VeciSub   ((a).hi, (b).hi) }
#define VeciMul256(a, b)      (v256u){ VeciMul   ((a).lo, (b).lo), VeciMul   ((a).hi, (b).hi) }
#define VeciDiv256(a, b)      (v256u){ VeciDiv   ((a).lo, (b).lo), VeciDiv   ((a).hi, (b).hi) }

#define VecLoadI256(ptr)      (v256u){ VecLoadI(ptr), VecLoadI((char*)(ptr) + 16) }
#define VecStoreI256(ptr, x)  do { VecStoreI(ptr, (x).lo); VecStoreI((char*)(ptr) + 16, (x).hi); } while(0)
#define VeciSet1_256(x)       (v256u){ VeciSet1(x)      , VeciSet1(x)                     }
#define VecSetBytes256(x)     (v256u){ VecSetBytes(x)   , VecSetBytes(x)                  }

#endif
typedef v128f f4;
typedef v128i i4;

// shared 
purefn f4 VCALL Vec3DistV    (f4 a, f4 b) { f4 x = VecSub(a, b); return Vec3LenV(x);  } 
purefn f32 VCALL Vec3DistfV   (f4 a, f4 b) { f4 x = VecSub(a, b); return Vec3LenfV(x); } 
purefn f32 VCALL Vec3DistSqrfV(f4 a, f4 b) { f4 x = VecSub(a, b); return Vec3DotfV(x, x); } 

purefn v128f VCALL VecClamp(v128f v, v128f vmin, v128f vmax)
{
    v = VecSelect(v, vmax, VecCmpGt(v, vmax));
    return VecSelect(v, vmin, VecCmpLt(v, vmin));
}

purefn u32 VCALL VecMaxElement(v128f a)
{
    v128f t = VecSwapPairs(a);
    v128f m = VecMax(a, t);
    t = VecSwapHalves(m);
    v128f max_val = VecMax(m, t);
    u32 mask = (u32)VecMovemask(VecCmpGe(a, max_val));
    return TrailingZeroCount32(mask);
}

#if defined(AX_SUPPORT_SSE) || defined(AX_ARM)
purefn float VCALL VecGetN(v128f v, int n)
{
    return ((float*)&v)[n & 3];
}

purefn v128f VCALL VecSetN(v128f v, int n, float f)
{
    ((float*)&v)[n & 3] = f;
    return v;
}

purefn int VCALL VeciGetN(v128u v, int n)
{
    return ((int*)&v)[n & 3];
}

purefn v128u VCALL VeciSetN(v128u v, int n, int i)
{
    ((int*)&v)[n & 3] = i;
    return v;
}
#endif 

#if defined(__cplusplus)
}
#endif

#endif // simd.h