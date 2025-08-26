#include <immintrin.h>

inline __m256 vsplat(float x) {
    return _mm256_set_ps(x, x, x, x, x, x, x, x);
}

inline __m256 vdistribute(const float* v, const size_t* i) {
    return _mm256_set_ps(v[i[7]], v[i[6]], v[i[5]], v[i[4]], v[i[3]], v[i[2]], v[i[1]], v[i[0]]);
}

inline __m256 vdot(__m256 ax, __m256 ay, __m256 bx, __m256 by) {
    __m256 result = _mm256_mul_ps(ax, bx);
    result = _mm256_fmadd_ps(ay, by, result);
    return result;
}

inline __m256 vcross(__m256 ax, __m256 ay, __m256 bx, __m256 by) {
    __m256 result = _mm256_mul_ps(ax, by);
    result = _mm256_fnmadd_ps(ay, bx, result);
    return result;
}
