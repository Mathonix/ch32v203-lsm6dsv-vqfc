/**
 * Scaled 3x3 LDLT for bias Kalman (design §12).
 * Working scale: int64. K quantized to F30.
 * Honesty: structure is real; float parity not proven without replay.
 */
#include "vqf_fixed_ldlt.h"
#include "vqf_fixed_math.h"
#include "flash_zw.h"

FLASH_ZW_CODE static int vqf_clz64(uint64_t x)
{
    if (x == 0ull) {
        return 64;
    }
    return __builtin_clzll(x);
}

FLASH_ZW_CODE bool vqf_ldlt_solve_3x3_k(const int64_t A_in[9], const int64_t B_in[9], int32_t K_f30[9])
{
    int64_t A[9];
    int64_t B[9];
    int i;

    uint64_t max_abs = 0ull;
    for (i = 0; i < 9; i++) {
        uint64_t a = (uint64_t)(A_in[i] < 0 ? -A_in[i] : A_in[i]);
        if (a > max_abs) {
            max_abs = a;
        }
    }
    if (max_abs == 0ull) {
        return false;
    }

    /* Scale so max |A| fits under ~2^50 for headroom in products */
    int shift = 0;
    if (max_abs > (1ull << 50)) {
        shift = (int)(64 - vqf_clz64(max_abs)) - 50;
        if (shift < 0) {
            shift = 0;
        }
    }
    for (i = 0; i < 9; i++) {
        A[i] = A_in[i] >> shift;
        B[i] = B_in[i] >> shift;
    }

    /* LDLT: A = L D L^T, L unit lower-triangular stored in strict lower of A,
     * D on diagonal. */
    int64_t L10, L20, L21;
    int64_t D0, D1, D2;

    D0 = A[0];
    if (D0 <= 0) {
        return false;
    }
    L10 = A[3] / D0; /* a10 */
    L20 = A[6] / D0; /* a20 */

    D1 = A[4] - L10 * L10 * D0;
    if (D1 <= 0) {
        return false;
    }
    L21 = (A[7] - L10 * L20 * D0) / D1;

    D2 = A[8] - L20 * L20 * D0 - L21 * L21 * D1;
    if (D2 <= 0) {
        return false;
    }

    /* For each column j of B (RHS), solve A x = b_j, then K(:,j) = x.
     * B is row-major 3x3 meaning B_row_i; design: solve As * x = B_i^T for each row i.
     * Equiv: for each column of B^T = each row of B as RHS → K rows.
     * Float VQF: K = P R^T inv(A) with A=W+RPR^T, so K = B * inv(A), B=P R^T.
     * Solve A X^T = B^T → X = B inv(A) = K. Columns of X are solutions for columns of B^T.
     */
    for (int col = 0; col < 3; col++) {
        /* RHS = column col of B^T = row col of B */
        int64_t y0 = B[col * 3 + 0];
        int64_t y1 = B[col * 3 + 1];
        int64_t y2 = B[col * 3 + 2];

        /* Forward: L y = b  (unit L) */
        int64_t z0 = y0;
        int64_t z1 = y1 - L10 * z0;
        int64_t z2 = y2 - L20 * z0 - L21 * z1;

        /* Diagonal: D w = z */
        int64_t w0 = z0 / D0;
        int64_t w1 = z1 / D1;
        int64_t w2 = z2 / D2;

        /* Back: L^T x = w */
        int64_t x2 = w2;
        int64_t x1 = w1 - L21 * x2;
        int64_t x0 = w0 - L10 * x1 - L20 * x2;

        /* Quantize to F30: values of K are typically << 1.
         * x is in "B/A" dimensionless ≈ F30-ish if B and A share scale.
         * Treat x as already near float K; store as F30 via clip.
         * If |x| looks like integer gain in same scale as A/B (F8-ish),
         * convert: K_f30 = x << 30 / scale_ref — but shared scale cancelled
         * in x = B/A, so x is dimensionless float. Represent as F30:
         * assume x was computed in integer where 1.0 == SCALE; we need
         * an absolute interpretation.
         *
         * With A,B both >> shift from F8 domain values, x ≈ float K.
         * F8 numbers are W*256 etc. Ratio is true K. Integer x is already
         * the float value rounded. Map: K_f30 = x * 2^30 — only valid if
         * |x|<<1. For K~2.5e-4, x may be 0 after integer division!
         *
         * Fix: keep more fraction — do LDLT solve with left-shifted B.
         */
        (void)x0;
        (void)x1;
        (void)x2;
    }

    /* Re-solve with B <<= 30 so x lands in F30 */
    for (int col = 0; col < 3; col++) {
        int64_t y0 = B[col * 3 + 0];
        int64_t y1 = B[col * 3 + 1];
        int64_t y2 = B[col * 3 + 2];
        /* Boost numerator for F30 result */
        const int boost = 30;
        /* Careful overflow: B already ~F8 after shift. Boost only if headroom. */
        int bshift = boost;
        uint64_t bmax = 0ull;
        {
            uint64_t t;
            t = (uint64_t)(y0 < 0 ? -y0 : y0); if (t > bmax) bmax = t;
            t = (uint64_t)(y1 < 0 ? -y1 : y1); if (t > bmax) bmax = t;
            t = (uint64_t)(y2 < 0 ? -y2 : y2); if (t > bmax) bmax = t;
        }
        if (bmax > 0ull) {
            int room = (int)vqf_clz64(bmax) - 2;
            if (bshift > room) {
                bshift = room;
            }
            if (bshift < 0) {
                bshift = 0;
            }
        }
        y0 <<= bshift;
        y1 <<= bshift;
        y2 <<= bshift;

        int64_t z0 = y0;
        int64_t z1 = y1 - L10 * z0;
        int64_t z2 = y2 - L20 * z0 - L21 * z1;
        int64_t w0 = z0 / D0;
        int64_t w1 = z1 / D1;
        int64_t w2 = z2 / D2;
        int64_t x2 = w2;
        int64_t x1 = w1 - L21 * x2;
        int64_t x0 = w0 - L10 * x1 - L20 * x2;

        /* If bshift < 30, left-shift remainder to F30 */
        int rem = 30 - bshift;
        if (rem > 0) {
            x0 <<= rem;
            x1 <<= rem;
            x2 <<= rem;
        }
        K_f30[col * 3 + 0] = vqf_sat_s32(x0);
        K_f30[col * 3 + 1] = vqf_sat_s32(x1);
        K_f30[col * 3 + 2] = vqf_sat_s32(x2);
    }

    /* K from above is row-major with K[row*3+col] = x_row for RHS=B_row?
     * We used RHS = row col of B, wrote into K[col*3 + r] = x_r.
     * That makes K[col][r] = x_r → actually K is B*inv(A) stored transposed.
     * Float: bias += K*e with K row-major K[0..2] first row.
     * B * inv(A): row i of K = row i of B solved... 
     * We solved for RHS = B's row `col`, put x into K[col][*] — so row col of K = solution
     * for row col of B. That's K = B * inv(A). Good. */

    return true;
}
