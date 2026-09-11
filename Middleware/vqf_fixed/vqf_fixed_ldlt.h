#ifndef VQF_FIXED_LDLT_H
#define VQF_FIXED_LDLT_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Scaled 3x3 LDLT: solve A * X = B for 3 right-hand sides (rows of B as columns of X).
 * A is symmetric 3x3 in int64 common scale; on success writes K 3x3 in F30.
 *
 * @param A_sym  row-major 9 elems (only upper/lower used; must be SPD after scale)
 * @param B      3x3 row-major (P * R^T) in same numeric scale as A for the solve
 * @param K_f30  output Kalman gain F30
 * @return true on success; false → skip measurement update
 */
bool vqf_ldlt_solve_3x3_k(const int64_t A_sym[9], const int64_t B[9], int32_t K_f30[9]);

#endif
