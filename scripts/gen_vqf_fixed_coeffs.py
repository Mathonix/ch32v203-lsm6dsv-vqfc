#!/usr/bin/env python3
"""Generate F30 VQF fixed-point filter/gain constants (design appendix B)."""
from __future__ import annotations

import math
import sys

F30 = 1 << 30


def q30(x: float) -> int:
    return int(round(x * F30))


def filter_coeffs(tau: float, Ts: float) -> tuple[list[float], list[float]]:
    fc = (math.sqrt(2) / (2 * math.pi)) / tau
    C = math.tan(math.pi * fc * Ts)
    D = C * C + math.sqrt(2) * C + 1
    b0 = C * C / D
    B = [b0, 2 * b0, b0]
    A = [2 * (C * C - 1) / D, (1 - math.sqrt(2) * C + C * C) / D]
    return B, A


def gain_from_tau(tau: float, Ts: float) -> float:
    return 1.0 - math.exp(-Ts / tau)


def main() -> int:
    rows = [
        ("ACC_LP", 3.0, 0.001),
        ("REST_GYR_1k", 0.5, 0.001),
        ("REST_GYR_4k", 0.5, 0.00025),
        ("REST_ACC", 0.5, 0.001),
        ("MAG_LP", 0.05, 0.01),
    ]
    for name, tau, Ts in rows:
        B, A = filter_coeffs(tau, Ts)
        print(name, [q30(x) for x in B], [q30(x) for x in A])
    print("K_MAG", q30(gain_from_tau(9.0, 0.01)))
    print("K_MAG_REF", q30(gain_from_tau(20.0, 0.01)))
    print("DT_HALF_1k", q30(0.0005))
    print("DT_HALF_4k", q30(0.000125))
    return 0


if __name__ == "__main__":
    sys.exit(main())
