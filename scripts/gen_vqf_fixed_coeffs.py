#!/usr/bin/env python3
"""Generate F30 VQF fixed-point filter/gain headers (design appendix B).

Writes Middleware/vqf_fixed/vqf_fixed_coeffs_{1k1k,4k1k}.h
Usage:
  python3 scripts/gen_vqf_fixed_coeffs.py
  python3 scripts/gen_vqf_fixed_coeffs.py --print-only
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

F30 = 1 << 30
ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "Middleware" / "vqf_fixed"


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


def emit_1k1k() -> str:
    Bacc, Aacc = filter_coeffs(3.0, 0.001)
    Brest, Arest = filter_coeffs(0.5, 0.001)
    lines = [
        "/**",
        " * PC-generated F30 constants: gyr=1 kHz, acc=1 kHz, tauAcc=3s, restTau=0.5s.",
        " * Regenerate via scripts/gen_vqf_fixed_coeffs.py — do not hand-edit.",
        " */",
        "#ifndef VQF_FIXED_COEFFS_1K1K_H",
        "#define VQF_FIXED_COEFFS_1K1K_H",
        "",
        "#include <stdint.h>",
        "",
        "/* dt/2 @ 1 kHz → F30 */",
        f"#define VQF_C1K_DT_HALF_F30          ((int32_t){q30(0.0005)})",
        "",
        "/* acc LP tau=3s @ 1kHz */",
        f"#define VQF_C1K_ACC_LP_B0            ((int32_t){q30(Bacc[0])})",
        f"#define VQF_C1K_ACC_LP_B1            ((int32_t){q30(Bacc[1])})",
        f"#define VQF_C1K_ACC_LP_B2            ((int32_t){q30(Bacc[2])})",
        f"#define VQF_C1K_ACC_LP_A1            ((int32_t){q30(Aacc[0])})",
        f"#define VQF_C1K_ACC_LP_A2            ((int32_t){q30(Aacc[1])})",
        "#define VQF_C1K_ACC_LP_INIT_SAMPLES  (3000u) /* round(tau/Ts) */",
        "",
        "/* rest gyr/acc LP tau=0.5s @ 1kHz (same coeffs) */",
        f"#define VQF_C1K_REST_LP_B0           ((int32_t){q30(Brest[0])})",
        f"#define VQF_C1K_REST_LP_B1           ((int32_t){q30(Brest[1])})",
        f"#define VQF_C1K_REST_LP_B2           ((int32_t){q30(Brest[2])})",
        f"#define VQF_C1K_REST_LP_A1           ((int32_t){q30(Arest[0])})",
        f"#define VQF_C1K_REST_LP_A2           ((int32_t){q30(Arest[1])})",
        "#define VQF_C1K_REST_LP_INIT_SAMPLES (500u)",
        "",
        "/* thresholds / clips from design §6 */",
        "#define VQF_C1K_BIAS_CLIP_F29        ((int32_t)18740330)   /* 2 deg/s */",
        "#define VQF_C1K_BIAS_CLIP_F25        ((int32_t)1171271)    /* same physical, F25 */",
        "#define VQF_C1K_REST_TH_GYR_F25      ((int32_t)1171271)",
        "#define VQF_C1K_REST_TH_GYR2_Q50     (1371875755441ull)",
        "#define VQF_C1K_REST_TH_ACC_F27      ((int32_t)6843200)    /* 0.5 m/s² in g */",
        "#define VQF_C1K_REST_TH_ACC2_Q54     (46829386240000ull)",
        "#define VQF_C1K_REST_MIN_SAMPLES     (1500u)               /* restMinT=1.5s @1kHz */",
        "",
        "/* Bias Kalman: P F18, V F18, W U64/F8 */",
        "#define VQF_C1K_BIAS_P0_F18          ((int32_t)655360000) /* 2500.0 */",
        "#define VQF_C1K_BIAS_V_F18           ((int32_t)262)       /* 0.001 */",
        "#define VQF_C1K_BIAS_W_REST_F8       (20738304ull)        /* 81009 */",
        "#define VQF_C1K_BIAS_W_MOTION_F8     (2560025600ull)      /* 10000100 */",
        "#define VQF_C1K_BIAS_W_VERT_F8       (25600256000000ull)  /* 100001000000 */",
        "",
        "/* 1/Ts for motion e: Ts=0.001 → 1000 */",
        "#define VQF_C1K_INV_ACC_TS           (1000)",
        "",
        "#endif /* VQF_FIXED_COEFFS_1K1K_H */",
        "",
    ]
    return "\n".join(lines)


def emit_4k1k() -> str:
    Bgyr, Agyr = filter_coeffs(0.5, 0.00025)
    lines = [
        "/**",
        " * PC-generated F30 constants: gyr=4 kHz, acc=1 kHz (design default).",
        " * Acc-side / Kalman / rest-acc coeffs live in vqf_fixed_coeffs_1k1k.h.",
        " * Regenerate via scripts/gen_vqf_fixed_coeffs.py — do not hand-edit.",
        " * Selected at runtime when vqf_fixed_config_t.gyr_hz >= 3500.",
        " */",
        "#ifndef VQF_FIXED_COEFFS_4K1K_H",
        "#define VQF_FIXED_COEFFS_4K1K_H",
        "",
        "#include <stdint.h>",
        "",
        f"#define VQF_C4K_DT_HALF_F30          ((int32_t){q30(0.000125)}) /* 0.000125 */",
        "",
        "/* rest gyr LP tau=0.5s @ 4 kHz */",
        f"#define VQF_C4K_REST_GYR_B0          ((int32_t){q30(Bgyr[0])})",
        f"#define VQF_C4K_REST_GYR_B1          ((int32_t){q30(Bgyr[1])})",
        f"#define VQF_C4K_REST_GYR_B2          ((int32_t){q30(Bgyr[2])})",
        f"#define VQF_C4K_REST_GYR_A1          ((int32_t){q30(Agyr[0])})",
        f"#define VQF_C4K_REST_GYR_A2          ((int32_t){q30(Agyr[1])})",
        "#define VQF_C4K_REST_GYR_INIT_SAMPLES (2000u) /* 0.5/0.00025 */",
        "",
        "/* Mag path coeffs (100 Hz) — unused while VQF_FIXED_ENABLE_MAG=0 */",
        f"#define VQF_C4K_MAG_LP_B0            ((int32_t){q30(filter_coeffs(0.05, 0.01)[0][0])})",
        f"#define VQF_C4K_MAG_LP_B1            ((int32_t){q30(filter_coeffs(0.05, 0.01)[0][1])})",
        f"#define VQF_C4K_MAG_LP_B2            ((int32_t){q30(filter_coeffs(0.05, 0.01)[0][2])})",
        f"#define VQF_C4K_MAG_LP_A1            ((int32_t){q30(filter_coeffs(0.05, 0.01)[1][0])})",
        f"#define VQF_C4K_MAG_LP_A2            ((int32_t){q30(filter_coeffs(0.05, 0.01)[1][1])})",
        f"#define VQF_C4K_K_MAG_F30            ((int32_t){q30(gain_from_tau(9.0, 0.01))})",
        f"#define VQF_C4K_K_MAG_REF_F30        ((int32_t){q30(gain_from_tau(20.0, 0.01))})",
        "",
        "#endif /* VQF_FIXED_COEFFS_4K1K_H */",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--print-only", action="store_true")
    args = ap.parse_args()
    h1 = emit_1k1k()
    h4 = emit_4k1k()
    if args.print_only:
        print(h1)
        print(h4)
        return 0
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "vqf_fixed_coeffs_1k1k.h").write_text(h1)
    (OUT_DIR / "vqf_fixed_coeffs_4k1k.h").write_text(h4)
    print(f"wrote {OUT_DIR / 'vqf_fixed_coeffs_1k1k.h'}")
    print(f"wrote {OUT_DIR / 'vqf_fixed_coeffs_4k1k.h'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
