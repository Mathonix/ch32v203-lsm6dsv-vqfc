# vqf_fixed replay (host)

Offline **float VQF-C vs vqf_fixed** comparison at design rates (**gyro 4 kHz / acc 1 kHz**).

## Build & run

```bash
# from repo root
make -C tools/vqf_fixed_replay
./tools/vqf_fixed_replay/build/vqf_fixed_replay --static --seconds 5
./tools/vqf_fixed_replay/build/vqf_fixed_replay --seconds 10

# or
make replay
```

Optional CSV (`t,gx,gy,gz,ax,ay,az` with gyr rad/s, acc m/s²), one row per gyro sample:

```bash
./tools/vqf_fixed_replay/build/vqf_fixed_replay --csv sample.csv
```

Host build defines `VQF_FIXED_HOST` so `FLASH_ZW` / `FLASH_NZW` attributes are no-ops.

## Measured numbers (this machine, gcc -O2)

| Scenario | Quat geodesic RMS | Max | Euler RMS (r/p/y) | vs 0.02° target |
|----------|-------------------|-----|-------------------|-----------------|
| `--static` 5 s (0 gyr, +1 g) | **0.000000°** | 0.000° | 0 / 0 / 0 | **PASS** |
| mild synthetic motion 10 s | **0.480°** | 1.36° | 0.151 / 0.455 / 0.009 | **below target** |

Design acceptance (Laidig-style): RMS &lt; 0.02°, p99 &lt; 0.05°. Static path matches; **motion / bias-Kalman fidelity is not yet within that envelope** — treat as engineering-grade, not bit-exact. Likely residual sources: scaled LDLT quantization, F18/F8 bias math, IIR init (mean vs float NaN-tau).

## What it links

- Float: `Middleware/vqf-c/vqf.c` (with local FIXES.md patches)
- Fixed: `Middleware/vqf_fixed/*.c` + generated `vqf_fixed_coeffs_{1k1k,4k1k}.h`

Regenerate coeffs after tau/rate changes:

```bash
python3 scripts/gen_vqf_fixed_coeffs.py
```
