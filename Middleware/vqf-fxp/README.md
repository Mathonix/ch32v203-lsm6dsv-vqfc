# vqf-fxp — fixed-point VQF-structured 6DOF

**Not** a full Laidig / dusking1 VQF fixed-point port. This module implements a
**practical VQF-structured** filter for the 1 kHz CH32V203 hot path:

- `gyrQuat` strapdown integration (bias-corrected gyro)
- inertial-frame accel LP + `accQuat` inclination correction (VQF-style)
- light gyro bias pull from tilt residual
- `quat6D = accQuat ⊗ gyrQuat`
- fixed-point Euler → millidegrees

## Q-format

- Quaternions: **Q30**
- Rates / accel: **Q16**
- All hot MACs: `int64_t` products (`mul`/`mulh` on `rv32imac`)

## API

See `vqf_fxp.h`. Wired as `ALGO_VQF` via `Middleware/fusion/fusion_vqf.c`;
`User/main.c` calls `vqfx_*` directly on the default path (no float).
