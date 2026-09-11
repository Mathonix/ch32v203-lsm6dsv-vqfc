# vqf_fixed replay (stub)

Offline **float vs fixed** comparison harness (design §18). Not wired into CI yet.

## Intent

1. Feed the same IMU stream to:
   - float `Middleware/vqf-c` (after FIXES.md patches)
   - fixed `Middleware/vqf_fixed`
2. Log quat6d / bias / restDetected each sample.
3. On PC (double): geodesic angle error `2*acos(|dot(qf,qx)|)`.

## Acceptance (design)

- RMS angle error vs float &lt; 0.02 deg, p99 &lt; 0.05 deg (engineering target).
- Until this harness exists, treat bias Kalman accuracy as **unverified**.

## Suggested layout (future)

```
tools/vqf_fixed_replay/
  host_main.c          # link both trees or call via FFI
  sample.csv           # t, gx,gy,gz, ax,ay,az
  compare.py           # error stats
```

Regenerate IIR constants with `scripts/gen_vqf_fixed_coeffs.py` when rates/tau change.
