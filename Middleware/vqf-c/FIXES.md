# VQF-C float baseline fixes (local)

Audited against design doc §3 before Full VQF fixed-point work. Upstream:
[DusKing1/vqf-c](https://github.com/DusKing1/vqf-c). Float sources remain for
PC replay / reference; firmware default path is `Middleware/vqf_fixed/` (not linked).

| Site | Bug | Fix |
|------|-----|-----|
| `setBiasEstimate` | `memcpy(..., sizeof(bias))` copies pointer width (4 B), not 3 floats | `sizeof(state.bias)` |
| `setRestBiasEstEnabled` | `fill(restLastSquaredDeviations, 3, …)` but array length 2 | length **2** (already applied) |
| `resetState` | loop wrote 18 NaNs into `restLastGyrLp[3]` (OOB) | removed; init `motionBiasEstRLpState` / `motionBiasEstBiasLpState` (already applied) |
| `resetState` / rest | `restLastSquaredDeviations` fill length 3 | length **2** (already applied) |
| `setTauAcc` | `newA[3]` + `memcpy(..., sizeof(newA))` into `accLpA[2]` | `newA[2]` + `sizeof(coeffs.accLpA)` |
| `initVqf` | `init_params()` never called (BSS-zero params) | call `init_params()` (already applied) |

## NaN filter init

Float `filterVec` still uses NaN sentinels for the tau-average init phase (matches
upstream C++). The fixed-point port (`vqf_fixed`) uses explicit
`initialized` / `init_count` / `init_sum` instead — no NaN on the MCU path.

## Honesty

These fixes make the float tree a safer replay baseline. They do **not** by
themselves validate fixed-point Kalman accuracy; that needs
`tools/vqf_fixed_replay/` float vs fixed comparison.
