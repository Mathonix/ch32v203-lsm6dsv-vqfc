# vqf-fxp (deprecated wrappers)

Legacy Q16 API (`vqfx_*`) now forwards to **`Middleware/vqf_fixed/`** (Full VQF
fixed-point, design domains F25/F27/F29/F30).

Prefer:
- `lsm6dsv_read_acc_gyr_fixed()` → F27/F25
- `vqf_fixed_update_gyr_f25` / `vqf_fixed_update_acc_f27`
