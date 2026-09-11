#!/usr/bin/env python3
"""Verify VQF 1 kHz hot path symbols live in zero-wait Flash (< 0x8000).

Also reports ALGO_RAM placement for Mahony/Comp (must be in SRAM @ 0x20000000+).
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ZW_LIMIT = 0x8000
ALGO_RAM_BASE = 0x20000000
ALGO_RAM_END = 0x20001000

REQUIRED = [
    "updateGyr",
    "updateAcc",
    "getQuat6D",
    "quatMultiply",
    "quatRotate",
    "filterVec",
    "norm",
    "normalize",
    "matrix3Multiply",
    "filterCoeffs",
    "gainFromTau",
    "lsm6dsv_read_acc_gyr",
    "platform_spi_xfer",
    "platform_lsm_cs",
    "fusion_sample_step",
    "fusion_run_1khz",
    "platform_uart_write_bytes",
    "platform_uart_send_euler_bin",
    "platform_can_send_euler",
    "atan2f",
    "asinf",
]

# VQF fusion wrappers should also be ZW
REQUIRED_VQF_WRAP = [
    "vqf_fusion_update",
    "vqf_fusion_get_quat",
]

HOT_FUNCS = (
    "updateGyr",
    "updateAcc",
    "fusion_sample_step",
    "fusion_run_1khz",
    "filterVec",
    "quatMultiply",
    "quatRotate",
    "norm",
    "normalize",
    "matrix3Multiply",
    "sinf",
    "_sinf",
    "cosf",
    "_cosf",
    "sqrt",
    "acos",
    "asinf",
    "atan2f",
    "atanf",
    "__kernel_sinf",
    "__kernel_cosf",
    "__rem_pio2f",
    "platform_uart_write_bytes",
    "platform_uart_send_euler_bin",
    "platform_can_send_euler",
    "vqf_fusion_update",
    "vqf_fusion_get_quat",
)

ALGO_RAM_SYMS = [
    "mahony_init",
    "mahony_update",
    "mahony_get_quat",
    "comp_init",
    "comp_update",
    "comp_get_quat",
]


def run(cmd: list[str]) -> str:
    return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT)


def parse_nm(elf: Path) -> dict[str, int]:
    out = run(["riscv64-unknown-elf-nm", "-n", str(elf)])
    syms: dict[str, int] = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        try:
            addr = int(parts[0], 16)
        except ValueError:
            continue
        name = parts[-1]
        syms[name] = addr
    return syms


def section_sizes(elf: Path) -> dict[str, int]:
    out = run(["riscv64-unknown-elf-size", "-A", str(elf)])
    sizes: dict[str, int] = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith("."):
            try:
                sizes[parts[0]] = int(parts[1])
            except ValueError:
                pass
    return sizes


def jal_targets(elf: Path, func: str) -> list[tuple[int, str]]:
    dump = run(["riscv64-unknown-elf-objdump", "-d", str(elf)])
    lines = dump.splitlines()
    start_re = re.compile(rf"^[0-9a-f]+ <{re.escape(func)}>:")
    label_re = re.compile(r"^[0-9a-f]+ <([^>]+)>:")
    jal_re = re.compile(r"\bj(?:al)?\s+\w+,([0-9a-f]+)\s+<([^>]+)>")
    jal_re2 = re.compile(r"\bj(?:al)?\s+([0-9a-f]+)\s+<([^>]+)>")
    collecting = False
    hits: list[tuple[int, str]] = []
    for line in lines:
        if start_re.match(line):
            collecting = True
            continue
        if collecting and label_re.match(line):
            break
        if not collecting:
            continue
        m = jal_re.search(line) or jal_re2.search(line)
        if m:
            addr = int(m.group(1), 16)
            name = m.group(2)
            if name.startswith(".") or name.startswith(func):
                continue
            hits.append((addr, name))
    return hits


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} firmware.elf", file=sys.stderr)
        return 2
    elf = Path(sys.argv[1])
    syms = parse_nm(elf)
    errors: list[str] = []

    print("=== Hot symbol addresses (must be < 0x8000) ===")
    print(f"{'symbol':<28} {'addr':>10}  zone")
    for name in REQUIRED + REQUIRED_VQF_WRAP:
        if name not in syms:
            # static wrappers may be local — try with prefix or skip soft fail
            errors.append(f"MISSING symbol: {name}")
            print(f"{name:<28} {'MISSING':>10}")
            continue
        addr = syms[name]
        zone = "ZW" if addr < ZW_LIMIT else "NZW"
        print(f"{name:<28} 0x{addr:08x}  {zone}")
        if addr >= ZW_LIMIT:
            errors.append(f"{name} @ 0x{addr:x} >= 0x8000")

    print("\n=== ALGO_RAM symbols (must be in [0x20000000, 0x20001000)) ===")
    for name in ALGO_RAM_SYMS:
        if name not in syms:
            errors.append(f"MISSING algo symbol: {name}")
            print(f"{name:<28} {'MISSING':>10}")
            continue
        addr = syms[name]
        ok = ALGO_RAM_BASE <= addr < ALGO_RAM_END
        print(f"{name:<28} 0x{addr:08x}  {'ALGO_RAM' if ok else 'BAD'}")
        if not ok:
            errors.append(f"{name} @ 0x{addr:x} not in ALGO_RAM")

    print("\n=== jal targets from hot funcs (must be < 0x8000) ===")
    print("(indirect calls via fusion_active are OK — not checked as jal)")
    for func in HOT_FUNCS:
        if func not in syms:
            continue
        bad = []
        for addr, name in jal_targets(elf, func):
            if addr >= ZW_LIMIT:
                # Allow calls into ALGO_RAM only from fusion_sample_step via jalr — jal is absolute
                bad.append((addr, name))
        status = "OK" if not bad else "FAIL"
        print(f"{func}: {status}")
        for addr, name in bad:
            print(f"  jal -> 0x{addr:08x} <{name}>")
            errors.append(f"{func} jal to {name} @ 0x{addr:x}")

    sizes = section_sizes(elf)
    zw = sizes.get(".init", 0) + sizes.get(".vector", 0) + sizes.get(".text_zw", 0)
    nzw = sizes.get(".text_nzw", 0) + sizes.get(".text", 0) + sizes.get(".fini", 0)
    algo = sizes.get(".algo_ram", 0)
    print(f"\n=== Flash / RAM usage ===")
    print(f"ZW  (init+vector+text_zw): {zw} / 32768 bytes ({100.0 * zw / 32768:.1f}%)")
    print(f"NZW (text_nzw+text+fini):  {nzw} / 192512 bytes ({100.0 * nzw / 192512:.1f}%)")
    print(f"ALGO_RAM (.algo_ram):      {algo} / 4096 bytes ({100.0 * algo / 4096:.1f}%)")
    if zw > 32768:
        errors.append(f"ZW overflow: {zw} > 32768")
    if algo > 4096:
        errors.append(f"ALGO_RAM overflow: {algo} > 3072")

    if errors:
        print("\nVERIFY FAILED:")
        for e in errors:
            print(" -", e)
        return 1
    print("\nVERIFY OK: VQF hot path in ZW; Mahony/Comp in ALGO_RAM.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
