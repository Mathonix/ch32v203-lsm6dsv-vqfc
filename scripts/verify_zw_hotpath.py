#!/usr/bin/env python3
"""Verify vqf_fixed 4 kHz gyr / 1 kHz acc hot path symbols live in HOT_RAM (SRAM @ 0x2000xxxx).

Also reports ALGO_RAM placement for Mahony/Comp and that soft-float/libm are
not in zero-wait Flash. Int64 div helpers may remain in ZW (Option B).
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ZW_LIMIT = 0x8000
HOT_RAM_BASE = 0x20000000
HOT_RAM_END = 0x20001000
ALGO_RAM_BASE = 0x20001000
ALGO_RAM_END = 0x20002000
RAM_BASE = 0x20002000
RAM_END = 0x20002800

REQUIRED = [
    "vqf_fixed_update_gyr_f25",
    "vqf_fixed_update_acc_f27",
    "vqf_fixed_get_quat6d_f30",
    "vqf_fixed_get_euler_mdeg",
    "lsm6dsv_read_acc_gyr_fixed",
    "lsm6dsv_read_gyr_fixed",
    "lsm6dsv_read_acc_fixed",
    "lsm6dsv_read_status",
    "platform_spi_xfer",
    "platform_lsm_cs",
    "fusion_run_1khz",
    "platform_uart_write_bytes",
    "platform_uart_send_euler_i16",
    "platform_can_send_euler_i16",
    "platform_millis",
]

# Soft-float / libm must NOT appear in ZW on this branch (default path is fxp)
FORBIDDEN_IN_ZW = [
    "__addsf3",
    "__mulsf3",
    "__divsf3",
    "atan2f",
    "asinf",
    "sinf",
    "cosf",
    "sqrtf",
    "sqrt",
    "acos",
]

HOT_FUNCS = (
    "vqf_fixed_update_gyr_f25",
    "vqf_fixed_update_acc_f27",
    "fusion_run_1khz",
    "lsm6dsv_read_acc_gyr_fixed",
    "platform_uart_send_euler_i16",
    "platform_can_send_euler_i16",
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


def zone_of(addr: int) -> str:
    if HOT_RAM_BASE <= addr < HOT_RAM_END:
        return "HOT_RAM"
    if ALGO_RAM_BASE <= addr < ALGO_RAM_END:
        return "ALGO_RAM"
    if RAM_BASE <= addr < RAM_END:
        return "RAM"
    if addr < ZW_LIMIT:
        return "ZW"
    return "NZW"


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} firmware.elf", file=sys.stderr)
        return 2
    elf = Path(sys.argv[1])
    syms = parse_nm(elf)
    errors: list[str] = []

    print("=== Hot symbol addresses (must be in HOT_RAM [0x20000000, 0x20001000)) ===")
    print(f"{'symbol':<32} {'addr':>10}  zone")
    # Updates + IMU/IO must be HOT_RAM; getters may be ZW Flash (HOT_RAM budget).
    HOT_REQUIRED = {
        "vqf_fixed_update_gyr_f25",
        "vqf_fixed_update_acc_f27",
        "lsm6dsv_read_acc_gyr_fixed",
        "lsm6dsv_read_gyr_fixed",
        "lsm6dsv_read_acc_fixed",
        "lsm6dsv_read_status",
        "platform_spi_xfer",
        "platform_lsm_cs",
        "fusion_run_1khz",
        "platform_uart_write_bytes",
        "platform_uart_send_euler_i16",
        "platform_can_send_euler_i16",
        "platform_millis",
    }
    for name in REQUIRED:
        if name not in syms:
            errors.append(f"MISSING symbol: {name}")
            print(f"{name:<32} {'MISSING':>10}")
            continue
        addr = syms[name]
        zone = zone_of(addr)
        print(f"{name:<32} 0x{addr:08x}  {zone}")
        if name in HOT_REQUIRED:
            if not (HOT_RAM_BASE <= addr < HOT_RAM_END):
                errors.append(f"{name} @ 0x{addr:x} not in HOT_RAM")
        else:
            if not (HOT_RAM_BASE <= addr < HOT_RAM_END or addr < ZW_LIMIT):
                errors.append(f"{name} @ 0x{addr:x} not in HOT_RAM/ZW")

    print("\n=== Integer helpers (divdi3) — ZW OK (Option B) ===")
    for name in ("__divdi3", "__udivdi3", "__umoddi3"):
        if name in syms:
            addr = syms[name]
            zone = zone_of(addr)
            print(f"{name:<32} 0x{addr:08x}  {zone}")
            # Prefer ZW or HOT_RAM; NZW would be a regression for hot path
            if addr >= ZW_LIMIT and not (HOT_RAM_BASE <= addr < HOT_RAM_END):
                errors.append(f"{name} @ 0x{addr:x} not in ZW/HOT_RAM")

    print("\n=== Soft-float / libm must not be in ZW ===")
    for name in FORBIDDEN_IN_ZW:
        if name not in syms:
            print(f"{name:<32} (not linked)")
            continue
        addr = syms[name]
        zone = zone_of(addr)
        print(f"{name:<32} 0x{addr:08x}  {zone}")
        if addr < ZW_LIMIT:
            errors.append(f"{name} still in ZW @ 0x{addr:x}")

    print("\n=== ALGO_RAM symbols (must be in [0x20001000, 0x20002000)) ===")
    for name in ALGO_RAM_SYMS:
        if name not in syms:
            errors.append(f"MISSING algo symbol: {name}")
            print(f"{name:<32} {'MISSING':>10}")
            continue
        addr = syms[name]
        ok = ALGO_RAM_BASE <= addr < ALGO_RAM_END
        print(f"{name:<32} 0x{addr:08x}  {'ALGO_RAM' if ok else 'BAD'}")
        if not ok:
            errors.append(f"{name} @ 0x{addr:x} not in ALGO_RAM")

    print("\n=== jal targets from hot funcs (HOT_RAM or ZW helpers OK) ===")
    print("(indirect calls via fusion_active are OK — not checked as jal)")
    for func in HOT_FUNCS:
        if func not in syms:
            continue
        bad = []
        for addr, name in jal_targets(elf, func):
            in_hot = HOT_RAM_BASE <= addr < HOT_RAM_END
            in_zw = addr < ZW_LIMIT
            # NZW / unexpected SRAM data region is bad for 1 kHz
            if not (in_hot or in_zw):
                bad.append((addr, name))
        status = "OK" if not bad else "FAIL"
        print(f"{func}: {status}")
        for addr, name in bad:
            print(f"  jal -> 0x{addr:08x} <{name}>")
            errors.append(f"{func} jal to {name} @ 0x{addr:x}")

    sizes = section_sizes(elf)
    zw = sizes.get(".init", 0) + sizes.get(".vector", 0) + sizes.get(".text_zw", 0)
    hot = sizes.get(".text.hot_ram", 0)
    nzw = sizes.get(".text_nzw", 0) + sizes.get(".text", 0) + sizes.get(".fini", 0)
    algo = sizes.get(".algo_ram", 0)
    data = sizes.get(".data", 0)
    bss = sizes.get(".bss", 0)
    stack = sizes.get(".stack", 0)
    print(f"\n=== Flash / RAM usage ===")
    print(f"ZW  (init+vector+text_zw): {zw} / 32768 bytes ({100.0 * zw / 32768:.1f}%)")
    print(f"HOT_RAM (.text.hot_ram):   {hot} / 4096 bytes ({100.0 * hot / 4096:.1f}%)")
    print(f"NZW (text_nzw+text+fini):  {nzw} / 192512 bytes ({100.0 * nzw / 192512:.1f}%)")
    print(f"ALGO_RAM (.algo_ram):      {algo} / 4096 bytes ({100.0 * algo / 4096:.1f}%)")
    print(f"RAM data+bss+stack:        {data}+{bss}+{stack} / 2048 bytes")
    if zw > 32768:
        errors.append(f"ZW overflow: {zw} > 32768")
    if hot > 4096:
        errors.append(f"HOT_RAM overflow: {hot} > 4096")
    if algo > 4096:
        errors.append(f"ALGO_RAM overflow: {algo} > 4096")
    if data + bss + stack > 2048:  # stack sized in link.ld (768)
        errors.append(f"RAM overflow: data+bss+stack {data+bss+stack} > 2048")

    dump = run(["riscv64-unknown-elf-objdump", "-d", str(elf)])
    mul_n = len(re.findall(r"\bmul\b", dump))
    mulh_n = len(re.findall(r"\bmulh\b", dump))
    div_n = len(re.findall(r"\bdiv\b", dump))
    print(f"\n=== RV M insn counts (whole image) ===")
    print(f"mul={mul_n} mulh={mulh_n} div={div_n}")

    if errors:
        print("\nVERIFY FAILED:")
        for e in errors:
            print(" -", e)
        return 1
    print("\nVERIFY OK: vqf_fixed updates in HOT_RAM; getters/helpers ZW OK; Mahony/Comp in ALGO_RAM.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
