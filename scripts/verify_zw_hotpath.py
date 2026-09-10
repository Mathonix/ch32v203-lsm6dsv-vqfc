#!/usr/bin/env python3
"""Verify VQF 1 kHz hot path symbols live in zero-wait Flash (< 0x8000)."""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ZW_LIMIT = 0x8000

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
    "platform_i2c_read",
    "platform_i2c_write",
    "vqf_sample_step",
    "vqf_run_1khz",
]

HOT_FUNCS = ("updateGyr", "updateAcc", "vqf_sample_step", "filterVec", "quatMultiply", "quatRotate", "norm", "normalize", "matrix3Multiply", "sinf", "_sinf", "cosf", "_cosf", "sqrt", "acos", "__kernel_sinf", "__kernel_cosf", "__rem_pio2f")


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


def section_sizes(elf: Path) -> tuple[int, int]:
    out = run(["riscv64-unknown-elf-size", "-A", str(elf)])
    sizes = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith("."):
            try:
                sizes[parts[0]] = int(parts[1])
            except ValueError:
                pass
    zw = sizes.get(".init", 0) + sizes.get(".vector", 0) + sizes.get(".text_zw", 0)
    nzw = sizes.get(".text_nzw", 0) + sizes.get(".text", 0) + sizes.get(".fini", 0)
    return zw, nzw


def jal_targets(elf: Path, func: str) -> list[tuple[int, str]]:
    dump = run(["riscv64-unknown-elf-objdump", "-d", str(elf)])
    lines = dump.splitlines()
    # Find function start
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
            # skip intra-function local labels like .Lxxx / offset-only
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
    for name in REQUIRED:
        if name not in syms:
            errors.append(f"MISSING symbol: {name}")
            print(f"{name:<28} {'MISSING':>10}")
            continue
        addr = syms[name]
        zone = "ZW" if addr < ZW_LIMIT else "NZW"
        print(f"{name:<28} 0x{addr:08x}  {zone}")
        if addr >= ZW_LIMIT:
            errors.append(f"{name} @ 0x{addr:x} >= 0x8000")

    print("\n=== jal targets from hot funcs (must be < 0x8000) ===")
    for func in HOT_FUNCS:
        if func not in syms:
            continue
        bad = []
        for addr, name in jal_targets(elf, func):
            # strip +offset from names like foo+0x10
            base = name.split("+", 1)[0]
            if addr >= ZW_LIMIT:
                bad.append((addr, name))
        status = "OK" if not bad else "FAIL"
        print(f"{func}: {status}")
        for addr, name in bad:
            print(f"  jal -> 0x{addr:08x} <{name}>")
            errors.append(f"{func} jal to {name} @ 0x{addr:x}")

    zw, nzw = section_sizes(elf)
    print(f"\n=== Flash usage ===")
    print(f"ZW  (init+vector+text_zw): {zw} / 32768 bytes ({100.0*zw/32768:.1f}%)")
    print(f"NZW (text_nzw+text+fini):  {nzw} / 196608 bytes ({100.0*nzw/196608:.1f}%)")
    if zw > 32768:
        errors.append(f"ZW overflow: {zw} > 32768")

    if errors:
        print("\nVERIFY FAILED:")
        for e in errors:
            print(" -", e)
        return 1
    print("\nVERIFY OK: all hot symbols and jal targets are in zero-wait Flash.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
