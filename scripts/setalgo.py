#!/usr/bin/env python3
"""Patch algo_cfg flag into a firmware .bin (or print OpenOCD poke commands).

Usage:
  python3 scripts/setalgo.py --id 1 build/firmware.bin
  python3 scripts/setalgo.py --id 0 --print-openocd

Flag layout @ CPU 0x00037000 / FPEC 0x08037000:
  magic=0x414C474F version=1 algo_id checksum(=magic^version^algo_id)
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

MAGIC = 0x414C474F
VERSION = 1
ADDR_CPU = 0x00037000
ADDR_FPEC = 0x08037000


def pack_cfg(algo_id: int) -> bytes:
    if algo_id not in (0, 1, 2):
        raise ValueError("algo_id must be 0..2")
    checksum = MAGIC ^ VERSION ^ algo_id
    return struct.pack("<IIII", MAGIC, VERSION, algo_id, checksum)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--id", type=int, required=True, help="0=VQF 1=Mahony 2=complementary")
    ap.add_argument("--print-openocd", action="store_true", help="print mww commands only")
    ap.add_argument("bin", nargs="?", type=Path, help="firmware.bin to patch at offset 0x37000")
    args = ap.parse_args()
    blob = pack_cfg(args.id)
    words = struct.unpack("<IIII", blob)

    if args.print_openocd:
        print(f"# algo_id={args.id} → FPEC 0x{ADDR_FPEC:08X}")
        print("# Erase 4K page first (device-specific), then:")
        for i, w in enumerate(words):
            print(f"mww 0x{ADDR_FPEC + i * 4:08X} 0x{w:08X}")
        return 0

    if args.bin is None:
        print("bin path required unless --print-openocd", file=sys.stderr)
        return 2

    data = bytearray(args.bin.read_bytes())
    # Image is linked from 0x0; offset == CPU address
    off = ADDR_CPU
    if len(data) < off + 16:
        # Extend with 0xFF (erased flash) up to page
        data.extend(b"\xFF" * (off + 16 - len(data)))
    data[off : off + 16] = blob
    args.bin.write_bytes(data)
    print(f"Patched {args.bin} @ offset 0x{off:X}: algo_id={args.id} {blob.hex()}")
    print("Flash the bin (ensure page 0x37000 is included) or use UART SETALGO / OpenOCD.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
