#!/usr/bin/env python3
"""
Convert eye material PNG files into a C header for dual_eye_render.cc.

Typical usage:
  python scripts/eye_asset_tools/convert_eye_assets.py ^
    --sclera my_sclera.png ^
    --iris my_iris.png ^
    --name my_eye ^
    --output main/display/my_eyes_data.h

Expected input sizes:
  sclera: 375x375
  iris:   80x512

The script outputs:
  const uint16_t <name>_sclera[375 * 375] = { ... };
  const uint16_t <name>_iris[80 * 512] = { ... };
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

from PIL import Image


SCLERA_SIZE = (375, 375)
IRIS_SIZE = (80, 512)


def rgb888_to_rgb565(r: int, g: int, b: int) -> int:
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def load_rgb565(path: Path, expected_size: tuple[int, int]) -> list[int]:
    img = Image.open(path).convert("RGB")
    if img.size != expected_size:
        raise ValueError(
            f"{path} size is {img.size}, expected {expected_size}. "
            "Please prepare the source image at the correct size first."
        )

    pixels: list[int] = []
    for r, g, b in img.getdata():
        pixels.append(rgb888_to_rgb565(r, g, b))
    return pixels


def format_c_array(name: str, width: int, height: int, values: Iterable[int]) -> str:
    values = list(values)
    lines = [f"const uint16_t {name}[{width} * {height}] = {{"]
    row: list[str] = []
    for idx, value in enumerate(values, start=1):
        row.append(f"0x{value:04X}")
        if len(row) == 12:
            lines.append("    " + ", ".join(row) + ",")
            row = []
    if row:
        lines.append("    " + ", ".join(row) + ",")
    lines.append("};")
    return "\n".join(lines)


def build_header(symbol_prefix: str, sclera_values: list[int], iris_values: list[int]) -> str:
    guard = f"{symbol_prefix.upper()}_EYES_DATA_H"
    sclera_symbol = f"{symbol_prefix}_sclera"
    iris_symbol = f"{symbol_prefix}_iris"
    return "\n".join(
        [
            f"#ifndef {guard}",
            f"#define {guard}",
            "",
            "#include <stdint.h>",
            "",
            format_c_array(sclera_symbol, SCLERA_SIZE[0], SCLERA_SIZE[1], sclera_values),
            "",
            format_c_array(iris_symbol, IRIS_SIZE[0], IRIS_SIZE[1], iris_values),
            "",
            f"#endif  // {guard}",
            "",
        ]
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert eye material PNGs to C arrays.")
    parser.add_argument("--sclera", type=Path, required=True, help="375x375 sclera PNG path")
    parser.add_argument("--iris", type=Path, required=True, help="80x512 iris PNG path")
    parser.add_argument("--name", required=True, help="symbol prefix, e.g. my_eye")
    parser.add_argument("--output", type=Path, required=True, help="output .h path")
    args = parser.parse_args()

    sclera_values = load_rgb565(args.sclera, SCLERA_SIZE)
    iris_values = load_rgb565(args.iris, IRIS_SIZE)

    header_text = build_header(args.name, sclera_values, iris_values)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(header_text, encoding="utf-8", newline="\n")

    print(f"Generated: {args.output}")
    print(f"Use in dual_eye_render.cc:")
    print(f"  #define EYE_SCLERA_MATERIAL {args.name}_sclera")
    print(f"  #define EYE_IRIS_MATERIAL {args.name}_iris")


if __name__ == "__main__":
    main()
