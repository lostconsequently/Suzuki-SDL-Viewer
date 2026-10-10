"""Generates the SDLView logo (sdlview.png 256 px + multi-size sdlview.ico) without external libraries.

Design: dark rounded tile, blue signal trace (data on the SDL line), green status dot.
Colours match the app palette. Run: python make_icon.py
"""

import math
import struct
import zlib
from pathlib import Path

BG = (0x14, 0x17, 0x1e)
BORDER = (0x2c, 0x32, 0x40)
TRACE = (0x4e, 0xa1, 0xff)
DOT = (0x3e, 0xcf, 0x8e)

# geometry in unit coordinates (0..1)
TILE_INSET, TILE_RADIUS, BORDER_W = 0.03, 0.22, 0.035
TRACE_POINTS = [(0.14, 0.62), (0.30, 0.62), (0.38, 0.34), (0.48, 0.80), (0.57, 0.44),
                (0.63, 0.62), (0.86, 0.62)]
TRACE_W = 0.085
DOT_C, DOT_R = (0.76, 0.28), 0.075


def rounded_rect_sd(x: float, y: float) -> float:
    """Signed distance to the rounded tile (negative inside)."""
    c, half = 0.5, 0.5 - TILE_INSET
    qx, qy = abs(x - c) - (half - TILE_RADIUS), abs(y - c) - (half - TILE_RADIUS)
    outside = math.hypot(max(qx, 0), max(qy, 0))
    return outside + min(max(qx, qy), 0) - TILE_RADIUS


def seg_dist(px: float, py: float, a: tuple, b: tuple) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def shade(x: float, y: float) -> tuple[float, float, float, float]:
    """Straight (non-premultiplied) RGBA in 0..1 for one sub-sample."""
    sd = rounded_rect_sd(x, y)
    if sd > 0:
        return 0.0, 0.0, 0.0, 0.0
    col = BORDER if sd > -BORDER_W else BG
    if math.hypot(x - DOT_C[0], y - DOT_C[1]) <= DOT_R:
        col = DOT
    elif min(seg_dist(x, y, a, b) for a, b in zip(TRACE_POINTS, TRACE_POINTS[1:])) <= TRACE_W / 2:
        col = TRACE
    return col[0] / 255, col[1] / 255, col[2] / 255, 1.0


def render(size: int, ss: int = 4) -> bytes:
    """RGBA bytes, anti-aliased by ss x ss supersampling."""
    out = bytearray()
    n = ss * ss
    for py in range(size):
        for px in range(size):
            r = g = b = a = 0.0
            for sy in range(ss):
                for sx in range(ss):
                    cr, cg, cb, ca = shade((px + (sx + 0.5) / ss) / size, (py + (sy + 0.5) / ss) / size)
                    r, g, b, a = r + cr * ca, g + cg * ca, b + cb * ca, a + ca
            if a > 0:
                out += bytes([round(r / a * 255), round(g / a * 255), round(b / a * 255), round(a / n * 255)])
            else:
                out += b"\x00\x00\x00\x00"
    return bytes(out)


def png(size: int, rgba: bytes) -> bytes:
    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + rgba[y * size * 4:(y + 1) * size * 4] for y in range(size))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def ico(images: list[tuple[int, bytes]]) -> bytes:
    """ICO with PNG-compressed entries (supported since Windows Vista)."""
    header = struct.pack("<HHH", 0, 1, len(images))
    offset = 6 + 16 * len(images)
    entries, blobs = b"", b""
    for size, data in images:
        entries += struct.pack("<BBBBHHII", size % 256, size % 256, 0, 0, 1, 32, len(data), offset)
        blobs += data
        offset += len(data)
    return header + entries + blobs


def main() -> None:
    here = Path(__file__).parent
    images = []
    for size in (256, 64, 48, 32, 24, 16):
        data = png(size, render(size, ss=3 if size >= 128 else 6))
        images.append((size, data))
        if size == 256:
            (here / "sdlview.png").write_bytes(data)
    (here / "sdlview.ico").write_bytes(ico(images))
    print("written:", here / "sdlview.png", here / "sdlview.ico")


if __name__ == "__main__":
    main()
