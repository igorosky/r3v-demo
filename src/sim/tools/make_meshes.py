#!/usr/bin/env python3
"""Writes the unit meshes the robot's visuals are made of, scaled to size in the URDF.

r3v-renderer draws only mesh visuals, so every visual is one of these with a `scale`, while the
collisions stay primitives for Gazebo:
- box.stl: a 1 x 1 x 1 cube centered at the origin (scale = size),
- cylinder.stl: radius 1, length 1 along z, centered (scale = radius radius length),
- sphere.stl: radius 1, centered (scale = radius radius radius).

Usage: make_meshes.py <output directory>
"""

import math
import struct
import sys
from pathlib import Path

SEGMENTS = 32
RINGS = 16


def write_stl(path, triangles):
    with open(path, "wb") as f:
        f.write(b"r3v_sim unit mesh".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(triangles)))
        for a, b, c in triangles:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            length = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(struct.pack("<3f", *(x / length for x in n)))
            for vertex in (a, b, c):
                f.write(struct.pack("<3f", *vertex))
            f.write(b"\0\0")


def quad(a, b, c, d):
    """Two counter-clockwise triangles of the quad a b c d."""
    return [(a, b, c), (a, c, d)]


def box():
    h = 0.5
    p = {(x, y, z): (x * h, y * h, z * h) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)}
    return (
        quad(p[1, -1, -1], p[1, 1, -1], p[1, 1, 1], p[1, -1, 1])
        + quad(p[-1, 1, -1], p[-1, -1, -1], p[-1, -1, 1], p[-1, 1, 1])
        + quad(p[1, 1, -1], p[-1, 1, -1], p[-1, 1, 1], p[1, 1, 1])
        + quad(p[-1, -1, -1], p[1, -1, -1], p[1, -1, 1], p[-1, -1, 1])
        + quad(p[-1, -1, 1], p[1, -1, 1], p[1, 1, 1], p[-1, 1, 1])
        + quad(p[-1, 1, -1], p[1, 1, -1], p[1, -1, -1], p[-1, -1, -1])
    )


def cylinder():
    triangles = []
    top, bottom = (0.0, 0.0, 0.5), (0.0, 0.0, -0.5)
    for i in range(SEGMENTS):
        a0 = 2 * math.pi * i / SEGMENTS
        a1 = 2 * math.pi * (i + 1) / SEGMENTS
        c0, s0, c1, s1 = math.cos(a0), math.sin(a0), math.cos(a1), math.sin(a1)
        triangles += quad((c0, s0, -0.5), (c1, s1, -0.5), (c1, s1, 0.5), (c0, s0, 0.5))
        triangles.append((top, (c0, s0, 0.5), (c1, s1, 0.5)))
        triangles.append((bottom, (c1, s1, -0.5), (c0, s0, -0.5)))
    return triangles


def sphere():
    def point(ring, segment):
        polar = math.pi * ring / RINGS
        azimuth = 2 * math.pi * segment / SEGMENTS
        return (
            math.sin(polar) * math.cos(azimuth),
            math.sin(polar) * math.sin(azimuth),
            math.cos(polar),
        )

    triangles = []
    for ring in range(RINGS):
        for segment in range(SEGMENTS):
            a, b = point(ring, segment), point(ring + 1, segment)
            c, d = point(ring + 1, segment + 1), point(ring, segment + 1)
            if ring > 0:
                triangles.append((a, b, d))
            if ring < RINGS - 1:
                triangles.append((b, c, d))
    return triangles


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    for name, triangles in (("box", box()), ("cylinder", cylinder()), ("sphere", sphere())):
        write_stl(out / f"{name}.stl", triangles)
        print(f"{out / name}.stl: {len(triangles)} triangles")


if __name__ == "__main__":
    main()
