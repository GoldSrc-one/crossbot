#!/usr/bin/env python3
"""Generates tworoom.map: a hand-designed two-room test fixture for
navgen's Milestone 2a leaf-extraction verification. Regenerate with
`python gen_tworoom_map.py > tworoom.map` if the layout ever needs to
change -- box brushes are written by formula here specifically to avoid
hand-transcription winding errors (verified against the Quake/GoldSrc
"normal = (p2-p1) x (p3-p1) points outward" .map convention).
"""

TEXTURE = "AAATRIGGER"


def box_brush(x0, y0, z0, x1, y1, z1, texture=TEXTURE):
    assert x0 < x1 and y0 < y1 and z0 < z1
    # Quake/GoldSrc .map convention computes each face's plane normal as
    # CrossProduct(p1-p2, p3-p2) (per the original qbsp3 map.c ParseBrush --
    # NOT (p2-p1)x(p3-p1)), and that normal must point outward. Point triples
    # below are ordered for that exact convention (verified by hand against
    # both formulas after an initial hlcsg "brush outside world" failure
    # traced to inverted normals from assuming the other cross-product order).
    faces = [
        ((x1, y1, z1), (x1, y0, z1), (x0, y0, z1)),  # top, +Z
        ((x1, y0, z0), (x1, y1, z0), (x0, y1, z0)),  # bottom, -Z
        ((x1, y1, z0), (x1, y1, z1), (x0, y1, z1)),  # north, +Y
        ((x0, y0, z0), (x0, y0, z1), (x1, y0, z1)),  # south, -Y
        ((x1, y0, z0), (x1, y0, z1), (x1, y1, z1)),  # east, +X
        ((x0, y1, z0), (x0, y1, z1), (x0, y0, z1)),  # west, -X
    ]
    lines = ["{"]
    for p1, p2, p3 in faces:
        pts = " ".join(f"( {p[0]} {p[1]} {p[2]} )" for p in (p1, p2, p3))
        lines.append(f"{pts} {texture} 0 0 0 1 1")
    lines.append("}")
    return "\n".join(lines)


def main():
    brushes = [
        (0, 0, -16, 512, 256, 0),      # floor
        (0, 0, 128, 512, 256, 144),    # ceiling
        (0, 0, 0, 16, 256, 128),       # west wall (room A outer)
        (496, 0, 0, 512, 256, 128),    # east wall (room B outer)
        (0, 240, 0, 512, 256, 128),    # north wall (both rooms)
        (0, 0, 0, 512, 16, 128),       # south wall (both rooms)
        (240, 16, 0, 272, 96, 128),    # shared wall, south-of-doorway
        (240, 160, 0, 272, 240, 128),  # shared wall, north-of-doorway
    ]

    print('{')
    print('"classname" "worldspawn"')
    print('"wad" "D:\\Downloads\\hammer\\zhlt.wad"')
    for b in brushes:
        print(box_brush(*b))
    print('}')

    print('{')
    print('"classname" "info_player_deathmatch"')
    print('"origin" "128 128 32"')
    print('}')
    print('{')
    print('"classname" "info_player_deathmatch"')
    print('"origin" "384 128 32"')
    print('}')
    print('{')
    print('"classname" "light"')
    print('"origin" "128 128 96"')
    print('"_light" "255 255 255 200"')
    print('}')
    print('{')
    print('"classname" "light"')
    print('"origin" "384 128 96"')
    print('"_light" "255 255 255 200"')
    print('}')


if __name__ == "__main__":
    main()
