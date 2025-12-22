#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Convert a 3D PCD point cloud map into a 2D occupancy grid (PGM + YAML).

Why:
- ROS navigation (map_server / AMCL / move_base) requires a 2D grid map (pgm+yaml).
- point_lio typically outputs a 3D point cloud map (e.g. scans.pcd).

This script:
- Loads an ASCII PCD file containing at least x y z fields.
- Filters points by Z range (ground slice / band-pass).
- Projects remaining points to XY plane.
- Rasterizes to a 2D grid at a given resolution.
- Writes:
  - PGM (P5, 8-bit)
  - YAML compatible with map_server.

Notes:
- This is a *simple* rasterizer. For best results you may need:
  - downsampling / outlier removal
  - careful z_min/z_max selection
  - manual cropping

Python:
- Compatible with Python 2.7 (ROS Melodic).

Usage example:
  pcd_to_occupancy_grid.py \
    --pcd ~/r300_navigation_ws/src/point_lio_unilidar-main/PCD/scans.pcd \
    --out_prefix ~/r300_navigation_ws/src/r300_function/maps/r300_map \
    --resolution 0.05 \
    --z_min -0.30 --z_max 0.30
"""

from __future__ import print_function

import argparse
import math
import os
import sys


def _parse_pcd_header(lines):
    """Parse PCD header; return (fields, points, data, header_line_count)."""
    fields = None
    points = None
    data = None

    for i, line in enumerate(lines):
        s = line.strip()
        if not s:
            continue
        if s.startswith('#'):
            continue

        parts = s.split()
        key = parts[0].upper()
        if key == 'FIELDS':
            fields = parts[1:]
        elif key == 'POINTS':
            try:
                points = int(parts[1])
            except Exception:
                points = None
        elif key == 'DATA':
            data = parts[1].lower() if len(parts) > 1 else None
            return fields, points, data, i + 1

    raise ValueError('Invalid PCD: missing DATA line')


def load_ascii_pcd_xyz(pcd_path):
    with open(pcd_path, 'r') as f:
        lines = f.readlines()

    fields, points_decl, data, header_lines = _parse_pcd_header(lines)

    if data != 'ascii':
        raise ValueError('Only ASCII PCD is supported right now (DATA ascii). Got: %s' % data)

    if not fields:
        raise ValueError('PCD missing FIELDS')

    try:
        ix = fields.index('x')
        iy = fields.index('y')
        iz = fields.index('z')
    except Exception:
        raise ValueError('PCD FIELDS must include x y z. Got: %s' % (' '.join(fields)))

    pts = []
    for line in lines[header_lines:]:
        s = line.strip()
        if not s:
            continue
        parts = s.split()
        if len(parts) < max(ix, iy, iz) + 1:
            continue
        try:
            x = float(parts[ix])
            y = float(parts[iy])
            z = float(parts[iz])
        except Exception:
            continue
        if math.isinf(x) or math.isinf(y) or math.isinf(z):
            continue
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            continue
        pts.append((x, y, z))

    # points_decl is advisory; prefer actual parsed count.
    return pts


def rasterize(points_xyz, resolution, z_min, z_max, padding_m=0.5):
    # Filter by height and project
    pts = []
    for x, y, z in points_xyz:
        if z < z_min or z > z_max:
            continue
        pts.append((x, y))

    if not pts:
        raise ValueError('No points left after filtering. Try widening z_min/z_max.')

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]

    min_x = min(xs) - padding_m
    max_x = max(xs) + padding_m
    min_y = min(ys) - padding_m
    max_y = max(ys) + padding_m

    width = int(math.ceil((max_x - min_x) / resolution))
    height = int(math.ceil((max_y - min_y) / resolution))

    if width <= 0 or height <= 0:
        raise ValueError('Invalid grid size computed: %dx%d' % (width, height))

    # map_server expects origin at lower-left of image.
    # In PGM, first row is top, so we need to flip Y when writing pixels.

    # Initialize to unknown 205.
    unknown = 205
    free = 254
    occ = 0

    grid = [[unknown for _ in range(width)] for _ in range(height)]

    for x, y in pts:
        gx = int((x - min_x) / resolution)
        gy = int((y - min_y) / resolution)
        if gx < 0 or gx >= width or gy < 0 or gy >= height:
            continue

        # Convert to image row index (top-down)
        row = (height - 1) - gy
        col = gx
        grid[row][col] = occ

    # Optionally mark untouched unknown as free? Keep unknown as unknown.
    # But many navigation stacks prefer free space; we keep unknown by default.

    origin_x = min_x
    origin_y = min_y

    return grid, width, height, origin_x, origin_y, free, occ, unknown


def write_pgm(path, grid, width, height):
    # P5 binary PGM
    with open(path, 'wb') as f:
        header = 'P5\n%d %d\n255\n' % (width, height)
        f.write(header.encode('ascii'))
        for r in range(height):
            row = grid[r]
            f.write(bytearray([int(v) & 0xFF for v in row]))


def write_yaml(path, image_filename, resolution, origin_x, origin_y, origin_yaw,
               negate=0, occupied_thresh=0.65, free_thresh=0.196):
    content = """image: {image}
resolution: {resolution}
origin: [{ox:.6f}, {oy:.6f}, {yaw:.6f}]
negate: {negate}
occupied_thresh: {occ}
free_thresh: {free}
""".format(
        image=image_filename,
        resolution=float(resolution),
        ox=float(origin_x),
        oy=float(origin_y),
        yaw=float(origin_yaw),
        negate=int(negate),
        occ=float(occupied_thresh),
        free=float(free_thresh),
    )
    with open(path, 'w') as f:
        f.write(content)


def main(argv):
    ap = argparse.ArgumentParser(description='Convert ASCII PCD to 2D occupancy PGM+YAML')
    ap.add_argument('--pcd', required=True, help='Path to input PCD (ASCII)')
    ap.add_argument('--out_prefix', required=True,
                    help='Output prefix path (without extension), e.g. .../maps/r300_map')
    ap.add_argument('--resolution', type=float, default=0.05, help='Grid resolution in meters/pixel')
    ap.add_argument('--z_min', type=float, default=-0.25, help='Min Z (meters) to keep')
    ap.add_argument('--z_max', type=float, default=0.25, help='Max Z (meters) to keep')
    ap.add_argument('--padding', type=float, default=0.5, help='Padding around bbox (meters)')
    ap.add_argument('--origin_yaw', type=float, default=0.0, help='Map YAML origin yaw (radians)')

    args = ap.parse_args(argv)

    pcd_path = os.path.expanduser(args.pcd)
    out_prefix = os.path.expanduser(args.out_prefix)

    out_dir = os.path.dirname(out_prefix)
    if out_dir and (not os.path.exists(out_dir)):
        os.makedirs(out_dir)

    pts = load_ascii_pcd_xyz(pcd_path)
    grid, width, height, ox, oy, _free, _occ, _unk = rasterize(
        pts,
        resolution=float(args.resolution),
        z_min=float(args.z_min),
        z_max=float(args.z_max),
        padding_m=float(args.padding),
    )

    pgm_path = out_prefix + '.pgm'
    yaml_path = out_prefix + '.yaml'

    write_pgm(pgm_path, grid, width, height)
    write_yaml(yaml_path, os.path.basename(pgm_path), float(args.resolution), ox, oy, float(args.origin_yaw))

    print('Wrote: %s' % pgm_path)
    print('Wrote: %s' % yaml_path)
    print('Grid: %dx%d  resolution=%.3f m/px' % (width, height, float(args.resolution)))
    print('Origin: [%.3f, %.3f, %.3f]' % (ox, oy, float(args.origin_yaw)))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

