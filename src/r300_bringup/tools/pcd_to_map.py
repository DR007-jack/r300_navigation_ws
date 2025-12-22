#!/usr/bin/env python3
"""
pcd_to_map.py
将单个 scans.pcd 投影为 2D 占据图 (PGM) 并生成 map.yaml。
用法示例:
  python3 pcd_to_map.py --input scans.pcd --output_dir ~/maps --resolution 0.05 --z_min -1.0 --z_max 2.0

说明:
 - 该脚本把点云在 XY 平面上投影并按栅格计数，计数高则判为占据，计数为0为自由，少量计数为未知。
 - 生成的 map.yaml 中 origin 设置为点云 bbox 左下角（需要时可手动调整以对齐机器人坐标系）。
"""
import argparse
import numpy as np
from PIL import Image
# Try to import open3d; if it fails (binary ABI mismatch in conda), fall back to pypcd
o3d = None
try:
    import open3d as o3d
except Exception as e:
    print("Warning: open3d import failed (will try pypcd fallback):", e)
    o3d = None
try:
    from pypcd import pypcd
except Exception:
    pypcd = None
import os
import yaml


def pcd_to_map(input_file, out_dir, resolution=0.05, z_min=-1.0, z_max=2.0, occ_threshold=3):
    assert os.path.exists(input_file), "input PCD not found: {}".format(input_file)
    print("Loading PCD:", input_file)
    pts = None
    if o3d is not None:
        # preferred fast path with Open3D
        pcd = o3d.io.read_point_cloud(input_file)
        pts = np.asarray(pcd.points)
    else:
        # fallback: try pypcd first, else try built-in binary reader
        if pypcd is not None:
            print("Using pypcd fallback to read PCD (slower).")
            pc = pypcd.PointCloud.from_path(input_file)
            # pypcd stores data in a structured numpy array
            data = pc.pc_data
            # try common field names
            for fname in (('x','y','z'), ('field_x','field_y','field_z')):
                if all(n in data.dtype.names for n in fname):
                    xs = data[fname[0]]
                    ys = data[fname[1]]
                    zs = data[fname[2]]
                    pts = np.vstack((xs, ys, zs)).T
                    break
            if pts is None:
                # try to find numeric columns by position
                arr = np.vstack([data[name] for name in data.dtype.names]).T
                if arr.shape[1] >= 3:
                    pts = arr[:, :3]
                else:
                    raise RuntimeError("pypcd could not find x,y,z fields in PCD")
        else:
            # attempt built-in binary PCD reader (handles common binary float32 PCDs)
            try:
                print("pypcd not available; trying built-in binary reader")
                pts = read_pcd_binary_xyz(input_file)
            except Exception as e:
                raise RuntimeError("Neither open3d nor pypcd are available to read PCD.\n" \
                                   "Tried built-in binary reader and failed: %s\n" \
                                   "If you are inside conda, try 'conda deactivate' or install open3d into your conda env,\n" \
                                   "or install pypcd via: pip3 install pypcd" % e)
    if pts.size == 0:
        raise RuntimeError("Empty point cloud")

    # filter by z
    mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    pts = pts[mask]
    print("Kept points after z-filter:", pts.shape[0])

    xs = pts[:, 0]
    ys = pts[:, 1]

    # compute bounds
    min_x, max_x = float(xs.min()), float(xs.max())
    min_y, max_y = float(ys.min()), float(ys.max())
    print("XY bounds:", (min_x, min_y), (max_x, max_y))

    res = float(resolution)
    width = int(np.ceil((max_x - min_x) / res)) + 1
    height = int(np.ceil((max_y - min_y) / res)) + 1
    print("map size (width x height) pixels:", width, height)

    ix = ((xs - min_x) / res).astype(np.int32)
    iy = ((ys - min_y) / res).astype(np.int32)

    # Vectorized accumulation: convert 2D indices to flat indices and bincount
    # This is much faster than Python loops for large point clouds.
    rows = height - 1 - iy
    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    ix_valid = ix[valid]
    rows_valid = rows[valid]
    n_valid = ix_valid.size
    print("Valid points inside bounds:", n_valid)
    if n_valid == 0:
        grid = np.zeros((height, width), dtype=np.uint32)
    else:
        flat_idx = rows_valid.astype(np.int64) * width + ix_valid.astype(np.int64)
        counts = np.bincount(flat_idx, minlength=width * height)
        grid = counts.reshape((height, width)).astype(np.uint32)

    occ = np.zeros((height, width), dtype=np.uint8) + 255
    occ[grid >= occ_threshold] = 0
    unknown_mask = (grid > 0) & (grid < occ_threshold)
    occ[unknown_mask] = 128

    os.makedirs(out_dir, exist_ok=True)
    out_pgm = os.path.join(out_dir, "map.pgm")
    Image.fromarray(occ).save(out_pgm)
    print("Saved PGM:", out_pgm)

    map_yaml = {
        "image": os.path.basename(out_pgm),
        "resolution": float(res),
        # origin is lower-left corner in world coords; adjust if needed
        "origin": [float(min_x - res / 2.0), float(min_y - res / 2.0), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    out_yaml = os.path.join(out_dir, "map.yaml")
    with open(out_yaml, "w") as f:
        yaml.dump(map_yaml, f, default_flow_style=False)
    print("Saved YAML:", out_yaml)
    print("Done. If the map appears flipped or offset, adjust origin in map.yaml accordingly.")


def read_pcd_binary_xyz(path):
    """Read binary PCD (common float32 layout) and return Nx3 numpy array of xyz."""
    with open(path, 'rb') as f:
        # read header
        header_lines = []
        while True:
            line = f.readline().decode('ascii', errors='ignore')
            header_lines.append(line)
            if line.strip().upper().startswith('DATA'):
                data_type = line.strip().split()[1].lower() if len(line.strip().split()) > 1 else 'binary'
                break
        header = ''.join(header_lines)

        # parse header fields
        fields = None
        sizes = None
        types = None
        counts = None
        width = None
        height = None
        points = None
        for hl in header_lines:
            toks = hl.strip().split()
            if len(toks) == 0:
                continue
            key = toks[0].upper()
            if key == 'FIELDS':
                fields = toks[1:]
            elif key == 'SIZE':
                sizes = list(map(int, toks[1:]))
            elif key == 'TYPE':
                types = toks[1:]
            elif key == 'COUNT':
                counts = list(map(int, toks[1:]))
            elif key == 'WIDTH':
                width = int(toks[1])
            elif key == 'HEIGHT':
                height = int(toks[1])
            elif key == 'POINTS':
                points = int(toks[1])

        if points is None:
            if width is not None and height is not None:
                points = width * height
            else:
                raise RuntimeError('Could not determine number of points in PCD')

        if data_type != 'binary' and data_type != 'binary_compressed':
            raise RuntimeError('read_pcd_binary_xyz called for non-binary PCD')

        if fields is None or sizes is None or types is None:
            raise RuntimeError('Malformed PCD header, missing FIELDS/SIZE/TYPE')

        # build numpy dtype (little endian assumed)
        dtype_list = []
        for name, t, s, c in zip(fields, types, sizes, counts if counts is not None else [1]*len(fields)):
            # handle only common cases
            if t == 'F' and s == 4:
                npdtype = '<f4'
            elif t == 'F' and s == 8:
                npdtype = '<f8'
            elif t == 'U' and s == 1:
                npdtype = 'u1'
            elif t == 'U' and s == 4:
                npdtype = '<u4'
            elif t == 'I' and s == 4:
                npdtype = '<i4'
            else:
                # fallback to raw bytes
                npdtype = ('u1', s)
            if c == 1:
                dtype_list.append((name, npdtype))
            else:
                dtype_list.append((name, npdtype, (c,)))

        np_dtype = np.dtype(dtype_list)

        # read binary block
        start = f.tell()
        data = f.read()
        # interpret buffer as structured array
        arr = np.frombuffer(data, dtype=np_dtype, count=points)
        # extract x,y,z
        if not all(k in arr.dtype.names for k in ('x', 'y', 'z')):
            # try lowercase names
            names = [n.lower() for n in arr.dtype.names]
            if all(k in names for k in ('x', 'y', 'z')):
                x = arr[arr.dtype.names[names.index('x')]]
                y = arr[arr.dtype.names[names.index('y')]]
                z = arr[arr.dtype.names[names.index('z')]]
            else:
                raise RuntimeError('x,y,z not found in PCD fields')
        else:
            x = arr['x']
            y = arr['y']
            z = arr['z']

        pts = np.vstack((x, y, z)).T
        return pts


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--input", "-i", required=True, help="input PCD file")
    p.add_argument("--output_dir", "-o", default=".", help="output dir for map.pgm and map.yaml")
    p.add_argument("--resolution", "-r", type=float, default=0.05, help="meters per pixel")
    p.add_argument("--z_min", type=float, default=-1.0, help="filter points below this z")
    p.add_argument("--z_max", type=float, default=2.0, help="filter points above this z")
    p.add_argument("--occupancy_threshold", type=int, default=3, help="cells with counts >= this considered occupied")
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()
    pcd_to_map(args.input, args.output_dir, args.resolution, args.z_min, args.z_max, args.occupancy_threshold)

