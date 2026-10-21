#!/usr/bin/env python3
# Copyright 2026 diting
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Regenerate the diting_3v3_field map assets for the pb2025 navigation stack.

Outputs (next to this script):
  diting_3v3_field.pgm  8-bit occupancy grid, 0.05 m/cell (0=occupied, 254=free)
  diting_3v3_field.yaml nav2 map_server metadata
  diting_3v3_field.pcd  downsampled prior cloud in MAP/world frame (binary PCD)

Geometry is derived from the 2027 season STL installed in the Gazebo model:
  .../models/diting_3v3_field/meshes/3v3_field.stl   (units: mm)
  SDF link pose: -6.3633 -4.333 0.5340 -1.5708 0 0, mesh scale 0.001

  p = vertices * 0.001
  world_x = p.x - 6.3633
  world_y = p.z - 4.333
  world_z = -p.y + 0.5340

Usage:
  python3 generate_diting_map.py
"""

import os
import sys

import numpy as np
import open3d as o3d
from PIL import Image
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
WS_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
MESH_PATH = os.path.join(
    WS_ROOT,
    "gazebo", "src", "rmu_gazebo_simulator", "rmu_gazebo_simulator",
    "resource", "models", "diting_3v3_field", "meshes", "3v3_field.stl",
)

PGM_PATH = os.path.join(HERE, "diting_3v3_field.pgm")
YAML_PATH = os.path.join(HERE, "diting_3v3_field.yaml")
PCD_PATH = os.path.join(HERE, "diting_3v3_field.pcd")
EVIDENCE_PATH = os.path.join(WS_ROOT, "log", "map_assets.txt")

RES = 0.05
MARGIN = 0.5
XMIN, XMAX = -6.8, 6.8
YMIN, YMAX = -4.8, 4.8
WIDTH = int(np.ceil((XMAX - XMIN) / RES))   # 272
HEIGHT = int(np.ceil((YMAX - YMIN) / RES))  # 192

SLICE_Z = 0.25
DILATE_ITERS = 2
FREE = 254
OCC = 0
UNKNOWN = 205

NUM_SAMPLES = 200000
VOXEL_SIZE = 0.02
RANDOM_SEED = 0
# The mesh contains full-footprint horizontal floor surfaces at z~0.00 and
# z~0.10 m. They dominate the cloud with a featureless plane that is useless
# (and harmful) for GICP, so ground/floor points below this height are dropped
# after the mandated 0.02 m voxel downsample.
GROUND_CLEARANCE = 0.12


def transform(pts):
    """Map mesh vertices (mm, Y-down) to world/map frame (m, Z-up)."""
    p = np.asarray(pts, dtype=np.float64) * 0.001
    out = np.empty_like(p)
    out[:, 0] = p[:, 0] - 6.3633
    out[:, 1] = p[:, 2] - 4.333
    out[:, 2] = -p[:, 1] + 0.5340
    return out


def slice_segments_paired(vertices, triangles, z):
    """Collect the two edge/plane intersection points per triangle."""
    tri = vertices[triangles]
    d = tri[:, :, 2] - z
    pts_by_tri = [None] * len(tri)
    segs = []
    for a, b in ((0, 1), (1, 2), (2, 0)):
        da, db = d[:, a], d[:, b]
        crossing = (da < 0.0) != (db < 0.0)
        idx = np.nonzero(crossing)[0]
        denom = db[idx] - da[idx]
        t = -da[idx] / denom
        p = tri[idx, a] + t[:, None] * (tri[idx, b] - tri[idx, a])
        for k, i in enumerate(idx):
            if pts_by_tri[i] is None:
                pts_by_tri[i] = [p[k]]
            else:
                pts_by_tri[i].append(p[k])
    for entry in pts_by_tri:
        if entry is not None and len(entry) >= 2:
            segs.append((entry[0], entry[1]))
    return segs


def rasterize(segments):
    img = np.full((HEIGHT, WIDTH), UNKNOWN, dtype=np.uint8)
    step = RES / 2.0
    for p0, p1 in segments:
        x0, y0 = p0[0], p0[1]
        x1, y1 = p1[0], p1[1]
        dist = np.hypot(x1 - x0, y1 - y0)
        n = max(1, int(np.ceil(dist / step)))
        t = np.linspace(0.0, 1.0, n + 1)
        xs = x0 + t * (x1 - x0)
        ys = y0 + t * (y1 - y0)
        ix = np.floor((xs - XMIN) / RES).astype(np.int64)
        iy = np.floor((ys - YMIN) / RES).astype(np.int64)
        ok = (ix >= 0) & (ix < WIDTH) & (iy >= 0) & (iy < HEIGHT)
        img[iy[ok], ix[ok]] = OCC
    return img


def main():
    if not os.path.isfile(MESH_PATH):
        sys.exit(f"source mesh not found: {MESH_PATH}")

    mesh = o3d.io.read_triangle_mesh(MESH_PATH)
    vertices = transform(np.asarray(mesh.vertices))
    triangles = np.asarray(mesh.triangles)

    # ---- occupancy grid -------------------------------------------------
    segments = slice_segments_paired(vertices, triangles, SLICE_Z)
    if not segments:
        sys.exit(f"no mesh/plane intersections at z={SLICE_Z}")
    base = rasterize(segments)
    occupied = base == OCC
    dilated = ndimage.binary_dilation(occupied, iterations=DILATE_ITERS)
    img = np.where(dilated, OCC, FREE).astype(np.uint8)

    # PGM stores row 0 at the TOP; map_server origin is the lower-left.
    Image.fromarray(np.flipud(img), mode="L").save(PGM_PATH, format="PPM")

    occ_ratio = float(np.count_nonzero(img == OCC)) / img.size

    # Center 1 m x 1 m box (world -0.5..0.5) must be fully free.
    x0 = int(np.floor((-0.5 - XMIN) / RES))
    x1 = int(np.ceil((0.5 - XMIN) / RES))
    y0 = int(np.floor((-0.5 - YMIN) / RES))
    y1 = int(np.ceil((0.5 - YMIN) / RES))
    center = img[y0:y1, x0:x1]
    center_free = bool(np.all(center == FREE))

    yaml_text = (
        f"image: {os.path.basename(PGM_PATH)}\n"
        f"resolution: {RES}\n"
        f"origin: [{XMIN}, {YMIN}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    with open(YAML_PATH, "w") as f:
        f.write(yaml_text)

    # ---- prior point cloud ---------------------------------------------
    o3d.utility.random.seed(RANDOM_SEED)
    sampled = mesh.sample_points_uniformly(number_of_points=NUM_SAMPLES)
    pts = transform(np.asarray(sampled.points))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd_down = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
    kept = np.asarray(pcd_down.points)[:, 2] >= GROUND_CLEARANCE
    pcd_down = pcd_down.select_by_index(np.nonzero(kept)[0])
    if not o3d.io.write_point_cloud(PCD_PATH, pcd_down, write_ascii=False):
        sys.exit(f"failed to write {PCD_PATH}")

    n_down = len(pcd_down.points)
    pcd_bbox = np.asarray(pcd_down.points).min(0)
    pcd_bbox_max = np.asarray(pcd_down.points).max(0)

    # ---- evidence -------------------------------------------------------
    regen = f"python3 {os.path.abspath(__file__)}"
    lines = [
        "diting_3v3_field map assets",
        f"source_mesh: {MESH_PATH}",
        f"slice_z: {SLICE_Z} m",
        f"pgm: {PGM_PATH}",
        f"pgm_shape: {img.shape[1]}x{img.shape[0]} (W x H, width x height)",
        f"resolution: {RES} m/cell",
        f"origin: [{XMIN}, {YMIN}, 0.0]",
        f"occupied_ratio: {occ_ratio:.6f} ({occ_ratio * 100:.2f} %)",
        f"occupied_ratio_expected: 0.10 <= r <= 0.35 -> "
        f"{'PASS' if 0.10 <= occ_ratio <= 0.35 else 'FAIL'}",
        f"center_1m_box_free: {center_free}",
        f"pcd: {PCD_PATH}",
        f"pcd_sampling: {NUM_SAMPLES} uniform surface samples, voxel_down_sample({VOXEL_SIZE})",
        f"pcd_ground_filter: keep z >= {GROUND_CLEARANCE} m (drop floor planes)",
        f"pcd_points: {n_down}",
        f"pcd_points_expected: 20000 <= n <= 120000 -> "
        f"{'PASS' if 20000 <= n_down <= 120000 else 'FAIL'}",
        f"pcd_bbox_min: [{pcd_bbox[0]:.4f}, {pcd_bbox[1]:.4f}, {pcd_bbox[2]:.4f}]",
        f"pcd_bbox_max: [{pcd_bbox_max[0]:.4f}, {pcd_bbox_max[1]:.4f}, {pcd_bbox_max[2]:.4f}]",
        f"regenerate: {regen}",
    ]
    os.makedirs(os.path.dirname(EVIDENCE_PATH), exist_ok=True)
    with open(EVIDENCE_PATH, "w") as f:
        f.write("\n".join(lines) + "\n")

    print("\n".join(lines))

    if not center_free:
        sys.exit("center 1 m box is not fully free")
    if not (0.10 <= occ_ratio <= 0.35):
        sys.exit(f"occupied ratio out of range: {occ_ratio:.4f}")


if __name__ == "__main__":
    main()
