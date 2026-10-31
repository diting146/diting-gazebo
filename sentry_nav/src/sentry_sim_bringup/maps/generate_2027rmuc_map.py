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
Regenerate the 2027RMUC map assets for the pb2025 navigation stack.

Outputs (next to this script):
  2027RMUC.pgm  8-bit occupancy grid, 0.05 m/cell (0=occupied, 254=free)
  2027RMUC.yaml nav2 map_server metadata
  2027RMUC.pcd  downsampled prior cloud in MAP/world frame (binary PCD)

Geometry is derived from the Gazebo model installed in the simulator:
  .../models/2027RMUC/meshes/2027RMUC.stl   (units: mm, Z-up)
  SDF link pose: -18.1400 -8.0016 -1.2000 0 0 0, mesh scale 0.001

  p = vertices * 0.001
  world_x = p.x - 18.1400
  world_y = p.y - 8.0016
  world_z = p.z - 1.2000

The arena is 28 m x 16 m (world x in [-14, 14]) and the mesh also contains
the radar base beside its left wall, so the generated grid covers
x in [-18.15, 14.05] (32.2 m) to keep the base inside the map. The arena
floor is not perfectly flat: the central highland, ring highlands, ramps and
corner bases rise above the nominal driving plane, so the occupancy grid
combines

  * a horizontal slice at z=0.25 m (walls and vertical faces), and
  * a top-down height test (fills the interior of raised plateaus).

Usage:
  python3 generate_2027rmuc_map.py
"""

import os
import sys

import numpy as np
import open3d as o3d
from PIL import Image
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
WS_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
SIM_ROOT = os.path.join(WS_ROOT, "gazebo") if os.path.isdir(os.path.join(WS_ROOT, "gazebo", "src")) else WS_ROOT
MESH_PATH = os.path.join(
    SIM_ROOT,
    "src", "rmu_gazebo_simulator", "rmu_gazebo_simulator",
    "resource", "models", "2027RMUC", "meshes", "2027RMUC.stl",
)

PGM_PATH = os.path.join(HERE, "2027RMUC.pgm")
YAML_PATH = os.path.join(HERE, "2027RMUC.yaml")
PCD_PATH = os.path.join(HERE, "2027RMUC.pcd")
EVIDENCE_PATH = os.path.join(WS_ROOT, "log", "map_assets_2027RMUC.txt")

RES = 0.05
XMIN, XMAX = -18.15, 14.05
YMIN, YMAX = -8.0, 8.0
WIDTH = int(np.ceil((XMAX - XMIN) / RES))   # 644
HEIGHT = int(np.ceil((YMAX - YMIN) / RES))  # 320

# Model frame -> world/map frame, mirroring models/2027RMUC/model.sdf.
OFFSET_X = 18.1400
OFFSET_Y = 8.0016
OFFSET_Z = 1.2000

SLICE_Z = 0.25
DILATE_ITERS = 2
FREE = 254
OCC = 0
UNKNOWN = 205

NUM_SAMPLES = 800000
VOXEL_SIZE = 0.02
RANDOM_SEED = 0
GROUND_CLEARANCE = 0.12

# Default sentry spawn used by bringup_sentry_simulation_launch.py.
SPAWN_X, SPAWN_Y = -5.3, 3.0


def transform(pts):
    """Map mesh vertices (mm, Z-up) to world/map frame (m)."""
    p = np.asarray(pts, dtype=np.float64) * 0.001
    p[:, 0] -= OFFSET_X
    p[:, 1] -= OFFSET_Y
    p[:, 2] -= OFFSET_Z
    return p


def slice_segments(vertices, triangles, z):
    """Return one (p0, p1) segment per triangle crossing the plane z."""
    tri = vertices[triangles]
    d = tri[:, :, 2] - z
    points_by_tri = {}
    for a, b in ((0, 1), (1, 2), (2, 0)):
        da, db = d[:, a], d[:, b]
        idx = np.nonzero((da < 0.0) != (db < 0.0))[0]
        if len(idx) == 0:
            continue
        t = -da[idx] / (db[idx] - da[idx])
        p = tri[idx, a] + t[:, None] * (tri[idx, b] - tri[idx, a])
        for k, i in enumerate(idx):
            points_by_tri.setdefault(int(i), []).append(p[k])

    segments = []
    for points in points_by_tri.values():
        if len(points) >= 2:
            segments.append((points[0], points[1]))
    return segments


def rasterize(segments):
    img = np.full((HEIGHT, WIDTH), FREE, dtype=np.uint8)
    step = RES / 2.0
    for p0, p1 in segments:
        dist = np.hypot(p1[0] - p0[0], p1[1] - p0[1])
        n = max(1, int(np.ceil(dist / step)))
        t = np.linspace(0.0, 1.0, n + 1)
        xs = p0[0] + t * (p1[0] - p0[0])
        ys = p0[1] + t * (p1[1] - p0[1])
        ix = np.floor((xs - XMIN) / RES).astype(np.int64)
        iy = np.floor((ys - YMIN) / RES).astype(np.int64)
        ok = (ix >= 0) & (ix < WIDTH) & (iy >= 0) & (iy < HEIGHT)
        img[iy[ok], ix[ok]] = OCC
    return img


def top_height_map(mesh):
    """Raycast straight down; returns top surface height per cell."""
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))

    xs = XMIN + RES * (np.arange(WIDTH) + 0.5)
    ys = YMIN + RES * (np.arange(HEIGHT) + 0.5)
    grid_x, grid_y = np.meshgrid(xs, ys)
    z_start = 6.0
    origins = np.stack(
        [grid_x.ravel(), grid_y.ravel(), np.full(grid_x.size, z_start)], axis=1
    ).astype(np.float32)
    directions = np.tile(
        np.array([[0.0, 0.0, -1.0]], np.float32), (len(origins), 1)
    )
    rays = o3d.core.Tensor(np.concatenate([origins, directions], axis=1))
    ans = scene.cast_rays(rays)
    t_hit = ans["t_hit"].numpy().reshape(HEIGHT, WIDTH)
    heights = z_start - t_hit
    return np.where(np.isfinite(heights), heights, -np.inf)


def cell_of(x, y):
    return int((y - YMIN) / RES), int((x - XMIN) / RES)


def main():
    if not os.path.isfile(MESH_PATH):
        sys.exit(f"source mesh not found: {MESH_PATH}")

    mesh = o3d.io.read_triangle_mesh(MESH_PATH)
    vertices = transform(np.asarray(mesh.vertices))
    triangles = np.asarray(mesh.triangles)

    world_mesh = o3d.geometry.TriangleMesh()
    world_mesh.vertices = o3d.utility.Vector3dVector(vertices)
    world_mesh.triangles = mesh.triangles

    # ---- occupancy grid -------------------------------------------------
    segments = slice_segments(vertices, triangles, SLICE_Z)
    if not segments:
        sys.exit(f"no mesh/plane intersections at z={SLICE_Z}")
    sliced = rasterize(segments) == OCC

    heights = top_height_map(world_mesh)
    raised = heights >= SLICE_Z

    occupied = sliced | raised
    dilated = ndimage.binary_dilation(occupied, iterations=DILATE_ITERS)
    img = np.where(dilated, OCC, FREE).astype(np.uint8)

    # PGM stores row 0 at the TOP; map_server origin is the lower-left.
    Image.fromarray(np.flipud(img), mode="L").save(PGM_PATH, format="PPM")

    occ_ratio = float(np.count_nonzero(img == OCC)) / img.size

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

    # ---- validation -----------------------------------------------------
    # The default launch spawn must be on free ground.
    spawn_row, spawn_col = cell_of(SPAWN_X, SPAWN_Y)
    spawn_free = bool(np.all(
        img[
            max(0, spawn_row - 3):spawn_row + 4,
            max(0, spawn_col - 3):spawn_col + 4,
        ] == FREE
    ))

    # The central highland must be blocked in the static map.
    center_row, center_col = cell_of(0.0, 0.0)
    center_blocked = bool(img[center_row, center_col] == OCC)

    # ---- evidence -------------------------------------------------------
    regen = f"python3 {os.path.abspath(__file__)}"
    lines = [
        "2027RMUC map assets",
        f"source_mesh: {MESH_PATH}",
        f"slice_z: {SLICE_Z} m",
        f"plateau_fill: top surface height >= {SLICE_Z} m",
        f"pgm: {PGM_PATH}",
        f"pgm_shape: {img.shape[1]}x{img.shape[0]} (W x H, width x height)",
        f"resolution: {RES} m/cell",
        f"origin: [{XMIN}, {YMIN}, 0.0]",
        f"occupied_ratio: {occ_ratio:.6f} ({occ_ratio * 100:.2f} %)",
        f"occupied_ratio_expected: 0.02 <= r <= 0.40 -> "
        f"{'PASS' if 0.02 <= occ_ratio <= 0.40 else 'FAIL'}",
        f"spawn_point: [{SPAWN_X}, {SPAWN_Y}] free_1m: {spawn_free}",
        f"center_cell_blocked: {center_blocked}",
        f"pcd: {PCD_PATH}",
        f"pcd_sampling: {NUM_SAMPLES} uniform surface samples, "
        f"voxel_down_sample({VOXEL_SIZE})",
        f"pcd_ground_filter: keep z >= {GROUND_CLEARANCE} m "
        f"(drop floor plates)",
        f"pcd_points: {n_down}",
        f"pcd_points_expected: 50000 <= n <= 500000 -> "
        f"{'PASS' if 50000 <= n_down <= 500000 else 'FAIL'}",
        f"pcd_bbox_min: [{pcd_bbox[0]:.4f}, {pcd_bbox[1]:.4f}, "
        f"{pcd_bbox[2]:.4f}]",
        f"pcd_bbox_max: [{pcd_bbox_max[0]:.4f}, {pcd_bbox_max[1]:.4f}, "
        f"{pcd_bbox_max[2]:.4f}]",
        f"regenerate: {regen}",
    ]
    os.makedirs(os.path.dirname(EVIDENCE_PATH), exist_ok=True)
    with open(EVIDENCE_PATH, "w") as f:
        f.write("\n".join(lines) + "\n")

    print("\n".join(lines))

    if not (0.02 <= occ_ratio <= 0.40):
        sys.exit(f"occupied ratio out of range: {occ_ratio:.4f}")
    if not spawn_free:
        sys.exit(f"default spawn ({SPAWN_X}, {SPAWN_Y}) is not free")
    if not center_blocked:
        sys.exit("central highland is not marked as occupied")


if __name__ == "__main__":
    main()
