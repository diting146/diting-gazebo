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

"""Rebuild meshes/2027RMUC.stl from the original 2027 RMUC field STL
(带雷达基座.STL).

The source file is a 7.2M-triangle / 361 MB binary STL. Gazebo collision
building and the Nav2 map generator do not need that resolution, so the
checked-in mesh is quadric-decimated to 400k triangles (~20 MB). The
decimation is deterministic for a given Open3D version.

Usage:
  python3 simplify_mesh.py /path/to/2027_rmuc_field.stl

The script depends on numpy and open3d.
"""

import os
import sys

import open3d as o3d

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_PATH = os.path.join(HERE, "meshes", "2027RMUC.stl")
TARGET_TRIANGLES = 400000


def main():
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} <source.stl>")
    source = os.path.abspath(sys.argv[1])
    if not os.path.isfile(source):
        sys.exit(f"source mesh not found: {source}")

    mesh = o3d.io.read_triangle_mesh(source)
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    simplified = mesh.simplify_quadric_decimation(
        target_number_of_triangles=TARGET_TRIANGLES
    )
    simplified.compute_vertex_normals()
    if not o3d.io.write_triangle_mesh(OUT_PATH, simplified, write_ascii=False):
        sys.exit(f"failed to write {OUT_PATH}")
    print(
        f"wrote {OUT_PATH}: {len(simplified.vertices)} vertices, "
        f"{len(simplified.triangles)} triangles"
    )


if __name__ == "__main__":
    main()
