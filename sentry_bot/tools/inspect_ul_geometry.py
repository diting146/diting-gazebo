#!/usr/bin/env python3
"""Read actual field and robot collision geometry, without changing map assets."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = ROOT / "gazebo" if (ROOT / "gazebo/src").is_dir() else ROOT
FIELD = (
    SIM_ROOT
    / "src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/2027RMUL/meshes/2027RMUL.stl"
)
ROBOT = ROOT / "sentry_nav/src/sentry_sim_bringup/models/sentry_rm25/model.sdf"


def transform(pose):
    values = list(
        map(float, (pose.text if pose is not None else "0 0 0 0 0 0").split())
    )
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", values[3:]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def main():
    model = ET.parse(ROBOT).getroot().find("model")
    frames = {"__model__": np.eye(4)}
    elements = {e.get("name"): e for e in model if e.tag in ("link", "joint", "frame")}

    def frame(name):
        if name not in frames:
            element = elements[name]
            pose = element.find("pose")
            relative = (
                pose.get("relative_to", "__model__")
                if pose is not None
                else "__model__"
            )
            frames[name] = frame(relative) @ transform(pose)
        return frames[name]

    bounds = []
    sensor_bounds = []
    for link in model.findall("link"):
        for collision in link.findall("collision"):
            pose = collision.find("pose")
            relative = (
                pose.get("relative_to", link.get("name"))
                if pose is not None
                else link.get("name")
            )
            matrix = frame(relative) @ transform(pose)
            geometry = collision.find("geometry")
            if geometry.find("box") is not None:
                half = (
                    np.array(list(map(float, geometry.findtext("box/size").split())))
                    / 2
                )
            elif geometry.find("cylinder") is not None:
                radius = float(geometry.findtext("cylinder/radius"))
                half = np.array(
                    [radius, radius, float(geometry.findtext("cylinder/length")) / 2]
                )
            else:
                raise ValueError("Unsupported collision: " + collision.get("name"))
            extent = np.abs(matrix[:3, :3]) @ half
            bounds.append((matrix[:3, 3] - extent, matrix[:3, 3] + extent))
    lower = np.min([a for a, b in bounds], axis=0)
    upper = np.max([b for a, b in bounds], axis=0)
    mesh = o3d.io.read_triangle_mesh(str(FIELD))
    v = np.asarray(mesh.vertices) * 0.001
    mesh.vertices = o3d.utility.Vector3dVector(
        np.column_stack((v[:, 0] - 6.3633, v[:, 2] - 4.333, -v[:, 1] + 0.534))
    )
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    # Probe upward inside the top-left passage, above the floor.
    rays = np.array(
        [
            [x, y, 0.12, 0, 0, 1]
            for x in (-2.65, -2.5, -2.35)
            for y in (3.1, 3.46, 3.82)
        ],
        dtype=np.float32,
    )
    hit = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy() + 0.12
    lidar = frame("lidar_link")
    # Actual rendered lidar cylinder, including its mount tilt.
    extent = np.array([0.03, 0.03, 0.025])
    lidar_upper = float(lidar[2, 3] + (np.abs(lidar[:3, :3]) @ extent)[2])
    ground_rays = rays.copy()
    ground_rays[:, 2] = 0.2
    ground_rays[:, 5] = -1
    ground = 0.2 - scene.cast_rays(o3d.core.Tensor(ground_rays))["t_hit"].numpy()
    # Flat-floor settled model origin follows wheel contact at lower bound.
    lidar_world_top = float(np.max(ground) - lower[2] + lidar_upper)
    report = {
        "lidar_visual_upper_model": lidar_upper,
        "tunnel_floor_z_max": float(np.max(ground)),
        "lidar_world_top_conservative": lidar_world_top,
        "conservative_envelope_difference": float(np.min(hit) - lidar_world_top),
        "field_mesh": str(FIELD),
        "robot_model": str(ROBOT),
        "robot_collision_lower_model": lower.tolist(),
        "robot_collision_upper_model": upper.tolist(),
        "robot_collision_height_from_lowest": float(upper[2] - lower[2]),
        "robot_collision_width": float(upper[1] - lower[1]),
        "passage_probe_xy": rays[:, :2].tolist(),
        "first_surface_above_z_012": hit.tolist(),
        "note": "Primitive collision AABBs; cylinder bounds are conservative. No visual geometry clearance claim.",
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
