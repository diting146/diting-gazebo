# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License");

"""Strict scene data loading; no ROS node, process, parameter or clock access."""

from dataclasses import dataclass
import math
from pathlib import Path
import re

import yaml


class _UniqueKeysLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str) or key in result:
            raise ValueError(f'Scene YAML has a non-string or duplicate key: {key!r}')
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeysLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def _keys(value, required, label):
    if not isinstance(value, dict) or set(value) != set(required):
        raise ValueError(f'{label} requires exactly: {", ".join(required)}')


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number (not a boolean)')
    return float(value)


@dataclass(frozen=True)
class TunnelGeometry:
    name: str
    x_min: float
    x_max: float
    y: float
    soft_cost_roi: tuple


@dataclass(frozen=True)
class SceneGeometry:
    frame_id: str
    tunnel_half_width: float
    tunnels: tuple
    route_points: tuple
    route_yaw: float

    @property
    def bt_tunnels(self):
        return ';'.join(str(v) for t in self.tunnels for v in (t.x_min, t.x_max, t.y))

    @property
    def planner_regions(self):
        return [v for tunnel in self.tunnels for v in tunnel.soft_cost_roi]


def validate_scene_geometry(document):
    _keys(document, ('schema_version', 'frame_id', 'tunnel_half_width', 'tunnels',
                     'default_route'), 'scene')
    if type(document['schema_version']) is not int or document['schema_version'] != 1:
        raise ValueError('scene schema_version must be integer 1')
    if document['frame_id'] != 'map':
        raise ValueError('Only the existing map-frame, axis-aligned scene is supported')
    half_width = _number(document['tunnel_half_width'], 'tunnel_half_width')
    if half_width <= 0.0:
        raise ValueError('tunnel_half_width must be positive')
    if not isinstance(document['tunnels'], list) or len(document['tunnels']) != 2:
        raise ValueError('This scene requires exactly two tunnels')
    tunnels, names = [], set()
    for item in document['tunnels']:
        _keys(item, ('id', 'x_min', 'x_max', 'y', 'soft_cost_roi'), 'tunnel')
        name = item['id']
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
            raise ValueError('tunnel id must be a nonempty identifier')
        if name in names:
            raise ValueError('tunnel ids must be unique')
        names.add(name)
        x_min, x_max, y = (_number(item[k], 'tunnel.' + k)
                           for k in ('x_min', 'x_max', 'y'))
        if x_min >= x_max:
            raise ValueError('tunnel x_min must be below x_max')
        roi = item['soft_cost_roi']
        if not isinstance(roi, list) or len(roi) != 4:
            raise ValueError('soft_cost_roi requires [xmin, ymin, xmax, ymax]')
        roi = tuple(_number(v, 'soft_cost_roi') for v in roi)
        if roi[0] >= roi[2] or roi[1] >= roi[3]:
            raise ValueError('soft_cost_roi bounds must be ordered')
        tunnels.append(TunnelGeometry(name, x_min, x_max, y, roi))
    route = document['default_route']
    _keys(route, ('points', 'yaw'), 'default_route')
    if (not isinstance(route['points'], list) or len(route['points']) != 3 or
            any(not isinstance(point, list) or len(point) != 2 for point in route['points'])):
        raise ValueError('default_route requires exactly three XY pairs')
    points = tuple(_number(v, 'default_route.points') for p in route['points'] for v in p)
    return SceneGeometry('map', half_width, tuple(tunnels), points,
                         _number(route['yaw'], 'default_route.yaw'))


def load_scene_geometry(filename):
    with open(filename, encoding='utf-8') as stream:
        return validate_scene_geometry(yaml.load(stream, Loader=_UniqueKeysLoader))


def default_scene_file():
    # Source/symlink installs are usable by pure Python tests without ROS imports.
    source_file = Path(__file__).resolve().parents[1] / 'config' / 'scene_geometry.yaml'
    if source_file.is_file():
        return str(source_file)
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('sentry_sim_bringup')) /
               'config' / 'scene_geometry.yaml')


def resolve_route(scene, points='', yaw=None):
    if isinstance(points, str) and not points.strip():
        points = list(scene.route_points)
    elif isinstance(points, str):
        points = yaml.safe_load(points)
    if not isinstance(points, (list, tuple)) or len(points) != 6:
        raise ValueError('route_points requires exactly three finite XY pairs')
    try:
        points = [_number(v, 'route_points') for v in points]
    except ValueError as error:
        raise ValueError('route_points requires exactly three finite XY pairs') from error
    if yaw is None or yaw == '':
        yaw = scene.route_yaw
    elif isinstance(yaw, str):
        yaw = yaml.safe_load(yaw)
    return points, _number(yaw, 'route_yaw')
