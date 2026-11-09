from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from sentry_sim_bringup.scene_geometry import (
    default_scene_file, load_scene_geometry, resolve_route, validate_scene_geometry,
)


SCENE = Path(__file__).resolve().parents[1] / 'config' / 'scene_geometry.yaml'


def test_existing_geometry_values_are_preserved():
    scene = load_scene_geometry(SCENE)
    assert scene.bt_tunnels == '-3.5;-1.5;3.475;1.5;3.5;-3.475'
    assert scene.planner_regions == [-4.0, 3.06, -1.0, 3.86, 1.0, -3.86, 4.0, -3.06]
    assert scene.route_points == (-4.0, 3.475, -2.5, 3.475, 0.0, 0.0)
    assert scene.route_yaw == 0.0
    assert scene.tunnel_half_width == 0.4
    assert Path(default_scene_file()) == SCENE


@pytest.mark.parametrize('bad', [None, {}, {'schema_version': 1}])
def test_missing_schema_is_rejected(bad):
    with pytest.raises(ValueError):
        validate_scene_geometry(bad)


@pytest.mark.parametrize('mutation', [
    lambda d: d.update(schema_version=True),
    lambda d: d.update(frame_id='odom'),
    lambda d: d.update(tunnel_half_width=True),
    lambda d: d.update(tunnel_half_width=float('nan')),
    lambda d: d.update(tunnel_half_width=0),
    lambda d: d.update(unknown='ignored'),
    lambda d: d['tunnels'][0].update(x_min=float('inf')),
    lambda d: d['tunnels'][0].update(x_max=-4),
    lambda d: d['tunnels'][0].update(y=True),
    lambda d: d['tunnels'][1].update(id='north'),
    lambda d: d['tunnels'][0].update(soft_cost_roi=[2, 0, 1, 1]),
    lambda d: d['default_route'].update(points=[[0, 0], [1, 1]]),
    lambda d: d['default_route'].update(yaw=float('nan')),
])
def test_invalid_scene_is_not_silently_normalized(mutation):
    data = deepcopy(yaml.safe_load(SCENE.read_text()))
    mutation(data)
    with pytest.raises(ValueError):
        validate_scene_geometry(data)


def test_duplicate_yaml_keys_are_rejected(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text(SCENE.read_text() + '\nframe_id: map\n')
    with pytest.raises(ValueError, match='duplicate'):
        load_scene_geometry(path)


def test_explicit_route_and_yaw_override_scene_without_adding_a_fourth_point():
    scene = load_scene_geometry(SCENE)
    assert resolve_route(scene, '[1,2,3,4,5,6]', '0.5') == ([1., 2., 3., 4., 5., 6.], 0.5)
    assert resolve_route(scene, '') == (list(scene.route_points), 0.0)
    with pytest.raises(ValueError):
        resolve_route(scene, [0.] * 8)
    with pytest.raises(ValueError):
        resolve_route(scene, '[true,0,0,0,0,0]')


@pytest.mark.parametrize('points', ['[]', [], 'null', None])
def test_explicit_empty_or_null_route_is_not_auto_start_default(points):
    with pytest.raises(ValueError, match='three finite XY pairs'):
        resolve_route(load_scene_geometry(SCENE), points)
