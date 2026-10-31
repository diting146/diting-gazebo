from types import SimpleNamespace as S
import pytest
from sentry_mission.route_validation import validate_route


def pose(x, y, frame="map"):
    return S(header=S(frame_id=frame), pose=S(position=S(x=x, y=y, z=0)))


def test_default_route_and_outside_tunnel_rejection():
    assert validate_route(pose(-4.1, 3.46), pose(-1.0, 3.46), pose(0, 0))
    with pytest.raises(ValueError):
        validate_route(pose(-4.1, 3.7), pose(-1.0, 3.7), pose(0, 0))
    with pytest.raises(ValueError):
        validate_route(pose(-2.5, 3.46), pose(-1.0, 3.46), pose(0, 0))
    with pytest.raises(ValueError):
        validate_route(pose(-4.1, 3.46, "odom"), pose(-1.0, 3.46), pose(0, 0))


def test_actual_static_map_rejects_old_handoff_points():
    from pathlib import Path
    import numpy as np
    from PIL import Image
    from sentry_mission.route_validation import validate_map_points

    root = Path(__file__).resolve().parents[4]
    image = root / "sentry_nav/src/sentry_sim_bringup/maps/2027RMUL.pgm"
    if not image.exists():
        pytest.skip("Sibling simulation workspace not installed")
    pixels = np.flipud(np.asarray(Image.open(image)))
    data = np.where(pixels < 50, 100, 0).ravel().tolist()
    grid = S(
        header=S(frame_id="map"),
        data=data,
        info=S(
            resolution=0.05,
            width=pixels.shape[1],
            height=pixels.shape[0],
            origin=S(position=S(x=-6.8, y=-4.8), orientation=S(x=0, y=0, z=0, w=1)),
        ),
    )
    assert validate_map_points((pose(-4.1, 3.46), pose(-1, 3.46), pose(0, 0)), grid)
    with pytest.raises(ValueError):
        validate_map_points((pose(-3.4, 3.46),), grid)


def test_reverse_tunnel_uses_same_audited_corridor():
    assert validate_route(pose(-1, 3.46), pose(-4.1, 3.46), pose(0, 0))
