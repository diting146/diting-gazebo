import math
import pytest
from sentry_motion.line_geometry import line_error, body_velocity
from sentry_motion.arbiter_core import ArbiterCore


def test_directed_geometry_and_body_transform():
    e = line_error((0, 0), (0, 2), (-0.1, 1, math.pi / 2))
    assert e.lateral == pytest.approx(0.1)
    assert e.remaining == pytest.approx(1)
    assert e.heading == pytest.approx(0)
    assert body_velocity((0, 0), (0, 2), math.pi / 2, 0.2, -0.1) == pytest.approx(
        (0.2, -0.1)
    )
    reverse = line_error((0, 2), (0, 0), (-0.1, 1, -math.pi / 2))
    assert reverse.lateral == pytest.approx(-0.1)


def test_switch_rejects_motion_and_discards_old_commands():
    c = ArbiterCore()
    c.heartbeat = 1
    assert not c.select("nav", 1)[0]
    c.stopped = True
    assert c.select("nav", 1)[0]
    c.receive("nav", (0.2, 0, 0), 1)
    assert c.output(1.1) == (0.2, 0, 0)
    assert not c.select("tunnel", 1.1)[0]
    c.stop("handoff")
    assert c.select("tunnel", 1.1)[0]
    c.receive("nav", (0.5, 0, 0), 1.11)
    assert c.output(1.11) == (0, 0, 0)
    assert c.output(4.2) == (0, 0, 0)
    assert c.owner == "stop"


def test_heartbeat_loss_and_nonfinite_latch_stop():
    c = ArbiterCore()
    c.stopped = True
    c.heartbeat = 1
    assert c.select("tunnel", 1)[0]
    c.receive("tunnel", (0.2, 0, 0), 1.49)
    assert c.output(1.51) == (0, 0, 0)
    assert c.owner == "stop"
    c.heartbeat = 2
    assert c.select("nav", 2)[0]
    c.receive("nav", (math.nan, 0, 0), 2)
    assert c.owner == "stop"


def test_invalid_line_rejected():
    with pytest.raises(ValueError):
        line_error((0, 0), (0, 0), (0, 0, 0))


def test_operator_stop_cannot_be_undone_by_a_queued_owner_request():
    c = ArbiterCore()
    c.stopped = True
    c.heartbeat = 1
    generation = c.generation
    c.stop("Operator stop")
    assert not c.select("nav", 1, generation)[0]
    assert c.owner == "stop"
    assert c.select("nav", 1, c.generation)[0]


def test_obstacle_envelope_keeps_low_obstacles_and_excludes_roof_and_walls():
    from sentry_motion.obstacle_geometry import blocked_by_points

    assert blocked_by_points([[0.45, 0, 0.08]])
    assert blocked_by_points([[0, 0.32, 0.08]])
    assert blocked_by_points([[-0.35, 0, 0.08]], aligning=True)
    assert not blocked_by_points([[0.45, 0, 0.18], [0.4, 0.4, 0.08], [0.45, 0, -0.08]])
    assert not blocked_by_points([[0, 0, 0.08]], aligning=True)


def test_fast_profile_extends_front_obstacle_guard():
    from sentry_motion.obstacle_geometry import blocked_by_points

    point = [[0.75, 0, 0.08]]
    assert not blocked_by_points(point)
    assert blocked_by_points(point, forward_distance=0.8)
    assert not blocked_by_points([[0.75, 0, 0.20]], forward_distance=0.8)
