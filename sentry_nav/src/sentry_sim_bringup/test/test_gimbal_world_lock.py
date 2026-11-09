# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License").

"""World-yaw hold through repeated turns and incomplete sensor feedback."""

import math
from types import SimpleNamespace

import pytest

from sentry_sim_bringup.gimbal_world_lock_control import WorldYawHold, wrap_angle


def measure(hold, now, yaw=0.0, rate=0.0, joint=0.0, stamp=None):
    stamp = now if stamp is None else stamp
    hold.update_feedback(yaw, rate, stamp, now)
    hold.update_joint(joint, stamp, now)
    return hold.command(now)


@pytest.mark.parametrize('rate', [-3.14, 3.14])
def test_continuous_rotation_preserves_world_target_and_joint_turns(rate):
    hold = WorldYawHold()
    measure(hold, 1_000_000_000)
    commands = []
    for step in range(1, 601):
        angle = rate * step * 0.02
        command = measure(hold, 1_000_000_000 + step * 20_000_000,
                          wrap_angle(angle), rate, -angle)
        assert command == pytest.approx(-angle - rate * 0.17, abs=1e-10)
        assert hold.target_yaw == 0.0
        commands.append(command)
    assert max(abs(b - a) for a, b in zip(commands, commands[1:])) < 0.064
    assert abs(commands[-1]) > 6 * math.pi


def test_heading_wrap_crossing_keeps_position_command_continuous():
    hold = WorldYawHold(feedforward_time_sec=0.0)
    measure(hold, 1_000_000_000, math.pi - 0.01, joint=0.0)
    command = measure(hold, 1_020_000_000, -math.pi + 0.01, joint=-0.02)
    assert command == pytest.approx(-0.02)


def test_integral_removes_a_stationary_servo_bias():
    hold = WorldYawHold(feedforward_time_sec=0.0)
    joint = 0.0
    measure(hold, 1_000_000_000)
    # A simple servo with a fixed 0.05-rad bias: proportional position commands
    # alone settle off target. Integrating measured world error removes it.
    for step in range(1, 1201):
        command = measure(hold, 1_000_000_000 + step * 10_000_000, joint=joint)
        joint += 4.0 * (command - joint - 0.05) * 0.01
    assert abs(joint) < 1e-5
    assert hold.integral == pytest.approx(0.05, abs=1e-5)


def test_integral_is_bounded_and_can_unwind():
    hold = WorldYawHold()
    measure(hold, 1_000_000_000)
    for step in range(1, 101):
        command = measure(hold, 1_000_000_000 + step * 20_000_000, yaw=0.5)
        assert math.isfinite(command)
    assert hold.integral == pytest.approx(-0.35)
    for step in range(101, 201):
        measure(hold, 1_000_000_000 + step * 20_000_000, yaw=-0.5)
    assert hold.integral == pytest.approx(0.35)


def test_both_feedback_streams_are_required():
    hold = WorldYawHold()
    assert hold.command(1_000_000_000) is None
    hold.update_feedback(0.0, 0.0, 1_020_000_000, 1_020_000_000)
    assert hold.command(1_020_000_000) is None
    hold.update_joint(0.0, 1_040_000_000, 1_040_000_000)
    assert hold.command(1_040_000_000) == 0.0


@pytest.mark.parametrize('kind', ['yaw', 'rate', 'joint'])
@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_measurement_invalidates_old_feedback(kind, value):
    hold = WorldYawHold()
    measure(hold, 1_000_000_000)
    values = {'yaw': 0.0, 'rate': 0.0, 'joint': 0.0}
    values[kind] = value
    assert measure(hold, 1_020_000_000, **values) is None


def test_stale_source_and_stale_reception_both_block_commands():
    hold = WorldYawHold()
    measure(hold, 1_000_000_000)
    measure(hold, 1_020_000_000, yaw=0.1)
    assert hold.integral < 0.0
    assert hold.command(1_300_000_000) is None
    assert hold.integral == 0.0
    # Re-delivery of an old sensor message does not make its data current.
    assert measure(hold, 1_320_000_000, stamp=1_000_000_000) is None
    assert measure(hold, 1_340_000_000) is not None


def test_clock_rollback_clears_old_data_integral_and_reference():
    hold = WorldYawHold()
    measure(hold, 2_000_000_000)
    measure(hold, 2_020_000_000, yaw=0.5)
    assert hold.integral < 0.0
    assert hold.command(1_000_000_000) is None
    assert hold.integral == 0.0 and hold.target_yaw is None
    assert hold.command(1_020_000_000) is None
    assert measure(hold, 1_040_000_000, yaw=1.0, joint=0.2) == pytest.approx(0.2)
    assert hold.target_yaw == pytest.approx(1.2)


def test_long_timer_gap_never_integrates_an_unobserved_interval():
    hold = WorldYawHold()
    measure(hold, 1_000_000_000)
    measure(hold, 2_000_000_000, yaw=0.5)
    assert hold.integral == 0.0
    measure(hold, 2_020_000_000, yaw=0.5)
    assert hold.integral == pytest.approx(-0.02)


def test_small_sensor_clock_lead_is_allowed_but_old_epoch_is_not():
    hold = WorldYawHold()
    assert measure(hold, 1_000_000_000, stamp=1_040_000_000) == 0.0
    assert measure(hold, 1_020_000_000, stamp=1_200_000_000) is None


@pytest.mark.parametrize('parameter,value', [
    ('feedforward_time_sec', math.nan), ('integral_gain', -1.0),
    ('integral_limit', math.inf), ('feedback_timeout_sec', 0.0),
])
def test_invalid_configuration_is_rejected(parameter, value):
    with pytest.raises(ValueError):
        WorldYawHold(**{parameter: value})


@pytest.mark.parametrize('q', [
    [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, math.nan, 1.0],
    [0.0, math.inf, 0.0, 1.0],
])
def test_invalid_orientation_is_rejected_before_yaw_feedback(q):
    from sentry_sim_bringup.sensor_gimbal_world_lock import _yaw_from_quaternion

    with pytest.raises(ValueError):
        _yaw_from_quaternion(SimpleNamespace(**dict(zip(('x', 'y', 'z', 'w'), q))))
