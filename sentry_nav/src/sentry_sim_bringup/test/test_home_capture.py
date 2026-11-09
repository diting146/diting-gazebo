"""Initial home capture independent of route dispatch, without a ROS executor."""

import json
import math
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import pytest
from rclpy.time import Time
from std_msgs.msg import String

from test_low_hp_return import Handle, hp, hp_task, trigger


@pytest.fixture
def home_task(hp_task):
    from sentry_sim_bringup.startup_navigation import StartupNavigation

    def create(**overrides):
        task = hp_task(auto_start=False, require_localization_status=True, **overrides)
        transform = TransformStamped()
        transform.transform.translation.x = -4.6
        transform.transform.translation.y = 3.475
        transform.transform.rotation.w = 1.0
        task.transform = transform
        task.node._buffer = SimpleNamespace(lookup_transform=lambda *_args: transform)
        task.node._home_readiness = StartupNavigation._home_readiness.__get__(task.node)
        return task

    return create


def sample(task, seconds=0.0, *, trusted=True, speed=0.0, lateral=0.0, angular=0.0,
           odom_offset=0, tf_offset=0):
    task.clock[0] += int(seconds * 1e9)
    odom = Odometry()
    odom.header.stamp = Time(nanoseconds=task.clock[0] + odom_offset).to_msg()
    odom.twist.twist.linear.x = speed
    odom.twist.twist.linear.y = lateral
    odom.twist.twist.angular.z = angular
    task.node._on_odom(odom)
    task.transform.header.stamp = Time(nanoseconds=task.clock[0] + tf_offset).to_msg()
    task.node._on_localization_status(String(data=json.dumps({
        'stamp_ns': task.clock[0], 'trusted': trusted})))
    task.node._tick()


def capture(task):
    sample(task)
    sample(task, 0.25)
    sample(task, 0.25)
    assert task.node._home_pose is not None


def test_idle_records_home_without_start_or_navigation_servers(home_task):
    task = home_task()
    task.route.ready = task.home.ready = False
    assert task.node._scan is None
    capture(task)
    assert not task.node.busy and not task.route.requests and not task.home.requests
    status = json.loads(task.outputs['route_status'][-1].data)
    assert status['home_state'] == 'captured' and status['home_blocked_reason'] == ''
    assert status['home_pose']['position'] == {'x': -4.6, 'y': 3.475, 'z': 0.0}
    assert status['home_pose']['stamp_ns'] == task.clock[0]


def test_half_second_stationary_interval_is_required(home_task):
    task = home_task()
    sample(task)
    sample(task, 0.49)
    assert task.node._home_pose is None
    sample(task, 0.01)
    assert task.node._home_pose is not None


@pytest.mark.parametrize('motion', [
    {'speed': 0.04}, {'lateral': -0.04}, {'angular': 0.06},
    {'speed': math.nan}, {'lateral': math.inf}, {'angular': math.nan},
])
def test_motion_or_invalid_velocity_restarts_home_static_interval(home_task, motion):
    task = home_task()
    sample(task)
    sample(task, 0.25, **motion)
    sample(task, 0.25)
    sample(task, 0.25)
    assert task.node._home_pose is None
    sample(task, 0.25)
    assert task.node._home_pose is not None


def test_untrusted_registration_restarts_home_interval(home_task):
    task = home_task()
    sample(task)
    sample(task, 0.25, trusted=False)
    sample(task, 0.25)
    sample(task, 0.25)
    assert task.node._home_pose is None
    sample(task, 0.25)
    assert task.node._home_pose is not None


@pytest.mark.parametrize('offsets', [
    {'odom_offset': -1_100_000_000}, {'odom_offset': 1},
    {'tf_offset': -1_100_000_000}, {'tf_offset': 1},
])
def test_stale_or_future_pose_inputs_cannot_record_home(home_task, offsets):
    task = home_task()
    task.clock[0] = 3_000_000_000
    sample(task, **offsets)
    sample(task, 0.5, **offsets)
    assert task.node._home_pose is None
    assert task.node._home_state == 'pending'


@pytest.mark.parametrize('invalid_pose', ['nan_position', 'zero_quaternion'])
def test_invalid_tf_pose_cannot_record_home(home_task, invalid_pose):
    task = home_task()
    if invalid_pose == 'nan_position':
        task.transform.transform.translation.x = math.nan
    else:
        task.transform.transform.rotation.w = 0.0
    sample(task)
    sample(task, 0.5)
    assert task.node._home_pose is None
    assert 'invalid pose' in task.node._home_blocked_reason


def test_first_home_is_not_changed_when_robot_moves_and_stops_elsewhere(home_task):
    task = home_task()
    capture(task)
    first = json.dumps(task.node._home_pose, sort_keys=True)
    task.transform.transform.translation.x = 0.0
    sample(task, 0.25, speed=0.5)
    sample(task, 1.0)
    sample(task, 0.5)
    assert json.dumps(task.node._home_pose, sort_keys=True) == first
    assert not task.route.requests


def test_default_route_waits_for_home_before_dispatch(home_task):
    task = home_task()
    assert trigger(task, 'start_route').success
    sample(task)
    sample(task, 0.49)
    assert not task.route.requests
    sample(task, 0.01)
    assert len(task.route.requests) == 1
    assert task.node._home_state == 'captured'
    assert len(task.route.requests[0][0].poses) == 3


def test_no_hp_route_remains_usable_without_stationary_home(home_task):
    task = home_task(enable_low_hp_return=False)
    assert trigger(task, 'start_route').success
    sample(task, speed=0.2)
    assert len(task.route.requests) == 1
    assert task.node._home_pose is None


@pytest.mark.parametrize('active', [False, True])
def test_clock_reset_invalidates_home_without_recapturing_distant_pose(home_task, active):
    task = home_task()
    capture(task)
    handle = None
    if active:
        assert trigger(task, 'start_route').success
        sample(task)
        handle = Handle()
        task.route.requests[0][1].set_result(handle)
    task.transform.transform.translation.x = -2.5
    task.clock[0] = 500_000_000
    sample(task)
    assert task.node._home_pose is None and task.node._home_state == 'invalidated'
    if handle is not None:
        assert len(handle.cancels) == 1
        handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    sample(task, 0.25)
    sample(task, 0.5)
    assert task.node._home_pose is None
    status = json.loads(task.outputs['route_status'][-1].data)
    assert status['home_state'] == 'invalidated'
    assert 'clock reset' in status['home_blocked_reason']


def test_early_low_hp_then_late_home_does_not_start_return_or_clear_failure_latch(home_task):
    task = home_task()
    hp(task, 49)
    assert task.node._return_state == 'failed' and task.node._low_hp_latched
    capture(task)
    hp(task, 100)
    assert task.node._low_hp_latched
    assert not task.home.requests and not task.route.requests
    assert task.node._return_state == 'failed'
    assert not trigger(task, 'start_route').success


def test_reset_home_blocks_hp_route_even_after_inputs_recover(home_task):
    task = home_task()
    capture(task)
    task.clock[0] = 500_000_000
    sample(task)
    assert trigger(task, 'start_route').success
    sample(task, 0.5)
    sample(task, 0.5)
    assert not task.route.requests
    assert task.node._phase == 'waiting_ready'
    assert task.node._home_state == 'invalidated'
