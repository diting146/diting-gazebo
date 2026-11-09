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

"""Imports and fixed-route task callbacks, without starting ROS nodes or Gazebo."""


from concurrent.futures import Future
import json
from types import SimpleNamespace

import pytest


def test_package_imports():
    """The top-level package must be importable."""
    import sentry_sim_bringup

    assert sentry_sim_bringup is not None


def test_gateway_module_imports():
    """The gateway module must expose a callable ``main``."""
    from sentry_sim_bringup import sim_serial_gateway

    assert callable(sim_serial_gateway.main)


# Exercise the real task callbacks with controlled futures and mocked ROS I/O.
# No node, executor, action server or simulator is started by these tests.
class _RouteAction:
    def __init__(self):
        self.requests = []

    def send_goal_async(self, goal, goal_uuid=None):
        future = Future()
        self.requests.append((goal, future))
        return future

    def server_is_ready(self):
        return True


class _RouteHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result = Future()
        self.cancels = []

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        future = Future()
        self.cancels.append(future)
        return future


@pytest.fixture
def route_task(monkeypatch):
    from rclpy.time import Time
    from sentry_sim_bringup import startup_navigation as module

    def create(overrides=None):
        parameters = overrides or {}
        action = _RouteAction()
        clock = [1_000_000_000]
        statuses, logs, services = [], [], {}
        monkeypatch.setattr(module.Node, '__init__', lambda *_args, **_kwargs: None)
        monkeypatch.setattr(module.StartupNavigation, 'declare_parameter',
                            lambda _self, name, default: SimpleNamespace(
                                value=parameters.get(name, default)))
        monkeypatch.setattr(module.StartupNavigation, 'get_clock',
                            lambda _self: SimpleNamespace(now=lambda: Time(nanoseconds=clock[0])))
        logger = SimpleNamespace(info=logs.append, error=logs.append, warning=logs.append)
        monkeypatch.setattr(module.StartupNavigation, 'get_logger', lambda _self: logger)
        monkeypatch.setattr(module, 'Buffer', lambda: object())
        monkeypatch.setattr(module, 'TransformListener', lambda *_args: object())
        monkeypatch.setattr(module, 'ActionClient', lambda *_args: action)
        monkeypatch.setattr(module.StartupNavigation, 'create_client', lambda *_args: object())
        monkeypatch.setattr(module.StartupNavigation, 'create_subscription',
                            lambda *_args: object())
        monkeypatch.setattr(module.StartupNavigation, 'create_timer', lambda *_args: object())
        monkeypatch.setattr(module.StartupNavigation, 'create_publisher',
                            lambda *_args: SimpleNamespace(
                                publish=lambda message: statuses.append(json.loads(message.data))))

        def service(_self, _kind, name, callback):
            services[name] = callback
            return object()

        monkeypatch.setattr(module.StartupNavigation, 'create_service', service)
        node = module.StartupNavigation()
        node._readiness = lambda _now: ('', None)
        node._home_readiness = lambda _now: ('no home pose supplied by this fixture', None)
        return SimpleNamespace(node=node, action=action, clock=clock,
                               statuses=statuses, logs=logs, services=services)

    return create


def _ready_and_send(task):
    from nav_msgs.msg import Odometry
    from rclpy.time import Time

    def update_odom():
        odom = Odometry()
        odom.header.stamp = Time(nanoseconds=task.clock[0]).to_msg()
        task.node._on_odom(odom)

    update_odom()
    task.node._tick()
    task.clock[0] += 3_100_000_000
    update_odom()
    task.node._tick()


def _trigger(task, name):
    from std_srvs.srv import Trigger
    return task.services[name](Trigger.Request(), Trigger.Response())


def test_route_default_sends_exact_three_poses_once_and_remains_reusable(route_task):
    from action_msgs.msg import GoalStatus

    task = route_task()
    assert set(task.services) == {'start_route', 'cancel_route', 'retry_return'}
    assert task.node._phase == 'waiting_ready'
    _ready_and_send(task)
    assert len(task.action.requests) == 1
    goal, response = task.action.requests[0]
    assert [(pose.pose.position.x, pose.pose.position.y) for pose in goal.poses] == [
        (-4.0, 3.475), (-2.5, 3.475), (0.0, 0.0)]
    assert all(pose.header.frame_id == 'map' and pose.pose.orientation.z == 0.0 and
               pose.pose.orientation.w == 1.0 for pose in goal.poses)
    handle = _RouteHandle()
    response.set_result(handle)
    assert task.node._phase == 'active'
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert not task.node.busy
    assert task.statuses[-1]['state'] == 'idle'
    last = task.statuses[-1]['last_result']
    assert last['outcome'] == 'succeeded'
    assert last['run_id'] == 1 and last['elapsed_sec'] >= last['navigation_elapsed_sec'] >= 0
    task.node._tick()
    task.node._tick()
    assert len(task.action.requests) == 1
    assert _trigger(task, 'start_route').success
    assert task.node._run_id == 2
    _ready_and_send(task)
    assert len(task.action.requests) == 2


def test_route_busy_rejects_requests_and_cancel_ack_is_not_terminal(route_task):
    from action_msgs.msg import GoalStatus

    task = route_task({'auto_start': False})
    assert _trigger(task, 'start_route').success
    assert not _trigger(task, 'start_route').success  # Waiting for readiness.
    _ready_and_send(task)
    assert not _trigger(task, 'start_route').success  # Awaiting acceptance.
    handle = _RouteHandle()
    task.action.requests[0][1].set_result(handle)
    assert not _trigger(task, 'start_route').success  # Active.
    assert _trigger(task, 'cancel_route').success
    assert len(handle.cancels) == 1
    assert not _trigger(task, 'start_route').success
    handle.cancels[0].set_result(SimpleNamespace(goals_canceling=[object()]))
    assert task.node._phase == 'canceling'
    assert not _trigger(task, 'start_route').success
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert task.node._last_result['outcome'] == 'canceled'
    assert _trigger(task, 'start_route').success
    assert len(task.action.requests) == 1  # No background retry or queued duplicate.


def test_route_cancels_only_owned_late_acceptance(route_task):
    from action_msgs.msg import GoalStatus

    task = route_task()
    _ready_and_send(task)
    unrelated = _RouteHandle()
    assert _trigger(task, 'cancel_route').success
    assert task.node._phase == 'canceling'
    assert not _trigger(task, 'start_route').success
    owned = _RouteHandle()
    task.action.requests[0][1].set_result(owned)
    assert len(owned.cancels) == 1
    assert not unrelated.cancels
    owned.cancels[0].set_result(SimpleNamespace(goals_canceling=[object()]))
    assert task.node.busy
    owned.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert not task.node.busy
    assert task.node._last_result['outcome'] == 'canceled'
    assert len(task.action.requests) == 1


def test_route_can_cancel_while_waiting_without_sending_motion(route_task):
    task = route_task()
    assert _trigger(task, 'cancel_route').success
    assert not task.node.busy
    assert not task.action.requests
    assert task.node._last_result['navigation_elapsed_sec'] is None
    assert not _trigger(task, 'cancel_route').success
    assert _trigger(task, 'start_route').success


@pytest.mark.parametrize('accepted', [False, True])
def test_route_rejection_or_abort_does_not_retry_and_accepts_new_trigger(route_task, accepted):
    from action_msgs.msg import GoalStatus

    task = route_task()
    _ready_and_send(task)
    handle = _RouteHandle(accepted)
    task.action.requests[0][1].set_result(handle)
    if accepted:
        handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_ABORTED))
    assert not task.node.busy
    assert task.node._last_result['outcome'] == ('failed' if accepted else 'rejected')
    task.node._tick()
    assert len(task.action.requests) == 1
    assert _trigger(task, 'start_route').success


def test_route_result_transport_error_does_not_release_unconfirmed_motion(route_task):
    task = route_task()
    _ready_and_send(task)
    handle = _RouteHandle()
    task.action.requests[0][1].set_result(handle)
    handle.result.set_exception(RuntimeError('result transport lost'))
    assert task.node._phase == 'canceling'
    assert len(handle.cancels) == 1
    assert not _trigger(task, 'start_route').success
    assert task.node._last_result is None


def test_route_old_result_cannot_reset_a_new_requested_run(route_task):
    from action_msgs.msg import GoalStatus

    task = route_task()
    _ready_and_send(task)
    handle = _RouteHandle()
    task.action.requests[0][1].set_result(handle)
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert _trigger(task, 'start_route').success
    task.node._on_result(1, handle.result)
    assert task.node._run_id == 2 and task.node._phase == 'waiting_ready'
    assert task.node._last_result['run_id'] == 1


def test_route_startup_timeout_can_be_triggered_again(route_task):
    task = route_task()
    task.node._readiness = lambda _now: ('not ready', None)
    task.node._tick()
    task.clock[0] += 121_000_000_000
    task.node._tick()
    assert not task.node.busy
    assert task.node._last_result['outcome'] == 'failed'
    assert not task.action.requests
    assert _trigger(task, 'start_route').success
    task.node._readiness = lambda _now: ('', None)
    _ready_and_send(task)
    assert len(task.action.requests) == 1


@pytest.mark.parametrize('points', [[], [0.0, 0.0], [0.0] * 8,
                                    [-4.0, 3.475, -2.5, float('nan'), 0.0, 0.0],
                                    [-4.0, 3.475, -2.5, float('inf'), 0.0, 0.0]])
def test_route_points_require_three_finite_xy_pairs(route_task, points):
    with pytest.raises(ValueError, match='three finite XY pairs'):
        route_task({'route_points': points})


@pytest.mark.parametrize('explicit', [False, True])
def test_standalone_task_uses_scene_defaults_and_preserves_explicit_route(
        route_task, tmp_path, explicit):
    import math
    import yaml
    from pathlib import Path
    from sentry_sim_bringup.scene_geometry import default_scene_file

    scene = yaml.safe_load(Path(default_scene_file()).read_text())
    scene['default_route'] = {'points': [[-4.1, 3.475], [-2.4, 3.475], [0.1, 0.0]],
                              'yaw': 0.25}
    filename = tmp_path / 'task_scene.yaml'
    filename.write_text(yaml.safe_dump(scene))
    parameters = {'scene_file': str(filename)}
    expected = [-4.1, 3.475, -2.4, 3.475, 0.1, 0.0]
    yaw = 0.25
    if explicit:
        expected, yaw = [1., 2., 3., 4., 5., 6.], 0.5
        parameters.update(route_points=expected, route_yaw=yaw)
    task = route_task(parameters)
    assert task.node._route == expected
    assert task.node._route_yaw == yaw
    _ready_and_send(task)
    goal = task.action.requests[0][0]
    assert [(pose.pose.position.x, pose.pose.position.y) for pose in goal.poses] == (
        list(zip(expected[::2], expected[1::2])))
    assert goal.poses[0].pose.orientation.z == pytest.approx(math.sin(yaw / 2.0))


@pytest.fixture
def gimbal_tf(monkeypatch):
    """Exercise timestamp throttling with captured TF output and no ROS executor."""
    from rclpy.time import Time
    from sentry_sim_bringup import sentry_gimbal_tf as module

    def create(rate=100.0):
        transforms, logs = [], []
        clock = [42_000_000_000]
        monkeypatch.setattr(module.Node, '__init__', lambda *_args, **_kwargs: None)
        monkeypatch.setattr(module.SentryGimbalTf, 'declare_parameter',
                            lambda _self, name, default: SimpleNamespace(
                                value=rate if name == 'max_publish_rate_hz' else default))
        monkeypatch.setattr(module.SentryGimbalTf, 'get_clock',
                            lambda _self: SimpleNamespace(now=lambda: Time(nanoseconds=clock[0])))
        monkeypatch.setattr(module.SentryGimbalTf, 'get_logger',
                            lambda _self: SimpleNamespace(info=logs.append, warning=logs.append))
        monkeypatch.setattr(module.SentryGimbalTf, 'create_subscription',
                            lambda *_args: object())
        monkeypatch.setattr(module, 'StaticTransformBroadcaster',
                            lambda *_args: SimpleNamespace(sendTransform=lambda _transforms: None))
        monkeypatch.setattr(module, 'TransformBroadcaster',
                            lambda *_args: SimpleNamespace(sendTransform=transforms.append))
        return SimpleNamespace(node=module.SentryGimbalTf(), transforms=transforms,
                               logs=logs, clock=clock)

    return create


def _gimbal_sample(stamp_ns, position=0.0, name='gimbal_yaw_joint'):
    from sensor_msgs.msg import JointState
    msg = JointState()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.name = [name]
    msg.position = [position]
    return msg


def test_gimbal_tf_drops_intermediate_samples_and_preserves_source_pose(gimbal_tf):
    import math
    sample = gimbal_tf()
    for stamp, yaw in [(1_000_000_000, 0.1), (1_001_000_000, 0.2),
                       (1_009_999_999, 0.3), (1_010_000_000, 0.4)]:
        sample.node._on_joint_states(_gimbal_sample(stamp, yaw))
    assert len(sample.transforms) == 2
    assert [(t.header.stamp.sec, t.header.stamp.nanosec) for t in sample.transforms] == [
        (1, 0), (1, 10_000_000)]
    assert sample.transforms[-1].transform.rotation.z == pytest.approx(math.sin(0.4 / 2))
    assert sample.transforms[-1].header.frame_id == 'gimbal_yaw_odom'
    assert sample.transforms[-1].child_frame_id == 'gimbal_yaw'


def test_gimbal_tf_limits_1000_hz_joint_stream_to_100_hz(gimbal_tf):
    sample = gimbal_tf()
    for index in range(1000):
        sample.node._on_joint_states(_gimbal_sample(1_000_000_000 + index * 1_000_000))
    assert len(sample.transforms) == 100
    assert sample.transforms[0].header.stamp.sec == 1
    assert sample.transforms[-1].header.stamp.nanosec == 990_000_000


def test_gimbal_tf_resumes_immediately_after_source_time_reversal(gimbal_tf):
    sample = gimbal_tf()
    for stamp in (10_000_000_000, 1_000_000_000, 1_001_000_000, 1_010_000_000):
        sample.node._on_joint_states(_gimbal_sample(stamp))
    assert [(t.header.stamp.sec, t.header.stamp.nanosec) for t in sample.transforms] == [
        (10, 0), (1, 0), (1, 10_000_000)]


def test_gimbal_tf_zero_rate_keeps_unlimited_source_updates(gimbal_tf):
    sample = gimbal_tf(0.0)
    for index in range(10):
        sample.node._on_joint_states(_gimbal_sample(1_000_000_000 + index))
    assert len(sample.transforms) == 10


def test_gimbal_tf_missing_joint_does_not_advance_published_time(gimbal_tf):
    sample = gimbal_tf()
    sample.node._on_joint_states(_gimbal_sample(2_000_000_000, name='other_joint'))
    assert sample.node._last_published_stamp_ns is None
    sample.node._on_joint_states(_gimbal_sample(1_000_000_000))
    assert len(sample.transforms) == 1


def test_gimbal_tf_zero_stamp_retains_existing_current_clock_fallback(gimbal_tf):
    sample = gimbal_tf()
    sample.node._on_joint_states(_gimbal_sample(0))
    assert sample.transforms[0].header.stamp.sec == 42


@pytest.mark.parametrize('rate', [-1.0, float('inf'), float('nan')])
def test_gimbal_tf_rejects_invalid_rate(gimbal_tf, rate):
    with pytest.raises(ValueError, match='max_publish_rate_hz'):
        gimbal_tf(rate)


def _registration(task, stamp=None, trusted=True, **extra):
    from std_msgs.msg import String
    status = {'stamp_ns': task.clock[0] if stamp is None else stamp, 'trusted': trusted, **extra}
    task.node._on_localization_status(String(data=json.dumps(status)))


def test_lio_readiness_rejects_fresh_tf_without_registration(route_task):
    task = route_task({'require_localization_status': True})
    assert task.node._localization_readiness(task.clock[0]) == 'map registration is not trusted'
    _registration(task, trusted=False)
    assert task.node._localization_readiness(task.clock[0]) == 'map registration is not trusted'
    _registration(task)
    assert task.node._localization_readiness(task.clock[0]) == ''


def test_ground_truth_does_not_require_gicp_status(route_task):
    task = route_task({'require_localization_status': False})
    assert task.node._localization_readiness(task.clock[0]) == ''


def test_registration_old_future_and_receiver_expiry_do_not_pass(route_task, monkeypatch):
    from sentry_sim_bringup import startup_navigation as module
    wall = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    task = route_task({'require_localization_status': True})
    _registration(task, stamp=task.clock[0] + 1)
    assert task.node._localization_readiness(task.clock[0]) == 'map registration status is stale'
    task.clock[0] += 2_000_000_000
    _registration(task, stamp=1_000_000_000)
    assert task.node._localization_readiness(task.clock[0]) == 'map registration status is stale'
    _registration(task)
    wall[0] += 1.1
    assert task.node._localization_readiness(task.clock[0]) == 'map registration status is stale'


@pytest.mark.parametrize('message', ['null', '[]', '{"stamp_ns": 1, "trusted": 1}',
                                     '{"stamp_ns": true, "trusted": true}',
                                     '{"stamp_ns": 0, "trusted": true}', 'broken'])
def test_invalid_registration_message_clears_readiness(route_task, message):
    from std_msgs.msg import String
    task = route_task({'require_localization_status': True})
    _registration(task)
    task.node._ready_since = task.clock[0]
    task.node._on_localization_status(String(data=message))
    assert task.node._ready_since is None
    assert task.node._localization_readiness(task.clock[0]) == 'map registration is not trusted'


def test_registration_failure_restarts_stable_period_before_goal(route_task):
    task = route_task({'require_localization_status': True})
    task.node._readiness = lambda now: (task.node._localization_readiness(now), None)
    _registration(task)
    task.node._tick()
    task.clock[0] += 2_000_000_000
    _registration(task, trusted=False)
    task.node._tick()
    assert task.node._ready_since is None and not task.action.requests
    for _index in range(4):
        task.clock[0] += 1_000_000_000
        _registration(task)
        task.node._tick()
    assert len(task.action.requests) == 1


def test_clock_reversal_clears_registration_and_recorded_home(route_task):
    task = route_task({'require_localization_status': True})
    task.node._readiness = lambda now: (task.node._localization_readiness(now), None)
    _registration(task)
    task.node._tick()
    task.node._home_pose = {'frame_id': 'map'}
    task.clock[0] -= 500_000_000
    task.node._tick()
    assert task.node._home_pose is None
    assert task.node._localization is None
    assert not task.action.requests


def test_home_is_confirmed_actual_pose_and_is_not_overwritten_on_repeat(route_task):
    from geometry_msgs.msg import TransformStamped
    from action_msgs.msg import GoalStatus
    task = route_task()
    transform = TransformStamped()
    transform.header.stamp.sec = 1
    transform.transform.translation.x = -5.1
    transform.transform.translation.y = 3.15
    transform.transform.rotation.w = 1.0
    task.node._readiness = lambda _now: ('', transform)
    task.node._home_readiness = lambda _now: ('', transform)
    _ready_and_send(task)
    home = task.statuses[-1]['home_pose']
    assert home['position'] == {'x': -5.1, 'y': 3.15, 'z': 0.0}
    assert home['stamp_ns'] == 1_000_000_000
    handle = _RouteHandle()
    task.action.requests[0][1].set_result(handle)
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert _trigger(task, 'start_route').success
    transform.transform.translation.x = 0.0
    _ready_and_send(task)
    assert task.statuses[-1]['home_pose'] == home


@pytest.mark.parametrize('phase', ['idle', 'waiting_ready'])
@pytest.mark.parametrize('reset_time', [0, 500_000_000])
def test_clock_reset_publishes_cleared_home_even_when_idle(route_task, phase, reset_time):
    task = route_task({'auto_start': False, 'require_localization_status': True})
    task.node._readiness = lambda now: (task.node._localization_readiness(now), None)
    task.node._phase = phase
    task.node._last_tick_ns = 1_000_000_000
    task.node._home_pose = {'frame_id': 'map', 'stamp_ns': 1_000_000_000}
    task.node._ready_since = 1_000_000_000
    task.node._publish_status()
    assert task.statuses[-1]['home_pose'] is not None
    task.clock[0] = reset_time
    task.node._tick()
    assert task.statuses[-1]['home_pose'] is None
    assert task.node._ready_since is None
    assert not task.action.requests
