# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise action ownership and low-HP return races without a ROS executor."""

from concurrent.futures import Future
import json
import math
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import pytest
from rclpy.time import Time
from std_msgs.msg import Int32
from std_srvs.srv import Trigger


class Action:
    def __init__(self):
        self.requests = []
        self.goal_ids = []
        self.ready = True

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, goal_uuid=None):
        future = Future()
        self.goal_ids.append(goal_uuid)
        self.requests.append((goal, future))
        return future


class Handle:
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
def hp_task(monkeypatch):
    from sentry_sim_bringup import startup_navigation as module

    def create(**overrides):
        parameters = {'enable_low_hp_return': True, 'ready_duration_sec': 0.0, **overrides}
        actions = {'navigate_through_poses': Action(), 'navigate_to_pose': Action()}
        clock, logs, outputs, subscriptions = [1_000_000_000], [], {}, {}
        monkeypatch.setattr(module.Node, '__init__', lambda *_args, **_kwargs: None)
        monkeypatch.setattr(module.StartupNavigation, 'declare_parameter',
                            lambda _self, name, default: SimpleNamespace(
                                value=parameters.get(name, default)))
        monkeypatch.setattr(module.StartupNavigation, 'get_clock',
                            lambda _self: SimpleNamespace(now=lambda: Time(nanoseconds=clock[0])))
        monkeypatch.setattr(module.StartupNavigation, 'get_logger',
                            lambda _self: SimpleNamespace(
                                info=logs.append, error=logs.append, warning=logs.append))
        monkeypatch.setattr(module, 'Buffer', lambda: object())
        monkeypatch.setattr(module, 'TransformListener', lambda *_args: object())
        monkeypatch.setattr(module, 'ActionClient', lambda _node, _kind, topic: actions[topic])
        monkeypatch.setattr(module.StartupNavigation, 'create_client', lambda *_args: object())
        monkeypatch.setattr(module.StartupNavigation, 'create_timer', lambda *_args: object())
        monkeypatch.setattr(module.StartupNavigation, 'create_service', lambda *_args: object())

        def subscription(_self, _kind, topic, callback, _qos):
            subscriptions[topic] = callback
            return object()

        def publisher(_self, _kind, topic, _qos):
            outputs[topic] = []
            return SimpleNamespace(publish=outputs[topic].append)

        monkeypatch.setattr(module.StartupNavigation, 'create_subscription', subscription)
        monkeypatch.setattr(module.StartupNavigation, 'create_publisher', publisher)
        node = module.StartupNavigation()
        transform = TransformStamped()
        transform.header.stamp = Time(nanoseconds=clock[0]).to_msg()
        transform.transform.translation.x = -5.1
        transform.transform.translation.y = 3.15
        transform.transform.rotation.z = math.sin(-1.4 / 2)
        transform.transform.rotation.w = math.cos(-1.4 / 2)

        def readiness(now):
            action = actions['navigate_to_pose' if node._task_kind == 'return_home'
                             else 'navigate_through_poses']
            return (node._localization_readiness(now) or
                    ('' if action.ready else 'navigation action is unavailable')), transform

        node._readiness = readiness
        node._home_readiness = lambda now: (node._localization_readiness(now), transform)
        return SimpleNamespace(node=node, route=actions['navigate_through_poses'],
                               home=actions['navigate_to_pose'], clock=clock,
                               outputs=outputs, logs=logs, subscriptions=subscriptions)

    return create


def tick(task, seconds=0.0, speed=0.0, angular=0.0):
    task.clock[0] += int(seconds * 1e9)
    odom = Odometry()
    odom.header.stamp = Time(nanoseconds=task.clock[0]).to_msg()
    odom.twist.twist.linear.x = speed
    odom.twist.twist.angular.z = angular
    task.node._on_odom(odom)
    task.node._tick()


def hp(task, value):
    task.node._on_hp(Int32(data=value))


def route_request(task):
    tick(task)
    tick(task, 0.6)


def route_active(task):
    route_request(task)
    handle = Handle()
    task.route.requests[-1][1].set_result(handle)
    return handle


def stop_then_dispatch(task):
    tick(task)
    tick(task, 0.6)


def trigger(task, name):
    return getattr(task.node, '_' + name)(Trigger.Request(), Trigger.Response())


def test_exact_threshold_and_active_return_ignore_repeated_low_hp(hp_task):
    task = hp_task()
    handle = route_active(task)
    hp(task, 100)
    hp(task, 50)
    assert not handle.cancels and not task.home.requests
    hp(task, 49)
    hp(task, 49)
    assert len(handle.cancels) == 1
    assert task.node._return_pending and task.node._low_hp_latched
    assert not trigger(task, 'start_route').success
    handle.cancels[0].set_result(SimpleNamespace(goals_canceling=[object()]))
    stop_then_dispatch(task)
    assert task.node.busy and not task.home.requests
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    stop_then_dispatch(task)
    assert len(task.home.requests) == 1
    for value in (49, 0, 100, 49):
        hp(task, value)
    assert len(task.home.requests) == 1
    assert task.node._low_hp_latched


def test_successful_return_rearms_at_full_hp_and_returns_to_same_home_again(hp_task):
    task = hp_task()
    route = route_active(task)
    home = json.loads(task.outputs['route_status'][-1].data)['home_pose']
    route.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    hp(task, 49)
    stop_then_dispatch(task)
    first_return = Handle()
    task.home.requests[0][1].set_result(first_return)
    first_return.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    first_result = dict(task.node._return_result)

    for value in (49, 50, 99):
        hp(task, value)
        assert task.node._low_hp_latched and not task.node.busy
        assert not trigger(task, 'start_route').success
    hp(task, 100)
    status = json.loads(task.outputs['route_status'][-1].data)
    assert not status['low_hp_latched'] and not task.node.busy
    assert status['home_pose'] == home
    assert status['return_result'] == first_result
    tick(task, 1.0)
    assert len(task.route.requests) == len(task.home.requests) == 1

    hp(task, 50)
    assert not task.node.busy and len(task.home.requests) == 1
    hp(task, 49)
    status = json.loads(task.outputs['route_status'][-1].data)
    assert status['low_hp_latched'] and status['return_result'] is None
    assert status['home_pose'] == home
    stop_then_dispatch(task)
    assert len(task.home.requests) == 2 and len(task.route.requests) == 1
    goal, response = task.home.requests[1]
    assert goal.pose.header.frame_id == home['frame_id']
    assert goal.pose.pose.position.x == home['position']['x']
    assert goal.pose.pose.position.y == home['position']['y']
    assert goal.pose.pose.orientation.z == home['orientation']['z']
    second_return = Handle()
    response.set_result(second_return)
    second_return.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert task.node._return_result['run_id'] > first_result['run_id']
    assert task.node._return_result['outcome'] == 'succeeded'
    assert task.node._home_pose == home


def test_full_hp_after_success_allows_manual_route_start_without_auto_departure(hp_task):
    task = hp_task()
    route = route_active(task)
    home = json.loads(task.outputs['route_status'][-1].data)['home_pose']
    route.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    hp(task, 49)
    stop_then_dispatch(task)
    returned = Handle()
    task.home.requests[0][1].set_result(returned)
    returned.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    hp(task, 100)
    tick(task, 1.0)
    assert len(task.route.requests) == 1 and not task.node.busy
    assert trigger(task, 'start_route').success
    tick(task)
    assert len(task.route.requests) == 2 and task.node._home_pose == home


def test_second_return_failure_does_not_rearm_using_first_success(hp_task):
    task = hp_task()
    route = route_active(task)
    route.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    hp(task, 49)
    stop_then_dispatch(task)
    first_return = Handle()
    task.home.requests[0][1].set_result(first_return)
    first_return.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    hp(task, 100)
    hp(task, 49)
    stop_then_dispatch(task)
    second_return = Handle()
    task.home.requests[1][1].set_result(second_return)
    hp(task, 100)
    assert task.node._return_result is None and task.node._low_hp_latched
    second_return.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_ABORTED))
    hp(task, 100)
    tick(task, 1.0)
    assert task.node._return_result['outcome'] == 'failed'
    assert task.node._low_hp_latched and not task.node.busy
    assert not trigger(task, 'start_route').success
    assert len(task.home.requests) == 2


@pytest.mark.parametrize('low_again_before_success', [False, True])
def test_full_hp_during_return_never_interrupts_and_only_current_hp_rearms(
        hp_task, low_again_before_success):
    task = hp_task()
    route = route_active(task)
    route.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    hp(task, 49)
    stop_then_dispatch(task)
    returned = Handle()
    task.home.requests[0][1].set_result(returned)
    hp(task, 100)
    tick(task, 0.1, speed=0.2)
    assert task.node.busy and task.node._low_hp_latched
    assert not returned.cancels and len(task.home.requests) == 1
    if low_again_before_success:
        hp(task, 49)
    returned.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    tick(task, 1.0)
    assert not task.node.busy and not task.node._return_pending
    assert len(task.home.requests) == 1
    assert task.node._low_hp_latched is low_again_before_success
    if low_again_before_success:
        hp(task, 49)
        tick(task, 1.0)
        assert len(task.home.requests) == 1
    else:
        hp(task, 49)
        stop_then_dispatch(task)
        assert len(task.home.requests) == 2


def test_return_uses_recorded_actual_pose_and_stops_spin_after_idle_success(hp_task):
    task = hp_task()
    route = route_active(task)
    home = json.loads(task.outputs['route_status'][-1].data)['home_pose']
    route.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert not task.outputs['cmd_spin']  # Successful route may keep its exit spin.
    hp(task, 49)
    assert task.outputs['cmd_spin'][-1].data == 0.0
    tick(task, angular=1.0)
    tick(task, 0.6, angular=0.2)
    assert not task.home.requests
    stop_then_dispatch(task)
    goal, future = task.home.requests[0]
    assert goal.pose.header.frame_id == 'map'
    assert goal.pose.pose.position.x == -5.1 and goal.pose.pose.position.y == 3.15
    assert goal.pose.pose.orientation.z == home['orientation']['z']
    handle = Handle()
    future.set_result(handle)
    stopped_outputs = len(task.outputs['cmd_vel_controller'])
    tick(task, 0.1, speed=0.2)
    assert len(task.outputs['cmd_vel_controller']) == stopped_outputs
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    status = json.loads(task.outputs['route_status'][-1].data)
    assert status['return_result']['outcome'] == 'succeeded'
    assert status['home_pose'] == home and not task.node.busy
    assert task.outputs['cmd_spin'][-1].data == 0.0
    assert task.outputs['cmd_vel_controller'][-1].linear.x == 0.0
    hp(task, 49)
    tick(task, 1.0)
    assert len(task.home.requests) == 1


def test_low_hp_during_goal_send_waits_for_late_acceptance_and_terminal(hp_task):
    task = hp_task()
    route_request(task)
    hp(task, 49)
    stop_then_dispatch(task)
    assert not task.home.requests
    owned = Handle()
    unrelated = Handle()
    task.route.requests[0][1].set_result(owned)
    assert len(owned.cancels) == 1 and not unrelated.cancels
    owned.cancels[0].set_result(SimpleNamespace(goals_canceling=[object()]))
    stop_then_dispatch(task)
    assert not task.home.requests
    owned.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    stop_then_dispatch(task)
    assert len(task.home.requests) == 1


def test_cancel_rejection_or_transport_error_retains_motion_ownership(hp_task):
    task = hp_task()
    handle = route_active(task)
    hp(task, 49)
    handle.cancels[0].set_result(SimpleNamespace(goals_canceling=[]))
    tick(task)
    assert len(handle.cancels) == 2
    handle.cancels[1].set_exception(RuntimeError('cancel transport failed'))
    stop_then_dispatch(task)
    assert task.node.busy and not task.home.requests
    assert not trigger(task, 'start_route').success
    assert len(handle.cancels) == 3


def test_unknown_result_does_not_confirm_cancellation(hp_task):
    task = hp_task()
    handle = route_active(task)
    hp(task, 49)
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_UNKNOWN))
    stop_then_dispatch(task)
    assert task.node.busy and not task.home.requests


@pytest.mark.parametrize('low_before_error', [False, True])
def test_lost_route_goal_response_never_releases_ownership_or_dispatches_home(
        hp_task, low_before_error):
    task = hp_task()
    route_request(task)
    if low_before_error:
        hp(task, 49)
    task.route.requests[0][1].set_exception(RuntimeError('acceptance response was lost'))
    if not low_before_error:
        hp(task, 49)
    stop_then_dispatch(task)
    assert task.node._phase == 'canceling' and task.node.busy
    assert task.node._goal_handle is None and task.node._return_pending
    assert task.node._last_result is None and not task.home.requests
    assert not trigger(task, 'start_route').success
    assert task.outputs['cmd_spin'][-1].data == 0.0


def test_synchronous_route_send_error_keeps_unknown_request_busy(hp_task):
    task = hp_task()

    def send(_goal, goal_uuid=None):
        raise RuntimeError('send may have reached the server')

    task.route.send_goal_async = send
    route_request(task)
    hp(task, 49)
    stop_then_dispatch(task)
    assert task.node.busy and task.node._phase == 'canceling'
    assert task.node._last_result is None and not task.home.requests
    assert not trigger(task, 'start_route').success


@pytest.mark.parametrize('synchronous', [False, True])
def test_unknown_return_send_outcome_stays_busy_and_stopped(hp_task, synchronous):
    task = hp_task()
    route = route_active(task)
    route.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    if synchronous:
        def send(_goal, goal_uuid=None):
            raise RuntimeError('return send outcome unknown')
        task.home.send_goal_async = send
    hp(task, 49)
    stop_then_dispatch(task)
    if not synchronous:
        task.home.requests[0][1].set_exception(RuntimeError('return response lost'))
    stopped_count = len(task.outputs['cmd_vel_controller'])
    tick(task, 0.25)
    assert task.node.busy and task.node._phase == 'canceling'
    assert task.node._return_result is None and not task.node._return_pending
    assert task.node._goal_handle is None
    assert len(task.outputs['cmd_vel_controller']) > stopped_count
    assert task.outputs['cmd_spin'][-1].data == 0.0
    assert not trigger(task, 'start_route').success
    hp(task, 100)
    tick(task, 0.25)
    assert task.node._low_hp_latched and task.node.busy
    assert task.node._return_result is None
    assert not trigger(task, 'start_route').success


def test_missing_home_cancels_start_without_using_origin(hp_task):
    task = hp_task()
    hp(task, 49)
    tick(task, 1.0)
    assert not task.route.requests and not task.home.requests and not task.node.busy
    assert task.node._home_pose is None
    assert task.node._return_result['outcome'] == 'failed'
    assert 'no recorded trusted home' in task.node._return_result['detail']
    hp(task, 100)
    assert task.node._low_hp_latched and not trigger(task, 'start_route').success


def test_return_waits_for_fresh_odom_and_action_server(hp_task):
    task = hp_task()
    handle = route_active(task)
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    hp(task, 49)
    task.node._odom = None
    task.node._tick()
    assert not task.home.requests
    task.home.ready = False
    stop_then_dispatch(task)
    assert task.node._phase == 'return_waiting_ready' and not task.home.requests
    task.home.ready = True
    tick(task, 0.25)
    assert len(task.home.requests) == 1


def test_return_waits_for_trusted_localization(hp_task):
    task = hp_task(require_localization_status=True)
    task.node._localization = {'stamp_ns': task.clock[0], 'trusted': True}
    from time import monotonic
    task.node._localization_received_wall = monotonic()
    handle = route_active(task)
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    task.node._localization['trusted'] = False
    hp(task, 49)
    stop_then_dispatch(task)
    assert not task.home.requests
    task.node._localization = {'stamp_ns': task.clock[0], 'trusted': True}
    task.node._localization_received_wall = monotonic()
    tick(task)
    assert len(task.home.requests) == 1


@pytest.mark.parametrize('outcome', ['rejected', 'failed'])
def test_return_rejected_or_failed_stays_stopped_without_retry(hp_task, outcome):
    task = hp_task()
    route = route_active(task)
    route.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    hp(task, 49)
    stop_then_dispatch(task)
    handle = Handle(accepted=outcome != 'rejected')
    task.home.requests[0][1].set_result(handle)
    if handle.accepted:
        handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_ABORTED))
    tick(task, 1.0)
    assert task.node._return_result['outcome'] == outcome
    assert not task.node.busy and len(task.home.requests) == 1
    assert task.outputs['cmd_spin'][-1].data == 0.0
    hp(task, 100)
    tick(task, 1.0)
    assert task.node._low_hp_latched and not trigger(task, 'start_route').success
    assert len(task.home.requests) == 1


def test_cancel_pending_return_disables_automatic_handoff(hp_task):
    task = hp_task()
    route = route_active(task)
    hp(task, 49)
    assert trigger(task, 'cancel_route').success
    route.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    stop_then_dispatch(task)
    assert not task.node.busy and not task.home.requests
    assert task.node._return_state == 'canceled'
    hp(task, 100)
    assert task.node._low_hp_latched and not trigger(task, 'start_route').success


def test_cancel_return_itself_waits_for_terminal_and_does_not_restart(hp_task):
    task = hp_task()
    route = route_active(task)
    route.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    hp(task, 49)
    stop_then_dispatch(task)
    handle = Handle()
    task.home.requests[0][1].set_result(handle)
    assert trigger(task, 'cancel_route').success
    handle.cancels[0].set_result(SimpleNamespace(goals_canceling=[object()]))
    tick(task, 1.0)
    assert task.node.busy
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    hp(task, 49)
    tick(task, 1.0)
    assert task.node._return_result['outcome'] == 'canceled' and not task.node.busy
    assert len(task.home.requests) == 1
    hp(task, 100)
    tick(task, 1.0)
    assert task.node._low_hp_latched and not trigger(task, 'start_route').success
    assert len(task.home.requests) == 1


def test_clock_reset_invalidates_home_and_cancels_owned_return(hp_task):
    task = hp_task()
    route = route_active(task)
    route.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    hp(task, 49)
    stop_then_dispatch(task)
    handle = Handle()
    task.home.requests[0][1].set_result(handle)
    task.clock[0] = 0
    task.node._tick()
    assert task.node._home_pose is None and len(handle.cancels) == 1
    assert task.node.busy
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    tick(task, 1.0)
    assert len(task.home.requests) == 1 and not task.node.busy


def test_disabled_low_hp_return_does_not_subscribe_or_create_motion_publishers(hp_task):
    task = hp_task(enable_low_hp_return=False, auto_start=False)
    assert 'hp' not in task.subscriptions
    assert 'cmd_spin' not in task.outputs and 'cmd_vel_controller' not in task.outputs
    hp(task, 49)
    assert not task.node._low_hp_latched and trigger(task, 'start_route').success
