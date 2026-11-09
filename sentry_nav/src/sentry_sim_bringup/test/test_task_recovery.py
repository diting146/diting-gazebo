"""Explicit return retry and observed external action ownership, without ROS I/O."""

import json
from types import SimpleNamespace
from uuid import uuid4

from action_msgs.msg import GoalStatus, GoalStatusArray
import pytest
from unique_identifier_msgs.msg import UUID

from test_low_hp_return import (
    Handle, hp, hp_task, route_active, route_request, stop_then_dispatch, tick, trigger,
)


TO_POSE = 'navigate_to_pose'
THROUGH = 'navigate_through_poses'


def status(task, action, goal_id=None, state=GoalStatus.STATUS_EXECUTING):
    message = GoalStatusArray()
    if goal_id is not None:
        entry = GoalStatus()
        entry.goal_info.goal_id = goal_id
        entry.status = state
        message.status_list.append(entry)
    task.node._on_action_status(action, message)


def external_id():
    return UUID(uuid=list(uuid4().bytes))


def task_status(task):
    return json.loads(task.outputs['route_status'][-1].data)


def idle_with_home(task):
    owned = route_active(task)
    owned.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert not task.node.busy and task.node._home_pose is not None
    return task.node._home_pose


def return_terminal(task, outcome):
    hp(task, 49)
    stop_then_dispatch(task)
    handle = Handle(accepted=outcome != 'rejected')
    task.home.requests[-1][1].set_result(handle)
    if handle.accepted:
        handle.result.set_result(SimpleNamespace(status={
            'failed': GoalStatus.STATUS_ABORTED,
            'canceled': GoalStatus.STATUS_CANCELED,
            'succeeded': GoalStatus.STATUS_SUCCEEDED,
        }[outcome]))
    return handle


@pytest.mark.parametrize('outcome', ['failed', 'canceled', 'rejected'])
@pytest.mark.parametrize('full_hp', [False, True])
def test_explicit_retry_keeps_home_and_latch_until_success(hp_task, outcome, full_hp):
    task = hp_task()
    home = idle_with_home(task)
    return_terminal(task, outcome)
    previous_run = task.node._run_id
    hp(task, 100 if full_hp else 49)
    assert task.node._low_hp_latched and not task.node.busy
    response = trigger(task, 'retry_return')
    assert response.success and 'accepted' in response.message
    assert task.node._run_id == previous_run + 1
    assert task.node._home_pose == home and task.node._low_hp_latched
    assert task.node._phase == 'return_stopping'
    assert not trigger(task, 'retry_return').success
    stop_then_dispatch(task)
    assert len(task.home.requests) == 2
    handle = Handle()
    task.home.requests[-1][1].set_result(handle)
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert task.node._low_hp_latched is not full_hp
    assert task.node._home_pose == home
    assert len(task.route.requests) == 1


@pytest.mark.parametrize('case', ['disabled', 'no_home', 'invalidated_home', 'no_latch',
                                  'pending', 'unconfirmed', 'external'])
def test_retry_rejects_unsafe_or_unavailable_state(hp_task, case):
    task = hp_task(enable_low_hp_return=case != 'disabled')
    if case != 'disabled':
        idle_with_home(task)
        return_terminal(task, 'failed')
    if case == 'no_home':
        task.node._home_pose = None
    elif case == 'invalidated_home':
        task.node._home_state = 'invalidated'
    elif case == 'no_latch':
        task.node._low_hp_latched = False
    elif case == 'pending':
        task.node._return_pending = True
    elif case == 'unconfirmed':
        task.node._cancel_requested = True
    elif case == 'external':
        status(task, TO_POSE, external_id())
    before = len(task.home.requests)
    assert not trigger(task, 'retry_return').success
    assert len(task.home.requests) == before


def test_retry_cannot_release_unknown_owned_result(hp_task):
    task = hp_task()
    route = route_active(task)
    hp(task, 49)
    route.result.set_exception(RuntimeError('result lost'))
    assert task.node.busy and task.node._return_pending
    assert not trigger(task, 'retry_return').success
    hp(task, 100)
    assert task.node._low_hp_latched and not trigger(task, 'retry_return').success
    assert not task.home.requests


def test_retry_after_cancel_before_return_dispatch(hp_task):
    task = hp_task()
    idle_with_home(task)
    hp(task, 49)
    assert trigger(task, 'cancel_route').success
    assert task.node._return_state == 'canceled'
    assert trigger(task, 'retry_return').success
    stop_then_dispatch(task)
    assert len(task.home.requests) == 1


def test_initial_unobserved_status_does_not_block_default_route(hp_task):
    task = hp_task()
    route_request(task)
    assert len(task.route.requests) == 1
    assert task_status(task)['external_observation_state'] == 'not_observed'
    assert task.route.goal_ids[0] is not None and any(task.route.goal_ids[0].uuid)
    assert TO_POSE + '/_action/status' in task.subscriptions
    assert THROUGH + '/_action/status' in task.subscriptions


def test_owned_uuid_registered_before_acceptance_is_not_external(hp_task):
    task = hp_task()
    route_request(task)
    own_id = task.route.goal_ids[0]
    status(task, THROUGH, own_id)
    assert not task.node._external_goals
    status(task, TO_POSE, own_id)
    assert task.node._external_goals  # Action name is part of the identity.


def test_observed_uuid_is_sticky_until_matching_real_terminal(hp_task):
    task = hp_task(auto_start=False)
    goal_id = external_id()
    status(task, TO_POSE, goal_id)
    assert not trigger(task, 'start_route').success
    status(task, TO_POSE)  # Omission is not completion.
    status(task, TO_POSE, goal_id, GoalStatus.STATUS_UNKNOWN)
    status(task, THROUGH, goal_id, GoalStatus.STATUS_CANCELED)
    assert task.node._external_goals
    status(task, TO_POSE, goal_id, GoalStatus.STATUS_ABORTED)
    assert not task.node._external_goals
    status(task, TO_POSE, goal_id)  # A delayed stale snapshot cannot resurrect it.
    assert not task.node._external_goals
    assert trigger(task, 'start_route').success


@pytest.mark.parametrize('terminal', [GoalStatus.STATUS_SUCCEEDED,
                                     GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED])
def test_low_hp_blocks_without_competing_commands_then_returns_once(hp_task, terminal):
    task = hp_task()
    home = idle_with_home(task)
    goal_id = external_id()
    status(task, TO_POSE, goal_id)
    outputs_before = {name: len(task.outputs[name]) for name in ('cmd_spin', 'cmd_vel_controller')}
    hp(task, 49)
    blocked_run = task.node._run_id
    assert task.node._phase == 'blocked_external_navigation'
    assert task.node._return_pending and task.node._low_hp_latched
    assert task_status(task)['return_state'] == 'blocked_external_navigation'
    assert not trigger(task, 'retry_return').success
    tick(task, 0.25)
    tick(task, 2.0)
    hp(task, 100)
    assert task.node._return_pending and task.node._low_hp_latched
    assert not task.home.requests
    assert {name: len(task.outputs[name]) for name in outputs_before} == outputs_before
    status(task, TO_POSE, goal_id, terminal)
    assert task.node._phase == 'return_stopping'
    assert task.node._run_id == blocked_run  # Resume the same pending return request.
    stop_then_dispatch(task)
    assert len(task.home.requests) == 1 and task.node._home_pose == home
    tick(task, 1.0)
    status(task, TO_POSE, goal_id, terminal)
    assert len(task.home.requests) == 1


def test_multiple_external_goals_must_all_finish(hp_task):
    task = hp_task()
    idle_with_home(task)
    first, second = external_id(), external_id()
    status(task, TO_POSE, first)
    status(task, THROUGH, second)
    hp(task, 49)
    status(task, TO_POSE, first, GoalStatus.STATUS_SUCCEEDED)
    assert task.node._return_pending and not task.home.requests
    tick(task, 1.0)
    assert task.node._phase == 'blocked_external_navigation'
    status(task, THROUGH, second, GoalStatus.STATUS_CANCELED)
    stop_then_dispatch(task)
    assert len(task.home.requests) == 1


def test_cancel_waiting_return_does_not_cancel_or_command_external_goal(hp_task):
    task = hp_task()
    idle_with_home(task)
    goal_id = external_id()
    status(task, TO_POSE, goal_id)
    hp(task, 49)
    before = len(task.outputs['cmd_vel_controller'])
    assert trigger(task, 'cancel_route').success
    assert not task.node.busy and not task.node._return_pending
    assert task.node._return_state == 'canceled' and task.node._low_hp_latched
    assert len(task.outputs['cmd_vel_controller']) == before
    assert task.node._external_goals and not task.home.requests
    status(task, TO_POSE, goal_id, GoalStatus.STATUS_CANCELED)
    tick(task, 1.0)
    assert not task.home.requests
    assert trigger(task, 'retry_return').success


def test_external_completion_cannot_release_unknown_owned_action(hp_task):
    task = hp_task()
    owned = route_active(task)
    goal_id = external_id()
    status(task, TO_POSE, goal_id)
    hp(task, 49)
    assert task.node._return_state == 'blocked_external_navigation'
    owned.result.set_exception(RuntimeError('unconfirmed action'))
    assert len(owned.cancels) == 1
    status(task, TO_POSE, goal_id, GoalStatus.STATUS_CANCELED)
    tick(task, 1.0)
    assert task.node._phase == 'canceling' and not task.home.requests
    assert not trigger(task, 'retry_return').success


def test_external_appearing_during_return_preparation_pauses_without_commands(hp_task):
    task = hp_task()
    idle_with_home(task)
    hp(task, 49)
    before = len(task.outputs['cmd_vel_controller'])
    goal_id = external_id()
    status(task, TO_POSE, goal_id)
    tick(task, 121.0)  # Waiting for an external owner does not become a readiness timeout.
    assert task.node._return_pending and task.node._return_state == 'blocked_external_navigation'
    assert len(task.outputs['cmd_vel_controller']) == before
    status(task, TO_POSE, goal_id, GoalStatus.STATUS_SUCCEEDED)
    stop_then_dispatch(task)
    assert len(task.home.requests) == 1


def test_clock_reset_keeps_external_uuid_unconfirmed_and_invalidates_home(hp_task):
    task = hp_task()
    idle_with_home(task)
    goal_id = external_id()
    status(task, TO_POSE, goal_id)
    hp(task, 49)
    task.clock[0] = 0
    task.node._tick()
    assert task.node._external_goals and task.node._home_pose is None
    assert task.node._home_state == 'invalidated'
    assert not task.node._return_pending and task.node._return_state == 'failed'
    assert not trigger(task, 'retry_return').success
