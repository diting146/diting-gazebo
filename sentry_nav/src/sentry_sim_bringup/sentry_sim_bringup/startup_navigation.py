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

"""Own one navigation goal at a time, including an optional low-HP return home."""

import json
import math
import time
from uuid import uuid4

from action_msgs.msg import GoalStatus, GoalStatusArray
from example_interfaces.msg import Float32
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from sentry_sim_bringup.scene_geometry import default_scene_file, load_scene_geometry, resolve_route
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, qos_profile_action_status_default, qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from unique_identifier_msgs.msg import UUID


class StartupNavigation(Node):
    """One owned action at a time; a terminal result makes the next trigger possible."""

    SERVERS = ('planner_server', 'controller_server', 'bt_navigator', 'velocity_smoother')
    RETURN_REARM_HP = 100
    ACTION_NAMES = ('navigate_to_pose', 'navigate_through_poses')
    TERMINAL_STATUSES = (
        GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
    )

    def __init__(self, **kwargs):
        super().__init__('startup_navigation', **kwargs)
        self._base_frame = self.declare_parameter('base_frame', 'sentry_robot1/chassis').value
        self._ready_duration = float(self.declare_parameter('ready_duration_sec', 3.0).value)
        self._startup_timeout = float(self.declare_parameter('startup_timeout_sec', 120.0).value)
        self._freshness = float(self.declare_parameter('input_timeout_sec', 1.0).value)
        self._require_localization = bool(
            self.declare_parameter('require_localization_status', False).value)
        timings = (self._ready_duration, self._startup_timeout, self._freshness)
        if (not all(math.isfinite(value) for value in timings) or
                self._ready_duration < 0 or self._startup_timeout <= 0 or self._freshness <= 0):
            raise ValueError('Invalid startup timing parameters')
        scene_file = self.declare_parameter('scene_file', default_scene_file()).value
        scene = load_scene_geometry(scene_file)
        self._route, self._route_yaw = resolve_route(
            scene, self.declare_parameter('route_points', list(scene.route_points)).value,
            self.declare_parameter('route_yaw', scene.route_yaw).value)
        if (len(self._route) != 6 or
                any(isinstance(value, bool) or not math.isfinite(float(value))
                    for value in self._route) or not math.isfinite(self._route_yaw)):
            raise ValueError('route_points must contain three finite XY pairs; route_yaw is finite')
        self._route = [float(value) for value in self._route]
        auto_start = self.declare_parameter('auto_start', True).value
        self._enable_low_hp_return = bool(
            self.declare_parameter('enable_low_hp_return', False).value)
        self._hp_topic = self.declare_parameter('hp_topic', 'hp').value
        self._return_hp_threshold = self.declare_parameter('return_hp_threshold', 50).value
        if (not isinstance(self._hp_topic, str) or not self._hp_topic.strip() or
                type(self._return_hp_threshold) is not int or self._return_hp_threshold < 0):
            raise ValueError('hp_topic must be nonempty and return_hp_threshold a nonnegative int')
        self._hp = None
        self._low_hp_latched = False
        self._return_pending = False
        self._return_state = 'not_requested'
        self._return_result = None
        self._task_kind = 'route'
        self._stopped_since = None
        self._localization = None
        self._localization_received_wall = None
        self._home_pose = None
        self._home_state = 'pending'
        self._home_blocked_reason = 'waiting for trusted stationary map pose'
        self._home_stopped_since = None
        self._last_tick_ns = None
        self._odom = None
        self._scan = None
        self._ready_since = None
        self._started = None
        self._last_log = None
        self._state_requests = {}
        self._active_at = {}
        self._phase = 'idle'
        self._run_id = 0
        self._goal_handle = None
        self._cancel_requested = False
        self._cancel_sent = False
        self._requested_wall = None
        self._sent_wall = None
        self._last_result = None
        self._owned_goal_ids = set()
        self._external_goals = {}
        self._external_terminal_ids = set()
        self._observed_action_status = set()
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._action = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._return_action = None
        self._spin_pub = None
        self._stop_pub = None
        if self._enable_low_hp_return:
            self._return_action = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self._spin_pub = self.create_publisher(Float32, 'cmd_spin', 1)
            self._stop_pub = self.create_publisher(Twist, 'cmd_vel_controller', 1)
            self.create_subscription(Int32, self._hp_topic, self._on_hp, 10)
        self._lifecycle_clients = {
            name: self.create_client(GetState, name + '/get_state') for name in self.SERVERS
        }
        self.create_subscription(Odometry, 'odometry', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, 'obstacle_scan', self._on_scan, qos_profile_sensor_data)
        for name in self.ACTION_NAMES:
            self.create_subscription(
                GoalStatusArray, name + '/_action/status',
                lambda message, action=name: self._on_action_status(action, message),
                qos_profile_action_status_default)
        if self._require_localization:
            self.create_subscription(
                String, 'localization_status', self._on_localization_status,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._status_pub = self.create_publisher(
            String, 'route_status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._start_service = self.create_service(Trigger, 'start_route', self._start_route)
        self._cancel_service = self.create_service(Trigger, 'cancel_route', self._cancel_route)
        self._retry_service = self.create_service(Trigger, 'retry_return', self._retry_return)
        self.create_timer(0.25, self._tick)
        self._publish_status()
        if auto_start:
            self._begin_route()
        else:
            self.get_logger().info('Fixed route is idle; call start_route to begin')

    @property
    def busy(self):
        """Include unacknowledged and canceling goals in motion ownership."""
        return self._phase != 'idle'

    def _publish_status(self):
        message = String()
        message.data = json.dumps({
            'run_id': self._run_id, 'state': self._phase,
            'last_result': self._last_result, 'home_pose': self._home_pose,
            'home_state': self._home_state,
            'home_blocked_reason': self._home_blocked_reason,
            'task_kind': self._task_kind, 'hp': self._hp,
            'low_hp_latched': self._low_hp_latched,
            'return_pending': self._return_pending, 'return_state': self._return_state,
            'return_result': self._return_result,
            'external_navigation_active': bool(self._external_goals),
            'external_goal_ids': [
                {'action': action, 'goal_id': goal_id.hex(), 'status': status}
                for (action, goal_id), status in sorted(self._external_goals.items())
            ],
            'external_observation_state': (
                'not_observed' if not self._observed_action_status else
                'observed' if len(self._observed_action_status) == len(self.ACTION_NAMES) else
                'partial'),
        }, allow_nan=False)
        self._status_pub.publish(message)

    def _begin_route(self):
        if self.busy or self._low_hp_latched or self._external_goals:
            return False
        self._task_kind = 'route'
        self._run_id += 1
        self._phase = 'waiting_ready'
        self._ready_since = None
        self._started = None
        self._last_log = None
        self._state_requests.clear()
        self._active_at.clear()
        self._goal_handle = None
        self._cancel_requested = False
        self._cancel_sent = False
        self._requested_wall = time.monotonic()
        self._sent_wall = None
        self._publish_status()
        self.get_logger().info(f'Route {self._run_id}: waiting for navigation readiness')
        return True

    def _start_route(self, _request, response):
        response.success = self._begin_route()
        response.message = (f'Route {self._run_id} requested' if response.success else
                            ('External navigation is active; no route request queued'
                             if self._external_goals else
                             'Low-HP return is latched; no route request queued'
                             if self._low_hp_latched else
                             f'Busy with route {self._run_id} ({self._phase}); no request queued'))
        return response

    def _cancel_route(self, _request, response):
        response.success = self.busy
        if not response.success:
            response.message = 'No route is active'
            return response
        if self._return_pending:
            self._return_state = 'canceled'
        self._return_pending = False
        if self._enable_low_hp_return:
            self._stop_motion()
        if self._phase in (
                'waiting_ready', 'return_stopping', 'return_waiting_ready',
                'blocked_external_navigation'):
            self._finish('canceled', 'Canceled before sending the action goal')
        else:
            self._cancel_requested = True
            self._phase = 'canceling'
            self._publish_status()
            self._request_cancel(self._run_id)
        response.message = (f'Cancellation requested for route {self._run_id}; '
                            'a new start is allowed only after its terminal result')
        return response

    def _retry_return(self, _request, response):
        """Retry a confirmed failed/canceled return without clearing its safety latch."""
        reason = ''
        if not self._enable_low_hp_return:
            reason = 'Low-HP return is disabled'
        elif self._external_goals:
            reason = 'External navigation is active'
        elif (self.busy or self._return_pending or self._goal_handle is not None or
                self._cancel_requested or self._cancel_sent):
            reason = 'Navigation is busy or its outcome is unconfirmed'
        elif self._home_pose is None or self._home_state != 'captured':
            reason = 'No valid recorded home pose'
        elif (not self._low_hp_latched or
                self._return_state not in ('failed', 'canceled', 'rejected')):
            reason = 'No confirmed failed/canceled return is available to retry'
        response.success = not reason
        if reason:
            response.message = reason + '; no retry queued'
        else:
            self._begin_return()
            response.message = f'Return {self._run_id} retry accepted; awaiting navigation result'
        return response

    def _new_owned_goal_id(self, action):
        goal_id = UUID(uuid=list(uuid4().bytes))
        # Register before send: a status callback can precede the acceptance response.
        self._owned_goal_ids.add((action, bytes(goal_id.uuid)))
        return goal_id

    def _on_action_status(self, action, message):
        """Remember observed external UUIDs until that UUID has a real terminal status."""
        before = (dict(self._external_goals), set(self._observed_action_status))
        self._observed_action_status.add(action)
        for entry in message.status_list:
            goal_id = bytes(entry.goal_info.goal_id.uuid)
            key = (action, goal_id)
            if not any(goal_id) or key in self._owned_goal_ids:
                continue
            if entry.status in self.TERMINAL_STATUSES:
                self._external_goals.pop(key, None)
                self._external_terminal_ids.add(key)
            elif key not in self._external_terminal_ids:
                # Missing/UNKNOWN is not proof that an accepted goal has stopped.
                self._external_goals[key] = int(entry.status)
        if self._external_goals and self._low_hp_latched:
            if self._task_kind == 'return_home' and self._phase in ('sending', 'active'):
                self._return_pending = True
                self._cancel_requested = True
                self._phase = 'canceling'
                self._request_cancel(self._run_id)
            if self._return_pending or self._phase in (
                    'return_stopping', 'return_waiting_ready'):
                self._return_pending = True
                self._return_state = 'blocked_external_navigation'
                self._ready_since = None
                self._stopped_since = None
        self._maybe_begin_return()
        if before != (self._external_goals, self._observed_action_status):
            self._publish_status()

    def _maybe_begin_return(self):
        if not self._return_pending:
            return
        if self._external_goals:
            if self._phase == 'idle':
                # Give a pending return its own run before it can be manually canceled.
                self._begin_return()
                if self._phase != 'return_stopping':
                    return
                self._phase = 'blocked_external_navigation'
                self._return_pending = True
            self._return_state = 'blocked_external_navigation'
            self._publish_status()
            return
        if self._phase == 'blocked_external_navigation':
            self._phase = 'return_stopping'
            self._return_state = 'stopping'
            self._return_pending = False
            self._started = None
            self._publish_status()
            return
        if not self.busy:
            self._begin_return()

    def _on_odom(self, message):
        self._odom = message

    def _on_scan(self, message):
        self._scan = message

    def _stop_motion(self):
        """Stop the spin source as well as translation through the normal command chain."""
        if self._external_goals:
            return  # Do not compete with an observed external controller or spin request.
        if self._spin_pub is not None:
            self._spin_pub.publish(Float32(data=0.0))
        if self._stop_pub is not None:
            self._stop_pub.publish(Twist())

    def _on_hp(self, message):
        self._hp = message.data
        self._rearm_low_hp_return()
        if (not self._enable_low_hp_return or self._low_hp_latched or
                self._hp >= self._return_hp_threshold):
            self._publish_status()
            return
        self._low_hp_latched = True
        self._return_pending = True
        self._return_state = ('blocked_external_navigation' if self._external_goals else
                              'canceling_route' if self.busy else 'stopping')
        self._stop_motion()
        self.get_logger().warning(
            f'HP {self._hp} < {self._return_hp_threshold}: return to recorded home requested')
        if not self.busy:
            self._maybe_begin_return()
        elif self._phase == 'waiting_ready':
            self._finish('canceled', 'Low HP canceled the route before dispatch')
        else:
            # A goal request still awaiting its reply owns motion too. Its late acceptance
            # must be canceled and its real terminal result observed before returning.
            self._cancel_requested = True
            self._phase = 'canceling'
            self._publish_status()
            self._request_cancel(self._run_id)

    def _rearm_low_hp_return(self):
        """Only a successful, finished return and recovered HP permit a new cycle."""
        if (self._enable_low_hp_return and self._low_hp_latched and
                not self.busy and not self._return_pending and
                self._return_state == 'succeeded' and self._home_pose is not None and
                self._hp is not None and
                self._hp >= max(self.RETURN_REARM_HP, self._return_hp_threshold)):
            self._low_hp_latched = False
            self.get_logger().info(
                f'HP recovered to {self._hp} after successful return; '
                'low-HP return rearmed without starting a route')

    def _begin_return(self):
        """Called only with no unconfirmed owned action; never invent a missing home."""
        self._return_pending = False
        self._task_kind = 'return_home'
        self._run_id += 1
        self._phase = 'return_stopping'
        self._return_state = 'stopping'
        self._return_result = None
        self._requested_wall = time.monotonic()
        self._sent_wall = None
        self._started = None
        self._ready_since = None
        self._stopped_since = None
        self._last_log = None
        self._goal_handle = None
        self._cancel_requested = False
        self._cancel_sent = False
        self._state_requests.clear()
        self._active_at.clear()
        self._stop_motion()
        if self._home_pose is None:
            self._finish('failed', 'Low-HP return has no recorded trusted home pose; staying stopped')
            return
        self._publish_status()

    def _stationary(self, now):
        return self._stationary_for(now, '_stopped_since')

    def _stationary_for(self, now, timer_attribute):
        if self._odom is None or not self._fresh(self._odom.header.stamp, now):
            setattr(self, timer_attribute, None)
            return False
        velocity = self._odom.twist.twist
        values = (velocity.linear.x, velocity.linear.y, velocity.angular.z)
        if (not all(math.isfinite(value) for value in values) or
                math.hypot(*values[:2]) > 0.03 or abs(values[2]) > 0.05):
            setattr(self, timer_attribute, None)
            return False
        since = getattr(self, timer_attribute)
        if since is None or now < since:
            since = now
            setattr(self, timer_attribute, since)
        return now - since >= 0.5e9

    def _send_return(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._home_pose['frame_id']
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        for name, value in self._home_pose['position'].items():
            setattr(goal.pose.pose.position, name, value)
        for name, value in self._home_pose['orientation'].items():
            setattr(goal.pose.pose.orientation, name, value)
        self._phase = 'sending'
        self._return_state = 'sending'
        self._sent_wall = time.monotonic()
        self._publish_status()
        run_id = self._run_id
        self.get_logger().info(f'Return {run_id}: sending recorded home {self._home_pose}')
        try:
            self._return_action.send_goal_async(
                goal, goal_uuid=self._new_owned_goal_id('navigate_to_pose')).add_done_callback(
                lambda future: self._on_goal_response(run_id, future))
        except Exception as error:
            # An exception is not proof that the server did not accept the request.
            self._unconfirmed_result(run_id, 'Return request outcome unknown: ' + str(error))

    def _on_localization_status(self, message):
        """A fresh TF alone is not evidence of successful map registration."""
        try:
            status = json.loads(message.data)
            stamp = status.get('stamp_ns')
            if (not isinstance(status, dict) or type(stamp) is not int or stamp <= 0 or
                    type(status.get('trusted')) is not bool):
                raise ValueError('invalid localization status')
        except (TypeError, ValueError, AttributeError):
            self._localization = None
            self._localization_received_wall = None
            self._ready_since = None
            return
        self._localization = status
        self._localization_received_wall = time.monotonic()
        if not status['trusted']:
            self._ready_since = None

    def _localization_readiness(self, now):
        if not self._require_localization:
            return ''
        status = self._localization
        if status is None or not status['trusted']:
            return 'map registration is not trusted'
        age = now - status['stamp_ns']
        wall_age = time.monotonic() - self._localization_received_wall
        if not 0 <= age <= self._freshness * 1e9 or not 0 <= wall_age <= self._freshness:
            return 'map registration status is stale'
        return ''

    def _home_readiness(self, now):
        """Check map pose inputs independently of route/action readiness."""
        reason = self._localization_readiness(now)
        if reason:
            return reason, None
        if self._odom is None or not self._fresh(self._odom.header.stamp, now):
            return 'no fresh operational odometry', None
        try:
            transform = self._buffer.lookup_transform('map', self._base_frame, Time())
        except TransformException:
            return 'map-to-chassis TF is unavailable', None
        if not self._fresh(transform.header.stamp, now):
            return 'map-to-chassis TF is stale', None
        position = transform.transform.translation
        rotation = transform.transform.rotation
        values = (position.x, position.y, position.z,
                  rotation.x, rotation.y, rotation.z, rotation.w)
        if (not all(math.isfinite(value) for value in values) or
                sum(value * value for value in values[3:]) < 1e-12):
            return 'map-to-chassis TF has invalid pose values', None
        return '', transform

    def _capture_home(self, now):
        """Latch the initial trusted stationary pose, including while the task is idle."""
        if self._home_pose is not None or self._home_state != 'pending':
            return
        if self._external_goals:
            reason, transform = 'external navigation is active before home capture', None
        elif self._phase not in ('idle', 'waiting_ready'):
            reason, transform = 'navigation owns motion before home capture', None
        else:
            reason, transform = self._home_readiness(now)
        if reason:
            self._home_stopped_since = None
        elif not self._stationary_for(now, '_home_stopped_since'):
            reason = 'waiting for 0.5 seconds of stationary odometry'
        if reason:
            if reason != self._home_blocked_reason:
                self._home_blocked_reason = reason
                self._publish_status()
            return
        if self._record_home(transform):
            self._publish_status()
            self.get_logger().info('Recorded first trusted stationary home pose')

    def _record_home(self, transform):
        """Record the first confirmed map pose; never re-arm a clock-reset home."""
        if (self._home_pose is not None or transform is None or
                self._home_state != 'pending'):
            return False
        position = transform.transform.translation
        rotation = transform.transform.rotation
        values = (position.x, position.y, position.z,
                  rotation.x, rotation.y, rotation.z, rotation.w)
        if (not all(math.isfinite(value) for value in values) or
                sum(value * value for value in values[3:]) < 1e-12):
            return False
        self._home_pose = {
            'frame_id': 'map',
            'stamp_ns': Time.from_msg(transform.header.stamp).nanoseconds,
            'position': dict(zip(('x', 'y', 'z'), values[:3])),
            'orientation': dict(zip(('x', 'y', 'z', 'w'), values[3:])),
        }
        self._home_state = 'captured'
        self._home_blocked_reason = ''
        return True

    def _fresh(self, stamp, now):
        age = now - Time.from_msg(stamp).nanoseconds
        return 0 <= age <= self._freshness * 1e9

    def _poll_states(self, now):
        for name, client in self._lifecycle_clients.items():
            future = self._state_requests.get(name)
            if future is not None and future.done():
                try:
                    active = future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
                except Exception:
                    active = False
                if active:
                    self._active_at[name] = now
                else:
                    self._active_at.pop(name, None)
                del self._state_requests[name]
            if name not in self._state_requests and client.service_is_ready():
                self._state_requests[name] = client.call_async(GetState.Request())

    def _readiness(self, now):
        if self._external_goals:
            return 'external navigation is active', None
        self._poll_states(now)
        reason = self._localization_readiness(now)
        if reason:
            return reason, None
        for name in self.SERVERS:
            if name not in self._active_at or not 0 <= now - self._active_at[name] <= 2e9:
                return name + ' is not active', None
        if self._odom is None or not self._fresh(self._odom.header.stamp, now):
            return 'no fresh operational odometry', None
        if (self._scan is None or not self._scan.ranges or
                not self._fresh(self._scan.header.stamp, now)):
            return 'no fresh obstacle scan', None
        try:
            transform = self._buffer.lookup_transform('map', self._base_frame, Time())
        except TransformException:
            return 'map-to-chassis TF is unavailable', None
        if not self._fresh(transform.header.stamp, now):
            return 'map-to-chassis TF is stale', None
        action = self._return_action if self._task_kind == 'return_home' else self._action
        if not action.server_is_ready():
            return 'navigation action is unavailable', None
        return '', transform

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        if self._last_tick_ns is not None and now < self._last_tick_ns:
            self._localization = None
            self._localization_received_wall = None
            self._home_pose = None
            self._home_state = 'invalidated'
            self._home_blocked_reason = 'clock reset invalidated home; restart task to confirm a new home'
            self._home_stopped_since = None
            self._ready_since = None
            self._return_pending = False
            if self.busy and self._phase != 'waiting_ready':
                self._stop_motion()
                if self._phase in (
                        'return_stopping', 'return_waiting_ready', 'blocked_external_navigation'):
                    self._finish('failed', 'Clock reset invalidated the recorded home pose')
                else:
                    self._cancel_requested = True
                    self._phase = 'canceling'
                    self._request_cancel(self._run_id)
            self._publish_status()
        self._last_tick_ns = now
        if now <= 0:
            return
        self._capture_home(now)
        if self._phase == 'blocked_external_navigation':
            self._maybe_begin_return()
            return
        if self._phase == 'canceling':
            if self._low_hp_latched:
                self._stop_motion()
            self._request_cancel(self._run_id)
            return
        if self._phase not in ('waiting_ready', 'return_stopping', 'return_waiting_ready'):
            return
        if self._task_kind == 'return_home' and self._external_goals:
            self._return_pending = True
            self._return_state = 'blocked_external_navigation'
            self._ready_since = None
            self._stopped_since = None
            return
        if self._task_kind == 'return_home' and self._return_pending:
            self._return_pending = False
            self._return_state = 'stopping'
            self._started = None
        if self._started is None or now < self._started:
            self._started = now
            self._ready_since = None
            self._active_at.clear()
        if now - self._started > self._startup_timeout * 1e9:
            self._finish('failed', 'Navigation readiness timed out')
            return
        if self._task_kind == 'return_home':
            self._stop_motion()
            if self._home_pose is None:
                self._finish('failed', 'Recorded home pose is unavailable; return remains stopped')
                return
            if not self._stationary(now):
                self._ready_since = None
                self._phase = 'return_stopping'
                return
            if self._phase == 'return_stopping':
                self._phase = 'return_waiting_ready'
                self._return_state = 'waiting_ready'
                self._publish_status()
        reason, transform = self._readiness(now)
        if (not reason and self._task_kind == 'route' and
                self._enable_low_hp_return and self._home_pose is None):
            reason = 'home pose is unavailable: ' + self._home_blocked_reason
        if reason:
            self._ready_since = None
            if self._last_log is None or now - self._last_log >= 5e9:
                self.get_logger().info(f'Waiting for {self._task_kind}: ' + reason)
                self._last_log = now
            return
        if self._ready_since is None or now < self._ready_since:
            self._ready_since = now
        if now - self._ready_since < self._ready_duration * 1e9:
            return
        if self._task_kind == 'return_home':
            self._send_return()
            return
        goal = NavigateThroughPoses.Goal()
        for index in range(0, len(self._route), 2):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = self._route[index:index + 2]
            pose.pose.orientation.z = math.sin(self._route_yaw / 2)
            pose.pose.orientation.w = math.cos(self._route_yaw / 2)
            goal.poses.append(pose)
        self._phase = 'sending'
        self._sent_wall = time.monotonic()
        self._publish_status()
        run_id = self._run_id
        self.get_logger().info(f'Route {run_id}: sending three map-frame poses {self._route}')
        try:
            self._action.send_goal_async(
                goal, goal_uuid=self._new_owned_goal_id('navigate_through_poses')).add_done_callback(
                lambda future: self._on_goal_response(run_id, future))
        except Exception as error:
            if self._enable_low_hp_return:
                self._unconfirmed_result(run_id, 'Route request outcome unknown: ' + str(error))
            else:
                self._finish('failed', 'Route request failed: ' + str(error))

    def _on_goal_response(self, run_id, future):
        if run_id != self._run_id or not self.busy:
            try:
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().error('Stale route response cleanup failed: ' + str(error))
            return
        try:
            handle = future.result()
            self._goal_handle = handle
            if not handle.accepted:
                self._finish('rejected', 'Route goal was rejected; it will not be retried')
                return
            self._phase = 'canceling' if self._cancel_requested else 'active'
            if self._task_kind == 'return_home':
                self._return_state = ('blocked_external_navigation' if self._external_goals else
                                      self._phase)
            self._publish_status()
            handle.get_result_async().add_done_callback(
                lambda result: self._on_result(run_id, result))
            if self._cancel_requested:
                self._request_cancel(run_id)
        except Exception as error:
            if (self._enable_low_hp_return or
                    self._goal_handle is not None and self._goal_handle.accepted):
                # A lost response can hide an accepted goal even when no handle arrived.
                # Keep ownership and keep stopping; do not dispatch a competing home goal.
                self._unconfirmed_result(run_id, error)
            else:
                self._finish('failed', 'Route goal response failed: ' + str(error))

    def _request_cancel(self, run_id):
        if (run_id != self._run_id or self._goal_handle is None or
                self._cancel_sent or not self.busy):
            return
        self._cancel_sent = True
        if self._task_kind == 'return_home':
            self._return_state = ('blocked_external_navigation' if self._external_goals else
                                  'canceling')
        try:
            self._goal_handle.cancel_goal_async().add_done_callback(
                lambda future: self._on_cancel_response(run_id, future))
        except Exception as error:
            self._cancel_sent = False
            self.get_logger().error('Owned route cancel request failed: ' + str(error))

    def _on_cancel_response(self, run_id, future):
        if run_id != self._run_id or not self.busy:
            return
        try:
            reply = future.result()
            if not reply.goals_canceling:
                self._cancel_sent = False
                self.get_logger().warning(
                    'Route cancel was not acknowledged; awaiting terminal result')
        except Exception as error:
            self._cancel_sent = False
            self.get_logger().error('Owned route cancel response failed: ' + str(error))
        # An accepted cancel is not terminal: keep busy until get_result_async completes.

    def _on_result(self, run_id, future):
        if run_id != self._run_id or not self.busy:
            return
        try:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._finish('succeeded', self._task_kind + ' succeeded')
            elif status == GoalStatus.STATUS_CANCELED:
                self._finish('canceled', self._task_kind + ' canceled')
            elif status == GoalStatus.STATUS_ABORTED:
                self._finish('failed', self._task_kind + ' failed, status=' + str(status))
            else:
                self._unconfirmed_result(run_id, 'nonterminal status=' + str(status))
        except Exception as error:
            self._unconfirmed_result(run_id, error)

    def _unconfirmed_result(self, run_id, error):
        # A transport error does not prove that an accepted goal has stopped.
        self._cancel_requested = True
        self._phase = 'canceling'
        if self._enable_low_hp_return:
            self._stop_motion()
        self._publish_status()
        self.get_logger().error('Route result unconfirmed; retaining busy state: ' + str(error))
        self._request_cancel(run_id)

    def _finish(self, outcome, detail):
        now = time.monotonic()
        self._last_result = {
            'run_id': self._run_id, 'outcome': outcome, 'detail': detail,
            'task_kind': self._task_kind,
            'elapsed_sec': max(0.0, now - self._requested_wall),
            'navigation_elapsed_sec': (max(0.0, now - self._sent_wall)
                                       if self._sent_wall is not None else None),
        }
        self._phase = 'idle'
        self._goal_handle = None
        self._cancel_requested = False
        self._cancel_sent = False
        if self._task_kind == 'return_home':
            self._return_state = outcome
            self._return_result = dict(self._last_result)
            self._stop_motion()
            self._rearm_low_hp_return()
        self._publish_status()
        self.get_logger().info(
            f'ROUTE_RESULT {json.dumps(self._last_result, allow_nan=False)}')
        if self._return_pending:
            self._maybe_begin_return()


def main(args=None):
    """Keep the task services available after every terminal navigation result."""
    rclpy.init(args=args)
    node = StartupNavigation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
