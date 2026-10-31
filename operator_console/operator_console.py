"""Local browser console for Nav2 tasks and the simulated sentry spin command."""

import argparse
import base64
from collections import deque
from concurrent.futures import Future, TimeoutError as FutureTimeout
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import queue
import threading
import time
from urllib.parse import urlsplit
import uuid
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from example_interfaces.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger
try:
    from sentry_interfaces.action import OpeningMission
    from sentry_interfaces.msg import MotionState
except ImportError:
    OpeningMission = MotionState = None
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


BUSY = {'sending', 'running', 'canceling'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def finite_number(value, name, limit):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f'{name} 必须是数字')
    if not math.isfinite(value) or abs(value) > limit:
        raise ValueError(f'{name} 超出允许范围 ±{limit}')
    return float(value)


class OperatorConsole(Node):
    """Keep ROS operations on one executor; share snapshots with HTTP threads."""

    def __init__(self):
        super().__init__('sentry_operator_console', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'chassis').value
        self.lock = threading.Lock()
        self.requests = queue.Queue(maxsize=32)
        self.map_payload = None
        self.map_data = None
        self.map_revision = 0
        self.pose = None
        self.last_odom = 0.0
        self.velocity = {'linear': 0.0, 'angular': 0.0}
        self.plan = []
        self.events = deque(maxlen=60)
        self.phase = 'idle'
        self.task = None
        self.goal_handle = None
        self.cancel_requested = False
        self.spin_request = None
        self.opening_active = False
        self.opening_defaults = None
        try:
            route_path = Path(get_package_share_directory('sentry_business_bringup')) / 'config/opening_2027rmul.yaml'
            self.opening_defaults = yaml.safe_load(route_path.read_text())
        except (PackageNotFoundError, OSError, ValueError):
            pass
        self.motion_state = None
        self.motion_seen = 0.0
        self.opening_client = (ActionClient(self, OpeningMission, '/sentry/opening_mission')
                               if OpeningMission is not None else None)
        self.business_stop = self.create_client(Trigger, '/sentry/stop')
        if MotionState is not None:
            self.create_subscription(MotionState, '/sentry/motion_state', self.on_motion_state, 10)
        self.owns_spin = False
        self.stop_until = 0.0
        self.feedback = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_pub = self.create_publisher(Float32, '/cmd_spin', 10)
        self.stop_pub = None
        self.create_subscription(Float32, '/cmd_spin', self.on_spin, 10)
        self.create_subscription(Odometry, '/odometry', self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(RosPath, '/plan', self.on_plan, 10)
        self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.state = {}
        self.event('info', '操控台已启动，等待地图与导航节点')

    def on_motion_state(self, msg):
        if self.stop_pub is not None:
            self.destroy_publisher(self.stop_pub)
            self.stop_pub = None
        self.motion_state = {'owner': msg.owner, 'stopped': msg.stopped, 'reason': msg.reason}
        self.motion_seen = time.monotonic()

    def emit_stop(self):
        if self.business_stop.service_is_ready():
            self.business_stop.call_async(Trigger.Request())
        elif self.motion_state is None:
            if self.stop_pub is None:
                self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.stop_pub.publish(Twist())

    def business_available(self):
        return self.motion_state is not None and time.monotonic() - self.motion_seen < 1.0

    def start_opening(self, body):
        if self.phase in BUSY or not self.ready():
            raise ValueError('已有任务运行或导航尚未就绪')
        if not self.business_available() or not self.opening_client or not self.opening_client.server_is_ready():
            raise ValueError('开局业务节点未就绪，请使用业务启动文件')
        if self.motion_state['owner'] != 'stop':
            raise ValueError('其他运动任务正在执行，请先停止')
        points = body.get('points')
        if not isinstance(points, list) or len(points) != 3:
            raise ValueError('请按入口、出口、中心顺序配置三个点')
        points = [self.check_point(p) for p in points]
        goal = OpeningMission.Goal()
        for name, point in zip(('entry', 'exit', 'center'), points):
            pose = PoseStamped(); pose.header.frame_id = self.frame
            pose.pose.position.x, pose.pose.position.y = point['x'], point['y']
            pose.pose.orientation.z = math.sin(point['yaw']/2)
            pose.pose.orientation.w = math.cos(point['yaw']/2)
            setattr(goal, name, pose)
        self.opening_active = True; self.cancel_requested = False
        self.phase = 'sending'; self.feedback = {}
        self.task = {'id': str(uuid.uuid4()), 'points': points, 'index': 0,
                     'auto_stop_spin': True, 'started_at': time.time()}
        self.opening_client.send_goal_async(goal, feedback_callback=self.on_opening_feedback).add_done_callback(self.on_opening_goal)

    def on_opening_feedback(self, msg):
        f = msg.feedback
        self.feedback = {'stage': f.stage, 'lateral_error': f.lateral_error,
                         'heading_error': f.heading_error, 'distance_remaining': f.remaining}
        self.task['index'] = 2 if f.stage == 'center' else 1 if f.stage.startswith('tunnel') else 0

    def on_opening_goal(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                raise ValueError('开局路线被拒绝，请检查通道范围或其他活动任务')
            self.phase = 'running'
            self.goal_handle.get_result_async().add_done_callback(self.on_opening_result)
            if self.cancel_requested:
                self.goal_handle.cancel_goal_async()
        except Exception as error:
            self.opening_active = False; self.fail(str(error))

    def on_opening_result(self, future):
        try:
            response = future.result()
            self.phase = 'canceled' if self.cancel_requested else 'succeeded' if response.status == 4 else 'canceled' if response.status == 5 else 'failed'
            if response.status == 4:
                self.task['index'] = len(self.task['points'])
            self.event('success' if response.status == 4 else 'info', response.result.message)
        except Exception as error:
            self.fail(str(error))
        finally:
            self.opening_active = False; self.goal_handle = None

    def event(self, level, message):
        self.events.append({'id': str(uuid.uuid4()), 'time': time.time(),
                            'level': level, 'message': message})

    def on_map(self, msg):
        self.map_revision += 1
        self.map_data = msg
        payload = {'revision': self.map_revision, 'frame': msg.header.frame_id,
                   'width': msg.info.width, 'height': msg.info.height,
                   'resolution': msg.info.resolution,
                   'origin': [msg.info.origin.position.x, msg.info.origin.position.y,
                              yaw_of(msg.info.origin.orientation)],
                   'data': base64.b64encode(bytes(v & 255 for v in msg.data)).decode('ascii')}
        with self.lock:
            self.map_payload = payload

    def on_odom(self, msg):
        v = msg.twist.twist
        if not all(math.isfinite(x) for x in (v.linear.x, v.linear.y, v.angular.z)):
            return
        self.last_odom = time.monotonic()
        self.velocity = {'linear': math.hypot(v.linear.x, v.linear.y),
                         'angular': v.angular.z}

    def on_spin(self, msg):
        if math.isfinite(msg.data):
            self.spin_request = float(msg.data)

    def on_plan(self, msg):
        # Avoid drawing another frame as though it were map coordinates.
        if msg.header.frame_id == self.frame:
            stride = max(1, len(msg.poses) // 500)
            self.plan = [[p.pose.position.x, p.pose.position.y]
                         for p in msg.poses[::stride]]

    def ready(self):
        return (self.client.server_is_ready() and self.pose is not None
                and self.pose['age'] < 1.5
                and time.monotonic() - self.last_odom < 2.0
                and self.map_data is not None
                and self.map_data.header.frame_id == self.frame)

    def set_spin(self, speed):
        if speed and self.motion_state is not None:
            raise ValueError('业务模式下关闭自旋')
        subscribers = {info.node_name for info in
                       self.get_subscriptions_info_by_topic('/cmd_spin')}
        if speed and (not self.ready() or not
                      {'fake_vel_transform', 'sim_serial_gateway'} <= subscribers):
            raise ValueError('定位或自旋控制节点未就绪，无法开启自旋')
        self.spin_pub.publish(Float32(data=speed))
        self.owns_spin = True
        self.spin_request = speed
        self.event('info', f'已发布自旋指令 {speed:+.2f} rad/s' if speed
                   else '已发布停转指令，底盘按角加速度限制减速')

    def check_point(self, point):
        x = finite_number(point.get('x'), 'X', 10000)
        y = finite_number(point.get('y'), 'Y', 10000)
        yaw = finite_number(point.get('yaw', 0.0), '朝向', math.pi * 2)
        m = self.map_data
        if m is None or m.header.frame_id != self.frame:
            raise ValueError('尚未收到目标坐标系的地图')
        dx, dy = x - m.info.origin.position.x, y - m.info.origin.position.y
        angle = yaw_of(m.info.origin.orientation)
        ix = math.floor((dx * math.cos(angle) + dy * math.sin(angle)) / m.info.resolution)
        iy = math.floor((-dx * math.sin(angle) + dy * math.cos(angle)) / m.info.resolution)
        if not (0 <= ix < m.info.width and 0 <= iy < m.info.height):
            raise ValueError('目标点位于地图范围外')
        value = m.data[iy * m.info.width + ix]
        if value < 0 or value >= 50:
            raise ValueError('目标位于未知区域或静态障碍上，请重新选点')
        return {'x': x, 'y': y, 'yaw': yaw,
                'name': str(point.get('name', '导航目标'))[:40]}

    def start_task(self, body):
        if self.motion_state is not None:
            raise ValueError('业务模式请使用 UL 开局任务')
        if self.phase in BUSY:
            raise ValueError('已有任务执行中，请先取消')
        if not self.ready():
            raise ValueError('地图、定位或 Nav2 尚未就绪')
        points = body.get('points')
        if not isinstance(points, list) or not 1 <= len(points) <= 20:
            raise ValueError('每个任务需要 1–20 个目标点')
        if not all(isinstance(p, dict) for p in points):
            raise ValueError('目标点格式错误')
        points = [self.check_point(p) for p in points]
        speed = finite_number(body.get('spin_speed', 0.0), '自旋速度', 3.14)
        auto_stop = body.get('auto_stop_spin', True)
        if not isinstance(auto_stop, bool):
            raise ValueError('任务结束停转选项格式错误')
        self.set_spin(speed)
        self.task = {'id': str(uuid.uuid4()), 'points': points, 'index': 0,
                     'auto_stop_spin': auto_stop, 'started_at': time.time()}
        self.cancel_requested = False
        self.feedback = {}
        self.plan = []
        self.send_next()

    def send_next(self):
        point = self.task['points'][self.task['index']]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point['x']
        goal.pose.pose.position.y = point['y']
        goal.pose.pose.orientation.z = math.sin(point['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(point['yaw'] / 2)
        self.phase = 'sending'
        self.feedback = {}
        self.event('info', f"下发目标 {self.task['index'] + 1}/{len(self.task['points'])} · {point['name']}")
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal)

    def on_feedback(self, msg):
        f = msg.feedback
        self.feedback = {'distance_remaining': (f.distance_remaining if
                             math.isfinite(f.distance_remaining) else None),
                         'elapsed': f.navigation_time.sec,
                         'eta': f.estimated_time_remaining.sec,
                         'recoveries': f.number_of_recoveries}

    def fail(self, message):
        self.phase = 'failed'
        self.goal_handle = None
        self.event('error', message)
        # Rejected/failed tasks must not leave task-started spin running.
        self.set_spin(0.0)

    def on_goal(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.fail('Nav2 拒绝了目标，请检查可达性及节点状态')
                return
            self.goal_handle.get_result_async().add_done_callback(self.on_result)
            if self.cancel_requested:
                self.request_cancel()
            else:
                self.phase = 'running'
                self.event('info', 'Nav2 已接受目标，正在执行')
        except Exception as error:
            self.fail(f'目标下发失败：{error}')

    def on_result(self, future):
        try:
            status = future.result().status
        except Exception as error:
            self.fail(f'读取任务结果失败：{error}')
            return
        self.goal_handle = None
        if self.cancel_requested or status == 5:
            self.phase = 'canceled'
            self.event('info', '任务已取消，剩余目标不再执行')
        elif status == 4:
            self.event('success', f"已到达 · {self.task['points'][self.task['index']]['name']}")
            self.task['index'] += 1
            if self.task['index'] < len(self.task['points']):
                if not self.ready():
                    self.fail('定位或导航连接中断，后续目标未下发')
                    return
                self.send_next()
                return
            self.phase = 'succeeded'
            self.event('success', '全部导航目标已完成')
        else:
            self.fail(f'导航未完成（状态码 {status}），剩余目标已停止')
            return
        if self.cancel_requested or self.task['auto_stop_spin']:
            self.set_spin(0.0)

    def request_cancel(self):
        self.cancel_requested = True
        self.phase = 'canceling'
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async().add_done_callback(self.on_cancel_response)

    def on_cancel_response(self, future):
        try:
            response = future.result()
            if not response.goals_canceling and self.phase == 'canceling':
                self.cancel_requested = False
                self.phase = 'running'
                self.event('error', 'Nav2 未确认取消，任务仍可能执行；请重试取消')
        except Exception as error:
            self.event('error', f'取消请求失败：{error}')

    def process(self, command, body):
        if command == 'opening':
            self.start_opening(body)
        elif command == 'task':
            self.start_task(body)
        elif command == 'spin':
            self.set_spin(finite_number(body.get('speed'), '自旋速度', 3.14))
        elif command == 'stop':
            if self.business_stop.service_is_ready():
                self.business_stop.call_async(Trigger.Request())
            if self.phase in BUSY:
                self.request_cancel()
                self.stop_until = time.monotonic() + 0.8
            else:
                self.event('info', '当前没有由本操控台管理的活动任务')
            self.set_spin(0.0)
        else:
            raise ValueError('未知操作')

    def tick(self):
        try:
            transform = self.buffer.lookup_transform(self.frame, self.robot_frame, Time())
            stamp = Time.from_msg(transform.header.stamp).nanoseconds
            self.pose = {'x': transform.transform.translation.x,
                         'y': transform.transform.translation.y,
                         'yaw': yaw_of(transform.transform.rotation),
                         'age': max(0.0, (self.get_clock().now().nanoseconds - stamp) * 1e-9)}
        except TransformException:
            self.pose = None
        for _ in range(8):
            try:
                command, body, future = self.requests.get_nowait()
            except queue.Empty:
                break
            if not future.set_running_or_notify_cancel():
                continue
            try:
                self.process(command, body)
                future.set_result({'ok': True})
            except Exception as error:
                self.event('error', str(error))
                future.set_result({'ok': False, 'error': str(error)})
        if time.monotonic() < self.stop_until:
            self.emit_stop()
        if self.pose is not None and not all(
                math.isfinite(v) for v in self.pose.values()):
            self.pose = None
        snapshot = {'business_available': self.business_available(), 'business_mode': self.motion_state is not None, 'motion': self.motion_state,
                    'opening_active': self.opening_active, 'ready': self.ready(), 'nav_available': self.client.server_is_ready(),
                    'map_revision': self.map_revision, 'frame': self.frame,
                    'pose': self.pose, 'odom_fresh': time.monotonic() - self.last_odom < 2,
                    'velocity': self.velocity, 'phase': self.phase,
                    'task': self.task, 'spin_request': self.spin_request,
                    'feedback': self.feedback, 'plan': self.plan,
                    'events': list(self.events), 'server_time': time.time()}
        with self.lock:
            self.state = copy.deepcopy(snapshot)


class ConsoleServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class ConsoleHandler(BaseHTTPRequestHandler):
    """Serve bundled assets and a small same-origin control API."""

    def log_message(self, format_string, *args):
        return

    def reply(self, code, value, mime='application/json; charset=utf-8'):
        data = value if isinstance(value, bytes) else json.dumps(value, allow_nan=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; style-src 'self'; script-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(data)

    def valid_host(self):
        port = self.server.server_address[1]
        if self.headers.get('Host') not in (f'127.0.0.1:{port}', f'localhost:{port}'):
            self.reply(403, {'ok': False, 'error': '仅允许本机访问'})
            return False
        return True

    def do_GET(self):
        if not self.valid_host():
            return
        path = urlsplit(self.path).path
        node = self.server.node
        if path == '/api/opening-route':
            self.reply(200, node.opening_defaults)
            return
        if path in ('/api/state', '/api/map'):
            with node.lock:
                value = copy.deepcopy(node.state if path.endswith('state') else node.map_payload)
            self.reply(200, value)
            return
        files = {'/': ('index.html', 'text/html; charset=utf-8'),
                 '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                 '/style.css': ('style.css', 'text/css; charset=utf-8')}
        if path not in files:
            self.reply(404, {'error': 'Not found'})
            return
        name, mime = files[path]
        self.reply(200, (self.server.assets / name).read_bytes(), mime)

    def do_POST(self):
        if not self.valid_host():
            return
        # JSON plus exact same-origin validation prevents cross-site commands.
        origin = self.headers.get('Origin')
        if origin and origin != 'http://' + self.headers.get('Host', ''):
            self.reply(403, {'ok': False, 'error': '不允许跨站控制请求'})
            return
        command = {'/api/opening': 'opening', '/api/task': 'task', '/api/spin': 'spin', '/api/stop': 'stop'}.get(urlsplit(self.path).path)
        if command is None:
            self.reply(404, {'ok': False, 'error': '未知接口'})
            return
        if self.headers.get_content_type() != 'application/json':
            self.reply(415, {'ok': False, 'error': '需要 JSON 请求'})
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 16384:
                raise ValueError('请求长度无效')
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError('请求必须是对象')
        except (ValueError, json.JSONDecodeError):
            self.reply(400, {'ok': False, 'error': '请求格式错误'})
            return
        future = Future()
        try:
            self.server.node.requests.put_nowait((command, body, future))
        except queue.Full:
            self.reply(503, {'ok': False, 'error': '控制请求过多，请稍后重试'})
            return
        try:
            result = future.result(timeout=3)
            self.reply(200 if result['ok'] else 409, result)
        except FutureTimeout:
            if future.cancel():
                self.reply(503, {'ok': False, 'error': 'ROS 响应超时，操作未执行'})
            else:
                self.reply(503, {'ok': False, 'error': '操作结果尚未确认，请查看任务状态后再操作'})


def main(args=None):
    parser = argparse.ArgumentParser(description='哨兵独立操控台')
    parser.add_argument('--port', type=int, default=8989)
    options, ros_args = parser.parse_known_args(args)
    if not 1024 <= options.port <= 65535:
        parser.error('port must be between 1024 and 65535')
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = OperatorConsole()
    assets = Path(__file__).resolve().parent / 'web'
    server = ConsoleServer(('127.0.0.1', options.port), ConsoleHandler)
    server.node, server.assets = node, assets
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f'操控台：http://127.0.0.1:{options.port}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        if rclpy.ok():
            if node.phase in BUSY:
                node.request_cancel()
                node.stop_until = time.monotonic() + 0.8
            if node.owns_spin:
                node.set_spin(0.0)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
