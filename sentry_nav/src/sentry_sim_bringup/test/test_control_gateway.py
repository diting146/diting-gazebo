"""Exercise the command relay without Gazebo or shared motion topics."""

import time
from types import SimpleNamespace
from uuid import uuid4

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sentry_sim_bringup.sim_serial_gateway import SimSerialGateway


@pytest.fixture(params=[False, True], ids=['system_clock', 'paused_sim_clock'])
def relay(request):
    rclpy.init()
    namespace = '/gateway_test_' + uuid4().hex
    gateway = SimSerialGateway(
        namespace=namespace,
        parameter_overrides=[
            Parameter('use_sim_time', value=request.param),
            Parameter('input_topic', value=namespace + '/input'),
            Parameter('output_topic', value=namespace + '/output'),
            Parameter('timeout_ms', value=120),
            Parameter('spin_topic', value=''),
            Parameter('idle_spin_speed', value=0.0),
        ],
    )
    observer = Node('observer', namespace=namespace)
    received = []
    subscription = observer.create_subscription(
        Twist, namespace + '/output', received.append, 100
    )
    publisher = observer.create_publisher(Twist, namespace + '/input', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    executor.add_node(observer)

    def wait(predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.002)
            if predicate():
                return True
        return False

    def command(x):
        message = Twist()
        message.linear.x = x
        publisher.publish(message)

    try:
        assert wait(lambda: publisher.get_subscription_count() > 0)
        yield SimpleNamespace(
            gateway=gateway, observer=observer, received=received,
            publisher=publisher, subscription=subscription, wait=wait,
            command=command, simulation=request.param,
        )
    finally:
        executor.remove_node(gateway)
        executor.remove_node(observer)
        executor.shutdown()
        gateway.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


def test_command_gap_stops_and_new_input_resumes(relay):
    relay.command(0.2)
    assert relay.wait(lambda: relay.received and relay.received[-1].linear.x == 0.2)
    assert relay.wait(lambda: relay.received[-1].linear.x == 0.0, timeout=0.8)
    assert relay.received[-1].angular.z == 0.0
    relay.command(0.1)
    assert relay.wait(lambda: relay.received[-1].linear.x == 0.1)
    relay.command(0.0)
    assert relay.wait(lambda: relay.received[-1].linear.x == 0.0, timeout=0.1)
    if relay.simulation:
        assert relay.gateway.get_clock().now().nanoseconds == 0


def test_repeated_keyboard_style_input_does_not_timeout(relay):
    relay.command(0.2)
    assert relay.wait(lambda: relay.received and relay.received[-1].linear.x == 0.2)
    start = len(relay.received)
    deadline = time.monotonic() + 0.35
    while time.monotonic() < deadline:
        relay.command(0.2)
        relay.wait(lambda: False, timeout=0.02)
    assert all(message.linear.x == 0.2 for message in relay.received[start:])
    assert relay.wait(lambda: relay.received[-1].linear.x == 0.0, timeout=0.8)


def test_manual_relay_has_no_global_spin_subscription(relay):
    assert relay.wait(lambda: bool(relay.gateway.get_subscriptions_info_by_topic(
        relay.gateway.get_namespace() + '/input')))
    subscriptions = relay.gateway.get_subscriptions_info_by_topic('/cmd_spin')
    assert not any(
        endpoint.node_name == relay.gateway.get_name()
        and endpoint.node_namespace == relay.gateway.get_namespace()
        for endpoint in subscriptions
    )
    relay.command(0.2)
    assert relay.wait(lambda: relay.received and relay.received[-1].linear.x == 0.2)
    assert relay.received[-1].angular.z == 0.0
