"""Bound high-rate joint traffic without changing the measured TF chain."""
from types import SimpleNamespace
import math
from sensor_msgs.msg import JointState
from sentry_sim_bringup.sentry_gimbal_tf import SentryGimbalTf


def publish_sequence(period_ns):
    transforms=[]
    node=SimpleNamespace(_find_joint_position=lambda msg:msg.position[0],
                         _last_publish_ns=None,_publish_period_ns=period_ns,
                         _gimbal_odom_frame='gimbal_yaw_odom',_gimbal_frame='gimbal_yaw',
                         _dynamic_broadcaster=SimpleNamespace(sendTransform=transforms.append))
    for i in range(1000):
        msg=JointState();msg.header.stamp.sec=1;msg.header.stamp.nanosec=i*1000000
        msg.position=[i*.001]
        SentryGimbalTf._on_joint_states(node,msg)
    return node,transforms


def test_business_rate_bound_preserves_timestamps_and_yaw():
    node,transforms=publish_sequence(10000000)
    assert len(transforms)==100
    last=transforms[-1]
    assert last.header.frame_id=='gimbal_yaw_odom' and last.child_frame_id=='gimbal_yaw'
    assert last.header.stamp.nanosec==990000000
    assert math.isclose(last.transform.rotation.z,math.sin(.99/2))
    # A restarted simulation epoch must not be suppressed by the previous clock.
    msg=JointState();msg.header.stamp.sec=0;msg.header.stamp.nanosec=1000000;msg.position=[0.]
    SentryGimbalTf._on_joint_states(node,msg)
    assert len(transforms)==101


def test_original_unlimited_rate_remains_available():
    _,transforms=publish_sequence(0)
    assert len(transforms)==1000
