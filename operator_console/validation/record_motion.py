"""Read-only telemetry for manual operator-console acceptance."""
import json,math,time,signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from example_interfaces.msg import Float32
rclpy.init();n=Node('operator_console_test_observer');errors=[];poses=[];commands=[];last=0.;finished=False
start=time.monotonic()
def pose(m):
 global last
 now=time.monotonic()
 if now-last<.1:return
 last=now;p=m.pose.pose.position;poses.append([now-start,p.x,p.y,m.twist.twist.angular.z])
n.create_subscription(Odometry,'/reference/odometry',pose,qos_profile_sensor_data)
n.create_subscription(Vector3Stamped,'/localization_error',lambda m:errors.append([time.monotonic()-start,m.vector.x,m.vector.y,m.vector.z]),100)
n.create_subscription(Float32,'/cmd_spin',lambda m:commands.append([time.monotonic()-start,m.data]),10)
def stop(*args):
 global finished
 finished=True
signal.signal(signal.SIGUSR1,stop)
try:
 while not finished and time.monotonic()-start<240:rclpy.spin_once(n,timeout_sec=.05)
finally:
 result={'duration_wall_s':time.monotonic()-start,'samples':len(errors),'max_xy_error_m':max((math.hypot(e[1],e[2]) for e in errors),default=None),'max_yaw_error_rad':max((abs(e[3]) for e in errors),default=None),'errors':errors,'poses':poses,'spin_commands':commands};(Path(__file__).resolve().parent / 'operator-console-motion-telemetry.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('poses','errors')},indent=2));n.destroy_node();rclpy.shutdown()
