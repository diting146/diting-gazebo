"""Wait for the Gazebo create service, then replace this process with the native spawner."""
import os
from pathlib import Path
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix


def main():
    deadline = time.monotonic() + 20.0
    service = '/world/default/create'
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(['ign', 'service', '-l'], capture_output=True,
                                    text=True, timeout=5.0)
        except subprocess.TimeoutExpired:
            continue
        if result.returncode == 0 and service in result.stdout.splitlines():
            executable = Path(get_package_prefix('ros_gz_sim')) / 'lib/ros_gz_sim/create'
            print('Gazebo create service ready; starting native robot spawn', flush=True)
            os.execv(str(executable), [str(executable), *sys.argv[1:]])
        time.sleep(.2)
    print(f'Timed out waiting for Gazebo service {service}', file=sys.stderr, flush=True)
    raise SystemExit(2)
