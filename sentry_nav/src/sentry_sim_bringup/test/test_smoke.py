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

"""Minimal import smoke tests for the sentry_sim_bringup package.

These tests do not start Gazebo or any ROS node; they only check that the
package and its gateway entry point can be imported.
"""


def test_package_imports():
    """The top-level package must be importable."""
    import sentry_sim_bringup

    assert sentry_sim_bringup is not None


def test_gateway_module_imports():
    """The gateway module must expose a callable ``main``."""
    from sentry_sim_bringup import sim_serial_gateway

    assert callable(sim_serial_gateway.main)
