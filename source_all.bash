#!/usr/bin/env bash
#
# Enter the rm_sentry_nav_ws environment.
#
#   source /home/diting/work/rm_sentry_nav_ws/source_all.bash
#
# The script sources ROS 2 Humble, the simulation workspace, then sentry_nav.
# Both gazebo/src and the GitHub repository's root src layout are supported.
# It is idempotent, keeps
# working from any current directory, and can also be executed directly (in
# that case a failure exits with status 1 instead of returning).

# Detect whether we are sourced or executed. BASH_SOURCE[0] differs from $0
# only when the file is sourced.
_sentry_ws_is_sourced=0
if [ "${BASH_SOURCE[0]:-$0}" != "$0" ]; then
  _sentry_ws_is_sourced=1
fi

_sentry_ws_source="${BASH_SOURCE[0]:-$0}"
_sentry_ws_root="$(cd "$(dirname "${_sentry_ws_source}")" >/dev/null 2>&1 && pwd)"
if [ -z "${_sentry_ws_root}" ]; then
  printf 'error: cannot resolve the workspace root from %s\n' \
    "${_sentry_ws_source}" >&2
  unset _sentry_ws_source _sentry_ws_root _sentry_ws_is_sourced
  return 1 2>/dev/null || exit 1
fi

# Fast DDS shared-memory transport is unreliable on this host for large
# map/costmap messages. Default to UDPv4 unless the caller overrides it.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

_ros_setup=/opt/ros/humble/setup.bash
_gazebo_root="${_sentry_ws_root}"
if [ -d "${_sentry_ws_root}/gazebo/src" ]; then
  _gazebo_root="${_sentry_ws_root}/gazebo"
fi
_gazebo_install="${_gazebo_root}/install"
_sentry_install="${_sentry_ws_root}/sentry_nav/install"
_gazebo_setup="${_gazebo_install}/setup.bash"
_sentry_setup="${_sentry_install}/setup.bash"

_sentry_ws_error() {
  printf 'error: %s\n' "$1" >&2
}

if [ ! -f "${_ros_setup}" ]; then
  _sentry_ws_error "ROS 2 Humble not found at ${_ros_setup}"
  _sentry_ws_error "install ROS 2 Humble, then re-run this script"
  unset _sentry_ws_source _sentry_ws_root _sentry_ws_is_sourced \
    _ros_setup _gazebo_root _gazebo_install _sentry_install _gazebo_setup _sentry_setup
  unset -f _sentry_ws_error 2>/dev/null
  return 1 2>/dev/null || exit 1
fi

if [ ! -f "${_gazebo_setup}" ]; then
  _sentry_ws_error "missing ${_gazebo_setup}"
  _sentry_ws_error "build it first:"
  _sentry_ws_error "  source /opt/ros/humble/setup.bash"
  _sentry_ws_error "  cd ${_gazebo_root} && colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
  unset _sentry_ws_source _sentry_ws_root _sentry_ws_is_sourced \
    _ros_setup _gazebo_root _gazebo_install _sentry_install _gazebo_setup _sentry_setup
  unset -f _sentry_ws_error 2>/dev/null
  return 1 2>/dev/null || exit 1
fi

if [ ! -f "${_sentry_setup}" ]; then
  _sentry_ws_error "missing ${_sentry_setup}"
  _sentry_ws_error "build it after the gazebo workspace:"
  _sentry_ws_error "  source /opt/ros/humble/setup.bash"
  _sentry_ws_error "  source ${_gazebo_install}/setup.bash"
  _sentry_ws_error "  cd ${_sentry_ws_root}/sentry_nav && colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
  unset _sentry_ws_source _sentry_ws_root _sentry_ws_is_sourced \
    _ros_setup _gazebo_root _gazebo_install _sentry_install _gazebo_setup _sentry_setup
  unset -f _sentry_ws_error 2>/dev/null
  return 1 2>/dev/null || exit 1
fi

# shellcheck disable=SC1090
. "${_ros_setup}"
# shellcheck disable=SC1090
. "${_gazebo_setup}"
# shellcheck disable=SC1090
. "${_sentry_setup}"
if [ -f "${_sentry_ws_root}/sentry_bot/install/setup.bash" ]; then
  . "${_sentry_ws_root}/sentry_bot/install/setup.bash"
fi

printf 'sentry_nav workspace ready: ROS_DOMAIN_ID=%s gazebo=%s sentry_nav=%s\n' \
  "${ROS_DOMAIN_ID}" "${_gazebo_install}" "${_sentry_install}"

unset _sentry_ws_source _sentry_ws_root _sentry_ws_is_sourced \
  _ros_setup _gazebo_root _gazebo_install _sentry_install _gazebo_setup _sentry_setup
unset -f _sentry_ws_error 2>/dev/null
