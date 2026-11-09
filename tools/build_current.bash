#!/usr/bin/env bash
# Build only the current dual-sentry simulation's two workspace layers.

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  printf '请执行此脚本，不要 source 它。\n' >&2
  return 2
fi
set -euo pipefail

usage() {
  cat <<'EOF'
用法：build_current.bash root|overlay|all [--check] [--workspace 目录]
  root     构建当前入口所需的 6 个根包
  overlay  构建 sentry_sim_bringup 的 17 个本地依赖包（先构建根层）
  all      依次构建根层和 overlay，每层使用独立干净环境
  --check  仅核对包选择、源码归属、已有缓存和安装路径，不构建
EOF
}

script_path="$(realpath -- "${BASH_SOURCE[0]}")"
workspace="$(dirname -- "$(dirname -- "$script_path")")"
mode=''
check_only=0
while (($#)); do
  case "$1" in
    root|overlay|all)
      [[ -z "$mode" ]] || { usage >&2; exit 2; }
      mode="$1"; shift ;;
    --check) check_only=1; shift ;;
    --workspace)
      (($# >= 2)) || { usage >&2; exit 2; }
      workspace="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ -n "$mode" ]] || { usage >&2; exit 2; }
workspace="$(realpath -e -- "$workspace")"
[[ -d "$workspace/src" && -d "$workspace/sentry_nav/src" ]] || {
  printf '错误：%s 不是当前两层源码布局。\n' "$workspace" >&2
  exit 1
}

# Re-enter once in a clean environment. No inherited workspace, virtualenv,
# compiler/linker flags, shell startup files or colcon defaults are reused.
if [[ "${CURRENT_BUILD_CLEAN_CHILD:-}" != 1 ]]; then
  clean_env=(
    "HOME=${HOME}" "USER=${USER:-$(id -un)}" "LOGNAME=${LOGNAME:-$(id -un)}"
    'PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    "LANG=${LANG:-C.UTF-8}" 'PYTHONDONTWRITEBYTECODE=1'
    'COLCON_EXTENSION_BLOCKLIST=colcon_core.argument_parser.colcon'
    'CURRENT_BUILD_CLEAN_CHILD=1'
  )
  for key in LC_ALL TERM http_proxy https_proxy all_proxy no_proxy \
      HTTP_PROXY HTTPS_PROXY ALL_PROXY NO_PROXY SSL_CERT_FILE SSL_CERT_DIR \
      REQUESTS_CA_BUNDLE CURL_CA_BUNDLE GIT_SSL_CAINFO; do
    if [[ -v "$key" ]]; then
      clean_env+=("$key=${!key}")
    fi
  done
  if [[ -n "${CMAKE_BUILD_PARALLEL_LEVEL:-}" ]]; then
    [[ "$CMAKE_BUILD_PARALLEL_LEVEL" =~ ^[1-9][0-9]*$ ]] || {
      printf '错误：CMAKE_BUILD_PARALLEL_LEVEL 应为正整数。\n' >&2
      exit 2
    }
    clean_env+=("CMAKE_BUILD_PARALLEL_LEVEL=$CMAKE_BUILD_PARALLEL_LEVEL")
  fi
  child_args=("$mode" --workspace "$workspace")
  ((check_only == 0)) || child_args+=(--check)
  exec env -i "${clean_env[@]}" /bin/bash --noprofile --norc \
    "$script_path" "${child_args[@]}"
fi

[[ -f /opt/ros/humble/setup.bash ]] || {
  printf '错误：未找到 /opt/ros/humble/setup.bash。\n' >&2
  exit 1
}
command -v colcon >/dev/null || { printf '错误：未安装系统 colcon。\n' >&2; exit 1; }

root_packages=(
  rmoss_gz_plugins rmu_gazebo_simulator pb2025_robot_description
  pb_omni_pid_pursuit_controller tdt_yastar_planner sentry_planner_server
)
duplicate_packages=(
  fake_vel_transform ign_sim_pointcloud_tool livox_ros_driver2 loam_interface
  pb2025_nav_bringup pb2025_sentry_nav pb_nav2_plugins pb_omni_pid_pursuit_controller
  pb_teleop_twist_joy point_lio pointcloud_to_laserscan sensor_scan_generation
  small_gicp_relocalization terrain_analysis terrain_analysis_ext
)

source_setup() {
  # ROS generated environment scripts are not written for nounset.
  set +u
  source "$1"
  set -u
}

validate_selection() {
  local layer="$1" listing="$2"
  CURRENT_BUILD_WS="$workspace" CURRENT_BUILD_LAYER="$layer" \
    CURRENT_BUILD_LISTING="$listing" /usr/bin/python3 - <<'PY'
import os
from pathlib import Path

ws = Path(os.environ['CURRENT_BUILD_WS'])
layer = os.environ['CURRENT_BUILD_LAYER']
root = {
    'rmoss_gz_plugins': 'src/rmoss_gazebo/rmoss_gz_plugins',
    'rmu_gazebo_simulator': 'src/rmu_gazebo_simulator/rmu_gazebo_simulator',
    'pb2025_robot_description': 'src/pb2025_robot_description',
    'pb_omni_pid_pursuit_controller': 'src/pb2025_sentry_nav/pb_omni_pid_pursuit_controller',
    'tdt_yastar_planner': 'src/pb2025_sentry_nav/tdt_yastar_planner',
    'sentry_planner_server': 'src/pb2025_sentry_nav/sentry_planner_server',
}
duplicates = '''fake_vel_transform ign_sim_pointcloud_tool livox_ros_driver2
loam_interface pb2025_nav_bringup pb2025_sentry_nav pb_nav2_plugins
pb_omni_pid_pursuit_controller pb_teleop_twist_joy point_lio
pointcloud_to_laserscan sensor_scan_generation small_gicp_relocalization
terrain_analysis terrain_analysis_ext'''.split()
overlay = {n: f'sentry_nav/src/pb2025_sentry_nav/{n}' for n in duplicates}
overlay.update({n: f'sentry_nav/src/{n}' for n in ['sentry_sim_bringup', 'sentry_tunnel_bt']})
expected = root if layer == 'root' else overlay
actual = {}
for line in os.environ['CURRENT_BUILD_LISTING'].splitlines():
    fields = line.split('\t')
    if len(fields) < 2 or fields[0] in actual:
        raise SystemExit(f'错误：无法唯一解析 colcon 包列表：{line!r}')
    actual[fields[0]] = Path(fields[1]).resolve()
if set(actual) != set(expected):
    raise SystemExit(f'错误：{layer} 包集合变化；缺少 {sorted(set(expected)-set(actual))}；'
                     f'额外 {sorted(set(actual)-set(expected))}')
build_base = ws / ('build' if layer == 'root' else 'sentry_nav/build')
install_base = ws / ('install' if layer == 'root' else 'sentry_nav/install')
for name, relative in sorted(expected.items()):
    source = (ws / relative).resolve()
    if actual[name] != source:
        raise SystemExit(f'错误：{name} 源码归属不符：{actual[name]}，预期 {source}')
    cache = build_base / name / 'CMakeCache.txt'
    if cache.is_file():
        data = {}
        for line in cache.read_text().splitlines():
            if '=' in line and not line.startswith(('//', '#')):
                key, value = line.split('=', 1)
                data[key.split(':', 1)[0]] = value
        for key, value in [('CMAKE_HOME_DIRECTORY', source),
                           ('CMAKE_INSTALL_PREFIX', install_base / name)]:
            if key in data and Path(data[key]).resolve() != value.resolve():
                raise SystemExit(f'错误：{cache} 的 {key} 指向 {data[key]}；不自动清理缓存')
        if layer == 'root':
            overlay_install = (ws / 'sentry_nav/install').resolve()
            for key, value in data.items():
                if key.endswith('_DIR') and value.startswith('/'):
                    if Path(value).resolve().is_relative_to(overlay_install):
                        raise SystemExit(f'错误：{cache} 的 {key} 指向 overlay；不自动清理缓存')
print(f'{layer}：包选择与源码归属通过（{len(expected)} 包）。')
PY
}

check_installed_prefixes() (
  local layer="$1" install_base
  if [[ "$layer" == root ]]; then
    install_base="$workspace/install"
  else
    install_base="$workspace/sentry_nav/install"
  fi
  if [[ ! -f "$install_base/local_setup.bash" ]]; then
    printf '%s：尚无安装环境，跳过安装路径核对（源码选择已检查）。\n' "$layer"
    return 0
  fi
  source_setup "$install_base/local_setup.bash"
  CURRENT_BUILD_WS="$workspace" CURRENT_BUILD_LAYER="$layer" \
    CURRENT_BUILD_CHECK_ONLY="$check_only" /usr/bin/python3 - <<'PY'
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix, PackageNotFoundError

ws = Path(os.environ['CURRENT_BUILD_WS'])
layer = os.environ['CURRENT_BUILD_LAYER']
if layer == 'root':
    names = ['rmoss_gz_plugins', 'rmu_gazebo_simulator', 'pb2025_robot_description',
             'pb_omni_pid_pursuit_controller', 'tdt_yastar_planner', 'sentry_planner_server']
    base = ws / 'install'
else:
    names = '''fake_vel_transform ign_sim_pointcloud_tool livox_ros_driver2
    loam_interface pb2025_nav_bringup pb2025_sentry_nav pb_nav2_plugins
    pb_omni_pid_pursuit_controller pb_teleop_twist_joy point_lio
    pointcloud_to_laserscan sensor_scan_generation small_gicp_relocalization
    terrain_analysis terrain_analysis_ext sentry_sim_bringup sentry_tunnel_bt'''.split()
    base = ws / 'sentry_nav/install'
for name in names:
    expected = (base / name).resolve()
    if not (expected / 'share/ament_index/resource_index/packages' / name).exists():
        if os.environ['CURRENT_BUILD_CHECK_ONLY'] != '1':
            raise SystemExit(f'错误：构建后未找到 {name} 的预期安装索引。')
        print(f'{layer}：{name} 尚未安装到预期位置，不能据此确认构建完成。')
        continue
    try:
        actual = Path(get_package_prefix(name)).resolve()
    except PackageNotFoundError:
        raise SystemExit(f'错误：{name} 安装索引存在，但当前环境无法解析。')
    if actual != expected:
        raise SystemExit(f'错误：{name} 实际来自 {actual}，预期 {expected}')
print(f'{layer}：已有包的安装路径核对完成。')
PY
)

run_layer() (
  local layer="$1" layer_root listing
  source_setup /opt/ros/humble/setup.bash
  if [[ "$layer" == root ]]; then
    layer_root="$workspace"
    selection=(--packages-select "${root_packages[@]}")
  else
    layer_root="$workspace/sentry_nav"
    if [[ -f "$workspace/install/local_setup.bash" ]]; then
      # Source only the explicitly selected underlay, never its stored parent chain.
      source_setup "$workspace/install/local_setup.bash"
    elif ((check_only == 0)); then
      printf '错误：overlay 构建前须先构建根层。\n' >&2
      exit 1
    else
      printf 'overlay：根层尚未安装；本次只能核对源码选择。\n'
    fi
    selection=(--packages-up-to sentry_sim_bringup \
      --allow-overriding "${duplicate_packages[@]}")
  fi
  cd "$layer_root"
  listing="$(colcon --log-base /dev/null list --ignore-user-meta --metas \
    --base-paths "$layer_root/src" "${selection[@]}")"
  validate_selection "$layer" "$listing"
  if ((check_only)); then
    check_installed_prefixes "$layer"
    return 0
  fi
  printf '开始构建 %s；不清理已有 build/install/log。\n' "$layer"
  colcon build --ignore-user-meta --metas --base-paths "$layer_root/src" \
    --build-base "$layer_root/build" --install-base "$layer_root/install" \
    --symlink-install "${selection[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release
  check_installed_prefixes "$layer"
)

case "$mode" in
  root|overlay) run_layer "$mode" ;;
  all) run_layer root; run_layer overlay ;;
esac
if ((check_only)); then
  printf '检查完成：未执行构建、依赖安装或下载。\n'
fi
