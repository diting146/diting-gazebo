#!/usr/bin/env bash
# Copyright 2026 diting
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Regenerate models/pb2025_sentry_robot.sdf from the upstream xmacro.
#
# Two plugin blocks are stripped after xmacro expansion:
#   * LightBarController: Gazebo's rendering/Sensors system aborts with
#     "Another item already exists with name:
#     <robot>::armor_0::light_bar_visual" and segfaults.
#   * ProjectileShooter: requires model://rm_projectile_17mm, which is absent
#     from this workspace, and shooting is not needed for navigation.
# No other plugin, link, joint, sensor or material is modified.

set -euo pipefail

PKG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GAZEBO_SRC="${GAZEBO_SRC:-/home/diting/work/rm_sentry_nav_ws/gazebo/src}"
PB_SRC="${PB2025_ROBOT_DESCRIPTION_SRC:-${GAZEBO_SRC}/pb2025_robot_description}"
RMU_SRC="${RMU_GAZEBO_SIMULATOR_SRC:-${GAZEBO_SRC}/rmu_gazebo_simulator/rmu_gazebo_simulator}"

if command -v xmacro4sdf >/dev/null 2>&1; then
  XMACRO4SDF="$(command -v xmacro4sdf)"
elif [ -x "${HOME}/.local/bin/xmacro4sdf" ]; then
  XMACRO4SDF="${HOME}/.local/bin/xmacro4sdf"
else
  echo "error: xmacro4sdf not found" >&2
  exit 1
fi

XMACRO_FILE="${PB_SRC}/resource/xmacro/pb2025_sentry_robot.sdf.xmacro"
OUTPUT="${PKG_ROOT}/models/pb2025_sentry_robot.sdf"

export IGN_GAZEBO_RESOURCE_PATH="${PKG_ROOT}/models/vendor"
export IGN_GAZEBO_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH}:${PB_SRC}/resource/models"
export IGN_GAZEBO_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH}:${RMU_SRC}/resource/models"

RAW="$(mktemp)"
STRIPPED="$(mktemp)"
trap 'rm -f "${RAW}" "${STRIPPED}"' EXIT

"${XMACRO4SDF}" "${XMACRO_FILE}" > "${RAW}"

python3 - "${RAW}" "${STRIPPED}" <<'PY'
import re
import sys

source, destination = sys.argv[1], sys.argv[2]
with open(source, encoding='utf-8') as handle:
    text = handle.read()

stripped = re.sub(
    r'[ \t]*<plugin filename="'
    r'(?:LightBarController|ProjectileShooter)".*?</plugin>\n',
    '',
    text,
    flags=re.S,
)
with open(destination, 'w', encoding='utf-8') as handle:
    handle.write(stripped)
PY

if grep -qE 'LightBarController|ProjectileShooter' "${STRIPPED}"; then
  echo "error: plugin stripping failed" >&2
  exit 1
fi

install -m 0644 "${STRIPPED}" "${OUTPUT}"
echo "wrote ${OUTPUT}"
