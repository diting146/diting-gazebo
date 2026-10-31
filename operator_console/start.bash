#!/usr/bin/env bash
set -e
console_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
console_workspace="$(cd -- "${console_root}/.." && pwd)"
source "${console_workspace}/source_all.bash"
exec python3 "${console_root}/operator_console.py" "$@"
