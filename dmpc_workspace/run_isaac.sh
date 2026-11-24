#!/usr/bin/env bash
# Forward workspace-level commands to the versioned DMPC repository launcher.
set -euo pipefail
workspace_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
exec bash "$workspace_root/dmpc/run_isaac.sh" "$@"
