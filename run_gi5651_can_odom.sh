#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_SETUP="${SCRIPT_DIR}/install/setup.bash"

if [[ ! -f "${INSTALL_SETUP}" ]]; then
    echo "Missing ${INSTALL_SETUP}. Build the package with colcon first." >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source "${INSTALL_SETUP}"

exec ros2 launch gi5651_can_odom gi5651_can_odom.launch.py "$@"
