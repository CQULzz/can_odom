#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_PY="${SCRIPT_DIR}/usbcan_to_socketcan.py"

SOCKETCAN_IF="${SOCKETCAN_IF:-vcan0}"
LIB_PATH="${LIB_PATH:-}"
DEVICE_TYPE="${DEVICE_TYPE:-4}"
DEVICE_INDEX="${DEVICE_INDEX:-0}"
CHANNEL="${CHANNEL:-0}"
BAUD="${BAUD:-500k}"

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run this script with sudo." >&2
    exit 1
fi

if [[ ! -f "${BRIDGE_PY}" ]]; then
    echo "Bridge script not found: ${BRIDGE_PY}" >&2
    exit 1
fi

if [[ -n "${LIB_PATH}" && "${LIB_PATH}" != "auto" && ! -f "${LIB_PATH}" ]]; then
    echo "Vendor library not found: ${LIB_PATH}" >&2
    exit 1
fi

modprobe vcan

if ! ip link show "${SOCKETCAN_IF}" >/dev/null 2>&1; then
    ip link add dev "${SOCKETCAN_IF}" type vcan
fi

ip link set "${SOCKETCAN_IF}" up

echo "SocketCAN interface ready: ${SOCKETCAN_IF}"
ip -details link show "${SOCKETCAN_IF}"

bridge_args=(
    --socketcan-if "${SOCKETCAN_IF}"
    --device-type "${DEVICE_TYPE}"
    --device-index "${DEVICE_INDEX}"
    --channel "${CHANNEL}"
    --baud "${BAUD}"
)

if [[ -n "${LIB_PATH}" ]]; then
    bridge_args+=(--lib-path "${LIB_PATH}")
fi

exec python3 "${BRIDGE_PY}" "${bridge_args[@]}"
