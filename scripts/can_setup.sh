#!/usr/bin/env bash
# Bring up a SocketCAN interface: can_setup.sh [iface] [bitrate]
set -euo pipefail
IFACE="${1:-can0}"
BITRATE="${2:-500000}"
# Jetson AGX Orin on-board CAN (mttcan) needs its kernel modules; USB adapters (gs_usb,
# peak_usb) are loaded automatically.
modprobe can || true
modprobe can_raw || true
modprobe mttcan 2>/dev/null || true
ip link set "$IFACE" down 2>/dev/null || true
ip link set "$IFACE" type can bitrate "$BITRATE" restart-ms 100 berr-reporting on
ip link set "$IFACE" txqueuelen 1000
ip link set "$IFACE" up
ip -details link show "$IFACE"
