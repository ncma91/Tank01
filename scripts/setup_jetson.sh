#!/usr/bin/env bash
# Tank01 – one-time setup of a Jetson AGX Orin (JetPack 6.x / Ubuntu 22.04).
# Usage: sudo ./scripts/setup_jetson.sh
# See docs/05_jetson.md. Idempotent: safe to run again.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_USER="${SUDO_USER:-$USER}"
ROS_DISTRO=humble

if [[ $EUID -ne 0 ]]; then echo "please run with sudo"; exit 1; fi

echo "==> Performance mode (MAXN)"
nvpmodel -m 0 || true
jetson_clocks || true

echo "==> ROS 2 ${ROS_DISTRO} apt repository"
apt-get update
apt-get install -y curl gnupg lsb-release software-properties-common
add-apt-repository -y universe
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
apt-get update

echo "==> ROS 2 packages"
apt-get install -y \
  ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-robot-localization \
  ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-spatio-temporal-voxel-layer ros-${ROS_DISTRO}-twist-mux \
  ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-teleop-twist-joy \
  ros-${ROS_DISTRO}-ublox-gps ros-${ROS_DISTRO}-ntrip-client \
  ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-diagnostic-aggregator \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rosbag2-storage-mcap \
  python3-colcon-common-extensions python3-rosdep python3-can python3-pytest \
  can-utils chrony gpsd gpsd-clients linuxptp
# Not in apt for Humble/arm64 – build from source in ros2_ws/src if needed:
#   ouster-ros, livox_ros_driver2, xsens_mti_driver, kiss-icp, patchwork-plusplus, zed-ros2-wrapper

[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || rosdep init
sudo -u "$TARGET_USER" rosdep update

echo "==> CAN interface (systemd)"
install -m 755 "$REPO_DIR/scripts/can_setup.sh" /usr/local/bin/tank01-can-setup
cat > /etc/systemd/system/tank01-can.service <<UNIT
[Unit]
Description=Tank01 CAN bus (can0, 500 kbit/s)
After=network-pre.target
[Service]
Type=oneshot
ExecStart=/usr/local/bin/tank01-can-setup can0 500000
RemainAfterExit=yes
[Install]
WantedBy=multi-user.target
UNIT

echo "==> udev rules"
install -m 644 "$REPO_DIR/udev/99-tank01.rules" /etc/udev/rules.d/99-tank01.rules
udevadm control --reload-rules && udevadm trigger
usermod -aG dialout,plugdev "$TARGET_USER"

echo "==> LiDAR network (eth1 192.168.10.1/24)"
if command -v nmcli >/dev/null; then
  nmcli con show tank01-lidar >/dev/null 2>&1 || \
    nmcli con add type ethernet ifname eth1 con-name tank01-lidar \
      ipv4.method manual ipv4.addresses 192.168.10.1/24 ipv6.method ignore || true
fi

echo "==> Environment for ${TARGET_USER}"
BASHRC="/home/${TARGET_USER}/.bashrc"
grep -q "tank01 env" "$BASHRC" || cat >> "$BASHRC" <<ENV
# --- tank01 env ---
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f ${REPO_DIR}/ros2_ws/install/setup.bash ] && source ${REPO_DIR}/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
ENV

echo "==> Autostart service (enable manually after commissioning!)"
sed -e "s|@USER@|${TARGET_USER}|g" -e "s|@REPO@|${REPO_DIR}|g" \
  "$REPO_DIR/scripts/tank01.service" > /etc/systemd/system/tank01.service

systemctl daemon-reload
systemctl enable --now tank01-can.service
echo "Done. Re-login, then: cd ${REPO_DIR}/ros2_ws && colcon build --symlink-install"
echo "Enable autostart only after commissioning: sudo systemctl enable tank01"
