"""ROS 2 node bridging /cmd_vel_safe and the Teensy base controller over CAN.

Publishes wheel odometry, joint states, battery state, e-stop status and diagnostics.
With ``use_fake_hardware: true`` the CAN bus is replaced by a simple simulated base
(commanded track speeds are integrated into encoder ticks) for desktop testing.
"""

from __future__ import annotations

import math
import struct
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Bool, Float32, String, UInt32
from tf2_ros import TransformBroadcaster

from tank01_base import can_protocol as proto
from tank01_base.kinematics import (Odometry, SlipEstimator, TrackParams, rate_limit,
                                    twist_to_tracks)


SAFETY_LEVELS = {'OK': 0, 'SLOW': 1, 'PAUSE': 2, 'STOP': 3, 'ESTOP': 4}


class FakeBus:
    """Stand-in for python-can's Bus: integrates drive commands into encoder ticks."""

    def __init__(self, params: TrackParams):
        self._p = params
        self._ticks = [0.0, 0.0]
        self._speeds = (0.0, 0.0)
        self._last = time.monotonic()
        self._outbox: list[tuple[int, bytes]] = []

    def send(self, msg) -> None:
        if msg.arbitration_id == proto.CMD_DRIVE:
            left_mm, right_mm, *_ = struct.unpack(proto.FORMATS[proto.CMD_DRIVE],
                                                  bytes(msg.data))
            self._speeds = (left_mm / 1000.0, right_mm / 1000.0)

    def recv(self, timeout: float = 0.0):
        now = time.monotonic()
        dt, self._last = now - self._last, now
        for i in (0, 1):
            self._ticks[i] += self._speeds[i] * dt / self._p.metres_per_tick
        if not self._outbox:
            self._outbox = [
                (proto.ENCODERS, struct.pack('<ii', int(self._ticks[0]), int(self._ticks[1]))),
                (proto.STATUS, struct.pack(proto.FORMATS[proto.STATUS],
                                           proto.BaseState.ACTIVE, 0, 0b111, 0, 5200, 80, 30)),
            ]
        can_id, data = self._outbox.pop(0)
        return _Frame(can_id, data)

    def shutdown(self) -> None:
        pass


class _Frame:
    def __init__(self, arbitration_id: int, data: bytes):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = False


class BaseNode(Node):

    def __init__(self) -> None:
        super().__init__('base_node')
        self.declare_parameters('', [
            ('can_interface', 'can0'),
            ('use_fake_hardware', False),
            ('track_width', 0.95),
            ('track_width_factor', 1.5),
            ('sprocket_radius', 0.105),
            ('ticks_per_rev', 4096),
            ('max_track_speed', 2.2),
            ('max_accel', 1.0),
            ('estimate_slip', False),
            ('cmd_timeout', 0.25),
            ('control_rate', 50.0),
            ('publish_tf', False),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
            ('left_joint', 'sprocket_left_joint'),
            ('right_joint', 'sprocket_right_joint'),
            ('invert_left_encoder', False),
            ('invert_right_encoder', False),
            # 'encoders': odometry from the track encoders (recommended)
            # 'command':  open-loop odometry from the commanded track speeds, for chassis
            #             without encoders (prototype) - much larger covariance
            ('odometry_source', 'encoders'),
        ])
        g = self.get_parameter
        self.params = TrackParams(
            track_width=g('track_width').value,
            track_width_factor=g('track_width_factor').value,
            sprocket_radius=g('sprocket_radius').value,
            ticks_per_rev=g('ticks_per_rev').value,
            max_track_speed=g('max_track_speed').value,
            max_accel=g('max_accel').value,
        )
        self.odom = Odometry(self.params)
        self.slip = SlipEstimator(self.params.track_width, self.params.track_width_factor)
        self.estimate_slip = g('estimate_slip').value
        self.cmd_timeout = g('cmd_timeout').value
        self.publish_tf = g('publish_tf').value
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value
        self.joint_names = [g('left_joint').value, g('right_joint').value]
        self.odom_source = g('odometry_source').value
        if self.odom_source not in ('encoders', 'command'):
            raise ValueError(f'odometry_source must be encoders|command, not {self.odom_source}')
        self.cov_scale = 1.0 if self.odom_source == 'encoders' else 25.0
        self.virtual_ticks = [0.0, 0.0]
        self.enc_sign = (-1 if g('invert_left_encoder').value else 1,
                         -1 if g('invert_right_encoder').value else 1)

        self.bus = self._open_bus(g('can_interface').value, g('use_fake_hardware').value)

        # state
        self.cmd = Twist()
        self.cmd_stamp = 0.0
        self.track_out = (0.0, 0.0)
        self.blade_rpm = 0.0
        self.cut_height = 0.07
        self.safety_level = 0
        self.heartbeat_rx = 0
        self.hb_counter = 0
        self.seq = 0
        self.last_enc_time: float | None = None
        self.status: proto.Status | None = None
        self.motors: dict[str, proto.MotorStatus] = {}
        self.bms: proto.BmsStatus | None = None

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, 'cmd_vel_safe', self._on_cmd, reliable)
        self.create_subscription(Float32, 'blade/cmd', self._on_blade, reliable)
        self.create_subscription(Float32, 'mower/height_cmd', self._on_height, reliable)
        self.create_subscription(UInt32, 'safety/heartbeat', self._on_heartbeat, reliable)
        self.create_subscription(String, 'safety/state', self._on_safety_state, reliable)
        self.create_subscription(Imu, 'imu/data', self._on_imu, 20)

        self.pub_odom = self.create_publisher(OdometryMsg, 'odom/wheel', 20)
        self.pub_joint = self.create_publisher(JointState, 'joint_states', 20)
        self.pub_batt = self.create_publisher(BatteryState, 'battery/state', 5)
        self.pub_estop = self.create_publisher(Bool, 'base/estop', reliable)
        self.pub_diag = self.create_publisher(DiagnosticArray, 'base/status', 5)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None

        rate = g('control_rate').value
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._control_loop)
        self.create_timer(0.002, self._poll_can)
        self.create_timer(1.0, self._slow_loop)
        self.get_logger().info('base_node up (fake=%s)' % g('use_fake_hardware').value)

    # ------------------------------------------------------------------ CAN
    def _open_bus(self, interface: str, fake: bool):
        if fake:
            return FakeBus(self.params)
        import can  # python3-can
        return can.interface.Bus(channel=interface, interface='socketcan',
                                 receive_own_messages=False)

    def _send(self, can_id: int, data: bytes) -> None:
        if isinstance(self.bus, FakeBus):
            self.bus.send(_Frame(can_id, data))
            return
        import can
        try:
            self.bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False),
                          timeout=0.005)
        except can.CanError as exc:
            self.get_logger().warn(f'CAN send failed: {exc}', throttle_duration_sec=2.0)

    def _poll_can(self) -> None:
        for _ in range(32):
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                return
            self._handle_frame(msg.arbitration_id, msg.data)
            if isinstance(self.bus, FakeBus):
                return

    def _handle_frame(self, can_id: int, data: bytes) -> None:
        decoded = proto.decode(can_id, data)
        if decoded is None:
            return
        if can_id == proto.ENCODERS:
            if self.odom_source == 'encoders':
                self._on_encoders(*decoded)
        elif can_id == proto.STATUS:
            self.status = decoded
            self.pub_estop.publish(Bool(data=decoded.estop))
        elif can_id == proto.MOTOR_LEFT:
            self.motors['left'] = decoded
        elif can_id == proto.MOTOR_RIGHT:
            self.motors['right'] = decoded
        elif can_id == proto.MOTOR_BLADE:
            self.motors['blade'] = decoded
        elif can_id == proto.BMS:
            self.bms = decoded

    # ------------------------------------------------------------- callbacks
    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = msg
        self.cmd_stamp = time.monotonic()

    def _on_blade(self, msg: Float32) -> None:
        self.blade_rpm = max(0.0, float(msg.data))

    def _on_height(self, msg: Float32) -> None:
        self.cut_height = float(msg.data)

    def _on_heartbeat(self, msg: UInt32) -> None:
        self.heartbeat_rx = time.monotonic()

    def _on_safety_state(self, msg: String) -> None:
        level = msg.data.split(':', 1)[0].strip()
        self.safety_level = SAFETY_LEVELS.get(level, SAFETY_LEVELS['STOP'])

    def _on_imu(self, msg: Imu) -> None:
        if not self.estimate_slip:
            return
        left, right = self.track_out
        chi = self.slip.update(left, right, msg.angular_velocity.z)
        self.params.track_width_factor = chi

    def _on_encoders(self, left: int, right: int) -> None:
        now = time.monotonic()
        dt = 0.0 if self.last_enc_time is None else now - self.last_enc_time
        self.last_enc_time = now
        self.odom.update(self.enc_sign[0] * left, self.enc_sign[1] * right, dt)
        self._publish_odom(left, right)

    # ------------------------------------------------------------- control
    def _control_loop(self) -> None:
        now = time.monotonic()
        stale_cmd = now - self.cmd_stamp > self.cmd_timeout
        supervisor_alive = now - self.heartbeat_rx < 0.5
        if stale_cmd or not supervisor_alive:
            v, w = 0.0, 0.0
        else:
            v, w = self.cmd.linear.x, self.cmd.angular.z
        tl, tr = twist_to_tracks(v, w, self.params)
        left = rate_limit(tl, self.track_out[0], self.params.max_accel, self.dt)
        right = rate_limit(tr, self.track_out[1], self.params.max_accel, self.dt)
        self.track_out = (left, right)
        if self.odom_source == 'command':
            for i, speed in enumerate(self.track_out):
                self.virtual_ticks[i] += speed * self.dt / self.params.metres_per_tick
            self._on_encoders(int(self.virtual_ticks[0]), int(self.virtual_ticks[1]))

        moving = abs(left) > 1e-3 or abs(right) > 1e-3 or abs(tl) > 1e-3 or abs(tr) > 1e-3
        flags = proto.DriveFlag.BEACON
        if moving:
            flags |= proto.DriveFlag.RELEASE_BRAKE
        mode = proto.DriveMode.AUTO if supervisor_alive else proto.DriveMode.STANDBY
        self.seq += 1
        self._send(proto.CMD_DRIVE, proto.encode_drive(left, right, mode, flags, self.seq))

        # Heartbeat is only forwarded while the safety supervisor is alive, so the Teensy
        # watchdog also covers a crashed supervisor, not just a crashed base_node.
        if supervisor_alive:
            self.hb_counter += 1
            self._send(proto.HEARTBEAT, proto.encode_heartbeat(self.hb_counter,
                                                               self.safety_level))
        if self.seq % 5 == 0:
            blade_on = self.blade_rpm > 0.0 and supervisor_alive
            self._send(proto.CMD_BLADE, proto.encode_blade(
                self.blade_rpm if blade_on else 0.0, self.cut_height, blade_on))

    def _slow_loop(self) -> None:
        self._send(proto.TIME_SYNC, proto.encode_time_sync(time.time_ns() // 1000))
        self._publish_battery()
        self._publish_diagnostics()

    # ------------------------------------------------------------- publish
    def _publish_odom(self, left_ticks: int, right_ticks: int) -> None:
        stamp = self.get_clock().now().to_msg()
        o = self.odom
        msg = OdometryMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = o.x
        msg.pose.pose.position.y = o.y
        msg.pose.pose.orientation.z = math.sin(o.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(o.yaw / 2.0)
        msg.twist.twist.linear.x = o.v
        msg.twist.twist.angular.z = o.omega
        # Tracks slip: trust v moderately, yaw rate poorly (the IMU dominates in the EKF).
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        for i, var in enumerate((0.05, 0.05, 1e3, 1e3, 1e3, 0.2)):
            pose_cov[i * 7] = var
        for i, var in enumerate((0.0025, 0.0025, 1e3, 1e3, 1e3, 0.01 + 0.1 * abs(o.omega))):
            twist_cov[i * 7] = var * (self.cov_scale if var < 1e3 else 1.0)
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.pub_odom.publish(msg)

        js = JointState()
        js.header.stamp = stamp
        js.name = self.joint_names
        rad_per_tick = 2.0 * math.pi / self.params.ticks_per_rev
        js.position = [self.enc_sign[0] * left_ticks * rad_per_tick,
                       self.enc_sign[1] * right_ticks * rad_per_tick]
        self.pub_joint.publish(js)

        if self.tf is not None:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = o.x
            t.transform.translation.y = o.y
            t.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(t)

    def _publish_battery(self) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        msg.present = True
        if self.bms is not None:
            msg.voltage = self.bms.voltage_v
            msg.current = -self.bms.current_a  # ROS: negative = discharging
            msg.percentage = self.bms.soc / 100.0
            msg.temperature = float(self.bms.temp_c)
        elif self.status is not None:
            msg.voltage = self.status.battery_v
            msg.percentage = self.status.soc / 100.0
        else:
            return
        self.pub_batt.publish(msg)

    def _publish_diagnostics(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus(name='tank01/base', hardware_id='teensy41')
        if self.status is None:
            st.level = DiagnosticStatus.ERROR
            st.message = 'no status from base controller'
        else:
            s = self.status
            st.level = DiagnosticStatus.OK if s.state == proto.BaseState.ACTIVE else (
                DiagnosticStatus.ERROR if s.state in (proto.BaseState.FAULT,
                                                      proto.BaseState.ESTOP)
                else DiagnosticStatus.WARN)
            st.message = s.state.name
            st.values = [KeyValue(key='safety_flags', value=hex(s.safety_flags)),
                         KeyValue(key='fault_code', value=str(s.fault_code)),
                         KeyValue(key='vesc_ok_mask', value=bin(s.vesc_ok_mask)),
                         KeyValue(key='track_width_factor',
                                  value=f'{self.params.track_width_factor:.3f}')]
            for name, m in self.motors.items():
                st.values.append(KeyValue(key=f'{name}_current_a', value=f'{m.current_a:.1f}'))
                st.values.append(KeyValue(key=f'{name}_temp_c', value=f'{m.temp_c:.1f}'))
        arr.status = [st]
        self.pub_diag.publish(arr)

    def destroy_node(self) -> bool:
        try:
            self._send(proto.CMD_DRIVE, proto.encode_drive(0.0, 0.0, proto.DriveMode.STANDBY,
                                                           0, self.seq + 1))
            self.bus.shutdown()
        except Exception as exc:  # noqa: BLE001 - best effort on shutdown
            self.get_logger().warn(f'shutdown: {exc}')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
