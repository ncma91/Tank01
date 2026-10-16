"""ROS 2 driver for the Unicore UM982 (prototype GNSS: RTK position + dual-antenna heading).

Publishes   /gnss/fix (NavSatFix), /gnss/heading (Imu, yaw only), /gnss/status (String)
Subscribes  /rtcm (rtcm_msgs/Message, optional) → written to the receiver
"""

from __future__ import annotations

import math
import threading

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String

from tank01_gnss import um982_protocol as p


class Um982Node(Node):

    def __init__(self) -> None:
        super().__init__('um982_node')
        self.declare_parameters('', [
            ('port', '/dev/tank01/gnss'),
            ('baudrate', 460800),
            ('frame_id', 'gnss_front'),
            ('heading_frame_id', 'base_link'),
            ('mount_offset_deg', 0.0),
            ('min_baseline_m', 0.3),
            ('verify_checksums', True),
            ('verify_unicore_crc', False),   # enable after checking with the real device
            ('config_commands', ['']),       # e.g. ['GPGGA 0.1', 'GPGST 0.1', 'UNIHEADINGA 0.1']
        ])
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.frame_id = g('frame_id')
        self.heading_frame_id = g('heading_frame_id')
        self.mount_offset = g('mount_offset_deg')
        self.min_baseline = g('min_baseline_m')
        self.verify_nmea = g('verify_checksums')
        self.verify_crc = g('verify_unicore_crc')
        self.last_gst: p.Gst | None = None
        self.bad_lines = 0

        self.ser = serial.Serial(g('port'), g('baudrate'), timeout=0.2)
        for cmd in g('config_commands'):
            if cmd:
                self.ser.write((cmd + '\r\n').encode('ascii'))
        self.write_lock = threading.Lock()

        self.pub_fix = self.create_publisher(NavSatFix, 'gnss/fix', qos_profile_sensor_data)
        self.pub_heading = self.create_publisher(Imu, 'gnss/heading', qos_profile_sensor_data)
        self.pub_status = self.create_publisher(String, 'gnss/status', 10)
        self._subscribe_rtcm()

        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        self.get_logger().info(f'UM982 on {g("port")} @ {g("baudrate")} baud')

    def _subscribe_rtcm(self) -> None:
        try:
            from rtcm_msgs.msg import Message as RtcmMessage
        except ImportError:
            self.get_logger().warn('rtcm_msgs not installed - no RTK corrections via /rtcm')
            return
        self.create_subscription(RtcmMessage, 'rtcm', self._on_rtcm, 10)

    def _on_rtcm(self, msg) -> None:
        with self.write_lock:
            self.ser.write(bytes(msg.message))

    # ------------------------------------------------------------------ input
    def _read_loop(self) -> None:
        while rclpy.ok():
            try:
                raw = self.ser.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f'serial error: {exc}', throttle_duration_sec=5.0)
                continue
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            try:
                self.handle_line(line)
            except (ValueError, IndexError) as exc:
                self.bad_lines += 1
                self.get_logger().debug(f'unparsable line ({exc}): {line[:60]}')

    def handle_line(self, line: str) -> None:
        if line.startswith('$'):
            if self.verify_nmea and not p.nmea_checksum_ok(line):
                self.bad_lines += 1
                return
            tag = line[3:6]
            if tag == 'GST':
                self.last_gst = p.parse_gst(line)
            elif tag == 'GGA':
                gga = p.parse_gga(line)
                if gga is not None:
                    self._publish_fix(gga)
        elif line.startswith('#UNIHEADINGA'):
            if self.verify_crc and not p.unicore_crc_ok(line):
                self.bad_lines += 1
                return
            hdg = p.parse_uniheading(line)
            if hdg is not None:
                self._publish_heading(hdg)

    # ----------------------------------------------------------------- output
    def _publish_fix(self, gga: p.Gga) -> None:
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.latitude, msg.longitude, msg.altitude = gga.lat, gga.lon, gga.alt
        msg.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GALILEO
        msg.status.status = {0: NavSatStatus.STATUS_NO_FIX, 1: NavSatStatus.STATUS_FIX,
                             2: NavSatStatus.STATUS_SBAS_FIX}.get(
            gga.quality, NavSatStatus.STATUS_GBAS_FIX)
        if self.last_gst is not None and not math.isnan(self.last_gst.alt_std):
            sx, sy, sz = self.last_gst.lon_std, self.last_gst.lat_std, self.last_gst.alt_std
            # GST can be optimistic for RTK fixed - never below 1 cm
            sx, sy, sz = max(sx, 0.01), max(sy, 0.01), max(sz, 0.02)
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            sx = sy = gga.nominal_std
            sz = 2.0 * gga.nominal_std
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        big = 1e6
        msg.position_covariance = [min(sx * sx, big), 0.0, 0.0,
                                   0.0, min(sy * sy, big), 0.0,
                                   0.0, 0.0, min(sz * sz, big)]
        self.pub_fix.publish(msg)
        self.pub_status.publish(String(
            data=f'{gga.quality_name} sats={gga.satellites} age={gga.age} bad={self.bad_lines}'))

    def _publish_heading(self, hdg: p.Heading) -> None:
        if not hdg.fixed or hdg.baseline_m < self.min_baseline:
            return  # only an RTK-fixed baseline gives a trustworthy heading
        yaw = p.heading_to_enu_yaw(hdg.heading_deg, self.mount_offset)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.heading_frame_id
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)
        std = math.radians(max(hdg.heading_std_deg, 0.1))
        msg.orientation_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, std * std]
        msg.angular_velocity_covariance[0] = -1.0       # not provided
        msg.linear_acceleration_covariance[0] = -1.0    # not provided
        self.pub_heading.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Um982Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
