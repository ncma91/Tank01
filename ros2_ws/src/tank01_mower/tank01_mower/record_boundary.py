"""Record a field boundary by driving along it (RTK fix required).

    ros2 run tank01_mower record_boundary --ros-args -p output:=feld.geojson -p name:="Wiese Nord"

Points are stored when the vehicle has moved at least ``min_spacing`` metres. Press
Ctrl-C to finish; the polygon is closed and written as a GeoJSON ``field`` feature.
"""

from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

from tank01_safety.geo import Datum


class BoundaryRecorder(Node):

    def __init__(self) -> None:
        super().__init__('record_boundary')
        self.declare_parameters('', [
            ('output', 'boundary.geojson'),
            ('name', 'Neues Feld'),
            ('role', 'field'),
            ('min_spacing', 0.5),
            ('max_std', 0.05),
        ])
        self.points: list[tuple[float, float, float]] = []
        self.datum: Datum | None = None
        self.last_enu: tuple[float, float] | None = None
        self.create_subscription(NavSatFix, 'gnss/fix', self._on_fix, qos_profile_sensor_data)
        self.get_logger().info('recording boundary - drive along the edge, Ctrl-C to finish')

    def _on_fix(self, msg: NavSatFix) -> None:
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        std = math.sqrt(max(msg.position_covariance[0], msg.position_covariance[4], 0.0))
        if std > self.get_parameter('max_std').value:
            self.get_logger().warn(f'position std {std:.2f} m too large - skipping',
                                   throttle_duration_sec=5.0)
            return
        if self.datum is None:
            self.datum = Datum(msg.latitude, msg.longitude, msg.altitude)
        e, n, _ = self.datum.to_enu(msg.latitude, msg.longitude, msg.altitude)
        if self.last_enu is None or \
                math.dist(self.last_enu, (e, n)) >= self.get_parameter('min_spacing').value:
            self.last_enu = (e, n)
            self.points.append((msg.longitude, msg.latitude, msg.altitude))
            if len(self.points) % 20 == 0:
                self.get_logger().info(f'{len(self.points)} points recorded')

    def write(self) -> None:
        if len(self.points) < 3:
            self.get_logger().error('fewer than 3 points - nothing written')
            return
        ring = [list(p[:2]) for p in self.points] + [list(self.points[0][:2])]
        lat0, lon0, alt0 = self.points[0][1], self.points[0][0], self.points[0][2]
        data = {
            'type': 'FeatureCollection',
            'properties': {'name': self.get_parameter('name').value,
                           'datum': [lat0, lon0, alt0]},
            'features': [{
                'type': 'Feature',
                'properties': {'role': self.get_parameter('role').value,
                               'name': self.get_parameter('name').value},
                'geometry': {'type': 'Polygon', 'coordinates': [ring]},
            }],
        }
        out = self.get_parameter('output').value
        with open(out, 'w', encoding='utf-8') as fh:
            json.dump(data, fh, indent=2)
        self.get_logger().info(f'wrote {len(self.points)} points to {out}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoundaryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
