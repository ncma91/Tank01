"""Safety supervisor node: gates every motion and blade command.

    /cmd_vel_mux  ──▶ [evaluate(safety_logic)] ──▶ /cmd_vel_safe
    /mower/blade_request ─────────────────────▶ /blade/cmd

It also emits /safety/heartbeat, which base_node forwards to the Teensy watchdog; if
this node dies the vehicle stops within ~200 ms.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String, UInt32

from tank01_safety.geo import Datum, datum_from_geojson, features_by_role, load_geojson
from tank01_safety.geofence import Geofence
from tank01_safety.safety_logic import Inputs, Level, Limits, apply_to_twist, evaluate


def quat_to_rpy(q) -> tuple[float, float, float]:
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


class SafetySupervisor(Node):

    def __init__(self) -> None:
        super().__init__('safety_supervisor')
        defaults = Limits()
        self.declare_parameters('', [
            ('rate', 50.0),
            ('cmd_timeout', 0.5),
            ('require_perception', True),
            ('geofence_file', ''),
            ('datum', [0.0, 0.0, 0.0]),
        ] + [(f'limits.{k}', v) for k, v in vars(defaults).items()])
        g = self.get_parameter
        self.limits = Limits(**{k: g(f'limits.{k}').value for k in vars(defaults)})
        self.cmd_timeout = g('cmd_timeout').value
        self.fence = self._load_fence(g('geofence_file').value, g('datum').value)

        required = ['imu', 'odom', 'base', 'localization']
        if g('require_perception').value:
            required.append('perception')
        self.inputs = Inputs(now=time.monotonic(), required=tuple(required))
        self.cmd = Twist()
        self.cmd_stamp = 0.0
        self.blade_request = 0.0
        self.last_summary = ''
        self.hb = 0

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        sensor = qos_profile_sensor_data
        self.create_subscription(Twist, 'cmd_vel_mux', self._on_cmd, reliable)
        self.create_subscription(Imu, 'imu/data', self._on_imu, sensor)
        self.create_subscription(Odometry, 'odom/wheel', self._seen('odom'), 10)
        self.create_subscription(Bool, 'base/estop', self._on_estop, reliable)
        self.create_subscription(Odometry, 'odometry/global', self._on_localization, 10)
        self.create_subscription(Float32, 'perception/nearest_person', self._on_person, 10)
        self.create_subscription(Float32, 'mower/blade_request', self._on_blade_request,
                                 reliable)
        self.create_subscription(Bool, 'mission/transit', self._on_transit, reliable)
        self.create_subscription(Bool, 'pause', self._on_pause, reliable)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_safe', reliable)
        self.pub_blade = self.create_publisher(Float32, 'blade/cmd', reliable)
        self.pub_state = self.create_publisher(String, 'safety/state', reliable)
        self.pub_hb = self.create_publisher(UInt32, 'safety/heartbeat', reliable)
        self.create_timer(1.0 / g('rate').value, self._tick)
        self.get_logger().info(f'safety supervisor up, required inputs: {required}, '
                               f'geofence: {"yes" if self.fence else "NO"}')

    def _load_fence(self, path: str, datum_param) -> Geofence | None:
        if not path:
            self.get_logger().warn('no geofence_file configured - geofence disabled')
            return None
        data = load_geojson(path)
        datum = (Datum(*datum_param) if any(datum_param) else datum_from_geojson(data))
        outer = features_by_role(data, 'geofence') or features_by_role(data, 'field')
        if not outer:
            raise RuntimeError(f'{path}: no geofence/field polygon')
        poly = datum.ring_to_enu(outer[0]['geometry']['coordinates'][0])
        keepouts = [datum.ring_to_enu(f['geometry']['coordinates'][0])
                    for f in features_by_role(data, 'keepout')
                    if f['geometry']['type'] == 'Polygon']
        return Geofence(poly, keepouts)

    # -------------------------------------------------------------- inputs
    def _seen(self, name: str):
        def cb(_msg) -> None:
            self.inputs.last_seen[name] = time.monotonic()
        return cb

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = msg
        self.cmd_stamp = time.monotonic()

    def _on_imu(self, msg: Imu) -> None:
        self.inputs.roll, self.inputs.pitch, _ = quat_to_rpy(msg.orientation)
        self.inputs.last_seen['imu'] = time.monotonic()

    def _on_estop(self, msg: Bool) -> None:
        self.inputs.estop_hw = msg.data
        self.inputs.last_seen['base'] = time.monotonic()

    def _on_localization(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.inputs.x, self.inputs.y = p.position.x, p.position.y
        self.inputs.yaw = quat_to_rpy(p.orientation)[2]
        cov = msg.pose.covariance
        self.inputs.pos_std = math.sqrt(max(cov[0], cov[7], 0.0))
        self.inputs.last_seen['localization'] = time.monotonic()

    def _on_person(self, msg: Float32) -> None:
        self.inputs.nearest_person = msg.data if msg.data > 0.0 else math.inf
        self.inputs.last_seen['perception'] = time.monotonic()

    def _on_blade_request(self, msg: Float32) -> None:
        self.blade_request = max(0.0, msg.data)
        self.inputs.mowing_requested = self.blade_request > 0.0

    def _on_transit(self, msg: Bool) -> None:
        self.inputs.transit = msg.data

    def _on_pause(self, msg: Bool) -> None:
        self.inputs.paused_by_operator = msg.data

    # ---------------------------------------------------------------- tick
    def _tick(self) -> None:
        now = time.monotonic()
        self.inputs.now = now
        decision = evaluate(self.inputs, self.limits, self.fence)

        v = w = 0.0
        if now - self.cmd_stamp < self.cmd_timeout:
            v, w = apply_to_twist(self.cmd.linear.x, self.cmd.angular.z, decision)
        out = Twist()
        out.linear.x, out.angular.z = v, w
        self.pub_cmd.publish(out)

        blade = self.blade_request if decision.blade_allowed else 0.0
        self.pub_blade.publish(Float32(data=float(blade)))

        summary = decision.summary()
        self.pub_state.publish(String(data=summary))
        if summary != self.last_summary:
            log = self.get_logger().warn if decision.level >= Level.PAUSE else \
                self.get_logger().info
            log(summary)
            self.last_summary = summary

        self.hb += 1
        self.pub_hb.publish(UInt32(data=self.hb & 0xFFFFFFFF))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
