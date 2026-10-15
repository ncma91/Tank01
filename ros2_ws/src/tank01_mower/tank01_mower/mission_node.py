"""Mission node: plans coverage for a GeoJSON field and drives it with Nav2.

Interfaces (see docs/06 and docs/08):
  action client  navigate_through_poses (nav2_msgs/NavigateThroughPoses)
  pub            /mower/blade_request, /mower/height_cmd, /mission/transit,
                 /mission/state (JSON), /mission/coverage_path
  sub            /safety/state, /battery/state, /odometry/global
  services       /mission/start, /mission/pause, /mission/resume, /mission/abort
"""

from __future__ import annotations

import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from tank01_mower.coverage import CoverageParams, Pose, densify, plan_coverage
from tank01_mower.field import load_field
from tank01_mower.mission_logic import Leg, Mission, Progress, State, return_reserve
from tank01_mower.routing import RouteGraph, path_length, resample


def poses_from_points(points, spacing: float) -> list[Pose]:
    pts = resample(points, spacing)
    out: list[Pose] = []
    for a, b in zip(pts, pts[1:]):
        seg = densify(a, b, spacing)
        out.extend(seg if not out else seg[1:])
    return out


class MissionNode(Node):

    def __init__(self) -> None:
        super().__init__('mission_node')
        self.declare_parameters('', [
            ('field_file', ''),
            ('field_name', ''),
            ('start', False),
            ('map_frame', 'map'),
            ('cut_width', 1.0),
            ('overlap', 0.10),
            ('headland_passes', 2),
            ('swath_angle_deg', -1.0),         # < 0 → auto
            ('min_swath_length', 2.0),
            ('waypoint_spacing', 2.0),
            ('route_waypoint_spacing', 20.0),
            ('mow_speed', 1.0),
            ('blade_rpm', 2400.0),
            ('cut_height', 0.07),
            ('max_poses_per_goal', 60),
            ('battery_capacity_wh', 9200.0),
            ('energy_wh_per_m', 0.35),
            ('progress_file', os.path.expanduser('~/.tank01/mission_state.json')),
            ('resume_delay', 3.0),
        ])
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.g = g
        self.frame = g('map_frame')

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_blade = self.create_publisher(Float32, 'mower/blade_request', reliable)
        self.pub_height = self.create_publisher(Float32, 'mower/height_cmd', reliable)
        self.pub_transit = self.create_publisher(Bool, 'mission/transit', reliable)
        self.pub_state = self.create_publisher(String, 'mission/state', reliable)
        self.pub_path = self.create_publisher(Path, 'mission/coverage_path', latched)

        self.create_subscription(String, 'safety/state', self._on_safety, reliable)
        self.create_subscription(BatteryState, 'battery/state', self._on_battery, 5)
        self.create_subscription(Odometry, 'odometry/global', self._on_odom, 10)

        self.create_service(Trigger, 'mission/start', self._srv_start)
        self.create_service(Trigger, 'mission/pause', self._srv_pause)
        self.create_service(Trigger, 'mission/resume', self._srv_resume)
        self.create_service(Trigger, 'mission/abort', self._srv_abort)

        self.nav = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.goal_handle = None
        self.goal_poses: list[Pose] = []
        self.leg: Leg | None = None
        self.robot: tuple[float, float] | None = None
        self.clear_since: float | None = None
        self.mission: Mission | None = None
        self.home_distance = 0.0

        self.create_timer(0.5, self._tick)
        if g('field_file'):
            self._load_mission()
            if g('start'):
                self._autostart_timer = self.create_timer(2.0, self._autostart_once)

    # ------------------------------------------------------------ planning
    def _load_mission(self) -> None:
        g = self.g
        fld = load_field(g('field_file'), name=g('field_name') or None)
        params = CoverageParams(
            cut_width=g('cut_width'), overlap=g('overlap'),
            headland_passes=g('headland_passes'),
            swath_angle_deg=None if g('swath_angle_deg') < 0 else g('swath_angle_deg'),
            min_swath_length=g('min_swath_length'), waypoint_spacing=g('waypoint_spacing'),
            mow_speed=g('mow_speed'))
        plan = plan_coverage(fld, params)
        approach = home = None
        if fld.routes and fld.depot and fld.entry:
            graph = RouteGraph(fld.routes)
            route = graph.shortest_path(fld.depot, fld.entry)
            self.home_distance = path_length(route)
            approach = poses_from_points(route, g('route_waypoint_spacing'))
            home = poses_from_points(route[::-1], g('route_waypoint_spacing'))
        elif fld.entry:
            home = [(fld.entry[0], fld.entry[1], 0.0)]
        self.mission = Mission(plan, approach, home,
                               progress=Progress.load(g('progress_file')))
        self._publish_path(plan.all_poses())
        self.get_logger().info(
            f'field "{fld.name}": {int(plan.stats["swaths"])} swaths, '
            f'{plan.stats["mow_length_m"]:.0f} m, ~{plan.stats["estimated_time_min"]:.0f} min')

    # ------------------------------------------------------------ nav2
    def _to_pose(self, p: Pose) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = p[0], p[1]
        ps.pose.orientation.z = math.sin(p[2] / 2.0)
        ps.pose.orientation.w = math.cos(p[2] / 2.0)
        return ps

    def _execute(self, leg: Leg | None) -> None:
        self.leg = leg
        self._publish_actuators()
        if leg is None or not leg.poses:
            self._publish_state()
            return
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_through_poses action server not available')
            self.mission.fail('nav2 not available')
            self.leg = None
            self._publish_actuators()
            self._publish_state()
            return
        # Long legs are split into chunks; the remainder is sent after success.
        n = self.g('max_poses_per_goal')
        self.goal_poses = leg.poses[:n]
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._to_pose(p) for p in self.goal_poses]
        fut = self.nav.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)
        self._publish_state()

    def _on_goal_response(self, fut) -> None:
        self.goal_handle = fut.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('goal rejected')
            self._execute(self.mission.on_leg_failed('rejected', self.robot))
            return
        self.goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, fb) -> None:
        remaining = fb.feedback.number_of_poses_remaining
        if self.leg is not None:
            remaining += max(0, len(self.leg.poses) - len(self.goal_poses))
        self.mission.on_feedback(remaining)

    def _on_result(self, fut) -> None:
        status = fut.result().status
        self.goal_handle = None
        m = self.mission
        if m.state not in (State.TRANSIT, State.MOWING, State.RETURNING):
            return  # cancelled because of pause/abort/safety
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.leg is not None and len(self.leg.poses) > len(self.goal_poses):
                rest = Leg(self.leg.kind, self.leg.poses[len(self.goal_poses):], self.leg.blade)
                self._execute(rest)
                return
            leg = m.on_leg_succeeded(self.robot)
            m.progress.save(self.g('progress_file'))
            self._execute(leg)
        else:
            self.get_logger().warn(f'navigation leg failed (status {status})')
            self._execute(m.on_leg_failed(f'status {status}', self.robot))

    def _cancel(self) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    # ------------------------------------------------------------ inputs
    def _on_odom(self, msg: Odometry) -> None:
        self.robot = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_safety(self, msg: String) -> None:
        if self.mission is None:
            return
        level = msg.data.split(':', 1)[0].strip()
        if self.mission.on_safety(level):
            self._cancel()
            self.leg = None
            self._publish_actuators()
            self._publish_state()
            self.clear_since = None
        elif level in ('OK', 'SLOW') and self.mission.state == State.PAUSED:
            self.clear_since = self.clear_since or time.monotonic()

    def _on_battery(self, msg: BatteryState) -> None:
        if self.mission is None or math.isnan(msg.percentage):
            return
        reserve = return_reserve(self.home_distance, self.g('energy_wh_per_m'),
                                 self.g('battery_capacity_wh'))
        if self.mission.on_battery(msg.percentage, reserve):
            self.get_logger().warn('battery low - returning home')
            self._cancel()
            self._execute(self.mission.current_leg(self.robot))

    def _tick(self) -> None:
        m = self.mission
        if m is None:
            return
        if (m.state == State.PAUSED and self.clear_since is not None
                and time.monotonic() - self.clear_since > self.g('resume_delay')):
            self.clear_since = None
            leg = m.on_safety_clear(self.robot)
            if leg is not None:
                self.get_logger().info('safety clear - resuming')
                self._execute(leg)
        self._publish_actuators()
        self._publish_state()

    def _autostart_once(self) -> None:
        self._autostart_timer.cancel()
        if self.mission is not None and self.mission.state == State.IDLE:
            self._execute(self.mission.start(self.robot))

    # ------------------------------------------------------------ outputs
    def _publish_actuators(self) -> None:
        blade = bool(self.leg and self.leg.blade and self.mission.state == State.MOWING)
        self.pub_blade.publish(Float32(data=float(self.g('blade_rpm')) if blade else 0.0))
        self.pub_height.publish(Float32(data=float(self.g('cut_height'))))
        self.pub_transit.publish(Bool(data=bool(self.leg and self.leg.transit)))

    def _publish_state(self) -> None:
        if self.mission is not None:
            self.pub_state.publish(String(data=json.dumps(self.mission.status())))

    def _publish_path(self, poses: list[Pose]) -> None:
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self._to_pose(p) for p in poses]
        self.pub_path.publish(path)

    # ------------------------------------------------------------ services
    def _srv_start(self, _req, res):
        if self.mission is None:
            res.success, res.message = False, 'no field loaded (parameter field_file)'
        elif self.mission.state not in (State.IDLE, State.DONE, State.FAULT):
            res.success, res.message = False, f'mission busy ({self.mission.state.value})'
        else:
            self._execute(self.mission.start(self.robot))
            res.success, res.message = True, 'started'
        return res

    def _srv_pause(self, _req, res):
        res.success = bool(self.mission and self.mission.pause())
        if res.success:
            self._cancel()
            self.leg = None
            self._publish_actuators()
        res.message = 'paused' if res.success else 'nothing to pause'
        return res

    def _srv_resume(self, _req, res):
        leg = self.mission.resume(self.robot) if self.mission else None
        res.success = leg is not None
        if leg is not None:
            self._execute(leg)
        res.message = 'resumed' if res.success else 'not paused'
        return res

    def _srv_abort(self, _req, res):
        if self.mission:
            self._cancel()
            self.mission.abort()
            self.leg = None
            self._publish_actuators()
        res.success, res.message = True, 'aborted'
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
