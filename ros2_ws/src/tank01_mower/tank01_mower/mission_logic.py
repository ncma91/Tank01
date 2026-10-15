"""Mission state machine (pure Python, no ROS).

The ROS node feeds events in (goal reached/failed, safety level, battery, operator
commands) and executes the returned :class:`Leg` with Nav2.
See docs/08_maehplanung_missionen.md, section 3.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path

from tank01_mower.coverage import CoveragePlan, Pose


class State(str, Enum):
    IDLE = 'IDLE'
    TRANSIT = 'TRANSIT'
    MOWING = 'MOWING'
    PAUSED = 'PAUSED'
    RETURNING = 'RETURNING'
    DONE = 'DONE'
    FAULT = 'FAULT'


ACTIVE = (State.TRANSIT, State.MOWING, State.RETURNING)
BLOCKING_SAFETY = ('PAUSE', 'STOP', 'ESTOP')


@dataclass
class Leg:
    kind: str                 # 'transit' | 'reposition' | 'swath' | 'headland' | 'return'
    poses: list[Pose]
    blade: bool

    @property
    def transit(self) -> bool:
        return not self.blade


@dataclass
class Progress:
    segment: int = 0
    pose: int = 0

    def save(self, path: str | Path) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(asdict(self)))

    @classmethod
    def load(cls, path: str | Path) -> 'Progress':
        try:
            return cls(**json.loads(Path(path).read_text()))
        except (OSError, ValueError, TypeError):
            return cls()


class Mission:

    def __init__(self, plan: CoveragePlan, approach: list[Pose] | None = None,
                 home: list[Pose] | None = None, reposition_threshold: float = 3.0,
                 max_retries: int = 2, progress: Progress | None = None):
        self.plan = plan
        self.approach = approach or []
        self.home = home or []
        self.reposition_threshold = reposition_threshold
        self.max_retries = max_retries
        self.progress = progress or Progress()
        self.state = State.IDLE
        self.reason = ''
        self._resume_state: State | None = None
        self._leg: Leg | None = None
        self._leg_offset = 0
        self._retries = 0
        self._repositioning = False

    # ------------------------------------------------------------- helpers
    @property
    def segments(self):
        return self.plan.segments

    def fraction_done(self) -> float:
        total = sum(len(s.poses) for s in self.segments) or 1
        done = sum(len(s.poses) for s in self.segments[:self.progress.segment])
        return min(1.0, (done + self.progress.pose) / total)

    def status(self) -> dict:
        return {'state': self.state.value, 'reason': self.reason,
                'segment': self.progress.segment, 'segments': len(self.segments),
                'progress': round(self.fraction_done(), 4),
                'leg': self._leg.kind if self._leg else None}

    def _set(self, state: State, reason: str = '') -> None:
        self.state = state
        self.reason = reason

    # ------------------------------------------------------------- legs
    def current_leg(self, robot: tuple[float, float] | None = None) -> Leg | None:
        if self.state == State.TRANSIT:
            self._leg, self._leg_offset = Leg('transit', self.approach, False), 0
        elif self.state == State.RETURNING:
            self._leg, self._leg_offset = Leg('return', self.home, False), 0
        elif self.state == State.MOWING:
            seg = self.segments[self.progress.segment]
            start = min(self.progress.pose, len(seg.poses) - 1)
            first = seg.poses[start]
            far = robot is not None and math.dist(robot, first[:2]) > self.reposition_threshold
            if far and not self._repositioning:
                # Drive to the segment start with the blade off before mowing on.
                self._repositioning = True
                self._leg, self._leg_offset = Leg('reposition', [first], False), start
            else:
                self._repositioning = False
                self._leg = Leg(seg.kind, seg.poses[start:], seg.blade)
                self._leg_offset = start
        else:
            self._leg = None
        return self._leg

    # ------------------------------------------------------------- events
    def start(self, robot: tuple[float, float] | None = None) -> Leg | None:
        if not self.segments:
            self._set(State.FAULT, 'empty coverage plan')
            return None
        if self.progress.segment >= len(self.segments):
            self.progress = Progress()
        self._set(State.TRANSIT if self.approach and self.progress == Progress()
                  else State.MOWING)
        return self.current_leg(robot)

    def on_feedback(self, poses_remaining: int) -> None:
        if self.state == State.MOWING and self._leg and not self._repositioning:
            self.progress.pose = self._leg_offset + max(0, len(self._leg.poses) - poses_remaining)

    def on_leg_succeeded(self, robot: tuple[float, float] | None = None) -> Leg | None:
        self._retries = 0
        if self.state == State.TRANSIT:
            self._set(State.MOWING)
        elif self.state == State.MOWING:
            if self._repositioning:
                return self.current_leg(None)   # now mow the segment itself
            self.progress = Progress(self.progress.segment + 1, 0)
            if self.progress.segment >= len(self.segments):
                if self.home:
                    self._set(State.RETURNING, 'coverage complete')
                else:
                    self._set(State.DONE, 'coverage complete')
                    return None
        elif self.state == State.RETURNING:
            self._set(State.DONE, self.reason or 'returned home')
            return None
        return self.current_leg(robot)

    def on_leg_failed(self, why: str = '', robot: tuple[float, float] | None = None
                      ) -> Leg | None:
        if self.state not in ACTIVE:
            return None
        self._retries += 1
        if self._retries > self.max_retries:
            if self.state == State.MOWING:
                # Skip the blocked segment instead of aborting the whole field.
                self._retries = 0
                self._repositioning = False
                self.progress = Progress(self.progress.segment, 0)
                return self.on_leg_succeeded(robot)
            self._set(State.FAULT, f'navigation failed: {why}')
            return None
        return self.current_leg(robot)

    def on_safety(self, level: str) -> bool:
        """Returns True if the running navigation goal must be cancelled."""
        if level in BLOCKING_SAFETY and self.state in ACTIVE:
            self._resume_state = self.state
            self._set(State.PAUSED, f'safety {level}')
            return True
        return False

    def on_safety_clear(self, robot: tuple[float, float] | None = None) -> Leg | None:
        if self.state == State.PAUSED and self.reason.startswith('safety') and \
                self._resume_state is not None:
            self._set(self._resume_state)
            self._resume_state = None
            self._repositioning = False
            return self.current_leg(robot)
        return None

    def pause(self) -> bool:
        if self.state in ACTIVE:
            self._resume_state = self.state
            self._set(State.PAUSED, 'operator')
            return True
        return False

    def resume(self, robot: tuple[float, float] | None = None) -> Leg | None:
        if self.state == State.PAUSED and self._resume_state is not None:
            self._set(self._resume_state)
            self._resume_state = None
            self._repositioning = False
            return self.current_leg(robot)
        return None

    def fail(self, reason: str) -> None:
        self._set(State.FAULT, reason)
        self._leg = None

    def abort(self) -> None:
        self._set(State.IDLE, 'aborted')
        self._leg = None

    def on_battery(self, soc: float, reserve: float) -> bool:
        """Switch to RETURNING when the state of charge drops below the reserve."""
        if soc < reserve and self.state in (State.TRANSIT, State.MOWING) and self.home:
            self._set(State.RETURNING, f'battery low ({soc:.0%})')
            self._repositioning = False
            return True
        return False


def return_reserve(distance_m: float, wh_per_m: float, capacity_wh: float,
                   margin: float = 1.3, floor: float = 0.10) -> float:
    """State of charge (0..1) needed to drive distance_m home plus a safety floor."""
    if capacity_wh <= 0:
        return 1.0
    return min(1.0, distance_m * wh_per_m * margin / capacity_wh + floor)
