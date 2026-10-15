from tank01_mower.coverage import CoverageParams, plan_coverage
from tank01_mower.field import Field
from tank01_mower.mission_logic import Mission, Progress, State, return_reserve
from tank01_safety.geo import Datum


def make_plan():
    fld = Field('f', Datum(47.0, 11.0), [(0, 0), (20, 0), (20, 6), (0, 6)])
    return plan_coverage(fld, CoverageParams(headland_passes=1), start=(0, 0))


def test_full_run_with_transit_and_return():
    plan = make_plan()
    m = Mission(plan, approach=[(-50, 0, 0), (0, 0, 0)], home=[(-50, 0, 3.14)])
    leg = m.start()
    assert m.state == State.TRANSIT and not leg.blade
    leg = m.on_leg_succeeded(robot=(0, 0))
    assert m.state == State.MOWING
    n = 0
    while m.state == State.MOWING:
        assert leg is not None
        robot = leg.poses[-1][:2]
        leg = m.on_leg_succeeded(robot=robot)
        n += 1
    assert n >= len(plan.segments)
    assert m.state == State.RETURNING and leg.kind == 'return'
    assert m.on_leg_succeeded() is None
    assert m.state == State.DONE


def test_reposition_with_blade_off():
    m = Mission(make_plan())
    leg = m.start(robot=(100, 100))
    assert leg.kind == 'reposition' and not leg.blade
    leg = m.on_leg_succeeded(robot=leg.poses[0][:2])
    assert leg.blade and leg.kind in ('swath', 'headland')


def test_safety_pause_and_resume_from_progress():
    m = Mission(make_plan())
    leg = m.start(robot=(0, 0))
    total = len(leg.poses)
    m.on_feedback(poses_remaining=total - 3)
    assert m.progress.pose == 3
    assert m.on_safety('STOP') is True
    assert m.state == State.PAUSED
    assert m.on_safety('STOP') is False      # already paused
    leg2 = m.on_safety_clear(robot=leg.poses[3][:2])
    assert m.state == State.MOWING
    assert leg2.poses[0] == leg.poses[3]


def test_operator_pause_not_cleared_by_safety():
    m = Mission(make_plan())
    m.start(robot=(0, 0))
    assert m.pause()
    assert m.on_safety_clear() is None
    assert m.resume(robot=(0, 0)) is not None


def test_failing_segment_is_skipped_after_retries():
    m = Mission(make_plan(), max_retries=1)
    m.start(robot=(0, 0))
    seg0 = m.progress.segment
    m.on_leg_failed('blocked')
    m.on_leg_failed('blocked')
    assert m.progress.segment == seg0 + 1
    assert m.state == State.MOWING


def test_transit_failure_faults():
    m = Mission(make_plan(), approach=[(5, 5, 0)], max_retries=0)
    m.start()
    assert m.on_leg_failed('no path') is None
    assert m.state == State.FAULT


def test_battery_return():
    m = Mission(make_plan(), home=[(0, 0, 0)])
    m.start(robot=(0, 0))
    assert not m.on_battery(0.5, 0.2)
    assert m.on_battery(0.15, 0.2)
    assert m.state == State.RETURNING


def test_progress_persistence(tmp_path):
    f = tmp_path / 'state.json'
    Progress(4, 7).save(f)
    assert Progress.load(f) == Progress(4, 7)
    assert Progress.load(tmp_path / 'missing.json') == Progress()


def test_return_reserve():
    assert return_reserve(0, 0.35, 9200) == 0.10
    assert 0.10 < return_reserve(5000, 0.35, 9200) < 0.4
