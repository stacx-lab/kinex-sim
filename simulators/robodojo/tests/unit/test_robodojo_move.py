"""The devices' goals without Isaac: how a move steps in its gear and ends, how a grasp and a release end, and the
objects they read and give."""
import numpy as np
import pytest

from kinex_sim_robodojo import grip, move

UPRIGHT = (0., 0., 0., 1.)


def pose(x, y, quaternion=UPRIGHT):
    return np.array([x, y, 0., *quaternion])


def test_a_motion_steps_from_the_measured_pose_in_its_gear():
    half_turn = (0., 0., 1., 0.)
    motion = move.Motion(pose(0., 0.), pose(1., 0., half_turn), "standard")
    step = motion.reference()
    assert step[:3] == pytest.approx([.04, 0., 0.]) and 2 * np.arccos(step[6]) == pytest.approx(.132)
    assert move.Motion(pose(0., 0.), pose(1., 0.), "precision").reference() == pytest.approx(pose(.01, 0.))
    assert move.Motion(pose(0., 0.), pose(.02, 0.), "precision").reference() == pytest.approx(pose(.01, 0.))
    assert move.Motion(pose(0., 0.), pose(.005, 0.), "approach").reference() == pytest.approx(pose(.005, 0.))


def test_a_motion_ends_reached_late_or_stalled():
    arrive = move.Motion(pose(0., 0.), pose(.2, 0.), "standard")
    assert [arrive.observe(pose(x, 0.)) for x in (0., .1, .171)] == [None, None, "reached"]  # within 3 cm
    assert move.Motion(pose(0., 0.), pose(.2, 0.), "precision").observe(pose(.171, 0.)) is None  # not within 5 mm
    assert arrive.succeeded("reached") and not arrive.succeeded("stalled")
    assert arrive.result("reached") == {
        "stop_reason": "reached", "pose": [.171, 0., 0., 0., 0., 0., 1.], "position_error_m": pytest.approx(.029),
        "orientation_error_rad": 0., "steps": 2, "recovered": False,
        "message": "reached: 0.029 m and 0.000 rad from the pose after 2 steps"}
    assert arrive.feedback() == {"position_error_m": pytest.approx(.029), "orientation_error_rad": 0., "steps": 2}
    # A flange that lags is late SETTLE steps after its gear would have arrived, here 5 steps for 20 cm.
    late = move.Motion(pose(0., 0.), pose(.2, 0.), "standard")
    outcomes = [late.observe(pose(.005 * step, 0.)) for step in range(5 + move.SETTLE + 1)]
    assert outcomes[:-1] == [None] * (5 + move.SETTLE) and outcomes[-1] == "late"
    # One that gains nothing over four steps has stalled.
    blocked = move.Motion(pose(0., 0.), pose(1., 0.), "standard")
    assert [blocked.observe(pose(min(.04 * step, .08), 0.)) for step in range(6)] == [None] * 5 + ["stalled"]
    assert blocked.result("unreachable", "no IK")["message"] == "no IK: 0.920 m and 0.000 rad from the pose after 5 steps"


def test_a_motion_that_moves_away_from_its_pose_twice_running_is_diverging():
    drift = move.Motion(pose(0., 0.), pose(1., 0.), "standard")
    # Its distance grows by more than 1 cm on each of two steps running; a single slip is forgiven.
    assert [drift.observe(pose(x, 0.)) for x in (0., .04, .02, .06, .04, .02)] == [None] * 5 + ["diverging"]
    turning = move.Motion(pose(0., 0.), pose(1., 0.), "standard")  # an angle counts for a tenth of its radians
    quarter, half = (0., 0., np.sqrt(.5), np.sqrt(.5)), (0., 0., 1., 0.)
    assert [turning.observe(pose(.04 * step, 0., q)) for step, q in enumerate((UPRIGHT, quarter, half))][-1] == "diverging"


def test_a_step_that_moves_an_arm_too_far_is_unsafe():
    moved = {"left_arm": (.06, .1), "right_arm": (.06, .1)}
    assert move.unsafe(moved, {"left_arm", "right_arm"}) == {}  # arms their goals move may go 0.2 m and 0.8 rad
    assert move.unsafe(moved, {"left_arm"}) == {"right_arm": "right_arm moved 0.060 m and 0.100 rad in one step"}
    assert list(move.unsafe({"left_arm": (.1, .9), "right_arm": (0., .2)}, {"left_arm"})) == ["left_arm"]


def test_a_recovered_motion_steps_to_a_pose_near_its_goal_and_arrives_at_the_goal():
    motion = move.Motion(pose(0., 0.), pose(.2, 0.), "standard")
    assert motion.result("unreachable")["recovered"] is False
    motion.observe(pose(.1, 0.))
    motion.recover(pose(.18, .01))  # as the backend does when a step has no IK solution
    assert motion.reference() == pytest.approx(pose(*(np.array([.1, 0.]) + .04 * np.array([.08, .01]) / np.hypot(.08, .01))))
    assert motion.observe(pose(.14, .005)) is None and motion.observe(pose(.18, .01)) == "reached"  # 2.2 cm from the goal
    assert motion.result("reached")["recovered"] is True and motion.result("reached")["position_error_m"] == pytest.approx(.0224, abs=1e-4)


def test_a_move_goal_is_a_pose_and_a_gear_or_says_why_it_is_none():
    start = pose(0., 0.)
    motion = move.motion({"pose": [1., 0., 0., 0., 0., 0., 2.], "profile": "approach"}, start)
    assert motion.goal == pytest.approx(pose(1., 0.)) and motion.tolerance == pytest.approx([.02, .05])
    assert move.motion({"pose": list(pose(1., 0.))}, start).profile == "standard"
    for bad, reason in (({}, "a move goal is {pose, profile}"), ({"pose": list(start), "speed": 1}, "a move goal is"),
                        ({"pose": list(start), "profile": "fast"}, "profile is one of standard, approach, precision"),
                        ({"pose": [0.] * 7}, "goal pose needs a unit quaternion"),
                        ({"pose": [0.] * 6}, "goal pose needs 7 values")):
        with pytest.raises(ValueError, match=reason):
            move.motion(bad, start)


def test_a_grasp_closes_until_the_fingers_stop_and_a_release_opens_to_its_opening():
    span = (.185, 1.)  # the fingers stop short of the closed command, which squeezes
    grasp = grip.grip("grasp", {}, span)
    assert grasp.command == [0.] and grasp.target == .185
    held = [grasp.observe(opening) for opening in (1., .8, .6, .5, .5, .5, .5, .5, .5, .5, .5)]
    assert held == [None] * 10 + ["stalled"] and grasp.succeeded("stalled")  # stopped on what is between them
    assert grasp.result("stalled") == {"stop_reason": "stalled", "opening": .5, "steps": 10,
                                       "message": "stalled: opening 0.500 after 10 steps"}
    empty = grip.grip("grasp", {}, span)
    assert [empty.observe(opening) for opening in (1., .6, .2)] == [None, None, "reached"]
    release = grip.grip("release", {}, span)
    assert release.command == [1.] and release.observe(.99) == "reached" and release.feedback() == {"opening": .99, "steps": 0}
    assert not release.succeeded("stalled") and not release.succeeded("late")
    part = grip.grip("release", {"opening": .1}, span)  # below what the fingers reach: judged at the stop
    assert part.command == [.1] and part.target == .185
    slow = grip.grip("release", {}, span)
    assert [slow.observe(.02 * step) for step in range(grip.MAX_STEPS + 1)][-1] == "late"
    for name, bad, reason in (("grasp", {"opening": 0.}, "a grasp goal has no fields"),
                              ("release", {"opening": 2.}, "a release goal is {opening}"),
                              ("release", {"width": .1}, "a release goal is {opening}")):
        with pytest.raises(ValueError, match=reason):
            grip.grip(name, bad, span)
