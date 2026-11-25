"""A backend's action goals in the world: their commands, their ends, and waits that end with them. No ROS.

The pair toy's move steps an arm's tip 0.1 m a 0.1 s step from (0.5, -0.5) to the goal's last pose; a wall stops
the tips at x = 1.2.
"""
import pytest

from kinex_sim.runtime.world import WorldRuntime
from tests.toy_backend import PairPoseToy

REACHED = ("succeeded", {"error_code": 0, "error_string": "reached"})


def world(**kwargs):
    sim = PairPoseToy(**kwargs)
    return WorldRuntime(sim, "stepped", lambda: None), sim


def aborted(reason):
    return "aborted", {"error_code": -1, "error_string": reason}


def start(runtime, arm, x, y):
    """Start an arm's goal; its end and the x its feedback says each step commands."""
    end, commanded = [], []
    runtime.start(f"{arm}/move", {"multi_dof_trajectory": {"points": [{"transforms": [{"translation": {"x": x, "y": y}}]}]}},
                  lambda *outcome: end.append(outcome),
                  lambda fields: commanded.append(fields["multi_dof_desired"]["transforms"][0]["translation"]["x"]))
    return end, commanded


def wait(runtime, *ends, seconds=(), first=False, timeout=0.):
    """A world wait run to its reply: for seconds of world time and for the goals with these ends, all of them or
    with first the first, up to timeout. Its result, whose done says which entries were over, the seconds then
    the goals, and how long it waited."""
    replies, start = [], runtime.time_s
    runtime.wait(first, list(seconds), [lambda end=end: bool(end) for end in ends], timeout, replies.append)
    while not replies:
        runtime.tick()
    return replies[0], replies[0].time_s - start


def test_a_goal_runs_in_the_backend_and_the_wait_for_it_ends_with_it():
    runtime, sim = world()
    end, commanded = start(runtime, "left", 1., -.5)
    result, waited = wait(runtime, end)
    assert result.success and result.done == (True,) and waited == pytest.approx(.5)
    assert end == [REACHED] and commanded == pytest.approx([.6, .7, .8, .9, 1.]) and not runtime.goals
    assert sim.position[:2] == pytest.approx([1., -.5])
    assert runtime.commands.action(9.)[:2] == pytest.approx([1., -.5])  # the arm keeps the goal's last command
    again, _ = start(runtime, "left", 1., -.5)  # already there: no step
    assert again == [REACHED] and runtime.steps == 5
    refused, _ = start(runtime, "left", 1., 1.9)  # a goal the backend cannot finish ends where it gets
    assert wait(runtime, refused)[1] == pytest.approx(1.6)
    assert refused == [aborted("the tip pose is beyond the 1.5 m reach")] and sim.position[1] == pytest.approx(1.1)


def test_a_wait_ends_at_its_timeout_and_leaves_its_goals_running():
    runtime, sim = world()
    left, _ = start(runtime, "left", .7, -.5)
    right, _ = start(runtime, "right", 1., -.5)
    result, waited = wait(runtime, left, right, timeout=.3)
    assert result.success and result.timed_out and result.done == (True, False) and waited == pytest.approx(.3)
    assert left == [REACHED] and not right
    runtime.end("right/move", "too slow")  # as a client cancels a goal it gives up on
    assert right == [aborted("too slow")]
    result, _ = wait(runtime, seconds=[.5], timeout=1.)
    assert not result.timed_out and sim.position[2] == pytest.approx(.8)  # it stays where the backend left it


def test_a_goal_the_backend_ends_leaves_its_group_where_the_backend_puts_it():
    runtime, sim = world()
    end, _ = start(runtime, "left", 1.5, -.5)  # the wall stops the tip at x = 1.2
    _, waited = wait(runtime, end)
    assert end == [aborted("stalled")] and waited == pytest.approx(.8)
    assert runtime.commands.action(9.)[:2] == pytest.approx([1.2, -.5])


def test_a_goal_and_a_groups_targets_replace_each_other():
    runtime, sim = world()
    goal, _ = start(runtime, "left", 1., -.5)
    wait(runtime, seconds=[.2])
    runtime.commands.set_trajectory("left", runtime.time_s, [.2], [[.5, -.5]])  # as a joint_target does
    runtime.release("left", "replaced by a command")
    assert goal == [aborted("replaced by a command")]
    wait(runtime, seconds=[.1])
    assert sim.position[:2] == pytest.approx([.65, -.5])  # from the goal's command, 0.8: no jump back
    wait(runtime, seconds=[.1])
    runtime.commands.set_trajectory("left", runtime.time_s, [1.], [[1.1, -.5]])
    wait(runtime, seconds=[.2])
    goal, _ = start(runtime, "left", .5, 0.)  # a goal takes the group from its trajectory
    wait(runtime, goal)
    assert goal == [REACHED] and sim.position[:2] == pytest.approx([.5, 0.])
    wait(runtime, seconds=[1.])
    assert sim.position[:2] == pytest.approx([.5, 0.])  # the trajectory is gone


def test_a_newer_goal_a_stop_and_the_episodes_end_end_the_active_goal():
    runtime, sim = world(max_steps=3)
    first, _ = start(runtime, "left", 1., -.5)
    second, _ = start(runtime, "left", .5, 0.)
    assert first == [aborted("replaced by a newer goal")]
    wait(runtime, seconds=[.1])
    runtime.hold("left")
    assert second == [aborted("stopped")] and sim.position[:2] == pytest.approx([.5, -.4])
    result, _ = wait(runtime, second)  # finished already: no step
    assert result.done == (True,) and runtime.steps == 1
    last, _ = start(runtime, "right", 1., -.5)
    result, _ = wait(runtime, last)
    assert result.done == (True,) and runtime.steps == 3  # the end finished the goal the wait was for
    assert result.message == "ok; the episode ended because the time limit was reached"
    assert last == [aborted("the episode ended because the time limit was reached")]
    late, _ = start(runtime, "right", 1., -.5)
    assert late == [aborted("the episode ended because the time limit was reached")]
