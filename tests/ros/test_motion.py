"""Actuator endpoints validate, replace and stop; an arm's move action reaches goals that world waits await."""
import json
import math
import pytest
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from custom_interfaces.action import Json
from custom_interfaces.srv import Wait
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, MultiDOFJointState
from std_msgs.msg import Float64, Header
from std_srvs.srv import Trigger
from geometry_msgs.msg import Quaternion, Transform, Twist, Vector3
from trajectory_msgs.msg import (JointTrajectory, JointTrajectoryPoint, MultiDOFJointTrajectory,
                                 MultiDOFJointTrajectoryPoint)
from kinex_sim_runtime.devices import STREAM_QOS
from unique_identifier_msgs.msg import UUID
from tests.ros.support import Graph, ended, execute


def trajectory(positions, names=('slide', 'rotate'), **point):
    return JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=positions, time_from_start=Duration(sec=1), **point)])


def endpoints(graph, device, command, message):
    states = []
    graph.node.create_subscription(JointState, f'{device}/joint_state', states.append, STREAM_QOS)
    publisher = graph.node.create_publisher(message, f'{device}/{command}', 10)
    stop = graph.node.create_client(Trigger, f'{device}/stop')
    graph.until(lambda: stop.service_is_ready() and publisher.get_subscription_count() and states)
    return states, publisher, stop


def send(graph, publisher, message):
    # Commands are admitted asynchronously; let the graph receive one before waiting.
    publisher.publish(message)
    graph.spin(.2)


def test_arm_target_validates_replaces_and_stops(tmp_path):
    graph = Graph(tmp_path)
    try:
        states, publisher, stop = endpoints(graph, 'axes', 'joint_target', JointTrajectory)
        position = lambda: [dict(zip(states[-1].name, states[-1].position))[name] for name in ('slide', 'rotate')]
        send(graph, publisher, trajectory([.2, .1]))
        assert graph.wait(1.5).success
        graph.until(lambda: position() == pytest.approx([.2, .1]))
        # Invalid replacements are refused and keep the active reference; the next wait's reply names them.
        for invalid in (trajectory([math.nan, .2]), trajectory([.2, .1], names=('slide',)),
                        trajectory([.2, .1], velocities=[0., 0.]), trajectory([9., .1]),
                        JointTrajectory(joint_names=['slide', 'rotate'])):
            send(graph, publisher, invalid)
        result = graph.wait(.5)  # ends at once: the refusals await this reply
        assert not result.success and result.waited_s == 0.
        assert result.message.startswith('ended early by a refused command; refused since the previous wait reply: ')
        for reason in ('axes target has non-finite values', 'trajectory must name each of',
                       'only position trajectories are supported', 'slide=9.0000 outside',
                       'trajectory needs one time per point and at least one point'):
            assert f'/axes/joint_target: {reason}' in result.message
        graph.spin(.1)
        assert position() == pytest.approx([.2, .1])
        send(graph, publisher, trajectory([.3, .2]))
        assert graph.wait(.3).success
        assert graph.call(stop, Trigger.Request()).success
        assert graph.wait(1.5).success
        graph.spin(.1)
        held = position()
        assert .2 < held[0] < .3 and .1 < held[1] < .2
    finally:
        graph.close()


def poses(*targets, frame='reach/base', tip='tip', **point):
    """A tip trajectory of (x, y, quaternion xyzw) targets one second apart."""
    return MultiDOFJointTrajectory(header=Header(frame_id=frame), joint_names=[tip], points=[
        MultiDOFJointTrajectoryPoint(transforms=[Transform(translation=Vector3(x=x, y=y),
                                                           rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))],
                                     time_from_start=Duration(sec=i + 1), **point)
        for i, (x, y, q) in enumerate(targets)])


def test_pose_target_moves_the_tip_validates_and_stops(tmp_path):
    graph = Graph(tmp_path, backend='pose_toy')
    try:
        palms = []
        graph.node.create_subscription(MultiDOFJointState, 'reach/pose_state', palms.append, STREAM_QOS)
        states, publisher, stop = endpoints(graph, 'reach', 'pose_target', MultiDOFJointTrajectory)
        position = lambda: list(states[-1].position)
        assert graph.wait(.3).success
        graph.spin(.1)
        assert position() == pytest.approx([.5, -.5])  # uncommanded, the simulator holds the arm
        # pose_state reports the measured tip in the device base, stamped with joint_state.
        palm = palms[-1]
        assert (palm.header.frame_id, list(palm.joint_names), palm.header.stamp) == (
            'reach/base', ['tip'], states[-1].header.stamp)
        assert (palm.transforms[0].translation.x, palm.transforms[0].translation.y) == (.5, -.5)
        upright = (0., 0., 0., 1.)
        send(graph, publisher, poses((1., 0., upright), (1.5, .5, (0., 0., 0., 2.))))
        assert graph.wait(1.).success
        graph.spin(.1)
        assert position() == pytest.approx([1., 0.])  # interpolated from the measured tip
        for invalid in (poses((0., 0., upright), frame='world'), poses((0., 0., upright), tip='palm'),
                        poses((0., 0., (0., 0., 0., 0.))), poses((math.nan, 0., upright)),
                        poses((0., 0., upright), velocities=[Twist()])):
            send(graph, publisher, invalid)
        result = graph.wait(1.)
        assert not result.success and result.message.startswith('ended early by a refused command; ')
        for reason in ('pose targets must be expressed in reach/base', 'pose targets must name the tip link tip once',
                       'reach tip pose needs a unit quaternion', 'reach tip pose has non-finite values',
                       'only pose trajectories are supported'):
            assert f'/reach/pose_target: {reason}' in result.message
        assert graph.wait(1.).success
        graph.spin(.1)
        assert position() == pytest.approx([1.5, .5])  # the active trajectory went on
        send(graph, publisher, poses((-1., -1., upright)))
        assert graph.wait(.5).success
        assert graph.call(stop, Trigger.Request()).success
        graph.spin(.1)
        held = position()
        assert graph.wait(1.).success
        graph.spin(.1)
        assert position() == pytest.approx(held) and -1. < held[0] < 1.5  # stop holds the measured tip
    finally:
        graph.close()


def goal(x, y):
    """A move goal to the tip position (x, y)."""
    return Json.Goal(request=json.dumps({'x': x, 'y': y}))


def arms(graph):
    clients = [ActionClient(graph.node, Json, f'{arm}/move') for arm in ('left', 'right')]
    graph.until(lambda: all(client.server_is_ready() for client in clients))
    return clients


def test_a_backends_action_runs_goals_and_a_wait_ends_with_its_goals(tmp_path):
    # The pair toy's own action: tips at (0.5, -0.5) step 0.1 m a 0.1 s step to a goal's {x, y}; a wall at x = 1.2.
    graph = Graph(tmp_path, backend='pair_pose_toy')
    try:
        left, right = arms(graph)
        samples = []
        near = execute(graph, left, goal(.7, -.5), samples.append)
        far = execute(graph, right, goal(1., -.5))
        assert graph.wait(0.).time_s == 0.  # goals advance no time
        first = graph.wait(None, goals=[near, far], first=True)  # wait_any: until the first of them finishes
        assert first.success and first.finished == [True, False] and first.waited_s == pytest.approx(.2)
        both = graph.wait(None, goals=[near, far])  # wait_all: until the last does
        assert both.success and both.finished == [True, True] and both.waited_s == pytest.approx(.3)
        assert ended(graph, near) == ended(graph, far) == (GoalStatus.STATUS_SUCCEEDED, True, 'reached')
        result = far.get_result_async()
        graph.until(result.done)
        assert json.loads(result.result().result.response) == {'x': pytest.approx(1.)}  # the result's own fields
        graph.spin(.1)
        assert [json.loads(sample.feedback.feedback)['x'] for sample in samples] == pytest.approx([.6, .7])
        # A wait without goals runs to its time, whatever goals do.
        back = execute(graph, left, goal(.5, -.5))
        assert graph.wait(1.).waited_s == pytest.approx(1.) and ended(graph, back)[0] == GoalStatus.STATUS_SUCCEEDED
        # A goal the backend ends itself, here stalled at the wall, is aborted with the backend's result.
        blocked = execute(graph, right, goal(1.5, -.5))
        assert graph.wait(None, goals=[blocked]).waited_s == pytest.approx(.3)
        assert ended(graph, blocked) == (GoalStatus.STATUS_ABORTED, False, 'stalled')
        # A goal still executing at the wait's timeout goes on; a client that gives up on it cancels it.
        slow = execute(graph, left, goal(1.1, -.5))
        cut = graph.wait(None, goals=[slow], limit=.2)
        assert cut.success and cut.timed_out and cut.finished == [False] and cut.waited_s == pytest.approx(.2)
        graph.until(slow.cancel_goal_async().done)
        assert ended(graph, slow) == (GoalStatus.STATUS_CANCELED, False, 'canceled')
        unknown = graph.call(graph.wait_client, Wait.Request(goals=[UUID(uuid=[7] * 16)]))
        assert not unknown.success and unknown.message == 'goals must be goals this world accepted'
        assert graph.wait(0.).time_s == pytest.approx(2.)
    finally:
        graph.close()


def test_a_goal_ends_when_cancelled_replaced_or_stopped_and_shares_its_arm_with_joint_targets(tmp_path):
    graph = Graph(tmp_path, backend='pair_pose_toy')
    try:
        left, _ = arms(graph)
        states, joints, stop = endpoints(graph, 'left', 'joint_target', JointTrajectory)
        cancelled = execute(graph, left, goal(1., -.5))
        assert graph.wait(.2).success
        cancel = cancelled.cancel_goal_async()
        graph.until(cancel.done)
        assert ended(graph, cancelled) == (GoalStatus.STATUS_CANCELED, False, 'canceled')
        assert graph.wait(0., goals=[cancelled]).waited_s == 0.  # finished: nothing to wait for
        first = execute(graph, left, goal(1., -.5))
        second = execute(graph, left, goal(.7, 0.))
        assert ended(graph, first)[2] == 'replaced by a newer goal'
        send(graph, joints, trajectory([.5, -.5], names=('x', 'y')))  # a joint target takes the arm from the goal
        assert ended(graph, second)[2] == 'replaced by a command'
        assert graph.wait(.5).success
        graph.spin(.1)
        assert list(states[-1].position) == pytest.approx([.6, -.45])  # from the goal's command, (0.7, -0.4)
        third = execute(graph, left, goal(.9, -.45))  # and a goal takes it from the trajectory
        assert graph.wait(.1).success
        assert graph.call(stop, Trigger.Request()).success and ended(graph, third)[2] == 'stopped'
        assert graph.wait(1.).success  # the stop held the joints
        graph.spin(.1)
        assert list(states[-1].position) == pytest.approx([.7, -.45])
        for invalid, reason in ((Json.Goal(request='{}'), 'a goal needs a tip pose'),
                                (goal(.7, 1.9), 'the tip pose is beyond the 1.5 m reach')):
            refused = execute(graph, left, invalid)
            graph.wait(None, goals=[refused])
            assert ended(graph, refused) == (GoalStatus.STATUS_ABORTED, False, reason)
        malformed = left.send_goal_async(Json.Goal(request='[1]'))  # a goal is a JSON object
        graph.until(malformed.done)
        assert not malformed.result().accepted
    finally:
        graph.close()


def test_a_realtime_goal_ends_without_a_wait(tmp_path):
    graph = Graph(tmp_path, mode='realtime', backend='pair_pose_toy')
    try:
        left, _ = arms(graph)
        assert ended(graph, execute(graph, left, goal(.8, -.5)))[:2] == (GoalStatus.STATUS_SUCCEEDED, True)
    finally:
        graph.close()


def test_gripper_target_persists_and_stop_keeps_it(tmp_path):
    graph = Graph(tmp_path)
    try:
        states, publisher, stop = endpoints(graph, 'finger', 'target', Float64)
        send(graph, publisher, Float64(data=-1.))
        assert graph.wait(.1).success
        graph.until(lambda: states[-1].position[0] == -1.)
        assert graph.wait(2.).success
        graph.spin(.1)
        assert states[-1].position[0] == -1.
        for value in (math.nan, math.inf, -2., 2.):
            send(graph, publisher, Float64(data=value))
        result = graph.wait(.1)
        assert not result.success and '/finger/target: finger target has non-finite values (2 commands)' in result.message
        assert graph.wait(0.).message == 'ok'  # named once
        graph.spin(.1)
        assert states[-1].position[0] == -1.
        stopped = graph.call(stop, Trigger.Request())
        assert stopped.success and stopped.message == 'keeping the grip target'
        assert graph.wait(.5).success
        graph.spin(.1)
        assert states[-1].position[0] == -1.  # stop keeps the grip target
    finally:
        graph.close()


@pytest.mark.parametrize('mode', ['stepped', 'realtime'])
def test_terminal_episode_rejects_commands_and_stays_stoppable(tmp_path, mode):
    graph = Graph(tmp_path, mode=mode, max_steps=10)
    try:
        states, publisher, stop = endpoints(graph, 'finger', 'target', Float64)
        # Realtime may already have terminated while the clients discovered each other.
        result = graph.wait(3.)
        assert not result.success
        assert 'episode' in result.message or 'terminal' in result.message
        send(graph, publisher, Float64(data=1.))
        graph.spin(.3)
        assert graph.owner.poll() is None
        assert states[-1].position[0] == 0.
        assert graph.call(stop, Trigger.Request()).success
    finally:
        graph.close()
