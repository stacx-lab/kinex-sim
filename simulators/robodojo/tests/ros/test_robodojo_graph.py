"""The RoboDojo graph: a Python 3.12 world node proxying Isaac Sim 5.1 in a worker."""
import json

import numpy as np
import psutil
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from custom_interfaces.action import Json
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, JointState, MultiDOFJointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kinex_sim_runtime.devices import STREAM_QOS
from tests.ros.support import Graph, ended, execute, seconds

# Isaac Sim boots, compiles shaders and loads the layout before serving.
STARTUP_S = 1200.


def workers(graph):
    """Native worker processes of this graph, identified by its leased ROS domain."""
    found = []
    for process in psutil.process_iter(["cmdline"]):
        try:
            if ("kinex_sim.core.worker_main" in " ".join(process.info["cmdline"] or ())
                    and process.environ().get("ROS_DOMAIN_ID") == str(graph.lease.domain)):
                found.append(process)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return found


def response(graph, handle):
    """A finished Json goal's result object."""
    reply = handle.get_result_async()
    graph.until(reply.done)
    return json.loads(reply.result().result.response)


def test_robodojo_graph_serves_its_devices_and_their_goals_from_a_native_worker(tmp_path):
    # The depth configuration serves everything the standard devices do, and registered depth; one Isaac boot.
    graph = Graph(tmp_path, backend="robodojo", mode="stepped", spectators=True, startup_s=STARTUP_S,
                  configuration="depth")
    node, last = graph.node, {}
    try:
        native = workers(graph)
        assert len(native) == 1 and "robodojo-native" in native[0].exe()
        for kind, topic, key in ((JointState, "left_arm/joint_state", "arm"),
                                 (JointState, "right_arm/joint_state", "right"),
                                 (JointState, "left_gripper/joint_state", "gripper"),
                                 (CameraInfo, "head_camera/camera_info", "info"),
                                 (MultiDOFJointState, "left_arm/pose_state", "flange"),
                                 (MultiDOFJointState, "right_arm/pose_state", "right_flange"),
                                 (Image, "world/spectator/third_person/image_raw", "spectator")):
            node.create_subscription(kind, topic, lambda msg, k=key: last.update({k: msg}), STREAM_QOS)
        moves = [ActionClient(node, Json, f"{arm}/move") for arm in ("left_arm", "right_arm")]
        grasp, release = (ActionClient(node, Json, f"left_gripper/{name}") for name in ("grasp", "release"))
        stop = node.create_client(Trigger, "left_arm/stop")
        graph.until(lambda: all(client.server_is_ready() for client in (*moves, grasp, release)) and stop.service_is_ready())
        # Goals are the devices' only commands.
        assert not node.count_subscribers("left_arm/joint_target") and not node.count_subscribers("left_gripper/target")
        assert graph.wait(.2).success
        graph.until(lambda: all(k in last for k in ("arm", "right", "gripper", "flange", "right_flange", "info", "spectator")))
        # Both arms keep the native joint names; devices disambiguate them.
        assert last["arm"].name == last["right"].name == [f"joint{i}" for i in range(1, 7)]
        assert last["gripper"].name == ["left_gripper"]
        assert (last["info"].width, last["info"].height, last["info"].header.frame_id) == (
            640, 480, "head_camera/base")
        assert last["info"].k[2] == 319.5 and (last["spectator"].width, last["spectator"].height) == (960, 720)
        assert (last["flange"].header.frame_id, list(last["flange"].joint_names)) == ("left_arm/base", ["flange"])

        def raised(key, dz, **fields):
            """The height and the move goal of a flange lifted dz from where it is."""
            pose = last[key].transforms[0]
            t, q = pose.translation, pose.rotation
            return t.z + dz, Json.Goal(request=json.dumps({"pose": [t.x, t.y, t.z + dz, q.x, q.y, q.z, q.w], **fields}))

        # Both arms at once, each in its gear: one wait ends with both.
        heights, goals = zip(raised("flange", .06), raised("right_flange", .04, profile="precision"))
        handles = [execute(graph, move, goal) for move, goal in zip(moves, goals)]
        both = graph.wait(None, 120., goals=handles, limit=3.)
        assert both.success and both.finished == [True, True] and both.waited_s < 1.
        for handle, tolerance in zip(handles, (.03, .005)):
            status, success, text = ended(graph, handle)
            assert (status, success) == (GoalStatus.STATUS_SUCCEEDED, True) and text.startswith("reached: "), text
            result = response(graph, handle)
            assert result["stop_reason"] == "reached" and result["position_error_m"] <= tolerance
        graph.spin(.1)
        for key, height, tolerance in zip(("flange", "right_flange"), heights, (.03, .005)):
            assert abs(last[key].transforms[0].translation.z - height) <= tolerance + 1e-3
        # A goal beyond reach is aborted where upstream's IK has no next step, with the reason.
        beyond = execute(graph, moves[0], raised("flange", 3.)[1])
        assert graph.wait(None, 120., goals=[beyond]).finished == [True]
        status, success, text = ended(graph, beyond)
        assert (status, success) == (GoalStatus.STATUS_ABORTED, False) and " IK " in text, text
        assert response(graph, beyond)["stop_reason"] == "unreachable"
        assert graph.call(stop, Trigger.Request()).success
        # A grasp closes the empty fingers to their stop, and a release opens them again.
        closing = execute(graph, grasp, Json.Goal(request="{}"))
        assert graph.wait(None, 120., goals=[closing], limit=2.).finished == [True]
        closed = response(graph, closing)
        assert ended(graph, closing)[:2] == (GoalStatus.STATUS_SUCCEEDED, True) and closed["stop_reason"] == "reached"
        assert closed["opening"] < .25  # the finger stop, short of the closed command
        opening = execute(graph, release, Json.Goal(request="{}"))
        assert graph.wait(None, 120., goals=[opening], limit=2.).finished == [True]
        assert response(graph, opening)["opening"] > .95
        image, depth = graph.frame("left_camera", depth=True, timeout=60.)
        assert image.header.frame_id == "left_camera/base" and (image.width, image.height) == (640, 480)
        assert np.frombuffer(bytes(image.data), np.uint8).std() > 1
        # Registered depth: same grid, stamp and optical frame; 16UC1 millimetres.
        assert (depth.encoding, depth.width, depth.height, depth.step) == ("16UC1", 640, 480, 1280)
        assert depth.header == image.header
        millimetres = np.frombuffer(bytes(depth.data), np.uint16)
        assert (millimetres > 0).mean() > .99 and 50 < np.median(millimetres) < 2000
        before = seconds(last["spectator"].header.stamp)
        assert graph.wait(.2).success
        graph.until(lambda: seconds(last["spectator"].header.stamp) > before, 60.)
    finally:
        graph.close()
    assert not any(process.is_running() and process.status() != psutil.STATUS_ZOMBIE for process in native)


def test_the_basic_configuration_takes_joint_and_grip_targets_on_topics(tmp_path):
    graph = Graph(tmp_path, backend="robodojo", mode="stepped", startup_s=STARTUP_S, configuration="basic")
    node, last = graph.node, {}
    try:
        for topic, key in (("left_arm/joint_state", "arm"), ("left_gripper/joint_state", "gripper")):
            node.create_subscription(JointState, topic, lambda msg, k=key: last.update({k: msg}), STREAM_QOS)
        joint_target = node.create_publisher(JointTrajectory, "left_arm/joint_target", 1)
        gripper_target = node.create_publisher(Float64, "left_gripper/target", 1)
        graph.until(lambda: joint_target.get_subscription_count() and gripper_target.get_subscription_count())
        assert not any("/move/" in name or "/grasp/" in name for name, _ in node.get_topic_names_and_types())
        assert graph.wait(.2).success
        graph.until(lambda: "arm" in last and "gripper" in last)
        target, opened = list(last["arm"].position), last["gripper"].position[0]
        target[0] += .1
        joint_target.publish(JointTrajectory(joint_names=list(last["arm"].name), points=[
            JointTrajectoryPoint(positions=target, time_from_start=Duration(nanosec=500_000_000))]))
        gripper_target.publish(Float64(data=0.))
        graph.spin(.1)
        assert graph.wait(1.).success
        graph.spin(.1)
        assert abs(last["arm"].position[0] - target[0]) < .03 and last["gripper"].position[0] < opened - .5
    finally:
        graph.close()
