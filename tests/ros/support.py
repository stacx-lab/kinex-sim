"""Subprocess ROS graphs, each on its own leased domain, with explicit cleanup."""
import json
import math
import os
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import rclpy
from custom_interfaces.srv import Now, Wait
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

from kinex_sim.runs.domains import DomainLease
from kinex_sim_runtime.devices import STREAM_QOS


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def execute(graph, client, message, feedback=None):
    """Send an action a goal; its handle once it is accepted, from when the world waits for it."""
    sent = client.send_goal_async(message, feedback_callback=feedback)
    graph.until(sent.done)
    assert sent.result().accepted
    return sent.result()


def ended(graph, handle):
    """A finished Json goal's (status, success, message)."""
    reply = handle.get_result_async()
    graph.until(reply.done)
    return reply.result().status, reply.result().result.success, reply.result().result.message


class Graph:
    def __init__(self, tmp_path, mode="stepped", backend="toy", max_steps=0, spectators=False,
                 startup_s=60., task="toy", early_stop=False, scene=None, configuration=None):
        self.lease = DomainLease()
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(self.lease.domain))
        self.processes, self.logs = [], []
        self.context = Context()
        rclpy.init(context=self.context, domain_id=self.lease.domain)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.node = Node("probe", context=self.context)
        self.last = {}
        self.node.create_subscription(Clock, "/clock", lambda msg: self.last.update(clock=msg), STREAM_QOS)
        self.wait_client = self.node.create_client(Wait, "world/wait")
        self.ended = set()  # the ids of the goals the world's waits have said ended
        self.now_client = self.node.create_client(Now, "world/now")
        self.stop_client = self.node.create_client(Trigger, "world/stop")
        self.tmp_path = tmp_path
        # Scene options travel as one JSON object, as launch passes them.
        scene = json.dumps(scene, separators=(",", ":"), sort_keys=True) if scene else None
        try:
            if backend.endswith("toy"):
                self.owner = self.start([sys.executable, "-m", "tests.ros.toy_server", "--ros-args",
                                         "-p", f"backend:={backend}",
                                         "-p", f"mode:={mode}", "-p", f"max_steps:={max_steps}",
                                         "-p", f"task:={task}",
                                         "-p", f"early_stop:={str(early_stop).lower()}",
                                         # rcl reads -p values as YAML: quoted, the object stays a string.
                                         *(["-p", f"scene:='{scene}'"] if scene else []),
                                         *(["-p", f"configuration:={configuration}"] if configuration else [])])
            else:
                self.owner = self.start(["ros2", "launch", "kinex_sim_runtime", "sim.launch.py",
                                         f"backend:={backend}", f"mode:={mode}",
                                         *([f"max_steps:={max_steps}"] if max_steps else []),
                                         *([f"scene:={scene}"] if scene else []),
                                         *([f"configuration:={configuration}"] if configuration else []),
                                         *(["spectator_rate_hz:=10.0"] if spectators else [])])
            # Some backends build scenes or start renderers for minutes before serving.
            self.until(self.wait_client.service_is_ready, startup_s)
            # Confirm that the owner loop is servicing requests after discovery.
            deadline = time.monotonic() + 5.
            while not self.wait(0.).success:
                assert time.monotonic() < deadline, "world never became responsive"
                self.spin(.05)
        except BaseException:
            self.close()
            raise

    def start(self, argv):
        log = open(self.tmp_path / f"process_{len(self.processes)}.log", "w+")
        self.logs.append(log)
        process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                                   env=self.env)
        self.processes.append(process)
        return process

    def until(self, predicate, timeout=15.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=.01)
        for log in self.logs:
            log.flush()
            log.seek(0)
            print(log.read())
        raise AssertionError("ROS graph timed out")

    def spin(self, duration=.2):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=.01)

    def call(self, client, request, timeout=15.):
        future = client.call_async(request)
        self.until(future.done, timeout)
        return future.result()

    def frame(self, camera, depth=False, timeout=15.):
        """A camera's current frame as a reader takes it: subscribe, then the first
        image and, with depth, the equally stamped depth. Returns (image, depth or None)."""
        from sensor_msgs.msg import Image
        from kinex_sim_runtime.devices import FRAME_QOS
        received = {"image": [], "depth": []}
        names = ("image", "depth") if depth else ("image",)
        subscriptions = [self.node.create_subscription(Image, f"{camera}/{name}",
                                                       lambda msg, name=name: received[name].append(msg),
                                                       FRAME_QOS) for name in names]

        def pair():
            depths = {seconds(m.header.stamp): m for m in received["depth"]}
            for image in received["image"]:
                if not depth or seconds(image.header.stamp) in depths:
                    return image, depths.get(seconds(image.header.stamp))
        try:
            self.until(lambda: pair() is not None, timeout)
            return pair()
        finally:
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)

    def now(self):
        return self.call(self.now_client, Now.Request())

    def wait(self, duration=0., timeout=15., goals=(), first=False, limit=0.):
        """Wait as a client does, by the world's waits, each of which a goal's end ends: for duration of world
        time, or with None until goals, handles of action goals, have ended, all of them or with first the first, at
        most limit seconds of world time. The last reply, with which goals had ended as finished, whether the
        limit ended it as timed_out and how long it took as waited_s."""
        start = self.now().time_s
        left = span = (limit or math.inf) if duration is None else duration
        while True:
            reply = self.call(self.wait_client, Wait.Request(seconds=float(left)), timeout)
            left = max(0., start + span - reply.time_s)
            self.ended |= {bytes(goal.uuid) for goal in reply.ended}
            finished = [bytes(handle.goal_id.uuid) in self.ended or handle.status >= 4 for handle in goals]
            met = duration is None and (any if first else all)(finished)
            late = reply.time_s >= start + span - 1e-9
            if met or late or not reply.success:
                return SimpleNamespace(success=reply.success, message=reply.message, time_s=reply.time_s,
                                       finished=finished, timed_out=late and not met and duration is None,
                                       waited_s=reply.time_s - start)
            self.spin(.02)  # a goal that ended as it started is known by its status

    def stop(self, process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    def close(self):
        for process in reversed(self.processes):
            self.stop(process)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.try_shutdown()
        for log in self.logs:
            log.close()
        self.lease.close()
