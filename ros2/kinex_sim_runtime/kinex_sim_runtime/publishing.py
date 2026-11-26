"""Owner-thread observation publication: clock, joint states, cameras and spectator streams."""
import threading
import time

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

from kinex_sim.core.devices import SampleSchedule, camera_rate
from . import conversions as conv
from .devices import STREAM_QOS, Camera

# The topic on which a synchronous recorder states the latest spectator sample it holds for every view, as a clock.
ACKNOWLEDGED = 'world/spectator/acknowledged'
# How long a synchronous recording holds the world for the recorder before the world stops with an error.
ACKNOWLEDGE_TIMEOUT_S = 60.


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Spectator:
    """The owner thread renders; a timer publishes the latest frame so reliable writes never block physics."""

    def __init__(self, node, view):
        self.view, self.frame = view, f'spectator_{view.name}'
        self.image = node.create_publisher(Image, f'world/spectator/{view.name}/image_raw', STREAM_QOS)
        self.info = node.create_publisher(CameraInfo, f'world/spectator/{view.name}/camera_info', STREAM_QOS)
        self._lock = threading.Lock()
        self._pending = None
        node.create_timer(.005, self._flush)

    def submit(self, stamp, sim, rgb):
        info = conv.camera_info(stamp, self.frame, self.view.width, self.view.height,
                                *sim.spectator_intrinsics(self.view.name))
        image = conv.image(stamp, self.frame, rgb)
        with self._lock:
            self._pending = (info, image)

    def _flush(self):
        with self._lock:
            pending, self._pending = self._pending, None
        if pending is not None:
            self.info.publish(pending[0])
            self.image.publish(pending[1])


class ObservationPublisher:
    """sync: a synchronous recording, in which the world takes no step while the recorder has not acknowledged the
    latest spectator sample (held())."""

    def __init__(self, node, spectator_rate_hz, sync=False):
        self.node, self.sim = node, node.sim
        self.clock = node.create_publisher(Clock, '/clock', STREAM_QOS)
        self.cameras = SampleSchedule(camera_rate(self.sim))
        self.spectator_schedule = SampleSchedule(spectator_rate_hz)
        self.spectators = [Spectator(node, view) for view in self.sim.spectators]
        self.sync = bool(sync and self.spectators)
        self.sampled_ns = None  # the latest spectator sample's stamp
        self.acknowledged_ns = -1
        self._held_since = None
        if self.sync:
            node.create_subscription(Clock, ACKNOWLEDGED, self._acknowledged, STREAM_QOS)

    def _acknowledged(self, msg):
        self.acknowledged_ns = max(self.acknowledged_ns, stamp_ns(msg.clock))

    def held(self):
        """Whether the world must wait for the recorder before its next step. Owner thread."""
        if not self.sync or self.sampled_ns is None or self.acknowledged_ns >= self.sampled_ns:
            self._held_since = None
            return False
        if self._held_since is None:
            self._held_since = time.monotonic()
        elif time.monotonic() - self._held_since > ACKNOWLEDGE_TIMEOUT_S:
            raise RuntimeError(f'the recorder has not acknowledged the spectator frame at {self.sampled_ns} ns '
                               f'for {ACKNOWLEDGE_TIMEOUT_S:.0f} s')
        return True

    def _sample(self, stamp):
        frames = self.sim.render_spectators()
        for spectator in self.spectators:
            spectator.submit(stamp, self.sim, frames[spectator.view.name])
        self.sampled_ns = stamp_ns(stamp)

    def publish(self):
        time_s = self.node.world.time_s
        stamp = conv.stamp(time_s)
        positions, velocities = self.sim.state()
        self.clock.publish(conv.clock(stamp))
        cameras = self.cameras.due(time_s)
        for device in self.node.devices:
            if not isinstance(device, Camera):
                device.publish(stamp, positions, velocities)
            elif cameras:
                device.publish(stamp)
        if self.spectators and self.spectator_schedule.due(time_s):
            self._sample(stamp)

    def final(self, timeout_s=10.):
        """Sample the spectators at the world's time unless the latest sample already shows it, publish the frames
        at once, and, in a synchronous recording, wait up to timeout_s for the recorder to acknowledge them. Owner
        thread; the executor delivers the acknowledgement."""
        if not self.spectators:
            return
        stamp = conv.stamp(self.node.world.time_s)
        if self.sampled_ns is not None and self.sampled_ns >= stamp_ns(stamp):
            return
        self._sample(stamp)
        for spectator in self.spectators:
            spectator._flush()
        deadline = time.monotonic() + timeout_s
        while self.sync and self.acknowledged_ns < self.sampled_ns and time.monotonic() < deadline:
            time.sleep(.005)
