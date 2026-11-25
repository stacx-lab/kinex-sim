"""World-time waits for ROS clients, independent of simulator or execution mode."""
import math
import time


class WorldClockClient:
    """Waits on world/wait from a standalone node; the world admits one wait at a time."""

    def __init__(self, node):
        from custom_interfaces.srv import Now, Wait
        self.node, self._type = node, Wait
        self.client, self.clock = node.create_client(Wait, "world/wait"), node.create_client(Now, "world/now")
        self._now = Now.Request()

    def _call(self, client, request, deadline):
        from rclpy.executors import SingleThreadedExecutor
        future = client.call_async(request)
        executor = SingleThreadedExecutor(context=self.node.context)
        executor.add_node(self.node)
        try:
            executor.spin_until_future_complete(future, timeout_sec=max(0., deadline - time.monotonic()))
        finally:
            executor.remove_node(self.node)
            executor.shutdown()
        if not future.done():
            client.remove_pending_request(future)
            raise TimeoutError("world wait wall-time timeout; the admitted wait remains pending")
        if future.result() is None:
            raise RuntimeError("world wait interrupted by ROS shutdown")
        return future.result()

    def wait(self, duration_s, *, timeout_s=300.):
        """Wait duration_s of world time, through the goals whose ends end the world's own waits sooner. Spins the
        node, which must not be attached to an executor, with a separate wall-time timeout: that does not cancel
        a wait already admitted; world/stop does."""
        if not math.isfinite(duration_s) or duration_s < 0:
            raise ValueError("duration_s must be finite and nonnegative")
        deadline = time.monotonic() + timeout_s
        if not self.client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(f"world clock unavailable: {self.client.srv_name}")
        until = self._call(self.clock, self._now, deadline).time_s + duration_s
        while True:
            reply = self._call(self.client, self._type.Request(seconds=float(duration_s)), deadline)
            duration_s = until - reply.time_s
            if not reply.success or duration_s < 1e-9:
                return reply
