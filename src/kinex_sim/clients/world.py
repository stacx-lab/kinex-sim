"""World-time waits for ROS clients, independent of simulator or execution mode."""
import math
import time


class WorldClockClient:
    """Waits on world/wait_all from a standalone node; the world admits one wait at a time."""

    def __init__(self, node, service="world/wait_all"):
        from custom_interfaces.srv import Wait
        self.node, self._type = node, Wait
        self.client = node.create_client(Wait, service)

    def wait(self, duration_s, *, timeout_s=300., goals=()):
        """Wait duration_s of world time, and until every one of goals, the ids of executing action goals (a goal
        handle's goal_id), has finished. Spins the node, which must not be attached to an executor, with a
        separate wall-time timeout: that does not cancel a wait already admitted; world/stop does."""
        from rclpy.executors import SingleThreadedExecutor
        if not math.isfinite(duration_s) or duration_s < 0:
            raise ValueError("duration_s must be finite and nonnegative")
        deadline = time.monotonic() + timeout_s
        if not self.client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(f"world clock unavailable: {self.client.srv_name}")
        future = self.client.call_async(self._type.Request(seconds=[float(duration_s)], goals=list(goals)))
        executor = SingleThreadedExecutor(context=self.node.context)
        executor.add_node(self.node)
        try:
            executor.spin_until_future_complete(future, timeout_sec=max(0., deadline - time.monotonic()))
        finally:
            executor.remove_node(self.node)
            executor.shutdown()
        if not future.done():
            self.client.remove_pending_request(future)
            raise TimeoutError("world wait wall-time timeout; the admitted wait remains pending")
        if future.result() is None:
            raise RuntimeError("world wait interrupted by ROS shutdown")
        return future.result()
