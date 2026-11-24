"""The world node's HandsOn backend: a proxy for Isaac Sim 5.1 in its Python 3.11 worker."""
from kinex_sim.core.worker import WorkerBackend


class HandsOnWorker(WorkerBackend):
    simulator = "handson"
    # Isaac Sim shutdown can take a while; the proxy kills the worker after this.
    close_timeout_s = 120.

    def measures(self):
        """The episode's native seed (HandsOnSim.measures)."""
        return self._call("measures")
