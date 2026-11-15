"""Synthetic position/scalar backend for shared conformance tests."""
import threading
import numpy as np
from rosbridge.core import ActionLayout, CommandGroup
from rosbridge.core.devices import device_inventory


class ToyBackend:
    """A differently shaped backend: a reordered arm group and a scalar gripper."""
    layout = ActionLayout(("slide", "finger", "rotate"), (
        CommandGroup("axes", (2, 0), (-2., -3.), (2., 3.)),
        CommandGroup("finger", (1,), (-1.,), (1.,), scalar=True),
    ))
    devices = device_inventory(layout, {})
    default_task = "toy"
    cameras = spectators = ()
    reward = 0.
    period = 0.1

    def __init__(self, max_steps=None, early_stop=False):
        self.owner = threading.get_ident()
        self.max_steps, self.early_stop = max_steps, early_stop
        self.closed = False
        self.reset("toy", 0, False)

    def configure_spectators(self, width, height, views=None):
        pass

    def reset(self, task, seed, randomize):
        assert threading.get_ident() == self.owner
        if task not in ("toy", "succeeds", "fails", "stumbles"):
            raise ValueError("unknown task")
        self.task, self.seed = task, seed
        self.steps, self.time_s, self.done, self.success, self.stop = 0, 0., False, False, None
        self.position = np.zeros(3)

    def state(self):
        assert threading.get_ident() == self.owner
        return self.position.copy(), np.zeros(3)

    def step(self, action):
        assert threading.get_ident() == self.owner
        if self.task == "fails" and self.steps == 1:
            raise FloatingPointError("the toy's physics failed")  # its second step
        self.position = action.copy()
        self.steps += 1
        self.time_s = self.steps * self.period
        self.success = self.task == "succeeds" and self.steps >= 2  # its goal holds from the second step
        if self.early_stop and self.task == "stumbles" and self.steps == 2:
            self.stop = ("failure", "toy_stumbled")  # a task failure of its own at its second step
        self.done = (self.max_steps is not None and self.steps >= self.max_steps
                     or self.early_stop and self.success or self.stop is not None)

    def close(self):
        assert threading.get_ident() == self.owner
        self.closed = True
