"""The grippers' grasp and release actions, after uni-robot-agent's: a goal commands the fingers closed or to an
opening, and ends once they are there or have stopped. No ROS and no Isaac: openings are positions of the grip
joint, 0 closed to 1 open, and a goal, its feedback and its result are the objects of the Json action."""
import numpy as np

# How near its target an opening has reached it: about 2 mm of pad gap.
TOLERANCE = .02
# Fingers that gain less than this on their target over WINDOW steps have stalled: about 0.5 mm of pad gap.
PROGRESS = .005
WINDOW = 8
# Control steps a goal has for fingers that neither arrive nor stall.
MAX_STEPS = 24


class Grip:
    """A gripper's goal: it commands the grip joint to command and ends when the opening is within TOLERANCE of
    target (reached), when the fingers stall, or after MAX_STEPS. target is command within what the fingers can
    reach: a closed command lies past the finger stop, to squeeze. A grasp that stalls has succeeded: the fingers
    stopped on what is between them."""

    def __init__(self, command, target, grasp):
        self.command, self.target, self.grasp = [command], target, grasp
        self.steps, self.errors, self.opening = -1, [], None  # observed once before its first step

    def observe(self, opening):
        """Take the opening measured after a step, or before the first: why the goal ends, or None while it goes on."""
        self.opening, self.steps = float(opening), self.steps + 1
        error = abs(self.opening - self.target)
        if error <= TOLERANCE:
            return "reached"
        self.errors = [*self.errors[1 - WINDOW:], error]
        if len(self.errors) == WINDOW and self.errors[0] - min(self.errors[1:]) <= PROGRESS:
            return "stalled"
        if self.steps >= MAX_STEPS:
            return "late"

    def succeeded(self, reason):
        return reason == "reached" or self.grasp and reason == "stalled"

    def feedback(self):
        return {"opening": self.opening, "steps": self.steps}

    def result(self, reason, text=None):
        """The action's result: why the goal ended (reached, stalled, late, unsafe_motion or ended), in words as
        its message, and the measured opening."""
        return {"stop_reason": reason, **self.feedback(),
                "message": f"{text or reason}: opening {self.opening:.3f} after {self.steps} steps"}


def grip(name, goal, span):
    """The Grip of a grasp goal, which closes, or of a release goal, the object {opening}, which opens to it, fully
    by default. span is the least and greatest opening the fingers can reach. A ValueError says why it is none."""
    if name == "grasp":
        if goal:
            raise ValueError(f"a grasp goal has no fields, not {sorted(goal)}")
        return Grip(0., float(span[0]), True)
    opening = goal.get("opening", 1.)
    if set(goal) - {"opening"} or isinstance(opening, bool) or not isinstance(opening, (int, float)) or not 0 <= opening <= 1:
        raise ValueError("a release goal is {opening}, a grip joint position from 0 (closed) to 1 (open)")
    return Grip(float(opening), float(np.clip(opening, *span)), False)
