"""The arms' move action: a goal is a flange pose, which an arm steps toward in a gear's steps until it is there.
No ROS and no Isaac: a goal, its feedback and its result are the objects of the Json action."""
import numpy as np

from kinex_sim.core.commands import CommandBuffer, slerp

# The gears, uni-robot-agent's motion profiles: the most a control step moves the flange (m), and how near its pose
# the flange has reached it (m, rad).
PROFILES = {"standard": (.04, .03, .1), "approach": (.04, .02, .05), "precision": (.01, .005, .05)}
# The most a step turns the flange for each metre it may move it: 0.13 rad with a 4 cm step.
TURN_RAD_PER_M = 3.3
# Control steps a goal has beyond those its gear needs for the way.
SETTLE = 25
# A flange that gains too little on its pose over this many steps has stalled.
WINDOW = 4
# A flange whose distance from its pose, in m plus this weight of its rad, grows by more than SLIP on each of two
# steps running is diverging.
ANGLE_WEIGHT, SLIP = .1, .01
# The most one step may move a flange (m, rad): of an arm a goal moves, and of one that holds still.
MOVED, HELD = (.2, .8), (.05, .25)


def offset(a, b):
    """Distance (m) and angle (rad) between poses [x, y, z, qx, qy, qz, qw] with unit quaternions."""
    return np.array([np.linalg.norm(b[:3] - a[:3]), 2 * np.arccos(min(1., abs(a[3:] @ b[3:])))])


class Motion:
    """A flange's goal: each step commands a pose at most the gear's step from the measured one, so a blocked
    flange is never driven far past where it is. The goal ends when the flange is within the gear's tolerance of
    its pose (reached), when it stalls or diverges, or SETTLE steps after the gear's steps would have brought it
    there (late). command is what the goal commands its arm, the joints of its latest step.

    The flange steps toward target: the goal's pose, or once recovered a pose near it that the arm can reach.
    Progress is measured against target, arrival always against the goal's own pose."""

    def __init__(self, pose, goal, profile):
        reach, *tolerance = PROFILES[profile]
        self.pose, self.goal, self.target, self.profile = np.asarray(pose), goal, goal, profile
        self.step, self.tolerance = np.array([reach, reach * TURN_RAD_PER_M]), np.array(tolerance)
        self.steps, self.command = -1, None  # observed once before its first step
        self.recovery = None  # None until a pose near the goal is sought, then whether one was found
        self.limit = int(np.ceil(max(offset(self.pose, goal) / self.step))) + SETTLE
        self._track()

    def _track(self):
        """Start measuring progress toward target."""
        self.gaps, self.score, self.worse = [], offset(self.pose, self.target) @ (1., ANGLE_WEIGHT), 0

    def recover(self, pose):
        """Step toward pose, one the arm can reach within the goal's tolerance, from now on."""
        self.target, self.recovery = np.asarray(pose), True
        self._track()

    def reference(self):
        """The pose to command for the next step."""
        distance, angle = offset(self.pose, self.target)
        reach, turn = self.step
        return np.r_[self.pose[:3] + (self.target[:3] - self.pose[:3]) * min(1., reach / max(distance, 1e-9)),
                     slerp(self.pose[3:], self.target[3:], min(1., turn / max(angle, 1e-9)))]

    def observe(self, pose):
        """Take the pose measured after a step, or before the first: why the goal ends, or None while it goes on."""
        self.pose, self.steps = np.asarray(pose), self.steps + 1
        if np.all(offset(self.pose, self.goal) <= self.tolerance):
            return "reached"
        if self.steps >= self.limit:
            return "late"
        gap = offset(self.pose, self.target)
        score = gap @ (1., ANGLE_WEIGHT)
        self.worse, self.score = (self.worse + 1 if score > self.score + SLIP else 0), score
        if self.worse >= 2:
            return "diverging"
        self.gaps = [*self.gaps[1 - WINDOW:], gap]
        gained = self.gaps[0] - np.min(self.gaps[1:], axis=0) if len(self.gaps) == WINDOW else np.inf
        if np.all(gained <= np.minimum(.1 * self.tolerance, .5 * self.step)):
            return "stalled"

    def succeeded(self, reason):
        return reason == "reached"

    def feedback(self):
        distance, angle = offset(self.pose, self.goal)
        return {"position_error_m": float(distance), "orientation_error_rad": float(angle), "steps": self.steps}

    def result(self, reason, text=None):
        """The action's result: why the goal ended (reached, stalled, diverging, late, unreachable, unsafe_motion or
        ended), in words as its message, the measured pose, how far it is from the goal's, and whether the flange
        went for a pose near the goal's that the arm could reach."""
        errors = self.feedback()
        return {"stop_reason": reason, "pose": [float(value) for value in self.pose], **errors,
                "recovered": bool(self.recovery),
                "message": f"{text or reason}: {errors['position_error_m']:.3f} m and "
                           f"{errors['orientation_error_rad']:.3f} rad from the pose after {self.steps} steps"}


def unsafe(moved, moving):
    """The arms a step moved too far, and how far, in words: moved maps each arm to its flange's (m, rad) over the
    step, and moving names the arms a goal moves; the others hold still."""
    return {arm: f"{arm} moved {distance:.3f} m and {angle:.3f} rad in one step" for arm, (distance, angle) in moved.items()
            if np.any(np.array([distance, angle]) > (MOVED if arm in moving else HELD))}


def motion(goal, pose):
    """The Motion of a goal, the action's object {pose, profile}, for the flange measured at pose. A ValueError says
    why it is none."""
    unknown = set(goal) - {"pose", "profile"}
    if unknown or "pose" not in goal:
        raise ValueError(f"a move goal is {{pose, profile}}, not {sorted(goal)}")
    profile = goal.get("profile", "standard")
    if profile not in PROFILES:
        raise ValueError(f"profile is one of {', '.join(PROFILES)}")
    return Motion(pose, CommandBuffer.unit_pose(goal["pose"], "goal pose"), profile)
