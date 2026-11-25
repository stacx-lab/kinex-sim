"""The arms' move action, uni-robot-agent's move: a goal is a pose of the gripper's TCP, which an arm steps toward in
a gear's steps until it is there. No ROS and no Isaac: a goal, its feedback and its result are the objects of the
Json action, and poses are [x, y, z, qx, qy, qz, qw] in the arm's base."""
import numpy as np

from kinex_sim.core.commands import CommandBuffer, slerp

# The gears, uni-robot-agent's motion profiles: the most a control step moves the TCP (m), and how near its pose the
# TCP has reached it (m, rad).
PROFILES = {"standard": (.04, .03, .1), "approach": (.04, .02, .05), "precision": (.01, .005, .05)}
# uni-robot-agent's bounds on a goal's own tolerances (m, rad), which replace its gear's, and on its steps.
LIMITS = {"position_tolerance_m": (.001, .1), "orientation_tolerance_rad": (.02, .5)}
MAX_STEPS = 24
# A reason's words in a result's message, where they differ from the reason.
WORDS = {"max_steps": "reached max_steps"}
FIELDS = ("pose", "profile", *LIMITS, "max_steps")
# The TCP, <gripper>/tcp: RoboEnv's parallel-gripper geometry, the pad centre in the flange frame, turned as the
# flange is. A turn about the TCP sweeps the flange RADIUS away.
TCP = np.array([.15007, -.000002, -.00144])
RADIUS = float(np.linalg.norm(TCP))
TOOL = np.r_[np.c_[np.eye(3), TCP], [[0., 0., 0., 1.]]]
# A TCP that gains too little on its pose over this many steps has stalled.
WINDOW = 4
# A TCP whose distance from its pose, in m plus this weight of its rad, grows by more than SLIP on each of two steps
# running is diverging.
ANGLE_WEIGHT, SLIP = .1, .01
# The most one step may move a TCP (m, rad): of an arm a goal moves, and of one that holds still.
MOVED, HELD = (.2, .8), (.05, .25)


def offset(a, b):
    """Distance (m) and angle (rad) between poses with unit quaternions."""
    return np.array([np.linalg.norm(b[:3] - a[:3]), 2 * np.arccos(min(1., abs(a[3:] @ b[3:])))])


def rotate(q, v):
    """v turned by the unit quaternion q, [x, y, z, w]."""
    return v + 2 * np.cross(q[:3], np.cross(q[:3], v) + q[3] * v)


def tcp(pose):
    """The TCP pose of a flange pose."""
    return np.r_[pose[:3] + rotate(pose[3:], TCP), pose[3:]]


def flange(pose):
    """The flange pose of a TCP pose."""
    return np.r_[pose[:3] - rotate(pose[3:], TCP), pose[3:]]


class Motion:
    """A TCP's goal: each step commands a pose at most the gear's step from the measured one, so a blocked TCP is
    never driven far past where it is. Judged after each step, never before the first, the goal succeeds when the
    TCP is within the tolerance of its pose (reached) or has taken max_steps steps (max_steps), and fails when it
    diverges or stalls. command is what the goal commands its arm, the joints of its latest step.

    The TCP steps toward target: the goal's pose, or once recovered a pose near it that the arm can reach.
    Progress is measured against target, arrival always against the goal's own pose."""

    def __init__(self, pose, goal, profile, tolerance=None, max_steps=MAX_STEPS):
        self.reach, *default = PROFILES[profile]
        self.pose, self.goal, self.target, self.profile = np.asarray(pose), goal, goal, profile
        self.tolerance, self.max_steps = np.array(default if tolerance is None else tolerance), max_steps
        self.steps, self.command = -1, None  # observed once before its first step
        self.recovery = None  # None until a pose near the goal is sought, then whether one was found
        self._track()

    def _track(self):
        """Start measuring progress toward target."""
        self.gaps, self.score, self.worse = [], offset(self.pose, self.target) @ (1., ANGLE_WEIGHT), 0

    def recover(self, pose):
        """Step toward pose, one the arm can reach within the goal's tolerance, from now on."""
        self.target, self.recovery = np.asarray(pose), True
        self._track()

    def reference(self):
        """The pose to command for the next step, uni-robot-agent's waypoint: the TCP turns toward target at most
        as far as sweeps the flange through half the gear's step, and moves on the straight line to target by what
        the sweep leaves of the step."""
        angle = offset(self.pose, self.target)[1]
        turn = min(angle, 2 * np.arcsin(self.reach / (4 * RADIUS)))
        way = self.target[:3] - self.pose[:3]
        reach = max(0., self.reach - 2 * RADIUS * np.sin(turn / 2))
        return np.r_[self.pose[:3] + way * min(1., reach / max(np.linalg.norm(way), 1e-9)),
                     slerp(self.pose[3:], self.target[3:], turn / max(angle, 1e-9))]

    def observe(self, pose):
        """Take the pose measured after a step, or before the first: why the goal ends, or None while it goes on."""
        self.pose, self.steps = np.asarray(pose), self.steps + 1
        if not self.steps:
            return None
        if np.all(offset(self.pose, self.goal) <= self.tolerance):
            return "reached"
        gap = offset(self.pose, self.target)
        score = gap @ (1., ANGLE_WEIGHT)
        self.worse, self.score = (self.worse + 1 if score > self.score + SLIP else 0), score
        if self.worse >= 2:
            return "diverging"
        self.gaps = [*self.gaps[1 - WINDOW:], gap]
        if len(self.gaps) == WINDOW and np.all(self.gaps[0] - np.min(self.gaps[1:], axis=0) <= .1 * self.tolerance):
            return "stalled"
        if self.steps >= self.max_steps:
            return "max_steps"

    def succeeded(self, reason):
        return reason in ("reached", "max_steps")

    def feedback(self):
        distance, angle = offset(self.pose, self.goal)
        return {"position_error_m": float(distance), "orientation_error_rad": float(angle), "steps": self.steps}

    def result(self, reason, text=None):
        """The action's result: why the goal ended (reached, stalled, diverging, max_steps, unreachable,
        unsafe_motion or ended), in words as its message, the measured TCP pose, how far it is from the goal's, and
        whether the TCP went for a pose near the goal's that the arm could reach."""
        errors = self.feedback()
        return {"stop_reason": reason, "pose": [float(value) for value in self.pose], **errors,
                "recovered": bool(self.recovery),
                "message": f"{text or WORDS.get(reason, reason)}: {errors['position_error_m']:.3f} m and "
                           f"{errors['orientation_error_rad']:.3f} rad from the pose after {self.steps} steps"}


def unsafe(moved, moving):
    """The arms a step moved too far, and how far, in words: moved maps each arm to its TCP's (m, rad) over the
    step, and moving names the arms a goal moves; the others hold still."""
    return {arm: f"{arm} moved {distance:.3f} m and {angle:.3f} rad in one step" for arm, (distance, angle) in moved.items()
            if np.any(np.array([distance, angle]) > (MOVED if arm in moving else HELD))}


def motion(goal, pose):
    """The Motion of a goal, the action's object {pose, profile, position_tolerance_m, orientation_tolerance_rad,
    max_steps} with uni-robot-agent's defaults and bounds, for the TCP measured at pose. A ValueError says why it is
    none."""
    if set(goal) - set(FIELDS) or "pose" not in goal:
        raise ValueError(f"a move goal is {{{', '.join(FIELDS)}}}, not {sorted(goal)}")
    profile = goal.get("profile", "standard")
    if profile not in PROFILES:
        raise ValueError(f"profile is one of {', '.join(PROFILES)}")
    tolerance = []
    for name, default, (low, high) in zip(LIMITS, PROFILES[profile][1:], LIMITS.values()):
        value = goal.get(name, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not low <= value <= high:
            raise ValueError(f"{name} is a number from {low:g} to {high:g}")
        tolerance.append(float(value))
    steps = goal.get("max_steps", MAX_STEPS)
    if isinstance(steps, bool) or not isinstance(steps, int) or not 1 <= steps <= MAX_STEPS:
        raise ValueError(f"max_steps is an integer from 1 to {MAX_STEPS}")
    return Motion(pose, CommandBuffer.unit_pose(goal["pose"], "goal pose"), profile, tolerance, steps)
