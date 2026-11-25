"""A pose an arm can reach within a goal's tolerance, for a move whose step has no IK solution: uni-robot-agent's
goal region search. No Isaac: the arm's kinematics come from its URDF, its IK candidates and its constraints from
the caller. Poses are [x, y, z, qx, qy, qz, qw] in the arm's base."""
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares, minimize
from scipy.spatial.transform import Rotation

# The search keeps this share of the tolerance, so the pose it finds is reached within the whole of it.
MARGIN = .8
# Optimizer starts, the iterations of each, and the wall time of a whole search.
STARTS, ITERATIONS, TIMEOUT_S = 4, 90, 10.


class Late(Exception):
    """The search has used its wall time."""


def chain(urdf, joints):
    """The forward kinematics of a serial chain of revolute joints, named in order, of a URDF: fk(q) is the last
    joint's child link in the first joint's parent link, a 4x4 transform."""
    found = {joint.get("name"): joint for joint in ET.parse(urdf).getroot().iter("joint")}
    links = []
    for name in joints:
        origin, fixed = found[name].find("origin"), np.eye(4)
        fixed[:3, :3] = Rotation.from_euler("xyz", [float(v) for v in origin.get("rpy", "0 0 0").split()]).as_matrix()
        fixed[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        links.append((fixed, np.array([float(v) for v in found[name].find("axis").get("xyz").split()])))

    def fk(q):
        pose = np.eye(4)
        for (fixed, axis), angle in zip(links, q):
            turn = np.eye(4)
            turn[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            pose = pose @ fixed @ turn
        return pose
    return fk


def search(fk, candidates, constraints, current, goal, limits, tolerance):
    """A pose within MARGIN of tolerance (m, rad) of goal that the arm can take, or None.

    fk(q) is the 4x4 transform at joints q of the frame a move controls; candidates are joints an IK for goal
    returned, converged or not; constraints(rows) gives, for joint rows, whether each satisfies the arm's
    constraints and each one's constraint costs, none positive when it does; current are the measured joints and
    limits their (lower, upper).
    A candidate already inside the region is taken, the nearest first; otherwise the joints nearest current are
    sought whose frame lies inside it. The pose is for a move's steps to go to: its joints are not commanded.
    """
    current, (low, high) = np.asarray(current, float), limits
    reach, turn = np.asarray(tolerance) * MARGIN
    target = np.eye(4)
    target[:3, :3], target[:3, 3] = Rotation.from_quat(goal[3:]).as_matrix(), goal[:3]
    deadline = time.monotonic() + TIMEOUT_S

    def errors(q):
        if time.monotonic() >= deadline:
            raise Late
        actual = fk(q)
        return np.r_[(actual[:3, 3] - target[:3, 3]) / reach,
                     Rotation.from_matrix(actual[:3, :3] @ target[:3, :3].T).as_rotvec() / turn]

    def inside(q):
        """Both parts of the region and the arm's constraints, each nonnegative when satisfied."""
        e = errors(q)
        return np.r_[1 - e[:3] @ e[:3], 1 - e[3:] @ e[3:], -constraints([q])[1][0]]

    def accepted(q):
        if not np.isfinite(q).all() or np.any(q < low) or np.any(q > high):
            return None
        e = errors(q)
        # The region itself, checked apart from the optimizer, whose convergence is not a radius. The nearest joints
        # lie on the region's edge, which rounding may put a millionth outside; MARGIN leaves far more than that.
        if np.linalg.norm(e[:3]) > 1 + 1e-6 or np.linalg.norm(e[3:]) > 1 + 1e-6 or not constraints([q])[0].all():
            return None
        actual = fk(q)
        return np.r_[actual[:3, 3], Rotation.from_matrix(actual[:3, :3]).as_quat()]

    try:
        near = sorted((np.asarray(q, float) for q in candidates), key=lambda q: np.linalg.norm(q - current))
        for q in near:
            if (pose := accepted(q)) is not None:
                return pose
        starts = [np.clip(current, low, high)]
        starts.insert(0, least_squares(errors, starts[0], bounds=(low, high), max_nfev=80,
                                       ftol=1e-9, xtol=1e-9, gtol=1e-9).x)
        for q in near:
            if len(starts) < STARTS and all(np.linalg.norm(q - start) > .1 for start in starts):
                starts.append(np.clip(q, low, high))
        for start in starts:
            fit = minimize(lambda q: .5 * np.sum((q - current) ** 2), start, jac=lambda q: q - current,
                           method="SLSQP", bounds=list(zip(low, high)), constraints={"type": "ineq", "fun": inside},
                           options={"maxiter": ITERATIONS, "ftol": 1e-9, "eps": 1e-4})
            if (pose := accepted(fit.x)) is not None:
                return pose
    except Late:
        pass
    return None
