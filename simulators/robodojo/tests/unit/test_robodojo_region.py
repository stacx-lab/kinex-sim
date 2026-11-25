"""The goal region search without Isaac: the arm's flange from its URDF, and a pose it can reach near a goal. It
runs where SciPy is, as the native worker's environment has it."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

SCIPY = importlib.util.find_spec("scipy") is not None
pytestmark = pytest.mark.skipif(not SCIPY, reason="needs SciPy")
if SCIPY:
    from scipy.spatial.transform import Rotation
    from kinex_sim_robodojo import move, region

ARM = Path(__file__).resolve().parents[4] / "catalog/robots/robodojo/robodojo-arx-x5/devices/left_arm/arm.urdf"
JOINTS = [f"joint{i}" for i in range(1, 7)]


def test_a_chain_is_the_flange_of_the_arms_urdf():
    import sys
    sys.path.insert(0, str(ARM.parents[6] / "tools/robots"))
    try:
        import rigs
    finally:
        sys.path.pop(0)
    q, expected = np.array([.3, .8, .9, -.4, .2, -.5]), np.eye(4)
    for name, angle in zip(JOINTS, q):  # the rig builders' own reading of a URDF origin
        joint = rigs.urdf_joints(ARM)[name]
        turn = np.eye(4)
        turn[:3, :3] = Rotation.from_rotvec(np.array(joint.find("axis").get("xyz").split(), float) * angle).as_matrix()
        expected = expected @ rigs.urdf_origin(joint) @ turn
    assert region.chain(ARM, JOINTS)(q) == pytest.approx(expected, abs=1e-9)


def test_a_goal_no_ik_reaches_is_replaced_by_a_reachable_pose_within_its_tolerance():
    fk, current = region.chain(ARM, JOINTS), np.array([0., .8, .9, -.4, 0., 0.])
    limits, tolerance = (np.full(6, -3.), np.full(6, 3.)), (.03, .1)

    def as_pose(matrix):
        return np.r_[matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()]

    def bounded(bound):
        """Constraints that forbid joint1 beyond bound, as an obstacle beside the arm would."""
        return lambda rows: (np.array([q[0] <= bound + 1e-9 for q in rows]), np.array([[q[0] - bound] for q in rows]))
    goal = as_pose(fk(current + [.2, 0., 0., 0., 0., 0.]))  # joint1 must turn 0.2 rad to reach it exactly
    assert max(move.offset(as_pose(fk(current)), goal) / tolerance) > 1  # and where the arm is lies outside its tolerance
    # A solution the IK returned that the constraints allow and that lies in the region is taken as it is.
    near = current + [.19, 0., 0., 0., 0., 0.]
    assert region.search(fk, [near], bounded(.19), current, goal, limits, tolerance) == pytest.approx(as_pose(fk(near)))
    # Without one, the joints nearest the measured ones are sought whose flange is inside 80% of the tolerance.
    found = region.search(fk, [], bounded(.15), current, goal, limits, tolerance)
    assert found is not None and np.all(move.offset(found, goal) <= np.multiply(tolerance, region.MARGIN) * (1 + 1e-5))
    assert not np.all(move.offset(found, goal) <= 1e-3)  # not the goal itself, which the constraints forbid
    blocked = lambda rows: (np.zeros(len(rows), bool), np.ones((len(rows), 1)))  # constraints nothing satisfies
    assert region.search(fk, [near], blocked, current, goal, limits, tolerance) is None
