"""HandsOn control acceptance: both ROKAE palms by pose, a right-hand curl and an RGB head-camera frame."""
import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion, Transform, Vector3
from sensor_msgs.msg import MultiDOFJointState
from std_msgs.msg import Header
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from rosbridge_runtime import dual_arm_demo
from rosbridge_runtime.devices import STREAM_QOS

# Index-finger MCP flexion, PIP and DIP (finger2 joints 1, 3 and 4) in the hand's finger-major order.
CURL = {4: .6, 6: .6, 7: .6}
# The arms hang mirrored from the station: world up is +y in right_arm/base and -y in left_arm/base.
UP = {"right": 1., "left": -1.}


class PalmDemo(dual_arm_demo.ControlDemo):
    """Arms take palm pose targets, which upstream's IK executes, and report the measured palm (pose_state)."""
    tolerance = .01  # metres

    def arms(self):
        for side in ("left", "right"):
            self.create_subscription(MultiDOFJointState, f"{side}_arm/pose_state",
                                     lambda msg, d=f"{side}_arm": self.last.update({d: msg}), STREAM_QOS)
        self.targets = {side: self.create_publisher(MultiDOFJointTrajectory, f"{side}_arm/pose_target", 1)
                        for side in ("left", "right")}

    def positions(self, side):
        t, q = self.last[f"{side}_arm"].transforms[0].translation, self.last[f"{side}_arm"].transforms[0].rotation
        return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]

    def target(self, side, pose, frame=None):
        x, y, z, qx, qy, qz, qw = pose
        self.targets[side].publish(MultiDOFJointTrajectory(
            header=Header(frame_id=frame or f"{side}_arm/base"), joint_names=["palm"],
            points=[MultiDOFJointTrajectoryPoint(
                transforms=[Transform(translation=Vector3(x=x, y=y, z=z), rotation=Quaternion(x=qx, y=qy, z=qz, w=qw))],
                time_from_start=Duration(sec=1))]))

    def moved(self, side, start, offset):
        """The palm offset metres up in the world."""
        goal = list(start)
        goal[1] += UP[side] * offset
        return goal

    def error(self, side, goal):
        return float(np.linalg.norm(np.subtract(self.positions(side)[:3], goal[:3])))

    def invalid(self, side):
        self.target(side, self.positions(side), frame="world")  # pose targets are in the arm base


def main():
    dual_arm_demo.main("handson_control_demo", "head_camera", depth=False, hand=CURL, offsets=(.05, .05),
                       demo=PalmDemo)


if __name__ == "__main__":
    main()
