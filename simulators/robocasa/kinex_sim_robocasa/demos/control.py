"""Finite ROS-only exercise of the base, torso, arm, gripper and camera endpoints; not a task policy."""
import json
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Transform, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, MultiDOFJointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import (JointTrajectory, JointTrajectoryPoint, MultiDOFJointTrajectory,
                                 MultiDOFJointTrajectoryPoint)

from kinex_sim.clients import WorldClockClient
from kinex_sim_runtime.devices import STREAM_QOS

CAMERAS = ('camera_left', 'camera_right', 'camera_wrist')


class ControlDemo(Node):
    def __init__(self):
        super().__init__('robocasa_control_demo')
        self.last, self.image_count = {}, dict.fromkeys(CAMERAS, 0)
        for device in ('arm', 'torso', 'gripper'):
            self.create_subscription(JointState, f'{device}/joint_state',
                                     lambda m, d=device: self.last.update({d: m}), STREAM_QOS)
        self.create_subscription(Odometry, 'base/odometry', lambda m: self.last.update(base=m), STREAM_QOS)
        self.create_subscription(MultiDOFJointState, 'arm/pose_state', lambda m: self.last.update(pose=m), STREAM_QOS)
        for camera in CAMERAS:
            self.create_subscription(Image, f'{camera}/image', lambda m, c=camera: self.on_image(c, m),
                                     qos_profile_sensor_data)
        # The arm takes flange pose targets, or in the basic configuration joint targets.
        self.joint_target = {device: self.create_publisher(JointTrajectory, f'{device}/joint_target', 1)
                             for device in ('arm', 'torso')}
        self.pose_target = self.create_publisher(MultiDOFJointTrajectory, 'arm/pose_target', 1)
        self.gripper_target = self.create_publisher(Float64, 'gripper/target', 1)
        self.velocity = self.create_publisher(Twist, 'base/velocity', 1)
        self.world = WorldClockClient(self)

    def on_image(self, camera, msg):
        if msg.encoding != 'rgb8' or len(msg.data) != msg.width * msg.height * 3:
            raise AssertionError(f'invalid {camera} RGB image')
        self.image_count[camera] += 1

    def wait(self, predicate, timeout=180.):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=.02)
        raise RuntimeError('timed out waiting for the RoboCasa graph')

    def spin(self, seconds=.1):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.01)

    def advance(self, seconds):
        # Works in both modes. Stepped admits ticks; realtime observes elapsed sim time.
        result = self.world.wait(seconds)
        if not result.success:
            raise RuntimeError(result.message)
        self.spin()

    def move(self, device, positions):
        self.joint_target[device].publish(JointTrajectory(
            joint_names=list(self.last[device].name),
            points=[JointTrajectoryPoint(positions=positions, time_from_start=Duration(sec=1))]))
        self.spin()
        self.advance(1.5)
        return abs(self.last[device].position[0] - positions[0])

    def reach(self, forward):
        """How far the flange moves forward after a pose target forward metres ahead of it."""
        start = self.last['pose'].transforms[0]
        position = start.translation
        target = Transform(translation=Vector3(x=position.x + forward, y=position.y, z=position.z),
                           rotation=start.rotation)
        message = MultiDOFJointTrajectory(joint_names=['flange'], points=[
            MultiDOFJointTrajectoryPoint(transforms=[target], time_from_start=Duration(sec=1))])
        message.header.frame_id = 'arm/base'
        self.pose_target.publish(message)
        self.spin()
        self.advance(2.)
        return self.last['pose'].transforms[0].translation.x - position.x

    def grip(self, target):
        self.gripper_target.publish(Float64(data=target))
        self.spin()
        self.advance(.6)
        return self.last['gripper'].position[0]

    def turn(self, seconds, rate=.5):
        # The base stops 0.5 s of world time after the last twist, so keep sending.
        twist = Twist()
        twist.angular.z = rate
        for _ in range(round(seconds / .25)):
            self.velocity.publish(twist)
            self.spin()
            self.advance(.25)
        self.advance(1.)
        odometry = self.last['base'].pose.pose.orientation
        return odometry.z, abs(self.last['base'].twist.twist.angular.z)

    def run(self):
        arm = (self.joint_target['arm'], self.pose_target)  # one of them, by configuration
        self.wait(lambda: any(p.get_subscription_count() for p in arm) and all(
            p.get_subscription_count() for p in (self.joint_target['torso'], self.gripper_target, self.velocity)))
        self.advance(.2)
        self.wait(lambda: all(k in self.last for k in ('arm', 'torso', 'gripper', 'base')))
        if self.pose_target.get_subscription_count():
            self.wait(lambda: 'pose' in self.last)
            forward, back = self.reach(.04), self.reach(-.04)
            if abs(forward - .04) > .005 or abs(back + .04) > .005:
                raise AssertionError(f'flange pose targets were not reached: {forward=}, {back=}')
            moved = {'flange_forward_m': forward}
        else:
            start = list(self.last['arm'].position)
            error = self.move('arm', [start[0] + .1, *start[1:]])
            if error > .04:
                raise AssertionError(f'arm joint target was not reached: {error=}')
            moved = {'arm_error_rad': error}
        torso_error = self.move('torso', [.15])
        if torso_error > .02:
            raise AssertionError(f'torso joint target was not reached: {torso_error=}')
        closed, opened = self.grip(1.), self.grip(-1.)
        if (closed, opened) != (1., -1.):
            raise AssertionError(f'gripper command did not reach its limits: {closed=}, {opened=}')
        turned, residual = self.turn(1.)
        if turned < .05 or residual > .05:
            raise AssertionError(f'base did not turn and stop: {turned=}, {residual=}')
        if not all(self.image_count.values()):
            raise AssertionError('missing observation camera stream')
        print(json.dumps({'result': 'PASS', **moved, 'torso_error_m': torso_error,
                          'gripper': {'closed': closed, 'opened': opened},
                          'base_odometry_qz': turned, 'image_count': self.image_count}), flush=True)


def main():
    rclpy.init()
    node = ControlDemo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
