#!/usr/bin/env python3
"""Pick-and-place mission: send 'pick_and_place' on /mission to start.

Steps (one per second of sim time, DELIVER takes a few seconds):
  PICKUP -> GRAB -> LIFT -> DELIVER (turn in place) -> PLACE -> RELEASE -> DONE
The gripper is a software interface only (no physics grasp yet).
Current state is published on /mission/state.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point

REACH_DOWN = Point(x=0.20, y=0.0, z=-0.12)   # relative to the shoulder pivot
CARRY = Point(x=0.16, y=0.0, z=0.10)
DELIVER_TURN_RATE = 0.6                       # rad/s while carrying
DELIVER_TICKS = 6                             # seconds of turning


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.state = 'IDLE'
        self.ticks = 0
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Point, '/arm_target', 10)
        self.grip_pub = self.create_publisher(Bool, '/gripper/command', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.sub = self.create_subscription(String, '/mission', self.mission_cb, 10)
        self.timer = self.create_timer(1.0, self.run)

    def mission_cb(self, msg):
        if msg.data == 'pick_and_place':
            self._go('PICKUP')
            self.get_logger().info('Pick-and-place mission started.')
        else:
            self.get_logger().warn(f'Unknown mission "{msg.data}" (try pick_and_place)')

    def _go(self, state):
        self.state = state
        self.ticks = 0
        self.state_pub.publish(String(data=state))
        self.get_logger().info(f'mission state: {state}')

    def run(self):
        s = self.state
        if s == 'PICKUP':
            self.cmd_pub.publish(Twist())
            self.arm_pub.publish(REACH_DOWN)
            self._go('GRAB')
        elif s == 'GRAB':
            self.grip_pub.publish(Bool(data=True))
            self._go('LIFT')
        elif s == 'LIFT':
            self.arm_pub.publish(CARRY)
            self._go('DELIVER')
        elif s == 'DELIVER':
            turn = Twist()
            turn.angular.z = DELIVER_TURN_RATE
            self.cmd_pub.publish(turn)
            self.ticks += 1
            if self.ticks >= DELIVER_TICKS:
                self.cmd_pub.publish(Twist())
                self._go('PLACE')
        elif s == 'PLACE':
            self.arm_pub.publish(REACH_DOWN)
            self._go('RELEASE')
        elif s == 'RELEASE':
            self.grip_pub.publish(Bool(data=False))
            self.arm_pub.publish(CARRY)
            self._go('DONE')


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
