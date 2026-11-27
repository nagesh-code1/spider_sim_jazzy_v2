#!/usr/bin/env python3
"""Arm IK node: /arm_target (Point) -> /arm_controller/commands.

FIX: the node used to be called 'arm_controller', the same name as the
ros2_control controller node, which gave two ROS nodes with one name.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from .arm_kinematics import inverse_kinematics

YAW_LIMIT = 3.14


class ArmIK(Node):
    def __init__(self):
        super().__init__('arm_ik')
        self.pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.sub = self.create_subscription(Point, '/arm_target', self.target_cb, 10)

    def target_cb(self, msg):
        shoulder, elbow = inverse_kinematics(msg.x, msg.z)
        yaw = max(-YAW_LIMIT, min(YAW_LIMIT, msg.y))
        cmd = [yaw, shoulder, elbow]
        self.pub.publish(Float64MultiArray(data=cmd))
        self.get_logger().info(
            f'arm target x={msg.x:.2f} z={msg.z:.2f} yaw={yaw:.2f} -> '
            f'shoulder={shoulder:.2f} elbow={elbow:.2f}')


def main():
    rclpy.init()
    node = ArmIK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
