#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.sub = self.create_subscription(Bool, '/gripper/command', self.cb, 10)
        self.pub = self.create_publisher(String, '/gripper/state', 10)
    def cb(self, msg):
        s = 'GRABBED' if msg.data else 'RELEASED'
        self.pub.publish(String(data=s))
        self.get_logger().info(s)

def main():
    rclpy.init()
    node = GripperController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
