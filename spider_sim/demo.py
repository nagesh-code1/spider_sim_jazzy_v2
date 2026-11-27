#!/usr/bin/env python3
"""Scripted ~2 minute demo (sim time): stand, walk, turn, arm moves, mission.

    ros2 run spider_sim demo --ros-args -p use_sim_time:=true

Exits by itself when the script is finished.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point

# (start_s, label, vx, wz) — the robot drives with these until the next entry
DRIVE_PLAN = [
    (0.0, 'stand', 0.0, 0.0),
    (6.0, 'walk forward', 0.10, 0.0),
    (26.0, 'stop', 0.0, 0.0),
    (30.0, 'turn left', 0.0, 0.6),
    (37.0, 'stop', 0.0, 0.0),
    (62.0, 'walk forward', 0.10, 0.0),
    (82.0, 'turn right', 0.0, -0.6),
    (92.0, 'walk forward', 0.10, 0.0),
    (100.0, 'stop', 0.0, 0.0),
]
# (time_s, arm target x, yaw, z)
ARM_PLAN = [
    (40.0, 0.20, 0.8, 0.05),
    (42.0, 0.20, -0.8, 0.05),
    (44.0, 0.22, 0.0, 0.0),
    (104.0, 0.16, 1.2, 0.10),
    (107.0, 0.16, -1.2, 0.10),
    (110.0, 0.22, 0.0, 0.0),
]
MISSION_AT = 47.0
MISSION_END = 61.0   # mission_manager owns /cmd_vel in between
END_AT = 120.0


class Demo(Node):
    def __init__(self):
        super().__init__('spider_demo')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Point, '/arm_target', 10)
        self.mission_pub = self.create_publisher(String, '/mission', 10)
        self.t0 = None
        self.label = None
        self.arm_done = 0
        self.mission_sent = False
        self.finished = False
        self.timer = self.create_timer(0.1, self.tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        now = self._now()
        if now <= 0.0:
            return  # waiting for /clock
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('demo started')
        t = now - self.t0

        if t >= END_AT:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('demo finished')
            self.finished = True
            return

        # during the mission the mission_manager owns /cmd_vel
        in_mission = MISSION_AT <= t < MISSION_END
        active = [p for p in DRIVE_PLAN if p[0] <= t][-1]
        if active[1] != self.label:
            self.label = active[1]
            self.get_logger().info(f't={t:5.1f}s  {self.label}')
        if not in_mission:
            cmd = Twist()
            cmd.linear.x = active[2]
            cmd.angular.z = active[3]
            self.cmd_pub.publish(cmd)

        while self.arm_done < len(ARM_PLAN) and ARM_PLAN[self.arm_done][0] <= t:
            _, x, yaw, z = ARM_PLAN[self.arm_done]
            self.arm_pub.publish(Point(x=x, y=yaw, z=z))
            self.get_logger().info(f't={t:5.1f}s  arm -> x={x} yaw={yaw} z={z}')
            self.arm_done += 1

        if not self.mission_sent and t >= MISSION_AT:
            self.mission_pub.publish(String(data='pick_and_place'))
            self.get_logger().info(f't={t:5.1f}s  mission pick_and_place')
            self.mission_sent = True


def main():
    rclpy.init()
    node = Demo()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
