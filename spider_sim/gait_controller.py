#!/usr/bin/env python3
"""Diagonal-pair trot gait: /cmd_vel (Twist) -> /leg_controller/commands.

Leg order (matches config/controllers.yaml): fl, fr, rl, rr; each leg is
[coxa, femur, tibia]. Legs are splayed at +-45 / +-135 deg in the URDF.

FIXES vs the original:
  * real diagonal pairs (fl+rr, fr+rl) instead of left pair / right pair
  * the foot is moved fore/aft with the coxa (hip yaw) joint, the femur only
    lifts it, so the robot actually walks and turns
  * when stopped it publishes the stand pose for a moment and then goes quiet,
    so manual `ros2 topic pub /leg_controller/commands ...` is not overwritten
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

STAND = [0.0, 0.8, 1.1]           # coxa, femur, tibia (rad)
MAX_VX = 0.10                      # m/s accepted on /cmd_vel
MAX_WZ = 0.8                       # rad/s accepted on /cmd_vel
STRIDE = 0.35                      # max coxa swing (rad) at full command
LIFT = 0.40                        # femur lift during swing (rad)
STEP_HZ = 1.0                      # gait cycles per second (sim time)
RATE_HZ = 50.0
IDLE_HOLD_TICKS = int(RATE_HZ)     # keep sending stand pose for 1 s after stopping
COXA_LIMIT = 0.8

# +1/-1: which coxa direction moves this foot forward (from the leg yaw)
FORWARD_SIGN = [-1.0, 1.0, -1.0, 1.0]   # fl, fr, rl, rr
PHASE_OFFSET = [0.0, math.pi, math.pi, 0.0]  # diagonal pairs: fl+rr, fr+rl


class GaitController(Node):
    def __init__(self):
        super().__init__('gait_controller')
        self.pub = self.create_publisher(Float64MultiArray, '/leg_controller/commands', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.update)
        self.vx = 0.0
        self.wz = 0.0
        self.phase = 0.0
        self.idle_ticks = 0

    def cmd_cb(self, msg):
        self.vx = max(-MAX_VX, min(MAX_VX, msg.linear.x))
        self.wz = max(-MAX_WZ, min(MAX_WZ, msg.angular.z))

    def _moving(self):
        return abs(self.vx) >= 0.005 or abs(self.wz) >= 0.02

    def update(self):
        if not self._moving():
            self.phase = 0.0
            if self.idle_ticks < IDLE_HOLD_TICKS:
                self.idle_ticks += 1
                self.pub.publish(Float64MultiArray(data=STAND * 4))
            return

        self.idle_ticks = 0
        self.phase = (self.phase + 2.0 * math.pi * STEP_HZ / RATE_HZ) % (2.0 * math.pi)
        fwd = STRIDE * self.vx / MAX_VX
        turn = STRIDE * self.wz / MAX_WZ
        cmd = []
        for leg in range(4):
            phi = self.phase + PHASE_OFFSET[leg]
            # x: -1 -> +1 while the foot is in the air, +1 -> -1 on the ground
            x = -math.cos(phi)
            coxa = STAND[0] + FORWARD_SIGN[leg] * fwd * x + turn * x
            coxa = max(-COXA_LIMIT, min(COXA_LIMIT, coxa))
            femur = STAND[1] - LIFT * max(0.0, math.sin(phi))
            cmd += [coxa, femur, STAND[2]]
        self.pub.publish(Float64MultiArray(data=cmd))


def main():
    rclpy.init()
    node = GaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
