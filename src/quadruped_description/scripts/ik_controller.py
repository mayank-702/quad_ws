#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

L1 = 0.15
L2 = 0.18

def leg_ik(x, z):
    D = (x*x + z*z - L1*L1 - L2*L2) / (2 * L1 * L2)
    D = max(min(D, 1.0), -1.0)
    val = max(0.0, 1.0 - D * D)
    knee = math.atan2(-math.sqrt(val), D)
    thigh = math.atan2(z, x) - math.atan2(L2 * math.sin(knee), L1 + L2 * math.cos(knee))
    return thigh, knee


class IKController(Node):

    def __init__(self):
        super().__init__('ik_controller')
        print("🔥 IK NODE STARTED")

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        self.start_time = time.time()

        # ── TUNABLE GAIT PARAMETERS ──────────────────────────────────────
        self.gait_freq   = 1.6    # was 0.8 → doubled = twice as fast
        self.step_length = 0.10   # was 0.06 → longer stride = more ground covered
        self.step_height = 0.05   # was 0.04 → slightly higher lift to clear ground
        self.base_z      = 0.24   # keep the same

        # Hip abduction offsets
        self.hip_fl = -0.09
        self.hip_fr =  0.09
        self.hip_rl = -0.09
        self.hip_rr =  0.09
        # ─────────────────────────────────────────────────────────────────

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

    def leg_motion(self, phase):
        s = math.sin(phase)
        c = math.cos(phase)
        half = self.step_length / 2.0

        if s >= 0:
            x = -c * half
            z = self.base_z + self.step_height * s
        else:
            x = -c * half
            z = self.base_z

        return leg_ik(x, z)

    def update(self):
        t     = time.time() - self.start_time
        phase = 2.0 * math.pi * self.gait_freq * t

        t1, k1 = self.leg_motion(phase)
        t2, k2 = self.leg_motion(phase + math.pi)

        msg = Float64MultiArray()
        msg.data = [
            self.hip_fl, t1, k1,
            self.hip_fr, t2, k2,
            self.hip_rl, t2, k2,
            self.hip_rr, t1, k1,
        ]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = IKController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
