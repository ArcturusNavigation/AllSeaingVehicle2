#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int64
from asv_interfaces.msg import ASVState

class Controller(Node):
    def __init__(self):
        super().__init__("xdrive_controller")
        l = 3.5
        w = 2
        self.max_output = 1000
        self.max_input = 1.1
        self.thrust_factor = self.max_output / self.max_input
        self.velocities = [0] * 3
        self.target_heading = None
        self.theta_kp = 6.1
        self.timestamp = time.time()
        self.previous_rotation = 0

        self.r = ((l ** 2 + w ** 2) / 2 - l * w) ** 0.5 / 2

        self.front_left_name = "frontleft_pwm"
        self.front_right_name = "frontright_pwm"
        self.back_left_name = "backleft_pwm"
        self.back_right_name = "backright_pwm"
        self.all_thruster_names = (
            self.front_left_name,
            self.front_right_name,
            self.back_left_name,
            self.back_right_name
        )

        self.create_subscription(
            Float64MultiArray, "/vrx/xytheta_velocities", self.update_velocities, 10
        )
        self.create_subscription(
            ASVState, "/allseaing_main/state", self.update_heading, 10
        )
        self.thrust_publishers = {}
        for thruster_prefix in self.all_thruster_names:
            self.thrust_publishers[thruster_prefix] = self.create_publisher(
                Int64, thruster_prefix, 10
            )

    def update_heading(self, msg):
        heading = -msg.nav_heading * math.pi/180
        if self.target_heading is None:
            # print(f"got initial heading = {heading}")
            self.target_heading = heading # first time initialization
        tx, ty, tn = self.velocities
        d = 2 ** (3 / 2)
        current_time = time.time()
        dt = current_time - self.timestamp
        self.timestamp = current_time
        theta_diff = 0
        if self.target_heading is not None:
            self.target_heading += self.previous_rotation * dt
            theta_diff = (self.target_heading - heading) % (2 * math.pi)
            if theta_diff > math.pi:
                theta_diff -= 2 * math.pi
            # print(f"theta error: {theta_diff:3.3f}, theta = {heading:3.3f}, desired = {self.target_heading:3.3f}")
        self.previous_rotation = tn
        tn += self.theta_kp * theta_diff
        tx, ty, tn = (self.restrict_input(inp) for inp in (tx, ty, tn))
        results = {
            self.back_right_name: (tx - ty) / d + tn / (4 * self.r),
            self.back_left_name: (ty + tx) / d - tn / (4 * self.r),
            self.front_left_name: (tx - ty) / d - tn / (4 * self.r),
            self.front_right_name: (ty + tx) / d + tn / (4 * self.r),
        }
        for name in self.all_thruster_names:
            float_msg = Int64()
            float_msg.data = int(results[name] * self.thrust_factor)
            self.thrust_publishers[name].publish(float_msg)

    def restrict_input(self, input):
        if input < -self.max_input:
            return -self.max_input
        if input > self.max_input:
            return self.max_input
        return input

    def update_velocities(self, msg):
        self.velocities = msg.data

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
