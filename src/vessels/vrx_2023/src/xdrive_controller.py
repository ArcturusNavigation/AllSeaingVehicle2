#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from sensor_msgs.msg import Imu

class Controller(Node):
    def __init__(self, in_sim = False):
        super().__init__("xdrive_controller")
        l = 3.5 # BOAT LENGTH
        w = 2 # BOAT WIDTH
        self.msg_type = Float64 if in_sim else Int64
        self.py_type = float if in_sim else int
        min_output = -1000 if in_sim else 1100
        max_output = 1000 if in_sim else 1900
        self.max_input = 1.1
        self.thrust_factor = (max_output - min_output) / (2 * self.max_input)
        self.midpoint = (max_output + min_output) / 2
        self.velocities = [0] * 3

        self.r = ((l ** 2 + w ** 2) / 2 - l * w) ** 0.5 / 2

        self.linear_factor = 1 # units (kg/s) arbitrary conversion between linear velocity and thrust, determined experimentally
        self.angular_factor = 0.32 if in_sim else 1 # units (kgm^2/s) arbitrary conversion between angular velocity and thrust, determined experimentally
        self.kptheta = 300
        self.kitheta = 0

        self.accumulated_error = 0
        self.timestamp = time.time()

        if in_sim:
            self.front_left_name = "/wamv/thrusters/front_left/thrust"
            self.front_right_name = "/wamv/thrusters/front_right/thrust"
            self.back_left_name = "/wamv/thrusters/back_left/thrust"
            self.back_right_name = "/wamv/thrusters/back_right/thrust"
        else:
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
            Imu, "/wamv/sensors/imu/imu/data", self.update_heading, 10
        )
        self.thrust_publishers = {}
        for thruster_prefix in self.all_thruster_names:
            self.thrust_publishers[thruster_prefix] = self.create_publisher(
                self.msg_type, thruster_prefix, 10
            )

    def get_thrust_values(self, tx, ty, tn):
        d = 2 ** (3 / 2)
        ang = tn * self.angular_factor / (4 * self.r)
        return {
            self.back_right_name: (tx - ty) * self.linear_factor / d + ang,
            self.back_left_name: (ty + tx) * self.linear_factor / d - ang,
            self.front_left_name: (tx - ty) * self.linear_factor / d - ang,
            self.front_right_name: (ty + tx) * self.linear_factor / d + ang,
        }

    def update_heading(self, msg):
        omega = msg.angular_velocity.z
        print(omega)
        tx, ty, tn = self.velocities

        omega_diff = (tn - omega) % (2 * math.pi)
        if omega_diff > math.pi:
            omega_diff -= 2 * math.pi
        
        current_time = time.time()
        self.accumulated_error += omega_diff * (current_time - self.timestamp)
        self.timestamp = current_time
        
        no_feedback_results = self.get_thrust_values(tx, ty, tn)
        feedback_results = self.get_thrust_values(0, 0, self.kptheta * omega_diff + self.kitheta * self.accumulated_error)

        for name in self.all_thruster_names:
            float_msg = self.msg_type()
            float_msg.data = self.py_type(self.restrict_input(
                no_feedback_results[name] + feedback_results[name]
            ) * self.thrust_factor + self.midpoint)
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

    controller = Controller(True)

    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
