#!/usr/bin/env python3
import rclpy
import time
import math
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Int64
from sensor_msgs.msg import Imu

from asv_interfaces.msg import ControlMessage

class PID:
    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d

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
        self.i = ControlMessage()
        self.i.vx = 0.0
        self.i.vy = 0.0
        self.i.angular = 0.0
        self.i.use_heading = False

        self.r = ((l ** 2 + w ** 2) / 2 - l * w) ** 0.5 / 2

        self.linear_factor = 1 # units (kg/s) arbitrary conversion between linear velocity and thrust, determined experimentally
        self.angular_factor = 0.32 if in_sim else 1 # units (kgm^2/s) arbitrary conversion between angular velocity and thrust, determined experimentally
       
        self.pid_omega = PID(10, 0, 0)
        self.pid_theta = PID(1.0, 0.0005, 0.5)

        self.max_angular_velocity = 1

        self.actual_omega = 0
        self.actual_theta = 0

        self.accumulated_error = 0
        self.last_error = None

        self.used_heading_last = False
        self.last_update_timestamp = None
        self.last_imu_data_timestamp = None
        self.required_imu_data_recentness = 1

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
            ControlMessage, "/control_input", self.update_control_input, 10
        )
        self.create_subscription(
            Imu, "/wamv/sensors/imu/imu/data", self.update_heading, 10
        )
        
        self.thrust_publishers = {}
        for thruster_prefix in self.all_thruster_names:
            self.thrust_publishers[thruster_prefix] = self.create_publisher(
                self.msg_type, thruster_prefix, 10
            )

        timer_period = 1/100  # seconds
        self.timer = self.create_timer(timer_period, self.update_thrust)

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
        self.actual_omega = msg.angular_velocity.z
        self.actual_theta = R.from_quat([
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ]).as_euler('xyz')[2]
        self.last_imu_data_timestamp = time.time()

    def update_thrust(self):
        print(self.actual_theta if self.i.use_heading else self.actual_omega)
        current_time = time.time()
        angular_input = 0 if self.i.use_heading else self.i.angular
        if self.last_imu_data_timestamp is not None and (current_time - self.last_imu_data_timestamp) <= self.required_imu_data_recentness:
            if self.i.use_heading:
                pid = self.pid_theta
                diff = (self.i.angular - self.actual_theta) % (2 * math.pi)
                if diff > math.pi:
                    diff -= 2 * math.pi
            else:
                pid = self.pid_omega
                diff = self.i.angular - self.actual_omega
            angular_input += pid.p * diff

            if self.last_update_timestamp is not None:
                dt = current_time - self.last_update_timestamp

                if self.about_zero(diff) or (self.accumulated_error > 0) != (diff > 0):
                    self.accumulated_error = 0
                self.accumulated_error += diff * dt

                d_comp = 0 if self.last_error is None else (diff - self.last_error) / dt

                angular_input += pid.i * self.accumulated_error + pid.d * d_comp
            
            self.last_error = diff

        results = self.get_thrust_values(self.i.vx, self.i.vy, self.restrict_input(angular_input, self.max_angular_velocity))

        for name in self.all_thruster_names:
            float_msg = self.msg_type()
            float_msg.data = self.py_type(self.restrict_input(
                results[name], self.max_input
            ) * self.thrust_factor + self.midpoint)
            self.thrust_publishers[name].publish(float_msg)

        self.last_update_timestamp = current_time

    def restrict_input(self, input, max_val):
        if input < -max_val:
            return -max_val
        if input > max_val:
            return max_val
        return input
    
    def about_zero(self, val):
        return abs(val) < 0.001

    def update_control_input(self, msg):
        self.i = msg
        if msg.use_heading != self.used_heading_last:
            self.accumulated_error = 0
            self.last_error = None
        self.used_heading_last = msg.use_heading

def main(args=None):
    rclpy.init(args=args)

    controller = Controller(True)

    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
