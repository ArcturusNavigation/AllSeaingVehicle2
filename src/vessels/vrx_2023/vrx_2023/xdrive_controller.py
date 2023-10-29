#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String


class Controller(Node):
    def __init__(self):
        super().__init__("xdrive_controller")
        l = 3.5
        w = 2
        max_thrust = 1000
        max_speed = 5
        self.thrust_factor = max_thrust / max_speed

        self.r = ((l ** 2 + w ** 2) / 2 - l * w) ** 0.5 / 2
        self.all_thruster_names = (
            "front_left",
            "front_right",
            "back_left",
            "back_right",
        )

        self.subscription = self.create_subscription(
            Float64MultiArray, "/vrx/xytheta_velocities", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.thrust_publishers = {}
        for thruster_prefix in self.all_thruster_names:
            self.thrust_publishers[thruster_prefix] = self.create_publisher(
                Float64, f"/wamv/thrusters/{thruster_prefix}/thrust", 10
            )

    def listener_callback(self, msg):
        tx, ty, tn = msg.data
        d = 2 ** (3 / 2)
        results = {
            "back_right": (tx - ty) / d + tn / (4 * self.r),
            "back_left": (ty + tx) / d - tn / (4 * self.r),
            "front_left": (tx - ty) / d - tn / (4 * self.r),
            "front_right": (ty + tx) / d + tn / (4 * self.r),
        }
        for name in self.all_thruster_names:
            float_msg = Float64()
            float_msg.data = results[name] * self.thrust_factor
            self.thrust_publishers[name].publish(float_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
