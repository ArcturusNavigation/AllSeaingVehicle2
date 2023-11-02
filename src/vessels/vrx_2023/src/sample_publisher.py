#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

class Pub(Node):

    def __init__(self):
        super().__init__('sample_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/vrx/xytheta_velocities', 10)
        self.declare_parameter('x_vel', 0.0)
        self.declare_parameter('y_vel', 0.0)
        self.declare_parameter('theta_vel', 0.0)

        self.x_vel = float(self.get_parameter('x_vel').value)
        self.y_vel = float(self.get_parameter('y_vel').value)
        self.theta_vel = float(self.get_parameter('theta_vel').value)

        timer_period = 1/60  # seconds
        self.timer = self.create_timer(timer_period, self.cb)
    
    def cb(self):
        msg = Float64MultiArray()
        msg.data = [self.x_vel, self.y_vel, self.theta_vel]
        self.publisher_.publish(msg)
            


def main(args=None):
    rclpy.init(args=args)

    pub = Pub()

    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()