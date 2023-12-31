#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from asv_interfaces.msg import ControlMessage

class Pub(Node):

    def __init__(self):
        super().__init__('sample_publisher')
        self.publisher_ = self.create_publisher(ControlMessage, '/control_input', 10)
        self.declare_parameter('vx', 0.0)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('use_heading', False)
        self.declare_parameter('angular', 0.0)

        self.msg = ControlMessage()
        self.msg.vx = float(self.get_parameter('vx').value)
        self.msg.vy = float(self.get_parameter('vy').value)
        self.msg.angular = float(self.get_parameter('angular').value)
        self.msg.use_heading = bool(self.get_parameter('use_heading').value)

        timer_period = 1/60  # seconds
        self.timer = self.create_timer(timer_period, self.cb)
    
    def cb(self):
        self.publisher_.publish(self.msg)
            


def main(args=None):
    rclpy.init(args=args)

    pub = Pub()

    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()