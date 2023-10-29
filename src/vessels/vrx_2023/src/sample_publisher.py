#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

class Pub(Node):

    def __init__(self):
        super().__init__('sample_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/vrx/xytheta_velocities', 10)
       
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.cb)
    
    def cb(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 1.0]
        self.publisher_.publish(msg)
            


def main(args=None):
    rclpy.init(args=args)

    pub = Pub()

    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()