#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import ParamVec
from nav_msgs.msg import Odometry 

class AcousticTrackingSub(Node):
    def __init__(self):
        super().__init__('acoustic_tracking_sub')
        self.pinger_subscription = self.create_subscription(
            ParamVec,
            '/wamv/sensors/acoustics/receiver/range_bearing',
            self.pinger_callback,
            10)
        self.odometry_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)

    def pinger_callback(self, msg):
        self.get_logger().info('Pinger heard: "%s"' % msg.params[0])

    def odom_callback(self, msg):
        self.get_logger().info('Odom heard x position: "%s"' % msg.pose.pose.position.x)

def main(args=None):
    rclpy.init(args=args)
    node = AcousticTrackingSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
