#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from protobuf_client_interfaces.msg import Gateway

class StationKeeping(Node):

    def __init__(self):
        super().__init__('stationkeeping')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrx/stationkeeping/goal',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Gateway, '/send_to_gateway', 10)

    def listener_callback(self, msg: PoseStamped):
        waypt_msg = Gateway()
        waypt_msg.gateway_key = "WPT_UPDATE_GPS"
        waypt_msg.gateway_string = f"{msg.pose.position.x},{msg.pose.position.y}"
        self.get_logger().info('Publishing: "%s"' % waypt_msg.gateway_string)
        self.publisher.publish(waypt_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StationKeeping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
