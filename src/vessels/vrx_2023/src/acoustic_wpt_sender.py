#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from protobuf_client_interfaces.msg import Gateway

class AcousticWPTSender(Node):

    def __init__(self):
        super().__init__('acoustic_wpt_sender')
        self.subscription = self.create_subscription(
            Point,
            '/pinger_coord',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Gateway, '/send_to_gateway', 10)

    def listener_callback(self, msg: Point):
        waypt_msg = Gateway()
        waypt_msg.gateway_key = "WPT_UPDATE"
        waypt_msg.gateway_string = f"points={msg.x},{msg.y}"
        self.get_logger().info('Publishing: "%s"' % waypt_msg.gateway_string)
        self.publisher.publish(waypt_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AcousticWPTSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
