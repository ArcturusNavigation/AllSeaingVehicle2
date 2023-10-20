#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from protobuf_client_interfaces.msg import Gateway

class WaypointPubSub(Node):

    def __init__(self):
        super().__init__('waypoint_pub_sub')
        self.subscription = self.create_subscription(
            PoseArray,
            '/vrx/wayfinding/waypoints',
            self.listener_callback,
            10)
        self.subscription
        self.publisher = self.create_publisher(Gateway, '/send_to_gateway', 10)

    def listener_callback(self, msg):
        waypt_msg = Gateway()
        inner_string = ""
        for i, pose in enumerate(msg.poses):
            inner_string += f"{pose.position.x},{pose.position.y}"
            if i < len(msg.poses) - 1:
                inner_string += ":"
        waypt_msg.gateway_key = "WPT_UPDATE_GPS"
        waypt_msg.gateway_string = inner_string
        self.get_logger().info('Publishing: "%s"' % waypt_msg.gateway_string)
        self.publisher.publish(waypt_msg)

def main(args=None):
    rclpy.init(args=args)
    pub_sub = WaypointPubSub()
    rclpy.spin(pub_sub)
    pub_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
