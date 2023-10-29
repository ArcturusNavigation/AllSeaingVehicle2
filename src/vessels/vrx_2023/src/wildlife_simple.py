#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from protobuf_client_interfaces.msg import Gateway

class WildlifeSimple(Node):

    def __init__(self):
        super().__init__('stationkeeping')
        self.animal1_sub = self.create_subscription(
            PoseStamped,
            '/vrx/wildlife/animal0/pose',
            lambda msg: self.pose_callback(msg, 0),
            10)
        self.animal2_sub = self.create_subscription(
            PoseStamped,
            '/vrx/wildlife/animal1/pose',
            lambda msg: self.pose_callback(msg, 1),
            10)
        self.animal3_sub = self.create_subscription(
            PoseStamped,
            '/vrx/wildlife/animal2/pose',
            lambda msg: self.pose_callback(msg, 2),
            10)
        self.publisher = self.create_publisher(Gateway, '/send_to_gateway', 10)

    def pose_callback(self, msg, num):
        waypt_msg = Gateway()
        waypt_msg.gateway_key = "ANIMAL_GPS"
        waypt_msg.gateway_string = f"id={num},animal={msg.header.frame_id},lat={msg.pose.position.x},lon={msg.pose.position.y}"
        self.get_logger().info('Publishing: "%s"' % waypt_msg.gateway_string)
        self.publisher.publish(waypt_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WildlifeSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
