#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from std_msgs.msg import String

class WaypointPubSub(Node):

    def __init__(self):
        super().__init__('waypoint_pub_sub')
        self.subscription = self.create_subscription(
            PoseArray,
            '/vrx/wayfinding/waypoints',
            self.listener_callback,
            10)
        self.subscription
        self.publisher_ = self.create_publisher(String, '/gateway_msgs', 10)

    def listener_callback(self, msg):
        new_msg = String()
        inner_string = "WAYPOINTS="
        for i, pose in enumerate(msg.poses):
            inner_string += f"{pose.position.y},{pose.position.x}"
            if i < len(msg.poses) - 1:
                inner_string += ":"
        new_msg.data = inner_string
        self.publisher_.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    pub_sub = WaypointPubSub()
    rclpy.spin(pub_sub)
    pub_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
