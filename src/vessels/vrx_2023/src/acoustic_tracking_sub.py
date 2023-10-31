#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import ParamVec
from geometry_msgs.msg import Point
from asv_interfaces.msg import ASVState

import math

class AcousticTrackingSub(Node):
    def __init__(self):
        self.pinger_bearing = 0
        self.pinger_range = 0
        self.pinger_elevation = 0
        self.nav_x = 0
        self.nav_y = 0
        self.heading = 0
        super().__init__('acoustic_tracking_sub')
        timer_period = 1/60
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pinger_subscription = self.create_subscription(
            ParamVec,
            '/wamv/sensors/acoustics/receiver/range_bearing',
            self.pinger_callback,
            10)

        self.odometry_subscription = self.create_subscription(
            ASVState,
            '/allseaing_main/state',
            self.odom_callback,
            10)
        self.pinger_coord_pub = self.create_publisher(Point, '/pinger_coord', 10)

    def timer_callback(self):
        orientation_angle = math.radians(self.heading)
        x_coord = self.nav_x
        y_coord = self.nav_y
        bearing = self.pinger_bearing
        pinger_radius = self.pinger_range * math.cos(self.pinger_elevation)
        new_x_coord = x_coord + pinger_radius * math.sin(bearing + orientation_angle)
        new_y_coord = y_coord + pinger_radius * math.cos(bearing + orientation_angle)
        point = Point()
        point.x = new_x_coord
        point.y = new_y_coord
        self.pinger_coord_pub.publish(point)

    def pinger_callback(self, msg):
        for element in msg.params:
            if element.name == "bearing":
                self.pinger_bearing = -element.value.double_value
            if element.name == "range":
                self.pinger_range = element.value.double_value
            if element.name == "elevation":
                self.pinger_elevation = element.value.double_value                

    def odom_callback(self, msg):
        self.nav_x = msg.nav_x
        self.nav_y = msg.nav_y
        self.heading = msg.nav_heading

def main(args=None):
    rclpy.init(args=args)
    node = AcousticTrackingSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
