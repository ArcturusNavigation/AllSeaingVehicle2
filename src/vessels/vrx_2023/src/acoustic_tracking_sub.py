#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import ParamVec
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point, Pose
import math
from asv_interfaces.msg import ASVState

class AcousticTrackingSub(Node):
    def __init__(self):
        pinger_bearing = None
        pinger_range = None
        nav_x = 0
        nav_y = 0
        heading = 0
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
        orientation_angle = math.radians(heading)
        x_coord = nav_x
        y_coord = nav_y
        bearing = self.pinger_bearing
        pinger_range = self.pinger_range
        new_x_coord = x_coord + pinger_range*math.sin(bearing+orientation_angle)
        new_y_coord = y_coord + pinger_range*math.cos(bearing+orientation_angle)
        point = Point(new_x_coord, new_y_coord,0)
        self.pinger_coord_pub.publish(point)

    def pinger_callback(self, msg):
        self.pinger_bearing = msg.params[1].value.double_value
        self.pinger_range = msg.params[2].value.double_value       

    def odom_callback(self, msg):
        self.nav_x = msg.nav_x
        self.nav_y = msg.nav_y
        self.heading = msg.heading



def main(args=None):
    rclpy.init(args=args)
    node = AcousticTrackingSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
