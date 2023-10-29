#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from perception_suite.pcl_utils import *

class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_listener')

        self.pcd_subscriber = self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/lidar_wamv_sensor/transformed_points', # TODO: CHANGE THIS TO A PARAMETER
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: PointCloud2):
        print("transformed_points:", read_points_list(msg)[:5000], '\n')

def main(args=None):
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)
    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()