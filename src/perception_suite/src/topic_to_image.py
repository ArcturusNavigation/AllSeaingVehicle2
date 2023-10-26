#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from perception_suite.pcl_utils import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()
class PCDListener(Node):

    def __init__(self):
        super().__init__('image_listener')

        self.img_sub = self.create_subscription(
            Image, 
            '/zed2i/zed_node/rgb/image_rect_color', 
            self.img_callback
        )

    def img_callback(self, msg: Image):
        
        # Converts a ROS Image topic to a photo (png, jpeg etc) and save it a folder "data" on your machine
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        bridge.imwrite('./data/current_image.png', cv_image)

def main(args=None):
    rclpy.init(args=args) 
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)
    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()