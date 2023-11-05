"""
References:
https://pytorch.org/hub/ultralytics_yolov5/
https://pypi.org/project/yolo5/
"""

import getpass
import torch
import cv_bridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from perception_suite.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
from utility.constants import IMG_WIDTH, IMG_HEIGHT
import time

class WriteBagToPNG(Node):

    def __init__(self):
        super().__init__('write_bag_to_png')
        
        self.bridge = cv_bridge.CvBridge()

        # Subscribers and publishers
        qos_profile = QoSProfile(depth=1)
        
        self.img_sub = self.create_subscription(
            Image, 

            # TODO Change topic to the camera topic thats in the simulator (look at vrx wiki)
            '/zed2i/zed_node/rgb/image_rect_color', 
            self.img_callback, 
            qos_profile 
        )

        self.SAMPLE_RATE = 1 # saves 1 image every SAMPLE_RATE seconds

    def img_callback(self, img):
        
        # img is a general ros message type. write the code below to convert this message to a png/jpeg
        # that's saved on the computer it's running on. Place the files in a directory called "data"
        # that's inside the vessels/vrx_2023 directory.
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)

        else:
            #1) Conversion from img to png/jpeg
            cv2.imwrite('AllSeaingVehicle2/src/vessels/vrx_2023/data/camera_image.jpeg', cv2_img)

        #2) Make sure you only save images every "n" seconds
        time.sleep(SAMPLE_RATE) # TODO unsure if this kind of delay will do Bad Things 


        
        #3) TODO *Bonus: Take in joy information so that images are only recorded when button is pressed. 
        # needs to subscribe to a joystick topic
        

def main(args=None):
    rclpy.init(args=args)
    detector = WriteBagToPNG()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
