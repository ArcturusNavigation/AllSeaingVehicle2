#!/usr/bin/env python3
import rclpy
import cv_bridge 
import cv2
from asv_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from utility.geometry import Vec2D # TODO: Perhaps reimplement Vec2D
from utility.constants import BUOY_CLASSES, IMG_WIDTH, IMG_HEIGHT # TODO: CHANGE TO PARAMETERS

from rclpy.node import Node

from std_msgs.msg import String


class BuoyDetector(Node):

    def __init__(self):
        super().__init__('buoy_detector')
        self.bridge = cv_bridge.CvBridge()
        self.bbox_mins = {}
        self.bbox_maxes = {}
        self.num_avg = 5
        self.centers = [Vec2D(
            IMG_WIDTH / 2,
            3 * IMG_HEIGHT / 4
        )]
        self.heading_error = 0

        # Subscribers
        self.bbox_sub = self.create_subscription(
            LabeledBoundingBox2DArray, 
            "/perception_suite/bounding_boxes", 
            self.bbox_callback, 
            10
        )
        self.img_sub = self.create_subscription(
            Image,
            "/zed2i/zed_node/rgb/image_rect_color",
            self.img_callback,
            10
        )
        
        # Publishers
        self.bbox_pub = self.create_publisher(
            LabeledBoundingBox2DArray,
            "/perception_suite/buoy_boxes",
            10
        )
        self.img_pub = self.create_publisher(
            Image,
            "/perception_suite/buoy_image",
            10
        )
        self.center_pub = self.create_publisher(
            Float64,
            "/perception_suite/buoy_center",
            10
        )
        self.buoy_heading_pub = self.create_publisher(
            Float32MultiArray,
            "/perception_suite/buoy_heading_error",
            10
        )

    def avg_center(self):
        center = Vec2D()
        for vec in self.centers:
            center += vec
        center /= len(self.centers)
        return center.to_int()

    def bbox_callback(self, bboxes):
        self.bbox_mins = {} 
        self.bbox_maxes = {}
        for bbox in bboxes.boxes:
            if bbox.label in BUOY_CLASSES.values():
                if bbox.label not in self.bbox_mins:
                    self.bbox_mins[bbox.label] = Vec2D(bbox.min_x, bbox.min_y)
                    self.bbox_maxes[bbox.label] = Vec2D(bbox.max_x, bbox.max_y)
                    continue
                
                self.bbox_mins[bbox.label].x = min(bbox.min_x, self.bbox_mins[bbox.label].x)
                self.bbox_mins[bbox.label].y = min(bbox.min_y, self.bbox_mins[bbox.label].y)
                self.bbox_maxes[bbox.label].x = max(bbox.max_x, self.bbox_maxes[bbox.label].x)
                self.bbox_maxes[bbox.label].y = max(bbox.max_y, self.bbox_maxes[bbox.label].y)
                
        new_bboxes = LabeledBoundingBox2DArray()
        new_bboxes.header.stamp = rclpy.Time.now()
        for label in self.bbox_mins:
            labeled_bbox = LabeledBoundingBox2D()
            labeled_bbox.label = label
            labeled_bbox.min_x = self.bbox_mins[label].x
            labeled_bbox.min_y = self.bbox_mins[label].y
            labeled_bbox.max_x = self.bbox_maxes[label].x
            labeled_bbox.max_y = self.bbox_maxes[label].y
            new_bboxes.boxes.append(labeled_bbox)

        # Accumulate center positions
        reds = 0
        if BUOY_CLASSES["EAST"] in self.bbox_maxes:
            reds = self.bbox_maxes[BUOY_CLASSES["EAST"]]
        elif BUOY_CLASSES["RED"] in self.bbox_maxes:
            reds = self.bbox_maxes[BUOY_CLASSES["RED"]]
        greens = IMG_WIDTH
        if BUOY_CLASSES["WEST"] in self.bbox_mins:
            greens = self.bbox_mins[BUOY_CLASSES["WEST"]]
        elif BUOY_CLASSES["GREEN"] in self.bbox_mins:
            greens = self.bbox_mins[BUOY_CLASSES["GREEN"]]
           
        center = (reds + greens) / 2
        self.centers.append(center)
        if len(self.centers) > self.num_avg:
            self.centers.pop(0)

        self.bbox_pub.publish(new_bboxes)

    def img_callback(self, img):
        try:
            img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        except cv_bridge.CvBridgeError as e:
            self.get_logger().info(str(e))

        # Draw bboxes
        for label in self.bbox_mins:
            cv2.rectangle(
                img, 
                (self.bbox_mins[label].x,  self.bbox_mins[label].y), 
                (self.bbox_maxes[label].x, self.bbox_maxes[label].y), 
                (0, 0, 255), 
                4
            )

        # Draw crosshairs
        center = self.avg_center()
        cv2.line(
            img,
            (center.x - 10, center.y),
            (center.x + 10, center.y),
            (255, 0, 0),
            2
        )
        cv2.line(
            img,
            (center.x, center.y - 10),
            (center.x, center.y + 10),
            (255, 0, 0),
            2
        )
        cv2.line(
            img,
            (center.x - 10, IMG_HEIGHT // 2),
            (center.x + 10, IMG_HEIGHT // 2),
            (0, 255, 0),
            2
        )
        cv2.line(
            img,
            (center.x, IMG_HEIGHT // 2 - 10),
            (center.x, IMG_HEIGHT // 2 + 10),
            (0, 255, 0),
            2
        )

        # Publish center
        self.center_pub.publish(center.x)

#        # Creating heading error object
#        heading_error = Float32MultiArray()
#        heading_error.data = np.zeros(2)
#
#        # Calculate heading error in x coord to be used by controller suite
#        heading_error_px = (IMG_WIDTH / 2.0) - center_point.x
#
#
#        # Assumes distance to buoy from robot is 2 meters, this defined how far to the left the robot can see
#        abstracted_fov_dist = 2 * math.tan(math.radians(ZED_FOV) / 1.0) 
#
#        # Converts pixels to distance space
#        pix_to_dist = abstracted_fov_dist / (IMG_WIDTH / 1.0)
#
#        # Turn heading error into a simulated distance and then calculate heading error angle from there
#
#        # Distance from center in camera frame
#        heading_error.data[0] = heading_error_px * pix_to_dist
#
#        # Heading angle from boat's heading
#        heading_error.data[1] = math.atan(heading_error.data[0] / 1.0)
#
#        # Publish heading error for controller
#        self.buoy_heading_pub.publish(heading_error)
        
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))

def main(args=None):
    rclpy.init(args=args)

    detector = BuoyDetector()

    rclpy.spin(detector)


if __name__ == '__main__':
    main()