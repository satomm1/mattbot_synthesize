import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DepthEstimator:
    def __init__(self):
        rospy.init_node('depth_estimator')
        self.bridge = CvBridge()
        self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_callback)
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        self.bounding_boxes = []
        self.depth_image = None

    def bounding_boxes_callback(self, msg):
        self.bounding_boxes = msg.bounding_boxes
        
        if self.depth_image is not None:
            self.process_bounding_boxes()

    def depth_image_callback(self, msg):
        # Save the depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def process_bounding_boxes(self):
        for box in self.bounding_boxes:
            if box.Class == "person":
                x_min = int(box.xmin)
                y_min = int(box.ymin)
                x_max = int(box.xmax)
                y_max = int(box.ymax)
                
                # Calculate center coordinates of the bounding box
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2

                # Get depth values within a 5x5 pixel region at the center of the bounding box
                region_depth_values = self.depth_image[center_y - 2:center_y + 3, center_x - 2:center_x + 3]

                # Remove invalid depth values (e.g., zero-depth values)
                region_depth_values = region_depth_values[region_depth_values != 0]


                # Get depth values within the bounding box
                # box_depth_values = self.depth_image[y_min:y_max, x_min:x_max]

                # Remove invalid depth values (e.g., zero-depth values)
                # box_depth_values = box_depth_values[box_depth_values != 0]

                # Calculate average depth for the bounding box
                if len(region_depth_values) > 0:
                    # average_depth = np.mean(box_depth_values)
                    average_depth = np.mean(region_depth_values)
                    rospy.loginfo("Average depth of person: {:.2f} meters".format(average_depth/1000))
                else:
                    rospy.loginfo("No valid depth values found within the bounding box")

if __name__ == '__main__':
    try:
        depth_estimator = DepthEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
