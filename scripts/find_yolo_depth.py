import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_msgs.msg import TFMessage


class DepthEstimator:
    def __init__(self):
        rospy.init_node('depth_estimator')
        self.bridge = CvBridge()
        self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_callback)
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        self.bounding_boxes = []
        self.depth_image = None
        
        self.K = np.array([[570.3405082258201,               0.0, 319.5],
                           [0.0,               570.3405082258201, 239.5],
                           [0.0,                             0.0,   1.0]])
        self.R = np.identity(3)
        self.P = np.array([[570.3405082258201,               0.0, 319.5, 0.0],
                           [              0.0, 570.3405082258201, 239.5, 0.0],
                           [              0.0,               0.0,   1.0, 0.0]])
                           
        self.K_inv = np.linalg.inv(self.K)
        self.P_inv = np.linalg.inv(self.P[:, :3])
        
        self.fx = 570.3405082258201
        self.fy = 570.3405082258201
        self.S = 0
        self.cx = 319.5
        self.cy = 239.5
        
        self.Converter = np.array([[1/self.fx, -self.S/(self.fx * self.fy), (self.S*self.cy - self.cx*self.fy)/(self.fx*self.fy)],
                                   [0, 1/self.fy, -self.cy/self.fy],
                                   [0, 0, 1]])
                                   
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.object_pub = rospy.Publisher('/object_loc', PointStamped, queue_size=10)

    def bounding_boxes_callback(self, msg):
        self.bounding_boxes = msg.bounding_boxes
        
        if self.depth_image is not None:
            self.process_bounding_boxes()

    def depth_image_callback(self, msg):
        # Save the depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def process_bounding_boxes(self):
        for box in self.bounding_boxes:
            if box.Class == "person" and box.probability > 0.8:
                x_min = int(box.xmin)
                y_min = int(box.ymin)
                x_max = int(box.xmax)
                y_max = int(box.ymax)
                
                # Calculate center coordinates of the bounding box
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                
                #rospy.loginfo("center: {:.2f}, {:.2f}".format(center_x, center_y))

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
                    average_depth = np.mean(region_depth_values)/1000
                    # rospy.loginfo("Average depth of person: {:.2f} meters".format(average_depth))
                    
                    pixel_coords_homogeneous = np.array([center_x, center_y, 1])
                    #camera_coords_homogeneous = average_depth * np.dot(np.dot(self.P_inv, self.R.T), np.dot(self.K_inv, pixel_coords_homogeneous))
                    
                    camera_coords_homogeneous = average_depth * self.Converter @ pixel_coords_homogeneous
                    
                    x_camera = camera_coords_homogeneous[0]
                    y_camera = -camera_coords_homogeneous[1]
                    z_camera = camera_coords_homogeneous[2]

                    # Camera coordinates of the bounding box center
                    bounding_box_center_camera = (x_camera, y_camera, z_camera)
                    coordinates_str = "Camera coordinates: ({:.2f}, {:.2f}, {:.2f})".format(x_camera, y_camera, z_camera)

                    # Log the coordinates using rospy.loginfo
                    # rospy.loginfo(coordinates_str)
                    
                    object_loc = PointStamped()
                    object_loc.header.frame_id = "camera_link"
                    object_loc.point.x = z_camera
                    object_loc.point.y = -x_camera
                    object_loc.point.z = y_camera
                    
                    object_map = PointStamped()
                    object_map.header.frame_id = "map"
                    object_map.point.x, object_map.point.y, object_map.point.z = self.transform_point(object_loc, "map")
                    
                    self.object_pub.publish(object_map)

                else:
                    # rospy.loginfo("No valid depth values found within the bounding box")
                    
    def transform_point(self, point, target_frame):
        # Wait for the transform to become available
        while not self.tfBuffer.can_transform(target_frame, point.header.frame_id, rospy.Time(0)):
            rospy.logwarn_throttle(1.0, "Waiting for transform from {} to {}...".format(point.header.frame_id, target_frame))
            rospy.sleep(0.1)

        # Perform the coordinate transformation
        transformed_point = self.tfBuffer.transform(point, target_frame)

        return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z
       
                

if __name__ == '__main__':
    try:
        depth_estimator = DepthEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
