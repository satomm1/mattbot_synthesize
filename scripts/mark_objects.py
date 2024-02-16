import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import numpy as np

class ObjectMarker:
    
    def __init__(self):
        rospy.init_node('mark_objects')
    
        self.marker_pub = rospy.Publisher('/object_marker', Marker, queue_size=10)
        rospy.Subscriber('/object_loc', PointStamped, self.object_loc_callback)
        
        self.marked_objects = {}  # Keep track of already marked objects
        self.Threshold = 1
        
        self.object_id_num = 0
        
    def object_loc_callback(self, msg):

        x_pos = msg.point.x
        y_pos = msg.point.y
	
        for key in self.marked_objects:
            x_marked, y_marked = self.marked_objects[key]
            if np.linalg.norm([x_marked-x_pos, y_marked-y_pos]) < self.Threshold:
                # Already marked this object, don't add
                return
	        
        msg.point.z = 0
        
        # Create a Marker message
        marker = Marker()
        marker.header = msg.header
        marker.ns = "object_marker"
        marker.id = self.object_id_num  
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.point
        print(msg.point)
        marker.pose.orientation.w = 1.0  # No rotation
        marker.scale.x = 0.1  # Size of the marker (diameter)
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color
        marker.lifetime = rospy.Duration()  # Marker should persist until explicitly deleted

        # Publish the Marker message
        self.marker_pub.publish(marker)

        # Add the object ID to the list of marked objects
        self.marked_objects[self.object_id_num] = (x_pos, y_pos)
            
        self.object_id_num += 1
   
 
if __name__ == '__main__':
    object_marker = ObjectMarker()
    rospy.spin()
    
