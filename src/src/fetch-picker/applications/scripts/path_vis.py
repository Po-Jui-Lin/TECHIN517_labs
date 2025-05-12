#!/usr/bin/env python
import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry

class NavPath(object):
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('path_visualizer')
        
        # Initialize path storage
        self._path = []
        self._last_position = None
        self._min_distance = 0.05  # Minimum distance to add a new point (in meters)
        
        # Create publisher for markers
        self._marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        
        # Wait for publisher to register
        rospy.sleep(0.5)
        
        # Subscribe to odometry
        rospy.Subscriber('odom', Odometry, self.callback)
        
        # Log startup
        rospy.loginfo("Path visualization started")

    def callback(self, msg):
        # Extract current position
        current_pos = msg.pose.pose.position
        
        # If this is our first point, add it
        if self._last_position is None:
            self._path.append(current_pos)
            self._last_position = current_pos
            self.publish_path()
            return
            
        # Calculate distance from last point
        dx = current_pos.x - self._last_position.x
        dy = current_pos.y - self._last_position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Only add point if it's far enough from the last one
        if distance >= self._min_distance:
            self._path.append(current_pos)
            self._last_position = current_pos
            self.publish_path()
    
    def publish_path(self):
        # Choose which marker type to use
        self.publish_line_strip()
        # Alternatively: self.publish_sphere_list()
    
    def publish_line_strip(self):
        """Publish the path as a line strip marker"""
        # Create line strip marker
        marker = Marker(
            type=Marker.LINE_STRIP,
            id=1,
            lifetime=rospy.Duration(0),  # 0 = forever
            scale=Vector3(0.05, 0, 0),   # Line width
            header=Header(frame_id='odom'),
            color=ColorRGBA(0.0, 0.0, 1.0, 0.8)  # Blue
        )
        
        # Add all points to the line strip
        marker.points = self._path
        
        # Publish the marker
        self._marker_publisher.publish(marker)
    
    def publish_sphere_list(self):
        """Publish the path as a list of small spheres"""
        # Create sphere list marker
        marker = Marker(
            type=Marker.SPHERE_LIST,
            id=2,
            lifetime=rospy.Duration(0),  # 0 = forever
            scale=Vector3(0.08, 0.08, 0.08),  # Sphere size
            header=Header(frame_id='odom'),
            color=ColorRGBA(1.0, 0.0, 0.0, 0.8)  # Red
        )
        
        # Add all points to the sphere list
        marker.points = self._path
        
        # Publish the marker
        self._marker_publisher.publish(marker)

def main():
    nav_path = NavPath()
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()