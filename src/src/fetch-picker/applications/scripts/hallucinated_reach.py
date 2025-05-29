#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import robot_api

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

class ArTagReader(object):
    def __init__(self):
        self.markers = []
        
    def callback(self, msg):
        self.markers = msg.markers

def main():
    rospy.init_node('hallucinated_reach')
    wait_for_time()
    
    # Start position
    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    start.pose.orientation.w = 1.0
    
    # Initialize arm
    arm = robot_api.Arm()
    
    # Move to start position
    rospy.loginfo("Moving to start position...")
    error = arm.move_to_pose(start)
    if error:
        rospy.logerr(f"Failed to move to start position: {error}")
        return
    
    # Set up AR tag reader
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)
    
    # Wait for markers
    rospy.loginfo("Waiting for AR markers...")
    while len(reader.markers) == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    rospy.loginfo(f"Found {len(reader.markers)} markers")
    
    # Try to reach each marker
    for marker in reader.markers:
        # Create target pose above the marker
        target = PoseStamped()
        target.header = marker.header
        
        # Position 10cm above the marker
        target.pose.position = marker.pose.pose.position
        target.pose.position.z += 0.1
        
        # Use identity orientation (gripper pointing down)
        target.pose.orientation.w = 1.0
        
        rospy.loginfo(f"Attempting to reach marker {marker.id}")
        error = arm.move_to_pose(target)
        
        if error is None:
            rospy.loginfo(f"Successfully reached marker {marker.id}")
            rospy.sleep(2.0)  # Pause at marker
            return
        else:
            rospy.logwarn(f"Failed to reach marker {marker.id}: {error}")
    
    rospy.logerr("Failed to reach any markers")

if __name__ == '__main__':
    main()