#!/usr/bin/env python

import rospy
import robot_api
import math
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def make_forward_marker(server, base):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "forward_marker"
    int_marker.description = "Move Forward 0.5m"
    int_marker.pose.position.x = 0.5
    int_marker.pose.orientation.w = 1
    
    # Create a blue arrow marker
    arrow_marker = Marker()
    arrow_marker.type = Marker.ARROW
    arrow_marker.pose.orientation.w = 1
    arrow_marker.scale.x = 0.3
    arrow_marker.scale.y = 0.05
    arrow_marker.scale.z = 0.05
    arrow_marker.color.r = 0.0
    arrow_marker.color.g = 0.0
    arrow_marker.color.b = 1.0
    arrow_marker.color.a = 1.0
    
    # Add control
    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(arrow_marker)
    int_marker.controls.append(button_control)
    
    # Add to server
    server.insert(int_marker, lambda feedback: handle_forward(feedback, base))

def make_rotate_markers(server, base):
    # Create rotate left marker
    left_marker = InteractiveMarker()
    left_marker.header.frame_id = "base_link"
    left_marker.name = "rotate_left_marker"
    left_marker.description = "Rotate CCW 30 degrees"
    left_marker.pose.position.y = 0.5
    left_marker.pose.orientation.w = 1
    
    # Create a green arrow for left rotation
    left_arrow = Marker()
    left_arrow.type = Marker.ARROW
    left_arrow.pose.orientation.x = 0
    left_arrow.pose.orientation.y = 0
    left_arrow.pose.orientation.z = 0.7071
    left_arrow.pose.orientation.w = 0.7071
    left_arrow.scale.x = 0.3
    left_arrow.scale.y = 0.05
    left_arrow.scale.z = 0.05
    left_arrow.color.r = 0.0
    left_arrow.color.g = 1.0
    left_arrow.color.b = 0.0
    left_arrow.color.a = 1.0
    
    # Add control
    left_control = InteractiveMarkerControl()
    left_control.interaction_mode = InteractiveMarkerControl.BUTTON
    left_control.always_visible = True
    left_control.markers.append(left_arrow)
    left_marker.controls.append(left_control)
    
    # Add to server
    server.insert(left_marker, lambda feedback: handle_rotate_left(feedback, base))
    
    # Create rotate right marker
    right_marker = InteractiveMarker()
    right_marker.header.frame_id = "base_link"
    right_marker.name = "rotate_right_marker"
    right_marker.description = "Rotate CW 30 degrees"
    right_marker.pose.position.y = -0.5
    right_marker.pose.orientation.w = 1
    
    # Create a red arrow for right rotation
    right_arrow = Marker()
    right_arrow.type = Marker.ARROW
    right_arrow.pose.orientation.x = 0
    right_arrow.pose.orientation.y = 0
    right_arrow.pose.orientation.z = -0.7071
    right_arrow.pose.orientation.w = 0.7071
    right_arrow.scale.x = 0.3
    right_arrow.scale.y = 0.05
    right_arrow.scale.z = 0.05
    right_arrow.color.r = 1.0
    right_arrow.color.g = 0.0
    right_arrow.color.b = 0.0
    right_arrow.color.a = 1.0
    
    # Add control
    right_control = InteractiveMarkerControl()
    right_control.interaction_mode = InteractiveMarkerControl.BUTTON
    right_control.always_visible = True
    right_control.markers.append(right_arrow)
    right_marker.controls.append(right_control)
    
    # Add to server
    server.insert(right_marker, lambda feedback: handle_rotate_right(feedback, base))

def handle_forward(feedback, base):
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo("Moving forward 0.5m")
        base.go_forward(0.5)

def handle_rotate_left(feedback, base):
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo("Rotating CCW 30 degrees")
        base.turn(30 * math.pi / 180)

def handle_rotate_right(feedback, base):
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo("Rotating CW 30 degrees")
        base.turn(-30 * math.pi / 180)

def main():
    rospy.init_node('interactive_marker_navigation')
    wait_for_time()
    
    # Create a base controller
    base = robot_api.Base()
    
    # Create interactive marker server
    server = InteractiveMarkerServer("navigation_markers")
    
    # Create markers
    make_forward_marker(server, base)
    make_rotate_markers(server, base)
    
    # Apply changes to server
    server.applyChanges()
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()