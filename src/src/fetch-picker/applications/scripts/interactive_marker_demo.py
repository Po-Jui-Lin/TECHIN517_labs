#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('interactive_marker_demo')
    wait_for_time()
    
    # Create an Interactive Marker Server
    server = InteractiveMarkerServer("simple_marker")
    
    # Create an Interactive Marker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "my_marker"
    int_marker.description = "Simple Click Control"
    int_marker.pose.position.x = 1  # 1 meter in front of the robot
    int_marker.pose.orientation.w = 1
    
    # Create a Marker (the visual part)
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0
    
    # Create control that contains the marker
    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)
    
    # Add the interactive marker to the server with callback
    server.insert(int_marker, handle_viz_input)
    server.applyChanges()
    
    # Keep the node alive
    rospy.spin()

def handle_viz_input(feedback):
    """Process feedback from the interactive marker.
    
    Args:
        feedback: The feedback message.
    """
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(feedback.marker_name + ' was clicked at position:\n' + 
                     str(feedback.pose.position))
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo(feedback.marker_name + ' was moved to position:\n' + 
                     str(feedback.pose.position))
    else:
        rospy.loginfo('Received event type: ' + str(feedback.event_type))

if __name__ == '__main__':
    main()