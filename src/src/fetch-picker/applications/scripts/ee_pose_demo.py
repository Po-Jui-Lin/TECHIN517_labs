#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('ee_pose_demo')
    wait_for_time()
    
    # Create a TransformListener
    listener = tf.TransformListener()
    
    # Important: Wait a bit for the listener to start receiving TF data
    rospy.sleep(0.5)
    
    # Set the rate for the loop (1 Hz)
    rate = rospy.Rate(1)
    
    # Choose the frame for the gripper - you could use any of these:
    # - l_gripper_finger_link
    # - r_gripper_finger_link
    # - gripper_link
    # - wrist_roll_link
    gripper_frame = 'gripper_link'  # This is usually the most appropriate choice
    
    while not rospy.is_shutdown():
        try:
            # Look up the transform between base_link and the gripper frame
            # rospy.Time(0) gets the latest available transform
            (trans, rot) = listener.lookupTransform('base_link', gripper_frame, rospy.Time(0))
            
            # Print the position (translation) and orientation (rotation)
            rospy.loginfo(f"Position: {trans}, Orientation: {rot}")
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Error: {e}")
        
        # Sleep to maintain the rate
        rate.sleep()

if __name__ == '__main__':
    main()