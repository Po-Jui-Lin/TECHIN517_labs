#!/usr/bin/env python

import rospy
import robot_api
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('cart_arm_demo')
    wait_for_time()
    
    # Initialize the arm
    arm = robot_api.Arm()
    
    # Register shutdown handler for safety
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)
    
    # Define the wave poses
    pose1 = Pose(
        Point(0.042, 0.384, 1.826), 
        Quaternion(0.173, -0.693, -0.242, 0.657)
    )
    pose2 = Pose(
        Point(0.047, 0.545, 1.822), 
        Quaternion(-0.274, -0.701, 0.173, 0.635)
    )
    
    # Create PoseStamped messages
    ps1 = PoseStamped()
    ps1.header.frame_id = 'base_link'
    ps1.pose = pose1
    
    ps2 = PoseStamped()
    ps2.header.frame_id = 'base_link'
    ps2.pose = pose2
    
    gripper_poses = [ps1, ps2]
    
    # Wave forever
    while not rospy.is_shutdown():
        for pose in gripper_poses:
            # Update the timestamp
            pose.header.stamp = rospy.Time.now()
            
            # Move to the pose
            error = arm.move_to_pose(pose)
            if error is not None:
                rospy.logerr(error)
            
            # Add a delay to let the arm settle
            rospy.sleep(1)

if __name__ == '__main__':
    main()