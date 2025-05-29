#!/usr/bin/env python
import rospy
import tf
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import robot_api
from pbd_arm_server import PbdProgram

class PbdExecutor:
    def __init__(self):
        self.arm = robot_api.Arm()
        self.gripper = robot_api.Gripper()
        self.tf_listener = tf.TransformListener()
        self.ar_markers = {}
        
        # Subscribe to AR markers
        self._ar_sub = rospy.Subscriber(
            '/ar_pose_marker', 
            AlvarMarkers, 
            self._ar_callback
        )
        
        # Ensure arm controller is running
        self.arm.freeze()
        rospy.sleep(1.0)
    
    def _ar_callback(self, msg):
        """Update tracked AR markers."""
        self.ar_markers = {marker.id: marker for marker in msg.markers}
    
    def execute_program(self, program):
        """Execute a PbD program."""
        rospy.loginfo("Executing program: {}".format(program.name))
        
        for i, step in enumerate(program.steps):
            rospy.loginfo("Executing step {}".format(i + 1))
            
            # Get target pose in base_link frame
            target_pose = self._get_transformed_pose(step.pose)
            
            if target_pose is None:
                rospy.logerr("Failed to get target pose for step {}".format(i + 1))
                continue
            
            # Set gripper state
            if step.gripper_state == 'open':
                self.gripper.open()
            elif step.gripper_state == 'close':
                self.gripper.close()
            
            # Move to pose
            error = self.arm.move_to_pose(target_pose)
            if error is not None:
                rospy.logerr("Failed to move to pose: {}".format(error))
                continue
            
            rospy.sleep(0.5)  # Small delay between steps
        
        rospy.loginfo("Program execution complete")
    
    def _get_transformed_pose(self, pbd_pose):
        """Transform a PbdPose to current base_link frame."""
        if pbd_pose.frame_id == 'base_link':
            # Already in base_link frame
            return pbd_pose.pose_stamped
        
        # Handle AR marker frames
        if pbd_pose.frame_id.startswith('ar_marker_'):
            try:
                # Wait for transform from base to marker
                self.tf_listener.waitForTransform(
                    'base_link', pbd_pose.frame_id,
                    rospy.Time(0), rospy.Duration(5.0)
                )
                
                # Transform pose from marker frame to base frame
                pose_in_base = self.tf_listener.transformPose(
                    'base_link', pbd_pose.pose_stamped
                )
                
                return pose_in_base
                
            except Exception as e:
                rospy.logerr("Transform failed: {}".format(e))
                return None
        
        return None

def main():
    rospy.init_node('pbd_executor')
    
    if len(sys.argv) < 2:
        print("Usage: rosrun applications pbd_executor.py <program_file>")
        return
    
    filename = sys.argv[1]
    
    # Load program
    try:
        program = PbdProgram.load_from_file(filename)
    except Exception as e:
        rospy.logerr("Failed to load program: {}".format(e))
        return
    
    # Execute program
    executor = PbdExecutor()
    rospy.sleep(1.0)  # Wait for connections
    
    executor.execute_program(program)

if __name__ == '__main__':
    main()