#!/usr/bin/env python
import rospy
import tf
import sys
from ar_track_alvar_msgs.msg import AlvarMarkers
import robot_api
from pbd_arm_server import PbdProgram, PbdStep, PbdPose

class PbdRecorder:
    def __init__(self):
        self.arm = robot_api.Arm()
        self.gripper = robot_api.Gripper()
        self.tf_listener = tf.TransformListener()
        self.current_program = None
        self.ar_markers = {}
        self.is_sim = rospy.get_param('/use_sim_time', False)
        
        # Subscribe to AR markers
        self._ar_sub = rospy.Subscriber(
            '/ar_pose_marker', 
            AlvarMarkers, 
            self._ar_callback
        )
        rospy.sleep(0.5)  # Wait for connections
    
    def _ar_callback(self, msg):
        """Update tracked AR markers."""
        self.ar_markers = {marker.id: marker for marker in msg.markers}
    
    def create_program(self, name):
        """Start creating a new program."""
        self.current_program = PbdProgram(name)
        if not self.is_sim:
            self.arm.relax()
        print("Program '{}' created. Arm relaxed for teaching.".format(name))
    
    def get_current_ee_pose(self):
        """Get current end-effector pose in base_link frame."""
        try:
            self.tf_listener.waitForTransform(
                'base_link', 'wrist_roll_link', 
                rospy.Time(0), rospy.Duration(5.0)
            )
            (trans, rot) = self.tf_listener.lookupTransform(
                'base_link', 'wrist_roll_link', rospy.Time(0)
            )
            
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.pose.position.x = trans[0]
            pose_stamped.pose.position.y = trans[1]
            pose_stamped.pose.position.z = trans[2]
            pose_stamped.pose.orientation.x = rot[0]
            pose_stamped.pose.orientation.y = rot[1]
            pose_stamped.pose.orientation.z = rot[2]
            pose_stamped.pose.orientation.w = rot[3]
            
            return pose_stamped
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Error: {}".format(e))
            return None
    
    def save_pose(self, frame_type='base', marker_id=None):
        """Save current pose relative to specified frame."""
        if self.current_program is None:
            print("No program active. Create one first.")
            return
        
        current_pose = self.get_current_ee_pose()
        if current_pose is None:
            print("Failed to get current pose.")
            return
        
        if frame_type == 'base':
            # Save relative to base_link
            pbd_pose = PbdPose(current_pose, 'base_link')
        elif frame_type == 'marker' and marker_id is not None:
            # Transform pose to be relative to AR marker
            marker_frame = 'ar_marker_{}'.format(marker_id)
            try:
                # Get transform from marker to base
                self.tf_listener.waitForTransform(
                    marker_frame, 'base_link',
                    rospy.Time(0), rospy.Duration(5.0)
                )
                pose_rel_marker = self.tf_listener.transformPose(
                    marker_frame, current_pose
                )
                pbd_pose = PbdPose(pose_rel_marker, marker_frame)
            except Exception as e:
                rospy.logerr("Failed to transform pose: {}".format(e))
                return
        else:
            print("Invalid frame type.")
            return
        
        # Get current gripper state
        gripper_state = 'open'  # You might want to check actual gripper state
        
        step = PbdStep(pbd_pose, gripper_state)
        self.current_program.add_step(step)
        print("Pose saved relative to {}".format(pbd_pose.frame_id))
    
    def set_gripper(self, state):
        """Set gripper to open or closed."""
        if state == 'open':
            self.gripper.open()
        elif state == 'close':
            self.gripper.close()
        
        # Update last step's gripper state if exists
        if self.current_program and self.current_program.steps:
            self.current_program.steps[-1].gripper_state = state
    
    def save_program(self, filename):
        """Save current program to file."""
        if self.current_program is None:
            print("No program to save.")
            return
        
        self.current_program.save_to_file(filename)
        print("Program saved to {}".format(filename))
        
        # Re-enable arm controller
        if not self.is_sim:
            self.arm.freeze()