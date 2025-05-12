#!/usr/bin/env python

import copy
import math
import numpy as np
import rospy
import tf
import tf.transformations as tft

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

import robot_api

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._menu_handler = MenuHandler()
        self._tf_listener = tf.TransformListener()
        rospy.sleep(0.5)  # Wait for TF to initialize
        
    def start(self):
        # Create interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "gripper_teleop"
        int_marker.description = "Gripper Teleop"
        int_marker.scale = 0.2  # Scale down the controls
        
        # Get current gripper pose as default
        try:
            self._tf_listener.waitForTransform('base_link', 'wrist_roll_link', rospy.Time(), rospy.Duration(2.0))
            (trans, rot) = self._tf_listener.lookupTransform('base_link', 'wrist_roll_link', rospy.Time(0))
            int_marker.pose.position.x = trans[0]
            int_marker.pose.position.y = trans[1]
            int_marker.pose.position.z = trans[2]
            int_marker.pose.orientation.x = rot[0]
            int_marker.pose.orientation.y = rot[1]
            int_marker.pose.orientation.z = rot[2]
            int_marker.pose.orientation.w = rot[3]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
            # Default position if transform isn't found
            int_marker.pose.position.x = 0.5
            int_marker.pose.position.y = 0.0
            int_marker.pose.position.z = 0.8
            int_marker.pose.orientation.w = 1.0
        
        # Create gripper marker control
        gripper_control = InteractiveMarkerControl()
        gripper_control.interaction_mode = InteractiveMarkerControl.MENU
        gripper_control.always_visible = True
        
        # Check if the pose is reachable
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = int_marker.header.frame_id
        pose_stamped.pose = int_marker.pose
        is_reachable = self._check_reachability(pose_stamped)
        
        # Add gripper markers to control
        markers = self.make_gripper_marker(pose_stamped, is_reachable)
        for marker in markers:
            gripper_control.markers.append(marker)
        
        # Add gripper control to interactive marker
        int_marker.controls.append(gripper_control)
        
        # Add 6DOF controls
        int_marker.controls.extend(self.make_6dof_controls())
        
        # Add marker to server
        self._im_server.insert(int_marker, self.handle_feedback)
        
        # Add menu entries
        self._menu_handler.insert("Go to pose", callback=self.handle_menu)
        self._menu_handler.insert("Open gripper", callback=self.handle_menu)
        self._menu_handler.insert("Close gripper", callback=self.handle_menu)
        
        # Apply menu to marker
        self._menu_handler.apply(self._im_server, int_marker.name)
        
        # Apply changes to server
        self._im_server.applyChanges()
        
        rospy.loginfo("Gripper teleop started")
        
    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Create PoseStamped for checking reachability
            pose_stamped = PoseStamped()
            pose_stamped.header = feedback.header
            pose_stamped.pose = feedback.pose
            
            # Check if the pose is reachable
            is_reachable = self._check_reachability(pose_stamped)
            
            # Update marker color based on reachability
            self._update_marker_color(feedback.marker_name, is_reachable)
            
            rospy.loginfo(f"Pose {'reachable' if is_reachable else 'unreachable'}: {feedback.pose.position}")
        
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:  # Go to pose
                self._go_to_pose(feedback)
            elif feedback.menu_entry_id == 2:  # Open gripper
                self._gripper.open()
            elif feedback.menu_entry_id == 3:  # Close gripper
                self._gripper.close()

    def _check_reachability(self, pose_stamped):
        """Check if a pose is reachable"""
        return self._arm.compute_ik(pose_stamped)

    def _update_marker_color(self, marker_name, is_reachable):
        """Update the color of the gripper marker based on reachability"""
        # Get the marker from the server
        int_marker = self._im_server.get(marker_name)
        
        # Update colors for all markers in the gripper control
        for control in int_marker.controls:
            if control.interaction_mode == InteractiveMarkerControl.MENU:
                for marker in control.markers:
                    if is_reachable:
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        marker.color.a = 0.6
                    else:
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                        marker.color.a = 0.6
        
        # Update marker in server
        self._im_server.insert(int_marker)
        self._im_server.applyChanges()

    def _go_to_pose(self, feedback):
        """Send the arm to the marker's pose"""
        pose_stamped = PoseStamped()
        pose_stamped.header = feedback.header
        pose_stamped.pose = feedback.pose
        
        # Check if the pose is reachable
        if not self._check_reachability(pose_stamped):
            rospy.logwarn("Cannot go to pose: pose is unreachable")
            return
        
        # Move arm to pose
        rospy.loginfo("Moving arm to pose")
        error = self._arm.move_to_pose(
            pose_stamped,
            allowed_planning_time=15.0,
            execution_timeout=10.0,
            num_planning_attempts=5,
            replan=True
        )
        
        if error:
            rospy.logerr(f"Failed to move to pose: {error}")
        else:
            rospy.loginfo("Successfully moved to pose")

    def handle_menu(self, feedback):
        """Handle menu feedback - this is a callback for MenuHandler"""
        self.handle_feedback(feedback)

    def make_gripper_marker(self, pose_stamped, is_reachable=True):
        # Define meshes for gripper parts
        GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
        L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
        R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
        
        # Get the transform from wrist_roll_link to gripper_link
        gripper_offset = self._get_gripper_offset()
        
        # Create markers for gripper parts
        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = GRIPPER_MESH
        gripper_marker.pose.position.x = gripper_offset[0]
        gripper_marker.pose.position.y = gripper_offset[1]
        gripper_marker.pose.position.z = gripper_offset[2]
        gripper_marker.pose.orientation.w = 1.0
        gripper_marker.scale.x = 1.0
        gripper_marker.scale.y = 1.0
        gripper_marker.scale.z = 1.0
        
        # Set color based on reachability
        if is_reachable:
            gripper_marker.color.r = 0.0
            gripper_marker.color.g = 1.0
            gripper_marker.color.b = 0.0
            gripper_marker.color.a = 0.6
        else:
            gripper_marker.color.r = 1.0
            gripper_marker.color.g = 0.0
            gripper_marker.color.b = 0.0
            gripper_marker.color.a = 0.6
        
        # Create left finger marker
        l_finger_marker = Marker()
        l_finger_marker.type = Marker.MESH_RESOURCE
        l_finger_marker.mesh_resource = L_FINGER_MESH
        l_finger_marker.pose.position.x = gripper_offset[0] + 0.117
        l_finger_marker.pose.position.y = gripper_offset[1] + 0.015
        l_finger_marker.pose.position.z = gripper_offset[2]
        l_finger_marker.pose.orientation.w = 1.0
        l_finger_marker.scale.x = 1.0
        l_finger_marker.scale.y = 1.0
        l_finger_marker.scale.z = 1.0
        l_finger_marker.color = gripper_marker.color
        
        # Create right finger marker
        r_finger_marker = Marker()
        r_finger_marker.type = Marker.MESH_RESOURCE
        r_finger_marker.mesh_resource = R_FINGER_MESH
        r_finger_marker.pose.position.x = gripper_offset[0] + 0.117
        r_finger_marker.pose.position.y = gripper_offset[1] - 0.015
        r_finger_marker.pose.position.z = gripper_offset[2]
        r_finger_marker.pose.orientation.w = 1.0
        r_finger_marker.scale.x = 1.0
        r_finger_marker.scale.y = 1.0
        r_finger_marker.scale.z = 1.0
        r_finger_marker.color = gripper_marker.color
        
        return [gripper_marker, l_finger_marker, r_finger_marker]

    def _get_gripper_offset(self):
        """Get the transform from wrist_roll_link to gripper_link"""
        try:
            self._tf_listener.waitForTransform('wrist_roll_link', 'gripper_link', rospy.Time(), rospy.Duration(2.0))
            (trans, rot) = self._tf_listener.lookupTransform('wrist_roll_link', 'gripper_link', rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
            # Default offset if transform isn't found
            return (0.16, 0.0, 0.0)

    def make_6dof_controls(self):
        controls = []
        
        # X axis control (move)
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        controls.append(copy.deepcopy(control))
        
        # Y axis control (move)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        controls.append(copy.deepcopy(control))
        
        # Z axis control (move)
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        controls.append(copy.deepcopy(control))
        
        # X axis control (rotate)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        controls.append(copy.deepcopy(control))
        
        # Y axis control (rotate)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        controls.append(copy.deepcopy(control))
        
        # Z axis control (rotate)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        controls.append(copy.deepcopy(control))
        
        return controls


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._menu_handler = MenuHandler()
        self._tf_listener = tf.TransformListener()
        rospy.sleep(0.5)  # Wait for TF to initialize
        
    def start(self):
        # Create object interactive marker
        obj_marker = InteractiveMarker()
        obj_marker.header.frame_id = "base_link"
        obj_marker.name = "object_teleop"
        obj_marker.description = "Object Teleop"
        obj_marker.scale = 0.2  # Scale down the controls
        
        # Set initial position
        obj_marker.pose.position.x = 0.7
        obj_marker.pose.position.y = 0.0
        obj_marker.pose.position.z = 0.7
        obj_marker.pose.orientation.w = 1.0
        
        # Create box control
        box_control = InteractiveMarkerControl()
        box_control.interaction_mode = InteractiveMarkerControl.MENU
        box_control.always_visible = True
        
        # Create box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.05
        box_marker.scale.y = 0.05
        box_marker.scale.z = 0.15
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0
        
        # Add box marker to control
        box_control.markers.append(box_marker)
        
        # Add box control to interactive marker
        obj_marker.controls.append(box_control)
        
        # Add 6DOF controls
        obj_marker.controls.extend(self.make_6dof_controls())
        
        # Add marker to server
        self._im_server.insert(obj_marker, self.handle_feedback)
        
        # Create pre-grasp, grasp, and lift markers
        self._create_gripper_pose_markers(obj_marker.pose)
        
        # Add menu entries
        self._menu_handler.insert("Execute Pick Sequence", callback=self.handle_menu)
        
        # Apply menu to marker
        self._menu_handler.apply(self._im_server, obj_marker.name)
        
        # Apply changes to server
        self._im_server.applyChanges()
        
        rospy.loginfo("Object teleop started")

    def _create_gripper_pose_markers(self, object_pose):
        """Create markers for pre-grasp, grasp, and lift poses"""
        # Calculate poses relative to object
        pre_grasp_pose = self._calculate_pre_grasp_pose(object_pose)
        grasp_pose = self._calculate_grasp_pose(object_pose)
        lift_pose = self._calculate_lift_pose(object_pose)
        
        # Check if all poses are reachable
        all_reachable = (
            self._check_reachability(self._pose_to_pose_stamped(pre_grasp_pose)) and
            self._check_reachability(self._pose_to_pose_stamped(grasp_pose)) and
            self._check_reachability(self._pose_to_pose_stamped(lift_pose))
        )
        
        # Create and add markers for each pose
        self._create_pose_marker("pre_grasp", pre_grasp_pose, all_reachable)
        self._create_pose_marker("grasp", grasp_pose, all_reachable)
        self._create_pose_marker("lift", lift_pose, all_reachable)

    def _pose_to_pose_stamped(self, pose):
        """Convert Pose to PoseStamped with base_link frame"""
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose = pose
        return ps

    def _create_pose_marker(self, name, pose, is_reachable):
        """Create a marker for a gripper pose"""
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = f"{name}_pose"
        int_marker.pose = pose
        int_marker.scale = 0.1  # Smaller than the object marker
        
        # Create control for the gripper markers
        gripper_control = InteractiveMarkerControl()
        gripper_control.interaction_mode = InteractiveMarkerControl.NONE
        gripper_control.always_visible = True
        
        # Add gripper markers to control
        pose_stamped = self._pose_to_pose_stamped(pose)
        markers = self.make_gripper_marker(pose_stamped, is_reachable)
        for marker in markers:
            gripper_control.markers.append(marker)
        
        # Add control to marker
        int_marker.controls.append(gripper_control)
        
        # Add marker to server
        self._im_server.insert(int_marker)

    def _calculate_pre_grasp_pose(self, object_pose):
        """Calculate pre-grasp pose relative to object pose"""
        # Pre-grasp is 10cm behind the object (in the object's frame)
        pre_grasp = copy.deepcopy(object_pose)
        
        # Create transform matrix from object pose
        obj_matrix = self._pose_to_matrix(object_pose)
        
        # Create a transform for pre-grasp (10cm back from object in X)
        pre_grasp_offset = np.array([
            [1, 0, 0, -0.1],  # 10cm back in X
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Multiply matrices to get pre-grasp in base_link frame
        result_matrix = np.dot(obj_matrix, pre_grasp_offset)
        
        # Convert back to Pose
        return self._matrix_to_pose(result_matrix)

    def _calculate_grasp_pose(self, object_pose):
        """Calculate grasp pose relative to object pose"""
        # Grasp is at the object's position
        grasp = copy.deepcopy(object_pose)
        return grasp

    def _calculate_lift_pose(self, object_pose):
        """Calculate lift pose relative to object pose"""
        # Lift is 10cm above the object (in the world frame)
        lift = copy.deepcopy(object_pose)
        lift.position.z += 0.1
        return lift

    def _pose_to_matrix(self, pose):
        """Convert a Pose to a 4x4 transformation matrix"""
        # Extract position and orientation
        pos = pose.position
        quat = pose.orientation
        
        # Convert quaternion to matrix
        matrix = tft.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        
        # Add translation
        matrix[0, 3] = pos.x
        matrix[1, 3] = pos.y
        matrix[2, 3] = pos.z
        
        return matrix

    def _matrix_to_pose(self, matrix):
        """Convert a 4x4 transformation matrix to a Pose"""
        pose = Pose()
        
        # Extract position
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        
        # Extract orientation
        quat = tft.quaternion_from_matrix(matrix)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # When object pose updates, update all associated gripper pose markers
            self._create_gripper_pose_markers(feedback.pose)
            self._im_server.applyChanges()
            
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == 1:  # Execute pick sequence
                self._execute_pick_sequence(feedback.pose)

    def _check_reachability(self, pose_stamped):
        """Check if a pose is reachable"""
        return self._arm.compute_ik(pose_stamped)

    def _execute_pick_sequence(self, object_pose):
        """Execute the pick sequence"""
        # Calculate poses
        pre_grasp_pose = self._calculate_pre_grasp_pose(object_pose)
        grasp_pose = self._calculate_grasp_pose(object_pose)
        lift_pose = self._calculate_lift_pose(object_pose)
        
        # Convert to PoseStamped
        pre_grasp_ps = self._pose_to_pose_stamped(pre_grasp_pose)
        grasp_ps = self._pose_to_pose_stamped(grasp_pose)
        lift_ps = self._pose_to_pose_stamped(lift_pose)
        
        # Check if all poses are reachable
        if not (self._check_reachability(pre_grasp_ps) and
                self._check_reachability(grasp_ps) and
                self._check_reachability(lift_ps)):
            rospy.logerr("Cannot execute pick sequence: one or more poses are unreachable")
            return
        
        # Execute the sequence
        rospy.loginfo("Executing pick sequence")
        
        # Open gripper
        self._gripper.open()
        
        # Move to pre-grasp
        error = self._arm.move_to_pose(
            pre_grasp_ps,
            allowed_planning_time=15.0,
            execution_timeout=10.0,
            num_planning_attempts=5
        )
        if error:
            rospy.logerr(f"Failed to move to pre-grasp: {error}")
            return
        
        # Move to grasp
        error = self._arm.move_to_pose(
            grasp_ps,
            allowed_planning_time=15.0,
            execution_timeout=10.0,
            num_planning_attempts=5
        )
        if error:
            rospy.logerr(f"Failed to move to grasp: {error}")
            return
        
        # Close gripper
        self._gripper.close()
        rospy.sleep(1.0)  # Wait for gripper to close
        
        # Move to lift
        error = self._arm.move_to_pose(
            lift_ps,
            allowed_planning_time=15.0,
            execution_timeout=10.0,
            num_planning_attempts=5
        )
        if error:
            rospy.logerr(f"Failed to move to lift: {error}")
            return
        
        rospy.loginfo("Pick sequence completed successfully")

    def handle_menu(self, feedback):
        """Handle menu feedback - this is a callback for MenuHandler"""
        self.handle_feedback(feedback)

    def make_gripper_marker(self, pose_stamped, is_reachable=True):
        # Define meshes for gripper parts
        GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
        L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
        R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
        
        # Get the transform from wrist_roll_link to gripper_link
        gripper_offset = self._get_gripper_offset()
        
        # Create markers for gripper parts
        gripper_marker = Marker()
        gripper_marker.type = Marker.MESH_RESOURCE
        gripper_marker.mesh_resource = GRIPPER_MESH
        gripper_marker.pose.position.x = gripper_offset[0]
        gripper_marker.pose.position.y = gripper_offset[1]
        gripper_marker.pose.position.z = gripper_offset[2]
        gripper_marker.pose.orientation.w = 1.0
        gripper_marker.scale.x = 1.0
        gripper_marker.scale.y = 1.0
        gripper_marker.scale.z = 1.0
        
        # Set color based on reachability
        if is_reachable:
            gripper_marker.color.r = 0.0
            gripper_marker.color.g = 1.0
            gripper_marker.color.b = 0.0
            gripper_marker.color.a = 0.6
        else:
            gripper_marker.color.r = 1.0
            gripper_marker.color.g = 0.0
            gripper_marker.color.b = 0.0
            gripper_marker.color.a = 0.6
        
        # Create left finger marker
        l_finger_marker = Marker()
        l_finger_marker.type = Marker.MESH_RESOURCE
        l_finger_marker.mesh_resource = L_FINGER_MESH
        l_finger_marker.pose.position.x = gripper_offset[0] + 0.117
        l_finger_marker.pose.position.y = gripper_offset[1] + 0.015
        l_finger_marker.pose.position.z = gripper_offset[2]
        l_finger_marker.pose.orientation.w = 1.0
        l_finger_marker.scale.x = 1.0
        l_finger_marker.scale.y = 1.0
        l_finger_marker.scale.z = 1.0
        l_finger_marker.color = gripper_marker.color
        
        # Create right finger marker
        r_finger_marker = Marker()
        r_finger_marker.type = Marker.MESH_RESOURCE
        r_finger_marker.mesh_resource = R_FINGER_MESH
        r_finger_marker.pose.position.x = gripper_offset[0] + 0.117
        r_finger_marker.pose.position.y = gripper_offset[1] - 0.015
        r_finger_marker.pose.position.z = gripper_offset[2]
        r_finger_marker.pose.orientation.w = 1.0
        r_finger_marker.scale.x = 1.0
        r_finger_marker.scale.y = 1.0
        r_finger_marker.scale.z = 1.0
        r_finger_marker.color = gripper_marker.color
        
        return [gripper_marker, l_finger_marker, r_finger_marker]

    def _get_gripper_offset(self):
        """Get the transform from wrist_roll_link to gripper_link"""
        try:
            self._tf_listener.waitForTransform('wrist_roll_link', 'gripper_link', rospy.Time(), rospy.Duration(2.0))
            (trans, rot) = self._tf_listener.lookupTransform('wrist_roll_link', 'gripper_link', rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")
            # Default offset if transform isn't found
            return (0.16, 0.0, 0.0)

    def make_6dof_controls(self):
        controls = []
        
        # X axis control (move)
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        controls.append(copy.deepcopy(control))
        
        # Y axis control (move)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        controls.append(copy.deepcopy(control))
        
        # Z axis control (move)
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        controls.append(copy.deepcopy(control))
        
        # X axis control (rotate)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        controls.append(copy.deepcopy(control))
        
        # Y axis control (rotate)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        controls.append(copy.deepcopy(control))
        
        # Z axis control (rotate)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        controls.append(copy.deepcopy(control))
        
        return controls


def main():
    rospy.init_node('interactive_gripper')
    
    # Wait for time to be initialized
    while rospy.Time().now().to_sec() == 0:
        pass
    
    arm = robot_api.Arm()
    gripper = robot_api.Gripper()
    
    # Create interactive marker servers
    gripper_im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    
    # Initialize teleop objects
    gripper_teleop = GripperTeleop(arm, gripper, gripper_im_server)
    auto_pick_teleop = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    
    # Start teleop interfaces
    gripper_teleop.start()
    auto_pick_teleop.start()
    
    rospy.spin()

if __name__ == '__main__':
    main()