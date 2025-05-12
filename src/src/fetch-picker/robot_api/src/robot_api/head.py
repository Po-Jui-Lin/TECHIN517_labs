#!/usr/bin/env python
# Import the necessary modules
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import math
import rospy

# Action server names
LOOK_AT_ACTION_NAME = 'head_controller/point_head'  # Name of the look-at action
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'  # Name of the pan/tilt action

# Joint names
PAN_JOINT = 'head_pan_joint'  # Name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # Name of the head tilt joint

PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.

class Head(object):
    """Head controls the Fetch's head.
    It provides two interfaces:
    head.look_at(frame_id, x, y, z)
    head.pan_tilt(pan, tilt) # In radians
    
    For example:
    head = robot_api.Head()
    head.look_at('base_link', 1, 0, 0.3)
    head.pan_tilt(0, math.pi/4)
    """
    # Joint limits in radians
    MIN_PAN = -1.57  # Minimum pan angle
    MAX_PAN = 1.57   # Maximum pan angle
    MIN_TILT = -0.76  # Minimum tilt angle
    MAX_TILT = 1.45  # Maximum tilt angle
    
    def __init__(self):
        # Create actionlib clients
        self.look_at_client = actionlib.SimpleActionClient(
            LOOK_AT_ACTION_NAME, control_msgs.msg.PointHeadAction)
        self.pan_tilt_client = actionlib.SimpleActionClient(
            PAN_TILT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        
        # Wait for both servers
        self.look_at_client.wait_for_server()
        self.pan_tilt_client.wait_for_server()
    
    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.
        
        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # Create goal
        goal = control_msgs.msg.PointHeadGoal()
        
        # Fill out the goal (we recommend setting min_duration to 1 second)
        goal.target.header.frame_id = frame_id
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(1.0)
        
        # Send the goal
        self.look_at_client.send_goal(goal)
        
        # Wait for result
        self.look_at_client.wait_for_result()
    
    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.
        
        Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # Check that the pan/tilt angles are within joint limits
        pan = min(max(pan, self.MIN_PAN), self.MAX_PAN)
        tilt = min(max(tilt, self.MIN_TILT), self.MAX_TILT)
        
        # Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        
        # Set positions of the two joints in the trajectory point
        point.positions = [pan, tilt]
        
        # Set time of the trajectory point
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)
        
        # Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        
        # Add joint names to the list
        goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        
        # Add trajectory point created above to trajectory
        goal.trajectory.points = [point]
        
        # Send the goal
        self.pan_tilt_client.send_goal(goal)
        
        # Wait for result
        self.pan_tilt_client.wait_for_result()