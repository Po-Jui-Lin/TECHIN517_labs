#!/usr/bin/env python
import rospy
import pickle
import tf
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from robot_controllers_msgs.msg import QueryControllerStatesGoal, ControllerState
from ar_track_alvar_msgs.msg import AlvarMarkers
import robot_api
import actionlib

class PbdPose:
    """Represents a single pose in a PbD program."""
    def __init__(self, pose_stamped, frame_id):
        self.pose_stamped = pose_stamped
        self.frame_id = frame_id  # 'base_link' or 'ar_marker_X'

class PbdStep:
    """Represents a single step in a PbD program."""
    def __init__(self, pose, gripper_state):
        self.pose = pose  # PbdPose object
        self.gripper_state = gripper_state  # 'open' or 'closed'

class PbdProgram:
    """Represents a complete PbD program."""
    def __init__(self, name):
        self.name = name
        self.steps = []
    
    def add_step(self, step):
        self.steps.append(step)
    
    def save_to_file(self, filename):
        with open(filename, 'wb') as f:
            pickle.dump(self, f)
    
    @staticmethod
    def load_from_file(filename):
        with open(filename, 'rb') as f:
            return pickle.load(f)