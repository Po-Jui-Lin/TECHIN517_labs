#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import actionlib
import control_msgs.msg
import trajectory_msgs
import rospy

ACTION_NAME = 'torso_controller/follow_joint_trajectory'
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            ACTION_NAME,
            control_msgs.msg.FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for torso controller...')
        self._client.wait_for_server()
        rospy.loginfo('Torso controller connected!')

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory

        # TODO: Send goal
        # TODO: Wait for result

        # Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if height < Torso.MIN_HEIGHT:
            rospy.logwarn('Requested height {} is less than min height {}'.format(
                height, Torso.MIN_HEIGHT))
            height = Torso.MIN_HEIGHT
        elif height > Torso.MAX_HEIGHT:
            rospy.logwarn('Requested height {} is greater than max height {}'.format(
                height, Torso.MAX_HEIGHT))
            height = Torso.MAX_HEIGHT
            
        # Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        # Set position of trajectory point
        point.positions = [height]
        # Set time of trajectory point
        point.time_from_start = rospy.Duration(TIME_FROM_START)

        # Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # Add joint name to list
        goal.trajectory.joint_names = [JOINT_NAME]
        # Add the trajectory point created above to trajectory
        goal.trajectory.points.append(point)

        # Send goal
        self._client.send_goal(goal)
        # Wait for result
        self._client.wait_for_result()