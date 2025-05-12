import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg  

from .arm_joints import ArmJoints

from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveGroupAction, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

from moveit_msgs.msg import OrientationConstraint

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # Create actionlib client
        self._client = actionlib.SimpleActionClient(
            'arm_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction
        )
        # Wait for server
        self._client.wait_for_server()

        self._move_group_client = actionlib.SimpleActionClient(
            'move_group',
            MoveGroupAction
        )
        # Wait for move group server
        self._move_group_client.wait_for_server()
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        
        # Set position of trajectory point
        point.positions = arm_joints.values()
        
        # Set time of trajectory point (5 seconds as per lab instructions)
        point.time_from_start = rospy.Duration(5.0)

        # Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        
        # Add joint names to list
        goal.trajectory.joint_names = ArmJoints.names()
        
        # Add the trajectory point created above to trajectory
        goal.trajectory.points.append(point)

        # Send goal
        self._client.send_goal(goal)
        
        # Wait for result
        self._client.wait_for_result()
        
        return self._client.get_result()
    
    def move_to_pose(self,
             pose_stamped,
             allowed_planning_time=10.0,
             execution_timeout=15.0,
             group_name='arm',
             num_planning_attempts=1,
             plan_only=False,
             replan=False,
             replan_attempts=5,
             tolerance=0.01,
             orientation_constraint=None
             ):
        """Moves the end-effector to a pose, using motion planning."""
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance

        if orientation_constraint is not None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)
        goal = goal_builder.build()
        
        # Send the goal
        self._move_group_client.send_goal(goal)
        # Use execution_timeout for wait_for_result()
        self._move_group_client.wait_for_result(rospy.Duration(execution_timeout))
        
        # Process the result
        result = self._move_group_client.get_result()
        
        if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
            return None
        else:
            error_code = result.error_code.val if result else MoveItErrorCodes.FAILURE
            return self.moveit_error_string(error_code)  # Use self. if you choose Option 1
    
    def cancel_all_goals(self):
        """Cancels all goals for both the joint trajectory and move group actions."""
        self._client.cancel_all_goals()  # existing action client from Lab 7
        self._move_group_client.cancel_all_goals()  # The MoveGroup client
    
    def moveit_error_string(self, val):
        """Returns a string associated with a MoveItErrorCode.
            
        Args:
            val: The val field from moveit_msgs/MoveItErrorCodes.msg
            
        Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
            if the value is invalid.
        """ 
        if val == MoveItErrorCodes.SUCCESS:
            return 'SUCCESS'
        elif val == MoveItErrorCodes.FAILURE:
            return 'FAILURE'
        elif val == MoveItErrorCodes.PLANNING_FAILED:
            return 'PLANNING_FAILED'
        elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
            return 'INVALID_MOTION_PLAN'
        elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
        elif val == MoveItErrorCodes.CONTROL_FAILED:
            return 'CONTROL_FAILED'
        elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
            return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
        elif val == MoveItErrorCodes.TIMED_OUT:
            return 'TIMED_OUT'
        elif val == MoveItErrorCodes.PREEMPTED:
            return 'PREEMPTED'
        elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
            return 'START_STATE_IN_COLLISION'
        elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
            return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
        elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
            return 'GOAL_IN_COLLISION'
        elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
            return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
        elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
            return 'GOAL_CONSTRAINTS_VIOLATED'
        elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
            return 'INVALID_GROUP_NAME'
        elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
            return 'INVALID_GOAL_CONSTRAINTS'
        elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
            return 'INVALID_ROBOT_STATE'
        elif val == MoveItErrorCodes.INVALID_LINK_NAME:
            return 'INVALID_LINK_NAME'                                      
        elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
            return 'INVALID_OBJECT_NAME'
        elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
            return 'FRAME_TRANSFORM_FAILURE'
        elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
            return 'COLLISION_CHECKING_UNAVAILABLE'
        elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
            return 'ROBOT_STATE_STALE'
        elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
            return 'SENSOR_INFO_STALE'
        elif val == MoveItErrorCodes.NO_IK_SOLUTION:
            return 'NO_IK_SOLUTION'
        else:
            return 'UNKNOWN_ERROR_CODE'
        

    def check_pose(self, 
               pose_stamped,
               allowed_planning_time=10.0,
               group_name='arm',
               tolerance=0.01):
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)
    
    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose.
        
        Note: if you are interested in returning the IK solutions, we have
        shown how to access them.
        
        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.
                
        Returns: True if the inverse kinematics were found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)
        error_str = self.moveit_error_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        joint_state = response.solution.joint_state
        for name, position in zip(joint_state.name, joint_state.position):
            if name in ArmJoints.names():
                rospy.loginfo('{}: {}'.format(name, position))
        return True