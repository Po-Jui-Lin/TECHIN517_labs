#! /usr/bin/env python

# TODO: import ????????_msgs.msg
import rospy
from geometry_msgs.msg import Twist # Import the Twist message
import copy
import math
import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
       # Create a publisher to the cmd_vel topic
       self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
       self._latest_odom = None
       self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # Create a Twist message
        msg = Twist()
        # Set linear velocity (x-direction)
        msg.linear.x = linear_speed
        # Set angular velocity (around z-axis)
        msg.angular.z = angular_speed
        # Publish the message
        self.pub.publish(msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # Publish 0 velocity
        self.move(0, 0)
        
    def _quaternion_to_yaw(self, q):
        """Converts a quaternion to a yaw angle (rotation about the z-axis).
        
        Args:
            q: A geometry_msgs/Quaternion message.
            
        Returns:
            The yaw angle in radians.
        """
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        x = m[0, 0]  # The x value of the x-axis (first column)
        y = m[1, 0]  # The y value of the x-axis
        return math.atan2(y, x)
    
    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.
        
        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.
        
        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # Wait until we've received at least one message on /odom
        while self._latest_odom is None:
            rospy.sleep(0.1)
        
        # Record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._latest_odom)
        rate = rospy.Rate(10)
        
        # Keep track of the distance traveled
        direction = -1 if distance < 0 else 1
        target_distance = abs(distance)
        distance_traveled = 0
        
        # Move until we've traveled the desired distance
        while distance_traveled < target_distance and not rospy.is_shutdown():
            # Get current position
            current = self._latest_odom
            
            # Calculate distance traveled
            dx = current.pose.pose.position.x - start.pose.pose.position.x
            dy = current.pose.pose.position.y - start.pose.pose.position.y
            distance_traveled = math.sqrt(dx*dx + dy*dy)
            
            # Calculate remaining distance and adjust speed if close to target
            remaining = target_distance - distance_traveled
            current_speed = min(speed, max(0.05, remaining))
            
            # Send velocity command
            self.move(direction * current_speed, 0)
            rate.sleep()
        
        # Stop the robot
        self.move(0, 0)

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.
        
        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # Wait until we've received at least one message on /odom
        while self._latest_odom is None:
            rospy.sleep(0.1)
        
        # Record start position
        start = copy.deepcopy(self._latest_odom)
        
        # Normalize angular_distance to be between -2π and 2π
        angular_distance = angular_distance % (2 * math.pi) if angular_distance > 0 else -((-angular_distance) % (2 * math.pi))
        
        # Get the starting yaw
        start_yaw = self._quaternion_to_yaw(start.pose.pose.orientation)
        
        # Calculate the target yaw
        target_yaw = start_yaw + angular_distance
        
        # Direction of rotation
        direction = -1 if angular_distance < 0 else 1
        
        rate = rospy.Rate(10)
        
        # Keep track of how much we've rotated
        angle_turned = 0
        target_angle = abs(angular_distance)
        
        while angle_turned < target_angle and not rospy.is_shutdown():
            # Get current position
            current = self._latest_odom
            current_yaw = self._quaternion_to_yaw(current.pose.pose.orientation)
            
            # Calculate the angle turned
            # This is tricky due to the wraparound nature of angles
            # We need to handle cases where we cross the -π/π boundary
            
            # Calculate the difference between current and start yaw
            delta_yaw = current_yaw - start_yaw
            
            # Normalize to [-π, π]
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
                
            # Take the absolute value to get distance turned
            angle_turned = abs(delta_yaw)
            
            # Calculate remaining angle and adjust speed if close to target
            remaining = target_angle - angle_turned
            current_speed = min(speed, max(0.25, remaining))
            
            # Send velocity command
            self.move(0, direction * current_speed)
            rate.sleep()
        
        # Stop the robot
        self.move(0, 0)