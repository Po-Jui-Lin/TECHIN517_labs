#!/usr/bin/env python

from moveit_python import PlanningSceneInterface
import rospy

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('lab26_obstacles')
    wait_for_time()
    
    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    
    # Remove any existing collision objects
    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('floor')
    
    # Add floor
    planning_scene.addBox('floor', 2, 2, 0.01, 0, 0, 0.01/2)
    
    # Add table
    # Adjust these values based on your setup
    table_size_x = 0.5
    table_size_y = 1
    table_size_z = 0.72
    table_x = 1.0
    table_y = 0.0
    table_z = table_size_z / 2
    
    planning_scene.addBox('table', table_size_x, table_size_y, table_size_z, 
                         table_x, table_y, table_z)
    
    rospy.sleep(2)
    rospy.loginfo("Added obstacles to planning scene")

if __name__ == '__main__':
    main()