#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
import perception
import rospy
import sys
import os

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('publish_instructor_cloud')
    wait_for_time()

    if len(sys.argv) < 2:
        print('Usage: rosrun applications publish_instructor_cloud.py <bag_file>')
        return

    bag_path = sys.argv[1]
    if not os.path.exists(bag_path):
        rospy.logerr(f'Bag file not found: {bag_path}')
        return

    camera = perception.MockCamera()
    cloud_msg, info_msg = camera.read_cloud(bag_path)

    if cloud_msg is None:
        rospy.logerr('Failed to read point cloud from bag file')
        return

    rospy.loginfo(f'Found camera info: {info_msg is not None}')
    rospy.loginfo(f'Publishing point cloud from {bag_path}')

    pub = rospy.Publisher('mock_point_cloud', PointCloud2, queue_size=1, latch=True)
    rate = rospy.Rate(2)  # 2 Hz

    try:
        while not rospy.is_shutdown():
            cloud_msg.header.stamp = rospy.Time.now()
            pub.publish(cloud_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
