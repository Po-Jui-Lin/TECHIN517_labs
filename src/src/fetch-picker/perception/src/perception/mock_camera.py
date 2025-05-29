# mock_camera.py
import rospy
import rosbag
from sensor_msgs.msg import PointCloud2, CameraInfo

class MockCamera(object):
    """A MockCamera reads saved point clouds from bag files."""

    def __init__(self):
        self.cloud_pub = rospy.Publisher(
            '/head_camera/depth_registered/points',
            PointCloud2, queue_size=1, latch=True
        )
        self.info_pub = rospy.Publisher(
            '/head_camera/rgb/camera_info',
            CameraInfo, queue_size=1, latch=True
        )

    def read_cloud(self, path):
        """
        Returns the sensor_msgs/PointCloud2 and CameraInfo from the bag file.
        """
        try:
            bag = rosbag.Bag(path, 'r')
            cloud_msg = None
            info_msg = None

            # (Optional) debug: list topics in the bag
            # print("Available topics:", bag.get_type_and_topic_info().topics.keys())

            for topic, msg, t in bag.read_messages():
                clean = topic.lstrip('/')  # strip any leading slash
                if clean == 'head_camera/depth_registered/points' and cloud_msg is None:
                    cloud_msg = msg
                elif clean == 'head_camera/rgb/camera_info' and info_msg is None:
                    info_msg = msg

                if cloud_msg and info_msg:
                    break

            bag.close()
            return cloud_msg, info_msg

        except Exception as e:
            rospy.logerr(f"Error reading bag file: {e}")
            return None, None

    def publish_cloud(self, cloud_msg, info_msg):
        """Publish the cloud and camera info messages."""
        if cloud_msg:
            cloud_msg.header.stamp = rospy.Time.now()
            self.cloud_pub.publish(cloud_msg)

        if info_msg:
            info_msg.header.stamp = rospy.Time.now()
            self.info_pub.publish(info_msg)
