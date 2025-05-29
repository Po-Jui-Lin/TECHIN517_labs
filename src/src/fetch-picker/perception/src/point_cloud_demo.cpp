#include "perception/crop.h"
#include "perception/downsample.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  
  // Create publishers
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  ros::Publisher downsample_pub =
      nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
  
  // Create processors
  perception::Cropper cropper(crop_pub);
  perception::Downsampler downsampler(downsample_pub);
  
  // Set up the pipeline: original cloud -> cropper -> downsampler
  ros::Subscriber crop_sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
  ros::Subscriber downsample_sub =
      nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);
  
  ros::spin();
  return 0;
}