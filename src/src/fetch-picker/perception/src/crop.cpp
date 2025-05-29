#include "perception/crop.h"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/common.h"
#include "ros/ros.h"
#include <Eigen/Dense>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {

Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {}

void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  // Convert from ROS message to PCL point cloud
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  // Create a new point cloud for the cropped result
  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  
  // Get crop parameters from parameter server
  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param("crop_min_x", min_x, 0.3);
  ros::param::param("crop_min_y", min_y, -1.0);
  ros::param::param("crop_min_z", min_z, 0.5);
  ros::param::param("crop_max_x", max_x, 0.9);
  ros::param::param("crop_max_y", max_y, 1.0);
  ros::param::param("crop_max_z", max_z, 1.5);
  
  // Set up the crop box filter
  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
  
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  crop.filter(*cropped_cloud);
  
  ROS_INFO("Cropped to %ld points", cropped_cloud->size());
  
  // Convert back to ROS message
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cropped_cloud, msg_out);
  msg_out.header = msg.header;  // Preserve the original header
  
  // Publish the cropped cloud
  pub_.publish(msg_out);
  
  // Optional: Find min/max points for debugging
  if (cropped_cloud->size() > 0) {
    PointC min_pcl;
    PointC max_pcl;
    pcl::getMinMax3D<PointC>(*cropped_cloud, min_pcl, max_pcl);
    ROS_INFO("Crop bounds: min=(%f, %f, %f), max=(%f, %f, %f)", 
             min_pcl.x, min_pcl.y, min_pcl.z,
             max_pcl.x, max_pcl.y, max_pcl.z);
  }
}

} // namespace perception