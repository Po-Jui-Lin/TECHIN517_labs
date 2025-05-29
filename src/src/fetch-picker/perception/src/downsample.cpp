#include "perception/downsample.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/common.h"  // Added for getMinMax3D
#include <Eigen/Dense>  // For any vector operations if needed

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
Downsampler::Downsampler(const ros::Publisher& pub) : pub_(pub) {}

void Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Downsampling point cloud with %ld points", cloud->size());
  
  // Create output cloud
  PointCloudC::Ptr downsampled_cloud(new PointCloudC());
  
  // Perform downsampling
  pcl::VoxelGrid<PointC> vox;
  vox.setInputCloud(cloud);
  
  // Get parameter for voxel size
  double voxel_size;
  ros::param::param("voxel_size", voxel_size, 0.01);
  vox.setLeafSize(voxel_size, voxel_size, voxel_size);
  vox.filter(*downsampled_cloud);
  
  ROS_INFO("Downsampled to %ld points", downsampled_cloud->size());
  
  // Find min/max points for debugging
  if (downsampled_cloud->size() > 0) {
    PointC min_point;
    PointC max_point;
    pcl::getMinMax3D<PointC>(*downsampled_cloud, min_point, max_point);
    
    // Calculate center
    PointC center_point;
    center_point.x = (min_point.x + max_point.x) / 2;
    center_point.y = (min_point.y + max_point.y) / 2;
    center_point.z = (min_point.z + max_point.z) / 2;
    
    ROS_INFO("Min: (%f, %f, %f)", min_point.x, min_point.y, min_point.z);
    ROS_INFO("Max: (%f, %f, %f)", max_point.x, max_point.y, max_point.z);
    ROS_INFO("Center: (%f, %f, %f)", center_point.x, center_point.y, center_point.z);
  }
  
  // Convert to ROS message and publish
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*downsampled_cloud, msg_out);
  msg_out.header = msg.header;
  pub_.publish(msg_out);
}
} // namespace perception