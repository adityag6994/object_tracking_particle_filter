#include <iostream>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

// jun 10 : 12:14 pm : voxel grid based downsampling working.
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for orignal and filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data here
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(cloud_filtered);  

  // Convert to ROS datatype
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  pub.publish(output);
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  // ros::Subscriber sub = nh.subscribe ("camera/depth/color/point", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
