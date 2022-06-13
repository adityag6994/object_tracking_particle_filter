#include <iostream>
#include <vector>
#include <fstream>
#include <limits>
#include <Eigen/Core>
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
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

ros::Publisher pub;

// jun 10 : 12:14 pm : voxel grid based downsampling working.
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for orignal and filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 *cloud_filtered (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2 *cloud_filtered_passthrough (new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2ConstPtr cloud_filteredPtr(cloud_filtered);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_plane (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Convert to PCL data here
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Downsize
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.02, 0.02, 0.02);
  sor.filter(*cloud_filtered);  

  // Filter Pass through
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloud_filteredPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-2.0, 2.0);
  pass.filter (*cloud_filtered_passthrough);

  // // Convert from PointCloud2 -> PointXYZRGB format, that is required for RANSAC
  // // pcl::fromPCLPointCloud2 (*cloud_filtered, *cloud_filtered_plane); // voxelized
  // pcl::fromPCLPointCloud2 (*cloud_filtered_passthrough, *cloud_filtered_plane); // voxelized + passthrough

  // // Find Plane 
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  // seg.setDistanceThreshold (0.02);
  // seg.setInputCloud (cloud_filtered_plane);

  // // Extract Plane using Inliers
  // int i = 0, nr_points = (int) cloud_filtered_plane->points.size ();
  // pcl::IndicesPtr remaining (new std::vector<int>);
  // remaining->resize (nr_points);
  // for (size_t i = 0; i < remaining->size (); ++i) { 
  //   (*remaining)[i] = static_cast<int>(i);
  // }

  // std::cout << "Points after downsampling : " << cloud_filtered_plane->points.size() << std::endl;

  // // While 30% of the original cloud is still there
  // while (remaining->size () > 0.9 * nr_points)
  // {
  //   // Segment the largest planar component from the remaining cloud
  //   seg.setIndices (remaining);
  //   seg.segment (*inliers, *coefficients);
  //   if (inliers->indices.size () == 0) break;

  //   // Extract the inliers
  //   std::vector<int>::iterator it = remaining->begin();
  //   for (size_t i = 0; i < inliers->indices.size (); ++i){
  //     int curr = inliers->indices[i];
  //     // Remove it from further consideration.
  //     while (it != remaining->end() && *it < curr) { ++it; }
  //     if (it == remaining->end()) break;
  //     if (*it == curr) it = remaining->erase(it);
  //     }
  //     i++;
  //   }

  //   std::cout << "Found " << i << " planes." << std::endl;

  // // Color points in plane
  // for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it)
  // {
  //   uint8_t r = 0, g = 255, b = 0;
  //   uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  //   cloud_filtered_plane->at(*it).rgb = *reinterpret_cast<float*>(&rgb);
  // }

  // pcl::PCLPointCloud2 outcloud; 
  // pcl::toPCLPointCloud2 (*cloud_filtered_plane, outcloud);
  
  // Convert to ROS datatype
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(*cloud_filtered_passthrough, output);

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