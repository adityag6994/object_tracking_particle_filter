#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/format.hpp>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <fstream>
#include <time.h>

struct pose{
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
};
using namespace pcl::tracking;


/***
NOTE : 
result_cloud : given point cloud
open camera node
run : rosrun tracker tracking_object /home/aditya/catkin_ws/src/tracker/src/teddy.pcd 
run  : rosrun rviz rviz
***/

/* Global Variables realted to low pass filter : with '__' */
double __T     = 0.3;
double __dt    = 0.1;
double __x     = 0;
double __y     = 0;
double __z     = 0;
double __roll  = 0;
double __pitch = 0;
double __yaw   = 0;
double __eXYZ  = 0.0001; //Check it !
double __eRPY  = 0.1;

/* To get the fps */
int __num_frames = 0, __current_count = 0;
time_t start, endl;

typedef pcl::PointXYZ PointType;
// typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZ RefPointType;
// typedef pcl::PointXYZRGB RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

pcl::PointCloud<PointType>::Ptr received_cloud_ptr;
sensor_msgs::PointCloud2 particle_cloud_msg, result_cloud_msg, down_cloud_msg;
ros::Publisher pub_praticles_cloud,pub_result_cloud, pub_down_cloud;
ros::Publisher pub_pose;
boost::shared_ptr<pcl::PointCloud<PointType> > result_cloud;

CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud;

boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int counter;

/*Low Pass Filter :To smoothen the pose estimate*/
double lwoPassFilterXYZ(double x, double y0, double dt, double T){
	double res = y0 + (x - y0) * (dt/(dt+T));
	if((res*res) <= __eXYZ){
		res = 0;
	}
	return res;
}

double lwoPassFilterRPY(double x, double y0, double dt, double T){
	double res = y0 + (x - y0) * (dt/(dt+T));
	if((res*res) <= __eRPY){
		res = 0;
	}
	return res;
}

void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (00.0, 10.0);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
}

void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
    pcl::ApproximateVoxelGrid<PointType> grid;
    grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
}

//Draw the current particles
// bool drawParticles ()
// {
//     ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
//     if (particles && new_cloud_)
//     {
//         //Set pointCloud with particle's points
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
//         for (size_t i = 0; i < particles->points.size (); i++)
//         {
//             pcl::PointXYZRGB point(255,0,0);

//             point.x = particles->points[i].x;
//             point.y = particles->points[i].y;
//             point.z = particles->points[i].z;
//             particle_cloud->points.push_back (point);
//         }

//         //Draw red particles
//         {

//             pcl::toROSMsg(*particle_cloud.get(),particle_cloud_msg );
//             particle_cloud_msg.header.frame_id = "royale_camera_optical_frame";
//             pub_praticles_cloud.publish(particle_cloud_msg );

//         }
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }



double x,y,z;
double quaternion_x,quaternion_y,quaternion_z,quaternion_w;
//ofstream myfile;
//myfile.open ("example.txt");
//std::ofstream outFile;
    
void publishObjectPose(Eigen::Affine3f &transformation)
{
    Eigen::Matrix3f rotationMatrix=transformation.rotation();

    x=transformation.translation().x();
    y=transformation.translation().y();
    z=transformation.translation().z();
    // Eigen::Quaternionf quaternion(rotationMatrix);
    // quaternion_x=quaternion.x();
    // quaternion_y=quaternion.y();
    // quaternion_z=quaternion.z();
    // quaternion_w=quaternion.w();
    float roll = atan2( rotationMatrix(2,1),rotationMatrix(2,2) );
    float pitch = atan2( -rotationMatrix(2,0), std::pow( rotationMatrix(2,1)*rotationMatrix(2,1) +rotationMatrix(2,2)*rotationMatrix(2,2) ,0.5  )  );
    float yaw = atan2( rotationMatrix(1,0),rotationMatrix(0,0) );
    float pi = 3.14;
    // std::cout<<"x: " << x <<std::endl;
    // std::cout<<"y: " << y <<std::endl;
    // std::cout<<"z: " << z <<std::endl;
    // std::cout<<"roll is:" << (roll*180)/pi << " | " << roll << std::endl;
    // std::cout<<"pitch is:" << (pitch*180)/pi << " | " << pitch << std::endl;
    // std::cout<<"yaw is:" << (yaw*180)/pi << " | " << yaw << std::endl;
    // std::cout << x << " " << y << " " << " " << z << " " << (roll*180)/pi << " " << (pitch*180)/pi << " " << (yaw*180)/pi << std::endl;
	
    // After applying low pass filter
    __x = lwoPassFilterXYZ(x, __x, __dt, __T);
    __y = lwoPassFilterXYZ(y, __y, __dt, __T);
    __z = lwoPassFilterXYZ(z, __z, __dt, __T);
    __roll  = lwoPassFilterRPY((roll*180)/pi, __roll, __dt, __T);
    __pitch = lwoPassFilterRPY((pitch*180)/pi, __pitch, __dt, __T);
    __yaw   = lwoPassFilterRPY((yaw*180)/pi, __yaw, __dt, __T);
    // std::cout << x << " " << y << " " << z << " " << (roll*180)/pi << " " << (pitch*180)/pi << " " << (yaw*180)/pi <<  " " << __x <<  " " << __y <<  " " << __z << " " << (__roll*180)/pi << " " << (__pitch*180)/pi << " " << (__yaw*180)/pi << std::endl;
    std::cout << x << " " << y << " " << z << " " << (roll*180)/pi << " " << (pitch*180)/pi << " " << (yaw*180)/pi <<  " " << __x <<  " " << __y <<  " " << __z << " " << __roll << " " << (__pitch) << " " << (__yaw) << std::endl;
	
	// std::cout<<"quaternion_x is:" <<quaternion_x <<std::endl;
    // std::cout<<"quaternion_y is:" <<quaternion_y <<std::endl;
    // std::cout<<"quaternion_z is:" <<quaternion_z <<std::endl;
    // std::cout<<"quaternion_w is:" <<quaternion_w <<std::endl;
    //myfile << x << " " << y << " " << z << " \n";
    

}

//Draw model reference point cloud
void drawResult ()
{
    ParticleT result = tracker_->getResult ();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

    publishObjectPose(transformation);


    //move close to camera a little for better visualization
    transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
//    CloudPtr result_cloud (new Cloud ());
    result_cloud.reset(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

    //Draw blue model reference point cloud
    {
        pcl::toROSMsg(*result_cloud,result_cloud_msg );
        result_cloud_msg.header.frame_id = "royale_camera_optical_frame";
        pub_result_cloud.publish(result_cloud_msg );
    }
}


void cloud_cb (const CloudConstPtr &cloud)
{
    cloud_pass_.reset (new Cloud);
    cloud_pass_downsampled_.reset (new Cloud);
    filterPassThrough (cloud, *cloud_pass_);
    gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

    pcl::toROSMsg(*cloud_pass_downsampled_,down_cloud_msg );
    // cloud_pass_downsampled_.header.frame_id = "/royale_camera_link_down";
    pub_down_cloud.publish(down_cloud_msg );

    if(counter < 10)
    {
        counter++;
    }else
    {
        //Track the object
        tracker_->setInputCloud (cloud_pass_downsampled_);
        tracker_->compute ();
        new_cloud_ = true;
    }
    // drawParticles();
}


template <typename PointT>
void computeMomentOfInertia(boost::shared_ptr<pcl::PointCloud<PointT> > cloud_ptr, Eigen::Vector3f &mass_center)
{

    pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
    feature_extractor.setInputCloud (cloud_ptr);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    //Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);
/*
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.1);
    viewer->initCameraParameters ();
    viewer->addPointCloud<pcl::PointXYZ> (no_plane_cloud, "sample cloud");

    //enable these lines to draw eigenvectors
    viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
*/

}



void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud_msg)
{
//    std::cout<<"point cloud recieved" <<std::endl;
//    printf ("Cloud: width = %d, height = %d\n", pointcloud_msg->width, pointcloud_msg->height);
    	
    received_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*pointcloud_msg.get(), *received_cloud_ptr.get());


    cloud_cb (received_cloud_ptr);
//    drawParticles();
    drawResult();




    // Eigen::Vector3f mass_center;
    //computeMomentOfInertia<pcl::PointXYZ>(result_cloud, mass_center);
    //x=mass_center(0,0) ;
    //y=mass_center(1,0);
    //z=mass_center(2,0);
    //std::cout << "x : " << x << " || y : " << y << " || z : " << x << std::endl;
}

// This function displays the help
void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

int main(int argc, char **argv)
{

    // Show help
    if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
    }


    ros::init(argc,argv,"tracking_object");
    ros::NodeHandle nh;
    std::string pointcloud_topic="/royale_camera_driver/point_cloud";
    ros::Subscriber sub_pcl= nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic,1 , pointcloudCallback);
    pub_praticles_cloud=nh.advertise<sensor_msgs::PointCloud2>("particles_cloud",1);
    pub_result_cloud=nh.advertise<sensor_msgs::PointCloud2>("result_cloud",1);
    pub_down_cloud=nh.advertise<sensor_msgs::PointCloud2>("cloud_pass_downsampled_",1);
    // pub_pose = nh.advertise<pose>("current_pose", 1);
    target_cloud.reset(new Cloud());
    pcl::PLYReader PLYFileReader;
   
    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    std::vector<int> filenames;
    bool file_is_pcd = false;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");	
    if (filenames.size () != 1)  {
    	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    	if (filenames.size () != 1) {
    	  showHelp (argv[0]);
    	  return -1;
    	} else {
    	  file_is_pcd = true;
    	}
    }


   if (file_is_pcd) {
    if (pcl::io::loadPCDFile (argv[filenames[0]], *target_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
   } else {
    if (pcl::io::loadPLYFile (argv[filenames[0]], *target_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
   }

   //Downsample in out cloud


//    if(PLYFileReader.read< pcl::PointXYZRGB >("/home/aditya/catkin_ws/src/pcd_files/tracking_models/spary.ply",*target_cloud) == -1)
 //  {
        ROS_INFO(" * ");
//	  std::cout << "pcd file not found" << std::endl;
 //       exit(-1);
  // }

    counter = 0;

    //Set parameters
    new_cloud_  = false;
    // downsampling_grid_size_ =  0;
    downsampling_grid_size_ =  0.0001;
    // downsampling_grid_size_ =  0.002;

    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
    (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;

    //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    //tracker->setMaximumParticleNum (1000);
    tracker->setMaximumParticleNum (500);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    tracker->setBinSize (bin_size);

    //Set all parameters for  ParticleFilter
    tracker_ = tracker;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    // tracker_->setParticleNum (600);
    tracker_->setParticleNum (300);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);

    //Setup coherence object for tracking
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
    (new ApproxNearestPairPointCloudCoherence<RefPointType> ());

    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
    = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    coherence->addPointCoherence (distance_coherence);

    boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);

    tracker_->setCloudCoherence (coherence);

    //prepare the model of tracker's target
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    CloudPtr transed_ref (new Cloud);
    CloudPtr transed_ref_downsampled (new Cloud);

    pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
    trans.translation ().matrix () = Eigen::Vector3f (c[1], c[2], c[0]);
    std::cout << "x : " << c[0] << " :: y : " << c[1] << " :: z : " << c[2] << std::endl;
    pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
    gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

    //set reference model and trans
    tracker_->setReferenceCloud (transed_ref_downsampled);
    tracker_->setTrans (trans);

    tf::TransformBroadcaster br;
    tf::Transform transform2;

    ros::Rate loop_rate(40);
    while(ros::ok())
    {
	  //ROS_INFO(" 9 ");
        ros::spinOnce();
	  //ROS_INFO(" 10 ");		
        loop_rate.sleep();
	  //ROS_INFO(" 11 ");		
        transform2.setOrigin( tf::Vector3(x, y, z) );
	  //ROS_INFO(" 12 ");		
        transform2.setRotation( tf::Quaternion( quaternion_x,quaternion_y,quaternion_z,quaternion_w  ) );
	  //ROS_INFO(" 13 ");		
        br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "royale_camera_optical_frame", "object"));
	  //ROS_INFO(" 14 ");		

    }
    // myfile.close();
    // std::cout << "finished!!" << std::endl;
     /**/
    return 0;
}
