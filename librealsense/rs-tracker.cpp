// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../../../examples/example.hpp" // Include short list of convenience functions for rendering

#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <stdio.h>

#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
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

#include <boost/format.hpp>
#include <pcl/pcl_config.h>

#include <mutex>
#include <thread>

using namespace pcl::tracking;

using RefPointType = pcl::PointXYZ;
using ParticleT = ParticleXYZRPY;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
// using CloudPtr = Cloud::Ptr;
using pcl_ptr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;
// using ParticleFilter = ParticleFilterTracker<RefPointType, ParticleT>;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter; 

// using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using pcl_nml = pcl::PointCloud<pcl::Normal>::Ptr;

pcl_ptr cloud_pass_;
pcl_ptr cloud_pass_downsampled_;
pcl_ptr target_cloud;

std::mutex mtx_;
// ParticleFilter::Ptr tracker_;
boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int counter;

void filterPassThrough(const CloudConstPtr &cloud, pcl_ptr &result){
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0.9, 1.0);
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (*result);
}

void gridSampleApprox (const CloudConstPtr &cloud, pcl_ptr &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (*result);
}

void filterPassThroughO(const CloudConstPtr &cloud, Cloud &result){
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (result);
}

void gridSampleApproxO(const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

//OpenNI Grabber's cloud Callback function
void cloud_cb (const CloudConstPtr &cloud)
{
  std::lock_guard<std::mutex> lock (mtx_);
  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);
  filterPassThroughO (cloud, *cloud_pass_);
  gridSampleApproxO (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

  if(counter < 10){
    counter++;
  }else{
    //Track the object
    tracker_->setInputCloud (cloud_pass_downsampled_);
    tracker_->compute ();
    new_cloud_ = true;
  }
}

// Struct for managing rotation of pointcloud view
struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
        ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; 
};

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points);

pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

float3 colors[] { { 0.8f, 0.1f, 0.3f }, 
                  { 0.1f, 0.9f, 0.5f },
                };

int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense PCL Pointcloud Example");
    // Construct an object to manage view state
    state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    // std::cout << "PCL_VERSION : " << PCL_VERSION << std::endl; 
    // std::cout << "PCL_VERSION_PRETTY : " <<PCL_VERSION_PRETTY << std::endl; # PCL_VERSION_PRETTY : 1.7.2
    
    //read pcd file
    target_cloud.reset(new Cloud());
    if(pcl::io::loadPCDFile (argv[1], *target_cloud) == -1){
        std::cout << "pcd file not found" << std::endl;
        exit(-1);
    } 

    // std::string device_id = std::string (argv[1]);

    counter = 0;
    new_cloud_ = false;
    downsampling_grid_size_ =  0.01f;
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    // KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>::Ptr tracker
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
    tracker->setMaximumParticleNum (1000);
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
    tracker_->setParticleNum (600);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);

    //Setup coherence object for tracking
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence
    (new ApproxNearestPairPointCloudCoherence<RefPointType>);

    DistanceCoherence<RefPointType>::Ptr distance_coherence
    (new DistanceCoherence<RefPointType>);
    coherence->addPointCoherence (distance_coherence);

    pcl::search::Octree<RefPointType>::Ptr search (new pcl::search::Octree<RefPointType> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);

    tracker_->setCloudCoherence (coherence);

    //prepare the model of tracker's target
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    pcl_ptr transed_ref (new Cloud);
    pcl_ptr transed_ref_downsampled (new Cloud);

    pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
    trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
    pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
    gridSampleApproxO (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

    //set reference model and trans
    tracker_->setReferenceCloud (transed_ref_downsampled);
    tracker_->setTrans (trans);

    // pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");
    // viewer_->runOnVisualizationThread (viz_cb, "viz_cb");
    
    while (app) // Application still alive?
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        auto pcl_points = points_to_pcl(points);

        pcl_ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_nml cloud_normals (new pcl::PointCloud<pcl::Normal> ());

        //Voxelisation
        // gridSampleApprox(pcl_points, cloud_ptr, 0.01f);

        //Filter Passthrough
        // pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        // filterPassThrough(cloud_ptr,cloud_filtered);

        cloud_cb(pcl_points);
        
        ParticleXYZRPY result = tracker_->getResult ();
        Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
        transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
        pcl_ptr result_cloud1 (new Cloud ());
        pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud1, transformation);
        // pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud1, 0, 0, 255);

        std::vector<pcl_ptr> layers;

        ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
        if (particles && new_cloud_){
            //Set pointCloud with particle's points
            pcl_ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
            for (const auto& particle: *particles){
                pcl::PointXYZ point;
                point.x = particle.x;
                point.y = particle.y;
                point.z = particle.z;
                particle_cloud->push_back (point);
            }            
            layers.push_back(particle_cloud);
            // std::cerr << "PointCloud particle_cloud: " << particle_cloud->width * particle_cloud->height << " data points (" << pcl::getFieldsList (*particle_cloud) << ")." << std::endl; //58067
        }

        // std::cerr << "PointCloud cloud_pass_downsampled_: " << cloud_pass_downsampled_->width * cloud_pass_downsampled_->height << " data points (" << pcl::getFieldsList (*cloud_pass_downsampled_) << ")." << std::endl; //407040
        // std::cerr << "PointCloud result_cloud1: " << result_cloud1->width * result_cloud1->height << " data points (" << pcl::getFieldsList (*cloud_ptr) << ")." << std::endl; //58067
        // std::cerr << "PointCloud particle_cloud: " << particle_cloud->width * particle_cloud->height << " data points (" << pcl::getFieldsList (*particle_cloud) << ")." << std::endl; //58067
        // std::cerr << "PointCloud cloud_ptr: " << cloud_ptr->width * cloud_ptr->height << " data points (" << pcl::getFieldsList (*cloud_ptr) << ")." << std::endl; //58067
        // std::cerr << "PointCloud cloud_filtered: " << cloud_filtered->width * cloud_filtered->height << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl; //2815

        layers.push_back(cloud_pass_downsampled_);
        layers.push_back(result_cloud1);
        draw_pointcloud(app, app_state, layers);
    }

    std::cout << " Finish " << std::endl;
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window& app, state& app_state)
{
    app.on_left_mouse = [&](bool pressed)
    {
        app_state.ml = pressed;
    };

    app.on_mouse_scroll = [&](double xoffset, double yoffset)
    {
        app_state.offset_x += static_cast<float>(xoffset);
        app_state.offset_y += static_cast<float>(yoffset);
    };

    app.on_mouse_move = [&](double x, double y)
    {
        if (app_state.ml)
        {
            app_state.yaw -= (x - app_state.last_x);
            app_state.yaw = std::max(app_state.yaw, -120.0);
            app_state.yaw = std::min(app_state.yaw, +120.0);
            app_state.pitch += (y - app_state.last_y);
            app_state.pitch = std::max(app_state.pitch, -80.0);
            app_state.pitch = std::min(app_state.pitch, +80.0);
        }
        app_state.last_x = x;
        app_state.last_y = y;
    };

    app.on_key_release = [&](int key)
    {
        if (key == 32) // Escape
        {
            app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
        }
    };
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points)
{
    // OpenGL commands that prep screen for the pointcloud
    glPopMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    float width = app.width(), height = app.height();

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_TEXTURE_2D);

    int color = 0;

    for (auto&& pc : points)
    {
        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

        glBegin(GL_POINTS);
        glColor3f(c.x, c.y, c.z);

        /* this segment actually prints the pointcloud */
        for (int i = 0; i < pc->points.size(); i++)
        {
            auto&& p = pc->points[i];
            if (p.z)
            {
                // upload the point and texture coordinates only for points we have depth data for
                glVertex3f(p.x, p.y, p.z);
            }
        }

        glEnd();
    }

    // OpenGL cleanup
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}
