/**
 * @file snap_shot_mode.cpp
 * @author Marc Eisoldt (meisoldt@uni-osnabrueck.de)
 * 
 * @brief Node to record a snap shot of all needed data during the Monte Carlo localization to be processed offline 
 * 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 */

#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <mcl/MCLConfig.h>

#include <random>
#include <cmath>
#include <vector>
#include <array>
#include <utility>

#include <particle_cloud.h>
#include <evaluation/model/naiv_evaluation.h>
#include <evaluation/model/likelihood_evaluation.h>
#include <evaluation/model/omp_likelihood_evaluation.h>

#include <map/hash_grid_map.h>
#include <map/sub_voxel_map.h>

#include <util/runtime_evaluator.h>

#include <map/map_util.h>
#include <util/util.h>

#include <util/mcl_file.h>

#include <iostream>
#include <string>

#include <boost/filesystem.hpp>

using namespace mcl;

std::vector<CudaPoint> free_map_;

// Reference to the node
std::shared_ptr<ros::NodeHandle> nh_p_;

geometry_msgs::Pose initial_pose_;
bool pose_initialized_ = false;

unsigned int number_particles_ = 800;
ParticleCloud particle_cloud_;

FLOAT_T init_sigma_x_;
FLOAT_T init_sigma_y_;
FLOAT_T init_sigma_z_;

FLOAT_T init_sigma_roll_;
FLOAT_T init_sigma_pitch_;
FLOAT_T init_sigma_yaw_;

int evaluation_model_;

std::string robot_frame_;
std::string scan_frame_;
std::string odom_frame_;
std::string map_frame_;

std::string save_dir = "mcl_snapshots/";

void responseCallback(mcl::MCLConfig& config, uint32_t level)
{
  number_particles_ = config.number_of_particles;
  
  init_sigma_x_ = config.init_sigma_x;
  init_sigma_y_ = config.init_sigma_y;
  init_sigma_z_ = config.init_sigma_z;
  init_sigma_roll_ = config.init_sigma_roll * M_PI / 180.0;
  init_sigma_pitch_ = config.init_sigma_pitch * M_PI / 180.0;
  init_sigma_yaw_ = config.init_sigma_yaw * M_PI / 180.0;

  evaluation_model_ = config.evaluation_model;

  robot_frame_ = config.robot_frame;
  scan_frame_ =  config.scan_frame;
  odom_frame_ = config.odom_frame;
  map_frame_ = config.map_frame;

  if(particle_cloud_.isInitialized())
  {
    particle_cloud_.resize(number_particles_);
  }
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_with_covariance)
{
  initial_pose_ = pose_with_covariance.pose.pose;

  initial_pose_.position.z = 0; //= -0.5;
  pose_initialized_ = true;

  ROS_INFO_STREAM("Initial pose received!");
}

void scanOdomCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud, const nav_msgs::Odometry::ConstPtr& odom)
{ 
  static tf2_ros::TransformBroadcaster broadcaster;
  static tf2_ros::Buffer tf_buffer;
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::TransformStamped scan_to_base; 

  /*** A priori ***/

  // Init particles
  if(pose_initialized_)
  {
    //particle_cloud_.initialize(initial_pose_, number_particles_, init_sigma_x_, init_sigma_y_, init_sigma_z_, init_sigma_roll_, init_sigma_pitch_, init_sigma_yaw_);
    particle_cloud_.initialize(number_particles_, free_map_, initial_pose_);
    pose_initialized_ = false;

    if(!boost::filesystem::exists(save_dir))
    {
        boost::filesystem::create_directory(save_dir);
        std::cout << "Create directory \"" << save_dir << '\"' << std::endl;
    }

    std::array<FLOAT_T, 16> tf_matrix;

    try
    {
      scan_to_base = tf_buffer.lookupTransform(robot_frame_, scan_frame_, cloud->header.stamp, ros::Duration(0.5));
    }
    catch (tf2::TransformException& e)
    {
      ROS_ERROR_STREAM("Couldn't transform robot frame to scan frame: " << e.what());
      throw;
    }

    tf2::Transform tf_scan_to_base;
    tf2::convert(scan_to_base.transform, tf_scan_to_base);

    tf_matrix[0] = tf_scan_to_base.getBasis()[0][0];
    tf_matrix[1] = tf_scan_to_base.getBasis()[0][1];
    tf_matrix[2] = tf_scan_to_base.getBasis()[0][2];
    tf_matrix[3] = tf_scan_to_base.getOrigin().getX();
    
    tf_matrix[4] = tf_scan_to_base.getBasis()[1][0];
    tf_matrix[5] = tf_scan_to_base.getBasis()[1][1];
    tf_matrix[6] = tf_scan_to_base.getBasis()[1][2];
    tf_matrix[7] = tf_scan_to_base.getOrigin().getY();
    
    tf_matrix[8] = tf_scan_to_base.getBasis()[2][0];
    tf_matrix[9] = tf_scan_to_base.getBasis()[2][1];
    tf_matrix[10] = tf_scan_to_base.getBasis()[2][2];
    tf_matrix[11] = tf_scan_to_base.getOrigin().getZ();

    std::vector<CudaPoint> points;
    points.reserve(cloud->width);
    std::vector<int> rings;
    rings.reserve(cloud->width);
    CudaPoint point;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<int> iter_ring(*cloud, "ring");

    for (; iter_x != iter_x.end(); ++iter_x)
    {
      point.x = iter_x[0];
      point.y = iter_x[1];
      point.z = iter_x[2];

      points.push_back(point);
      rings.push_back(iter_ring[0]);

      ++iter_ring;
    }

    std::cout << "Save current mcl data..." << std::endl;
    MCLFile file(save_dir + "mcl_" + std::to_string(cloud->header.stamp.toNSec()) + ".mcl");
    file.write(points, rings, particle_cloud_.particles(), tf_matrix, initial_pose_.position.x, initial_pose_.position.y, initial_pose_.position.z, initial_pose_.orientation.w, initial_pose_.orientation.x, initial_pose_.orientation.y, initial_pose_.orientation.z);
    std::cout << "Current mcl data saved" << std::endl;
  }
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
      std::cout << "usage: " << argv[0] << " <map-file>" << std::endl;
      return 0; 
    }

    ros::init(argc, argv, "snap_shot_node");
    ros::NodeHandle n;
    nh_p_.reset(new ros::NodeHandle("~"));

    auto map = createTSDFMap<CudaSubVoxelMap<FLOAT_T, FLOAT_T>, FLOAT_T, FLOAT_T>(argv[1], free_map_);
    std::cout << "Loaded free map" << std::endl;

    dynamic_reconfigure::Server<mcl::MCLConfig> server;
    dynamic_reconfigure::Server<mcl::MCLConfig>::CallbackType callbackType;

    callbackType = boost::bind(&responseCallback, _1, _2);
    server.setCallback(callbackType);

    // Meaningfull initialization
    initial_pose_.orientation.w = 1.0;

    // Initialize subscribers
    ros::Subscriber sub_initial_pose = n.subscribe("initialpose", 1, initialPoseCallback);

    // Initialize synchronized subscriber for the scan and odom topic
    message_filters::Subscriber<sensor_msgs::PointCloud2> scan2_sub(n, "velodyne_points", 1);

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(n, "odom", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), scan2_sub, odom_sub);
    sync.registerCallback(boost::bind(&scanOdomCallback, _1, _2)) ;

    std::cout << "Ready to save mcl data" << std::endl;

    ros::spin();
}