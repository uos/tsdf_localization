/**
 * @file tsdf_mcl.cpp
 * @author Marc Eisoldt (meisoldt@uni-osnabrueck.de)
 * @author Alexander Mock (amock@uni-osnabrueck.de)
 * 
 * @brief Node for an implementation of the global three dimensional Monte Carlo Localization with a three dimensional laser scanner and an odometry estimation in a three dimensional TSDF map
 * 
 * @version 0.2
 * @date 2025-04-01
 * 
 * @copyright Copyright (c) 2025
 */

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/path.hpp>

#include <std_srvs/srv/empty.hpp>

// #include <dynamic_reconfigure/server.h>
// #include <tsdf_localization/MCLConfig.h>

#include <random>
#include <cmath>
#include <vector>
#include <array>
#include <utility>
#include <fstream>

#include <tsdf_localization/particle_cloud.h>
#include <tsdf_localization/evaluation/model/naiv_evaluation.h>
#include <tsdf_localization/evaluation/model/likelihood_evaluation.h>
#include <tsdf_localization/evaluation/model/omp_likelihood_evaluation.h>

#include <tsdf_localization/resampling/wheel_resampler.h>
#include <tsdf_localization/resampling/novel_resampling.h>

#include <tsdf_localization/evaluation/tsdf_evaluator.h>
#include <tsdf_localization/map/hash_grid_map.h>
#include <tsdf_localization/cuda/cuda_sub_voxel_map.h>

#include <tsdf_localization/util/runtime_evaluator.h>

#include <tsdf_localization/map/map_util.h>
#include <tsdf_localization/util/util.h>

#include <tsdf_localization/util/imu_accumulator.h>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

namespace tsdf_localization
{

class TSDFMCLNode : public rclcpp::Node
{
public:
  TSDFMCLNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
  : Node("tsdf_mcl", options)
  {
    std::cout << "Create TSDF MCL Node..." << std::endl;

    float sigma = SIGMA;
    float a_hit = A_HIT;
    float a_rand = A_RAND;
    float a_max = A_MAX;
    float max_range = MAX_RANGE;

    // TODO: Init params
    // std::string cloud_topic;
    // std::string imu_topic;

    // Definining parameters and loading initial values
    {
      rcl_interfaces::msg::ParameterDescriptor map_file_pdesc; 
      map_file_pdesc.name = "map_file";
      map_file_pdesc.type = rclcpp::ParameterType::PARAMETER_STRING;  
      map_file_pdesc.description = "The path to a tsdf map (h5 format)";
      map_file_name_ = this->declare_parameter<std::string>(map_file_pdesc.name, "", map_file_pdesc);

      rcl_interfaces::msg::ParameterDescriptor map_frame_pdesc; 
      map_frame_pdesc.name = "map_frame";
      map_frame_pdesc.type = rclcpp::ParameterType::PARAMETER_STRING;  
      map_frame_pdesc.description = "Name of the map frame";
      map_frame_ = this->declare_parameter<std::string>(map_frame_pdesc.name, "map", map_frame_pdesc);

      rcl_interfaces::msg::ParameterDescriptor odom_frame_pdesc; 
      odom_frame_pdesc.name = "odom_frame";
      odom_frame_pdesc.type = rclcpp::ParameterType::PARAMETER_STRING;  
      odom_frame_pdesc.description = "Name of the odometry frame";
      odom_frame_ = this->declare_parameter<std::string>(odom_frame_pdesc.name, "odom", odom_frame_pdesc);

      rcl_interfaces::msg::ParameterDescriptor robot_frame_pdesc; 
      robot_frame_pdesc.name = "robot_frame";
      robot_frame_pdesc.type = rclcpp::ParameterType::PARAMETER_STRING;  
      robot_frame_pdesc.description = "Name of the robot frame";
      robot_frame_ = this->declare_parameter<std::string>(robot_frame_pdesc.name, "base_link", robot_frame_pdesc);

      rcl_interfaces::msg::ParameterDescriptor per_point_pdesc; 
      per_point_pdesc.name = "per_point";
      per_point_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
      per_point_pdesc.description = "Per point parallelization? Per particle parallelization otherwise.";
      per_point_ = this->declare_parameter<bool>(per_point_pdesc.name, false, per_point_pdesc);
    
      rcl_interfaces::msg::ParameterDescriptor init_global_pdesc; 
      init_global_pdesc.name = "init_global";
      init_global_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
      init_global_pdesc.description = "Initialize for global localization?";
      init_global_ = this->declare_parameter<bool>(init_global_pdesc.name, false, init_global_pdesc);
    
      rcl_interfaces::msg::ParameterDescriptor a_hit_pdesc; 
      a_hit_pdesc.name = "a_hit";
      a_hit_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
      a_hit_pdesc.description = "a hit";
      a_hit = this->declare_parameter<double>(a_hit_pdesc.name, (double)A_HIT, a_hit_pdesc);

      rcl_interfaces::msg::ParameterDescriptor a_rand_pdesc; 
      a_rand_pdesc.name = "a_rand";
      a_rand_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
      a_rand_pdesc.description = "a rand";
      a_rand = this->declare_parameter<double>(a_rand_pdesc.name, (double)A_RAND, a_rand_pdesc);

      rcl_interfaces::msg::ParameterDescriptor a_max_pdesc; 
      a_max_pdesc.name = "a_max";
      a_max_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
      a_max_pdesc.description = "a max";
      a_max = this->declare_parameter<double>(a_max_pdesc.name, (double)A_MAX, a_max_pdesc);

      rcl_interfaces::msg::ParameterDescriptor max_range_pdesc; 
      max_range_pdesc.name = "max_range";
      max_range_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
      max_range_pdesc.description = "max range";
      max_range = this->declare_parameter<double>(max_range_pdesc.name, (double)MAX_RANGE, max_range_pdesc);

      rcl_interfaces::msg::ParameterDescriptor use_imu_pdesc; 
      use_imu_pdesc.name = "use_imu";
      use_imu_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
      use_imu_pdesc.description = "Use the IMU information in the motion update. If not the odometry is used.";
      use_imu_ = this->declare_parameter<bool>(use_imu_pdesc.name, false, use_imu_pdesc);
    
      rcl_interfaces::msg::ParameterDescriptor use_os_pdesc; 
      use_os_pdesc.name = "use_os";
      use_os_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
      use_os_pdesc.description = "Use an ouster as input of the sensor update. If not a velodyne is used.";
      use_os_ = this->declare_parameter<bool>(use_os_pdesc.name, false, use_os_pdesc);
      
      rcl_interfaces::msg::ParameterDescriptor ignore_motion_pdesc; 
      ignore_motion_pdesc.name = "ignore_motion";
      ignore_motion_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
      ignore_motion_pdesc.description = "Ignores any sensor information in the motion update and only applies noise to every particle.";
      ignore_motion_ = this->declare_parameter<bool>(ignore_motion_pdesc.name, false, ignore_motion_pdesc);
      
      rcl_interfaces::msg::ParameterDescriptor use_best_pose_pdesc; 
      use_best_pose_pdesc.name = "use_best_pose";
      use_best_pose_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
      use_best_pose_pdesc.description = "Uses the particle with the best weight as current pose estimation. Else, the weighted average of all particles is used.";
      use_best_pose_ = this->declare_parameter<bool>(use_best_pose_pdesc.name, false, use_best_pose_pdesc);
      
      rcl_interfaces::msg::ParameterDescriptor reduction_cell_size_pdesc; 
      reduction_cell_size_pdesc.name = "reduction_cell_size";
      reduction_cell_size_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
      reduction_cell_size_pdesc.description = "Reduction cell size";
      reduction_cell_size_ = this->declare_parameter<double>(max_range_pdesc.name, 0.064, max_range_pdesc);

      rcl_interfaces::msg::ParameterDescriptor print_runtime_stats_pdesc; 
      print_runtime_stats_pdesc.name = "print_runtime_stats";
      print_runtime_stats_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
      print_runtime_stats_pdesc.description = "Ignores any sensor information in the motion update and only applies noise to every particle.";
      print_runtime_stats_ = this->declare_parameter<bool>(print_runtime_stats_pdesc.name, true, print_runtime_stats_pdesc);      
    }


    if(map_file_name_ == "")
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Please provide a map!");
      throw std::runtime_error("ERROR: Please provide a map!");
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading map from '" << map_file_name_ << "'");

    auto map = createTSDFMap<CudaSubVoxelMap<FLOAT_T, FLOAT_T>, FLOAT_T, FLOAT_T>(map_file_name_, free_map_, sigma);
    
    std::cout << "Reduction cell size is: " << reduction_cell_size_ << std::endl;
    tsdf_evaluator_ptr_ = std::make_shared<TSDFEvaluator>(
      map, per_point_, a_hit, a_rand, a_max, max_range, reduction_cell_size_
    );
    std::cout << "TSDF evaluator created!" << std::endl;

    if (use_imu_)
    {
      std::cout << "Use IMU for motion update..." << std::endl;
    }
    else if (!ignore_motion_)
    {
      std::cout << "Use odometry for motion update..." << std::endl;
    }
    else
    {
      std::cout << "Not using any sensor for motion update..." << std::endl;
    }

    resampler_ptr_ = std::make_unique<ResidualSystematicResampler>();
    
    ss_stamp << "stamp:\n";

    // dynamic_reconfigure::Server<tsdf_localization::MCLConfig> server;
    // dynamic_reconfigure::Server<tsdf_localization::MCLConfig>::CallbackType callbackType;

    // callbackType = boost::bind(&responseCallback, _1, _2);
    // server.setCallback(callbackType);

    // Meaningfull initialization
    initial_pose_.orientation.w = 1.0;

    // TODO: Init subscriber and service

    // // Initialize subscribers

    // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr 
    //   sub_initial_pose_;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr 
    //   sub_pcd_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr 
    //   sub_imu_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr 
    //   sub_odom_;

    sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10, 
      [=](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) -> void
      { 
        initialPoseCallback(msg);
      });
    // TODO: // ros::ServiceServer serv_global_loc = n.advertiseService("global_localization", globalLocalizationCallback);
    srv_global_loc_ = this->create_service<std_srvs::srv::Empty>(
      "global_localization", std::bind(&TSDFMCLNode::globalLocalizationCallback, this, _1, _2));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data", 10, 
      [=](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) -> void
      { 
        imuCallback(msg);
      });
    
    if(!use_os_)
    {

      // Initialize synchronized subscriber for the scan and odom topic
      message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub;
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcd_sub;
      
      odom_sub.subscribe(this, "odom");
      pcd_sub.subscribe(this, "cloud");

      uint32_t queue_size = 10;
      using MySyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
      sub_odom_pcd_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(queue_size), odom_sub, pcd_sub);

      sub_odom_pcd_->setAgePenalty(0.50);
      sub_odom_pcd_->registerCallback(std::bind(&TSDFMCLNode::scanOdomCallback, this, _1, _2));

    } else {
      sub_os_pcd_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", 10, 
        [=](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) -> void
        { 
          osCallback(msg);
        });
    }

    // Initialize publisher
    particles_pub_ptr_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particles", 1);
    current_pose_pub_ptr_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("current_pose", 1);

    tf_broadcaster_ptr_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  ~TSDFMCLNode() 
  {
    // Save evaluation results during the shutdown of the node

    std::ostringstream filename;
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);  

    filename << "mcl_times_" << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S") << ".log";

    std::ofstream time_output(filename.str());
    time_output << RuntimeEvaluator::get_instance().to_string(true) << "\n\n" << ss_stamp.str();
    time_output.close();

    tsdf_evaluator_ptr_.reset();

    std::cout << "Finished mcl 3d!" << std::endl;
  }

  /**
 * @brief Callback to get the initial pose etsimation for the algorithm  
 */
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose_with_covariance)
  {
    initial_pose_ = pose_with_covariance->pose.pose;

    initial_pose_.position.z = -0.1;
    pose_initialized_ = true;

    // TODOs:
    // - remove the constant -0.1
    // - use the covariance

    std::cout << "Initial pose received!" << std::endl;;
  }

  // TODO: Implement service
  void globalLocalizationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    const std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    global_initialized_ = true;
    std::cout << "Global localization triggered!" << std::endl;
  }

  /**
   * @brief Main callcack for the MCL algorithm to perform the initialization of the particle cloud, the motion update, the sensor update and the resampling based on the received data
   * 
   * @param odom Odometry estimation to perform a motion update
   * @param cloud Scan cloud to perform a sensor update based on the given TSDF map
   */
  void scanOdomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odom,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
  { 
    static tf2::BufferCore tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    static auto& eval = RuntimeEvaluator::get_instance();

    /*** A priori ***/

    if(!particle_cloud_.isInitialized() || pose_initialized_ || global_initialized_)
    {
      eval.start("init");
      if(!particle_cloud_.isInitialized() || global_initialized_)
      {
        std::cout << "Execute global particle initialization..." << std::endl;
        particle_cloud_.initialize(number_particles_, free_map_, initial_pose_);
        global_initialized_ = false;
      } else if(pose_initialized_) {
        std::cout << "Execute local particle initialization..." << std::endl;
        particle_cloud_.initialize(initial_pose_, number_particles_, init_sigma_x_, init_sigma_y_, init_sigma_z_, init_sigma_roll_, init_sigma_pitch_, init_sigma_yaw_);
        pose_initialized_ = false;
      }
      eval.stop("init");
    }
    // Resample particles
    else
    {
      
      eval.start("motion update");
      
      if (ignore_motion_)
      {
        particle_cloud_.motionUpdate(lin_scale_, ang_scale_);
      }
      else
      {
        if (use_imu_)
        {
          ImuAccumulator::Data imu_data;
          imu_acc_.getAndResetData(imu_data);
          particle_cloud_.motionUpdate(lin_scale_, imu_data);
        }
        else
        {
          particle_cloud_.motionUpdate(*odom);
        }
      }
      
      eval.stop("motion update");

      if (ignore_motion_ || use_imu_ || particle_cloud_.refDist() >= delta_update_dist_ || particle_cloud_.refAngle() >= delta_update_angle_)
      {
        particle_cloud_.resetRef();

        /*** A Posteriori ***/
    
        std::shared_ptr<EvaluationModel> model;

        switch(evaluation_model_)
        {
          case 0:
            model = std::make_shared<NaivEvaluation>();
            break;
          case 1:
            model = std::make_shared<OMPLikelihoodEvaluation>(100000);
            break;
          default:
            model = std::make_shared<OMPLikelihoodEvaluation>(100000);
        }

        eval.start("sensor update");

        try
        {
          current_pose = tsdf_evaluator_ptr_->evaluateParticles(
              particle_cloud_, *cloud, robot_frame_, cloud->header.frame_id, use_cuda_);
          current_pose.pose.position.z -= init_diff_z;

          FLOAT_T max_value = 0.0;
          FLOAT_T max_index = -1;

          for (auto index = 0u; index < particle_cloud_.size(); ++index)
          {
            auto value = particle_cloud_[index].second;

            if (value > max_value)
            {
              max_value = value;
              max_index = index;
            }
          }

          best_pose.pose.position.x = particle_cloud_[max_index].first[0];
          best_pose.pose.position.y = particle_cloud_[max_index].first[1];
          best_pose.pose.position.z = particle_cloud_[max_index].first[2] - init_diff_z;

          tf2::Quaternion q;
          q.setRPY(particle_cloud_[max_index].first[3], particle_cloud_[max_index].first[4], particle_cloud_[max_index].first[5]);

          tf2::convert(q, best_pose.pose.orientation);

          if (use_best_pose_)
          {
            current_pose.pose.position.x = best_pose.pose.position.x;
            current_pose.pose.position.y = best_pose.pose.position.y;
            current_pose.pose.position.z = best_pose.pose.position.z;
            current_pose.pose.orientation = best_pose.pose.orientation;
          }
        }
        catch (tf2::TransformException& e)
        {
          std::cerr << "Could not evaluate!" << std::endl;
        }

        eval.stop("sensor update");

        ss_stamp << cloud->header.stamp.sec << "\n";

        evaluated_ = true;

        eval.start("resampling");
        resampler_ptr_->resample(particle_cloud_);
        eval.stop("resampling");
      }
    }

    /*** Visualize particle cloud ***/

    geometry_msgs::msg::Pose pose;

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = this->get_clock()->now();
    pose_array.header.frame_id = map_frame_;
    pose_array.poses.resize(number_particles_);

    for(auto index = 0u; index < particle_cloud_.size(); ++index)
    {
      pose.position.x = particle_cloud_[index].first[0];
      pose.position.y = particle_cloud_[index].first[1];
      pose.position.z = particle_cloud_[index].first[2];
      
      tf2::Quaternion tf_quaternion;
      tf_quaternion.setRPY(particle_cloud_[index].first[3], particle_cloud_[index].first[4], particle_cloud_[index].first[5]);

      tf2::convert(tf_quaternion, pose.orientation);

      pose_array.poses[index] = pose;
    }

    // Visualize new particles
    particles_pub_ptr_->publish(pose_array);

    if (evaluated_)
    {

      geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
      pose_stamped.header = pose_array.header;
      pose_stamped.pose = current_pose;
      current_pose_pub_ptr_->publish(pose_stamped);

      /*** Provide a transform from the odom frame to the map fram based on the best estimated pose ***/ 

      tf2::Transform map_to_base;
      tf2::convert(current_pose.pose, map_to_base);
      geometry_msgs::msg::TransformStamped odom_to_base;

      // Retrieve the current transform from odom frame to scanner frame
      try
      {
        odom_to_base = tf_buffer.lookupTransform(odom_frame_, robot_frame_, tf2::TimePointZero);
      }
      catch (tf2::TransformException& e)
      {
        std::cout << "Couldn't transform from '" << odom_frame_ << "' to '" << robot_frame_ << "'." << std::endl;
        // Sleep for 0.1 secs
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        return;
      }

      tf2::Transform tf_odom_to_base;
      tf2::convert(odom_to_base.transform, tf_odom_to_base);

      // Calculate the transform from the map frame to the odom frame
      tf2::Transform map_to_odom = map_to_base * tf_odom_to_base.inverse();

      geometry_msgs::msg::TransformStamped stamped_transform;
      stamped_transform.header.stamp = cloud->header.stamp;
      stamped_transform.header.frame_id = map_frame_;
      stamped_transform.child_frame_id = odom_frame_;
      tf2::convert(map_to_odom, stamped_transform.transform);

      tf_broadcaster_ptr_->sendTransform(stamped_transform);
    }

    if (print_runtime_stats_)
    {
      std::cout << eval.to_string() << std::endl;
    }
    else
    {

    }
  }

  void osCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
  {
    static tf2::BufferCore tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    static auto& eval = RuntimeEvaluator::get_instance();

    /*** A priori ***/

    // Init particles
    if(!particle_cloud_.isInitialized() || pose_initialized_ || global_initialized_)
    {
      eval.start("init");
      if(!particle_cloud_.isInitialized() || global_initialized_)
      {
        std::cout << "Execute global particle initialization..." << std::endl;
        particle_cloud_.initialize(number_particles_, free_map_, initial_pose_);
        global_initialized_ = false;
      } else if(pose_initialized_) {
        std::cout << "Execute local particle initialization..." << std::endl;
        particle_cloud_.initialize(initial_pose_, number_particles_, init_sigma_x_, init_sigma_y_, init_sigma_z_, init_sigma_roll_, init_sigma_pitch_, init_sigma_yaw_);
        pose_initialized_ = false;
      }
      eval.stop("init");
    }
    // Resample particles
    else
    {
      
      eval.start("motion update");
      
      if (ignore_motion_)
      {
        particle_cloud_.motionUpdate(lin_scale_, ang_scale_);
      }
      else
      {
        
        if (use_imu_)
        {
          ImuAccumulator::Data imu_data;
          imu_acc_.getAndResetData(imu_data);
          particle_cloud_.motionUpdate(lin_scale_, imu_data);
        }
      }
      
      eval.stop("motion update");

      if (!ignore_sensor_update_ && ignore_motion_ || use_imu_ || particle_cloud_.refDist() >= delta_update_dist_ || particle_cloud_.refAngle() >= delta_update_angle_)
      {
        particle_cloud_.resetRef();

        /*** A Posteriori ***/
    
        std::shared_ptr<EvaluationModel> model;

        switch(evaluation_model_)
        {
          case 0:
            model = std::make_shared<NaivEvaluation>();
            break;
          case 1:
            model = std::make_shared<OMPLikelihoodEvaluation>(100000);
            break;
          default:
            model = std::make_shared<OMPLikelihoodEvaluation>(100000);
        }

        eval.start("sensor update");

        try
        {
          current_pose = tsdf_evaluator_ptr_->evaluateParticles(
            particle_cloud_, *cloud, 
            robot_frame_, cloud->header.frame_id, 
            use_cuda_, true);
          
          FLOAT_T max_value = 0.0;
          FLOAT_T max_index = -1;

          for (auto index = 0u; index < particle_cloud_.size(); ++index)
          {
            auto value = particle_cloud_[index].second;

            if (value > max_value)
            {
              max_value = value;
              max_index = index;
            }
          }

          best_pose.pose.position.x = particle_cloud_[max_index].first[0];
          best_pose.pose.position.y = particle_cloud_[max_index].first[1];
          best_pose.pose.position.z = particle_cloud_[max_index].first[2] - init_diff_z;

          tf2::Quaternion q;
          q.setRPY(particle_cloud_[max_index].first[3], particle_cloud_[max_index].first[4], particle_cloud_[max_index].first[5]);

          tf2::convert(q, best_pose.pose.orientation);

          current_pose.pose.orientation = best_pose.pose.orientation;
        }
        catch (tf2::TransformException& e)
        {
          std::cerr << "Could not evaluate!" << std::endl;
        }
        catch (std::runtime_error& e)
        {
          std::cout << "CUDA Execption occured!" << std::endl;
          return;
        }

        eval.stop("sensor update");

        ss_stamp << cloud->header.stamp.sec << "\n";

        evaluated_ = true;

        eval.start("resampling");
        resampler_ptr_->resample(particle_cloud_);
        eval.stop("resampling");
      }
    }

    /*** Visualize particle cloud ***/

    geometry_msgs::msg::Pose pose;

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = this->get_clock()->now();
    pose_array.header.frame_id = map_frame_;
    pose_array.poses.resize(number_particles_);

    for(auto index = 0u; index < particle_cloud_.size(); ++index)
    {
      pose.position.x = particle_cloud_[index].first[0];
      pose.position.y = particle_cloud_[index].first[1];
      pose.position.z = particle_cloud_[index].first[2];
      
      tf2::Quaternion tf_quaternion;
      tf_quaternion.setRPY(particle_cloud_[index].first[3], particle_cloud_[index].first[4], particle_cloud_[index].first[5]);

      tf2::convert(tf_quaternion, pose.orientation);

      pose_array.poses[index] = pose;
    }

    // Visualize new particles
    particles_pub_ptr_->publish(pose_array);

    if (evaluated_)
    {

      geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
      pose_stamped.header = pose_array.header;
      pose_stamped.pose = current_pose;
      current_pose_pub_ptr_->publish(pose_stamped);

      /// Provide a transform from the odom frame to the map frame based on the best estimated pose

      tf2::Transform map_to_base;
      tf2::convert(current_pose.pose, map_to_base);
      
      geometry_msgs::msg::TransformStamped stamped_transform;
      stamped_transform.header.stamp = cloud->header.stamp;
      
      stamped_transform.header.frame_id = cloud->header.frame_id;
      stamped_transform.child_frame_id = map_frame_;
      tf2::convert(map_to_base.inverse(), stamped_transform.transform);

      tf_broadcaster_ptr_->sendTransform(stamped_transform);
    }

    if (print_runtime_stats_)
    {
      std::cout << eval.to_string() << std::endl;
    }
    else
    {
      
    }
  }

  /**
   * @brief Callback to receive imu dat for the motion update 
   */
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
  {
    imu_acc_.update(*imu);
  }

private:
  // IMU accumulator to merge IMU measurents until a new motion update should be performed
  ImuAccumulator imu_acc_;

  // Initial estimate, where the robot might be located
  geometry_msgs::msg::Pose initial_pose_;

  // Was a new initial pose estimation received? 
  bool pose_initialized_ = false;

  // Trigger the global localization procedure
  bool global_initialized_ = false;

  // Was the particle cloud in the current MCL iteration already evaluated by the sensor update?
  bool evaluated_ = false;

  // Use the IMU information in the motion update. If not the odometry is used 
  bool use_imu_ = false;

  // Use an ouster as input of the sensor update. If not a velodyne is used 
  bool use_os_ = false;

  // Ignores any sensor information in the motion update and only applies noise to every particle
  bool ignore_motion_ = false;

  // Uses the particle with the best weight as current pose estimation. If not the weighted average of all particles is used
  bool use_best_pose_ = false;

  // Fix number of particles, which should be estimated in every step of the particle filter
  unsigned int number_particles_ = 800;

  // provides methods for particle initializing and applying an action model
  ParticleCloud particle_cloud_;

  // Pointer to the TSDF evaluator for the sensor update
  std::shared_ptr<TSDFEvaluator> tsdf_evaluator_ptr_;

  // Pointer to the resampler to generate a new particle set
  std::unique_ptr<Resampler> resampler_ptr_;

  // Standard deviation for the mcl startup with an initial pose estimation
  FLOAT_T init_sigma_x_;
  FLOAT_T init_sigma_y_;
  FLOAT_T init_sigma_z_;
  FLOAT_T init_sigma_roll_;
  FLOAT_T init_sigma_pitch_;
  FLOAT_T init_sigma_yaw_;

  // Evaluation model for the sensor update (deprecated)
  int evaluation_model_;

  // TF frames for the robot model
  std::string robot_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  // Pose changes until a sensor update should be started
  FLOAT_T delta_update_dist_ = 0.1;
  FLOAT_T delta_update_angle_ = 10.0 * M_PI / 180.0;

  // Should the cuda acceleration of the sensor update be used?
  bool use_cuda_ = true;
  // Should the per point implementation of the sensor update be used?
  bool per_point_ = false;
  // Should the particle set be initialized uniform in the given map?
  bool init_global_ = false;

  bool print_runtime_stats_ = true;

  bool ignore_sensor_update_= false;

  double reduction_cell_size_ = 0.064;

  // Parameter for the uncertainty model of the motion update
  FLOAT_T a_1 = 0.1;
  FLOAT_T a_2 = 0.1;
  FLOAT_T a_3 = 0.1;
  FLOAT_T a_4 = 0.1;
  FLOAT_T a_5 = 0.1;
  FLOAT_T a_6 = 0.1;
  FLOAT_T a_7 = 0.1;
  FLOAT_T a_8 = 0.1;
  FLOAT_T a_9 = 0.1;
  FLOAT_T a_10 = 0.1;
  FLOAT_T a_11 = 0.1;
  FLOAT_T a_12 = 0.1;

  // Scale of the motion model that only applies noise to the particles
  FLOAT_T lin_scale_ = 1.0;
  FLOAT_T ang_scale_ = 1.0;

  // Name of the file used to load the map of the environment
  std::string map_file_name_;//  = "/home/marc/ros_ws/tsdf_maps/sim_map.h5";

  std::vector<CudaPoint> free_map_;

  // Pose estimation based on a weighting average of the particle set
  geometry_msgs::msg::PoseWithCovariance current_pose;
  // Pose estimation based on best particle in the particle set
  geometry_msgs::msg::PoseWithCovariance best_pose;

  // Initial offset in z direction
  FLOAT_T init_diff_z = 0.0;

  std::stringstream ss_stamp;

  // Logging of ground truth error for the best pose estimation
  std::vector<double> best_diff_x;
  std::vector<double> best_diff_y;
  std::vector<double> best_diff_z;
  std::vector<double> best_diff_roll;
  std::vector<double> best_diff_pitch;
  std::vector<double> best_diff_yaw;

  // Logging of ground truth error for the average pose estimation
  std::vector<double> avg_diff_x;
  std::vector<double> avg_diff_y;
  std::vector<double> avg_diff_z;
  std::vector<double> avg_diff_roll;
  std::vector<double> avg_diff_pitch;
  std::vector<double> avg_diff_yaw;

  
  // Subscriber + Publisher + Services
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr 
    sub_initial_pose_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_global_loc_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr 
    sub_os_pcd_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr 
    sub_imu_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>>> 
    sub_odom_pcd_;

  // Publisher for the current particles
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr 
    particles_pub_ptr_;

  // Publisher of the current estimated pose
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr 
    current_pose_pub_ptr_; 




};

} // namespace tsdf_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tsdf_localization::TSDFMCLNode)