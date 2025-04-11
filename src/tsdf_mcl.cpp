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

#include <tsdf_localization/tsdf_mcl_params.hpp>

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

std::unique_ptr<Resampler> make_resampler(
  int resampling_method_)
{
  switch(resampling_method_)
  {
    case 0:
      return std::make_unique<WheelResampler>();
      break;
    case 1:
      return std::make_unique<ResidualResampler>();
      break;
    case 2:
      return std::make_unique<SystematicResampler>();
      break;
    case 3:
      return std::make_unique<ResidualSystematicResampler>();
      break;
    case 4:
      return std::make_unique<MetropolisResampler>(50);
      break;
    default:
      return std::make_unique<RejectionResampler>();
      break;
  }
}

class TSDFMCLNode : public rclcpp::Node
{
public:
  TSDFMCLNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
  : Node("tsdf_mcl", options)
  {
    std::cout << "Create TSDF MCL Node..." << std::endl;

    // share clocks
    particle_cloud_.setClock(this->get_clock());
    imu_acc_.setClock(this->get_clock());

    // TODO: Init params
    // std::string cloud_topic;
    // std::string imu_topic;

    // Definining parameters and loading initial values
    declareMCLParams(this);
    // get parameters initially
    getParameters();
    // get events when parameters are changed
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TSDFMCLNode::parametersCallback, this, _1));

    if(map_file_name_ == "")
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Please provide a map!");
      throw std::runtime_error("ERROR: Please provide a map!");
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading map from '" << map_file_name_ << "'");

    auto map = createTSDFMap<CudaSubVoxelMap<FLOAT_T, FLOAT_T>, FLOAT_T, FLOAT_T>(map_file_name_, free_map_, sigma_);
    
    std::cout << "Reduction cell size is: " << reduction_cell_size_ << std::endl;
    tsdf_evaluator_ptr_ = std::make_shared<TSDFEvaluator>(
      map, per_point_, a_hit_, a_rand_, a_max_, max_range_, reduction_cell_size_
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

    // resampler_ptr_ = std::make_unique<ResidualSystematicResampler>();
    // resampler_ptr_ = std::make_unique<ResidualResampler>();
    
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
      odom_sub_.subscribe(this, "odom");
      pcd_sub_.subscribe(this, "cloud");

      uint32_t queue_size = 10;
      using MySyncPolicy = message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
      sub_odom_pcd_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(queue_size), odom_sub_, pcd_sub_);

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

    if(particle_cloud_.isInitialized())
    {
      particle_cloud_.resize(number_particles_);
    }
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



  void getParameters()
  {
    map_file_name_       = this->get_parameter("map_file").as_string();
    map_frame_           = this->get_parameter("map_frame").as_string();
    odom_frame_          = this->get_parameter("odom_frame").as_string();
    robot_frame_         = this->get_parameter("robot_frame").as_string();
    per_point_           = this->get_parameter("per_point").as_bool();
    init_global_         = this->get_parameter("init_global").as_bool();
    a_hit_               = this->get_parameter("a_hit").as_double();
    a_rand_              = this->get_parameter("a_rand").as_double();
    a_max_               = this->get_parameter("a_max").as_double();
    max_range_           = this->get_parameter("max_range").as_double();
    use_imu_             = this->get_parameter("use_imu").as_bool();
    use_os_              = this->get_parameter("use_os").as_bool();
    ignore_motion_       = this->get_parameter("ignore_motion").as_bool();
    use_best_pose_       = this->get_parameter("use_best_pose").as_bool();
    reduction_cell_size_ = this->get_parameter("reduction_cell_size").as_double();
    print_runtime_stats_ = this->get_parameter("print_runtime_stats").as_bool();

    // old dynamic reconfigure
    number_particles_    = this->get_parameter("number_particles").as_int();
    init_sigma_x_        = this->get_parameter("init_sigma_x").as_double();
    init_sigma_y_        = this->get_parameter("init_sigma_y").as_double();
    init_sigma_z_        = this->get_parameter("init_sigma_z").as_double();
    init_sigma_roll_     = this->get_parameter("init_sigma_roll").as_double();
    init_sigma_pitch_    = this->get_parameter("init_sigma_pitch").as_double();
    init_sigma_yaw_      = this->get_parameter("init_sigma_yaw").as_double();
    delta_update_dist_   = this->get_parameter("resampling.delta_update_dist").as_double();
    delta_update_angle_  = this->get_parameter("resampling.delta_update_angle").as_double();
    use_cuda_            = this->get_parameter("use_cuda").as_bool();

    a_1                  = this->get_parameter("motion_update.a_1").as_double();
    a_2                  = this->get_parameter("motion_update.a_2").as_double();
    a_3                  = this->get_parameter("motion_update.a_3").as_double();
    a_4                  = this->get_parameter("motion_update.a_4").as_double();
    a_5                  = this->get_parameter("motion_update.a_5").as_double();
    a_6                  = this->get_parameter("motion_update.a_6").as_double();
    a_7                  = this->get_parameter("motion_update.a_7").as_double();
    a_8                  = this->get_parameter("motion_update.a_8").as_double();
    a_9                  = this->get_parameter("motion_update.a_9").as_double();
    a_10                 = this->get_parameter("motion_update.a_10").as_double();
    a_11                 = this->get_parameter("motion_update.a_11").as_double();
    a_12                 = this->get_parameter("motion_update.a_12").as_double();

    particle_cloud_.setMotionParameters(a_1, a_2, a_3, a_4, a_5, a_6, a_7, a_8, a_9, a_10, a_11, a_12);

    lin_scale_           = this->get_parameter("motion_update.lin_scale").as_double();
    ang_scale_           = this->get_parameter("motion_update.ang_scale").as_double();

    evaluation_model_    = this->get_parameter("sensor_update.evaluation_model").as_int();
  
    int resampling_method_ = this->get_parameter("resampling.method").as_int();
    resampler_ptr_ = make_resampler(resampling_method_);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.

    for(const auto &param: parameters)
    {
      std::cout << param.get_name() << " changed!" << std::endl;

      if(param.get_name() == "number_particles")
      {
        int number_particles = param.as_int();
        if(number_particles != number_particles_)
        {
          number_particles_ = number_particles;
          particle_cloud_.resize(number_particles_);
        }

        if(particle_cloud_.isInitialized())
        {
          particle_cloud_.resize(number_particles_);
        }
      }

      if(param.get_name() == "resampling.method")
      {
        int resampling_method_ = this->get_parameter("resampling.method").as_int();
        resampler_ptr_ = make_resampler(resampling_method_);
      }

      // if(param.get_name() == "motion_model.a_1")
      // {
      //   std::cout <<  "TODO!" << std::endl;
      // }

      if(param.get_name() == "use_cuda")
      {
        use_cuda_ = param.as_bool();
      }
    }

    return result;
  }


  /**
 * @brief Callback to get the initial pose etsimation for the algorithm  
 */
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose_with_covariance)
  {
    number_particles_ = this->get_parameter("number_particles").as_int();

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
    number_particles_ = this->get_parameter("number_particles").as_int();

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
    // RCLCPP_INFO(this->get_logger(), "scanOdomCallback!");
    static tf2::BufferCore tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    static auto& eval = RuntimeEvaluator::get_instance();

    /*** A priori ***/

    if(!particle_cloud_.isInitialized() || pose_initialized_ || global_initialized_)
    {
      eval.start("init");
      particle_cloud_.setClock(this->get_clock());
      if(!particle_cloud_.isInitialized() || global_initialized_)
      {
        RCLCPP_INFO(this->get_logger(), "Execute global particle initialization...");
        particle_cloud_.initialize(number_particles_, free_map_, initial_pose_);
        global_initialized_ = false;
      } else if(pose_initialized_) {
        RCLCPP_INFO(this->get_logger(), "Execute local particle initialization...");
        particle_cloud_.initialize(initial_pose_, number_particles_, init_sigma_x_, init_sigma_y_, init_sigma_z_, init_sigma_roll_, init_sigma_pitch_, init_sigma_yaw_);
        pose_initialized_ = false;
      }
      eval.stop("init");
    }
    else 
    { // Resample particles
      // RCLCPP_INFO(this->get_logger(), "ELSE!");
      eval.start("motion update");
      
      if(ignore_motion_)
      {
        // RCLCPP_INFO(this->get_logger(), "MOTION UPDATE: ignore");
        particle_cloud_.motionUpdate(lin_scale_, ang_scale_);
      }
      else
      {
        if(use_imu_)
        {
          // RCLCPP_INFO(this->get_logger(), "MOTION UPDATE: imu");
          ImuAccumulator::Data imu_data;
          imu_acc_.getAndResetData(imu_data);
          particle_cloud_.motionUpdate(lin_scale_, imu_data);
        }
        else
        {
          // RCLCPP_INFO(this->get_logger(), "MOTION UPDATE: odom msg");
          particle_cloud_.motionUpdate(*odom);
        }
      }
      
      eval.stop("motion update");


      // RCLCPP_INFO_STREAM(this->get_logger(), "SENSOR UPDATE ? " << particle_cloud_.refDist() << " >= " << delta_update_dist_ << " || " << particle_cloud_.refAngle() << " >= " << delta_update_angle_);
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

        // RCLCPP_INFO(this->get_logger(), "SENSOR UPDATE");
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
          RCLCPP_ERROR(this->get_logger(), "Could not evaluate!");
        }

        eval.stop("sensor update");

        // RCLCPP_INFO(this->get_logger(), "SENSOR UPDATE END");

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
          std::cout << "CUDA Exception occured!" << std::endl;
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

  float sigma_ = SIGMA;
  float a_hit_ = A_HIT;
  float a_rand_ = A_RAND;
  float a_max_ = A_MAX;
  float max_range_ = MAX_RANGE;

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

  // parameter callback
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // Subscriber + Publisher + Services
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr 
    sub_initial_pose_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_global_loc_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr 
    sub_os_pcd_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr 
    sub_imu_;
  // Initialize synchronized subscriber for the scan and odom topic
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcd_sub_;
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