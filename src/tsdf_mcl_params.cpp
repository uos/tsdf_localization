

#include <tsdf_localization/tsdf_mcl_params.hpp>

#include <tsdf_localization/util/util.h>


#include <rclcpp/rclcpp.hpp>


namespace tsdf_localization
{

void declareMCLParams(rclcpp::Node* node)
{
  rcl_interfaces::msg::ParameterDescriptor map_file_pdesc; 
  map_file_pdesc.name = "map_file";
  map_file_pdesc.type = rclcpp::ParameterType::PARAMETER_STRING;  
  map_file_pdesc.description = "The path to a tsdf map (h5 format)";
  node->declare_parameter<std::string>(map_file_pdesc.name, "", map_file_pdesc);

  rcl_interfaces::msg::ParameterDescriptor map_frame_pdesc; 
  map_frame_pdesc.name = "map_frame";
  map_frame_pdesc.type = rclcpp::ParameterType::PARAMETER_STRING;  
  map_frame_pdesc.description = "Name of the map frame";
  node->declare_parameter<std::string>(map_frame_pdesc.name, "map", map_frame_pdesc);

  rcl_interfaces::msg::ParameterDescriptor odom_frame_pdesc; 
  odom_frame_pdesc.name = "odom_frame";
  odom_frame_pdesc.type = rclcpp::ParameterType::PARAMETER_STRING;  
  odom_frame_pdesc.description = "Name of the odometry frame";
  node->declare_parameter<std::string>(odom_frame_pdesc.name, "odom", odom_frame_pdesc);

  rcl_interfaces::msg::ParameterDescriptor robot_frame_pdesc; 
  robot_frame_pdesc.name = "robot_frame";
  robot_frame_pdesc.type = rclcpp::ParameterType::PARAMETER_STRING;  
  robot_frame_pdesc.description = "Name of the robot frame";
  node->declare_parameter<std::string>(robot_frame_pdesc.name, "base_link", robot_frame_pdesc);

  rcl_interfaces::msg::ParameterDescriptor per_point_pdesc; 
  per_point_pdesc.name = "per_point";
  per_point_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
  per_point_pdesc.description = "Per point parallelization? Per particle parallelization otherwise.";
  node->declare_parameter<bool>(per_point_pdesc.name, false, per_point_pdesc);

  rcl_interfaces::msg::ParameterDescriptor init_global_pdesc; 
  init_global_pdesc.name = "init_global";
  init_global_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
  init_global_pdesc.description = "Initialize for global localization?";
  node->declare_parameter<bool>(init_global_pdesc.name, false, init_global_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_hit_pdesc; 
  a_hit_pdesc.name = "a_hit";
  a_hit_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_hit_pdesc.description = "a hit";
  node->declare_parameter<double>(a_hit_pdesc.name, (double)A_HIT, a_hit_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_rand_pdesc; 
  a_rand_pdesc.name = "a_rand";
  a_rand_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_rand_pdesc.description = "a rand";
  node->declare_parameter<double>(a_rand_pdesc.name, (double)A_RAND, a_rand_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_max_pdesc; 
  a_max_pdesc.name = "a_max";
  a_max_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_max_pdesc.description = "a max";
  node->declare_parameter<double>(a_max_pdesc.name, (double)A_MAX, a_max_pdesc);

  rcl_interfaces::msg::ParameterDescriptor max_range_pdesc; 
  max_range_pdesc.name = "range_max";
  max_range_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  max_range_pdesc.description = "max range";
  node->declare_parameter<double>(max_range_pdesc.name, (double)MAX_RANGE, max_range_pdesc);

  rcl_interfaces::msg::ParameterDescriptor use_imu_pdesc; 
  use_imu_pdesc.name = "use_imu";
  use_imu_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
  use_imu_pdesc.description = "Use the IMU information in the motion update. If not the odometry is used.";
  node->declare_parameter<bool>(use_imu_pdesc.name, false, use_imu_pdesc);

  rcl_interfaces::msg::ParameterDescriptor use_os_pdesc; 
  use_os_pdesc.name = "use_os";
  use_os_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
  use_os_pdesc.description = "Use an ouster as input of the sensor update. If not a velodyne is used.";
  node->declare_parameter<bool>(use_os_pdesc.name, false, use_os_pdesc);
  
  rcl_interfaces::msg::ParameterDescriptor ignore_motion_pdesc; 
  ignore_motion_pdesc.name = "ignore_motion";
  ignore_motion_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
  ignore_motion_pdesc.description = "Ignores any sensor information in the motion update and only applies noise to every particle.";
  node->declare_parameter<bool>(ignore_motion_pdesc.name, false, ignore_motion_pdesc);
  
  rcl_interfaces::msg::ParameterDescriptor use_best_pose_pdesc; 
  use_best_pose_pdesc.name = "use_best_pose";
  use_best_pose_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
  use_best_pose_pdesc.description = "Uses the particle with the best weight as current pose estimation. Else, the weighted average of all particles is used.";
  node->declare_parameter<bool>(use_best_pose_pdesc.name, false, use_best_pose_pdesc);
  
  rcl_interfaces::msg::ParameterDescriptor reduction_cell_size_pdesc; 
  reduction_cell_size_pdesc.name = "reduction_cell_size";
  reduction_cell_size_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  reduction_cell_size_pdesc.description = "Reduction cell size";
  node->declare_parameter<double>(reduction_cell_size_pdesc.name, 0.064, reduction_cell_size_pdesc);

  rcl_interfaces::msg::ParameterDescriptor print_runtime_stats_pdesc; 
  print_runtime_stats_pdesc.name = "print_runtime_stats";
  print_runtime_stats_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
  print_runtime_stats_pdesc.description = "Ignores any sensor information in the motion update and only applies noise to every particle.";
  node->declare_parameter<bool>(print_runtime_stats_pdesc.name, true, print_runtime_stats_pdesc);

  // old dynamic reconfigure
  rcl_interfaces::msg::ParameterDescriptor number_particles_pdesc; 
  number_particles_pdesc.name = "number_particles";
  number_particles_pdesc.type = rclcpp::ParameterType::PARAMETER_INTEGER;  
  number_particles_pdesc.description = "Number of particles";
  {
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 1000000;
    range.step = 1;
    number_particles_pdesc.integer_range.push_back(range);
  };
  node->declare_parameter<int>(number_particles_pdesc.name, 100000, number_particles_pdesc);


  rcl_interfaces::msg::ParameterDescriptor init_guess_noise_pdesc;
  init_guess_noise_pdesc.name = "initial_guess_noise";
  init_guess_noise_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;  
  init_guess_noise_pdesc.description = "Initial guess noise. X Y Z ROLL PITCH YAW";
  node->declare_parameter<std::vector<double> >(
    init_guess_noise_pdesc.name, 
    {0.5, 0.5, 0.5, 0.0, 0.0, 0.5}, // defaults
    init_guess_noise_pdesc);

  rcl_interfaces::msg::ParameterDescriptor delta_update_dist_pdesc;
  delta_update_dist_pdesc.name = "resampling.delta_update_dist";
  delta_update_dist_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  delta_update_dist_pdesc.description = "Moved distance at which a resampling step should executed";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 10.0;
    delta_update_dist_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(delta_update_dist_pdesc.name, 0.03, delta_update_dist_pdesc);

  rcl_interfaces::msg::ParameterDescriptor delta_update_angle_pdesc;
  delta_update_angle_pdesc.name = "resampling.delta_update_angle";
  delta_update_angle_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  delta_update_angle_pdesc.description = "Rotated angle at which a resampling step should executed (in radian)";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 2.0 * M_PI;
    delta_update_angle_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(delta_update_angle_pdesc.name, 0.1, delta_update_angle_pdesc);


  // gen.add("use_cuda", bool_t, 0, "Using cuda for the sensor-update", True)
  rcl_interfaces::msg::ParameterDescriptor use_cuda_pdesc;
  use_cuda_pdesc.name = "use_cuda";
  use_cuda_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;  
  use_cuda_pdesc.description = "Use cuda for the sensor-update";
  node->declare_parameter<bool>(use_cuda_pdesc.name, true, use_cuda_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_pdesc;
  a_pdesc.name = "motion_update.a";
  a_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
  a_pdesc.description = "Parameter for the motion model";
  node->declare_parameter<std::vector<double> >(a_pdesc.name, 
    {1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0}, 
    a_pdesc);
  
  // gen.add("lin_scale", double_t, 0, "Linear scale of the motion update that only applies noise to the particles", 0.1, 0.0, 10.0)
  // gen.add("ang_scale", double_t, 0, "Angular scale of the motion update that only applies noise to the particles", 0.1, 0.0, 10.0)
  rcl_interfaces::msg::ParameterDescriptor lin_scale_pdesc;
  lin_scale_pdesc.name = "motion_update.lin_scale";
  lin_scale_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  lin_scale_pdesc.description = "Linear scale of the motion update that only applies noise to the particles";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 10.0;
    lin_scale_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(lin_scale_pdesc.name, 0.1, lin_scale_pdesc);

  rcl_interfaces::msg::ParameterDescriptor ang_scale_pdesc;
  ang_scale_pdesc.name = "motion_update.ang_scale";
  ang_scale_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  ang_scale_pdesc.description = "Angular scale of the motion update that only applies noise to the particles";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 10.0;
    ang_scale_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(ang_scale_pdesc.name, 0.1, ang_scale_pdesc);

  // TODO: how to make enum here?
  rcl_interfaces::msg::ParameterDescriptor evaluation_model_pdesc;
  evaluation_model_pdesc.name = "sensor_update.evaluation_model";
  evaluation_model_pdesc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  evaluation_model_pdesc.description = "0 = naiv, 1 = likelihood.";
  {
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 0;
    range.to_value = 1;
    range.step = 1;
    evaluation_model_pdesc.integer_range.push_back(range);
  };
  node->declare_parameter<int>(evaluation_model_pdesc.name, 1, evaluation_model_pdesc);


  rcl_interfaces::msg::ParameterDescriptor resampling_method_pdesc;
  resampling_method_pdesc.name = "resampling.method";
  resampling_method_pdesc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  resampling_method_pdesc.description = "0 = WheelResampler, 1 = ResidualResampler, 2 = SystematicResampler, 3 = ResidualSystematicResampler, 4 = MetropolisResampler(50), 5 = RejectionResampler().";
  {
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 0;
    range.to_value = 5;
    range.step = 1;
    resampling_method_pdesc.integer_range.push_back(range);
  };
  node->declare_parameter<int>(resampling_method_pdesc.name, 1, resampling_method_pdesc);


  // // enum hack
  // rcl_interfaces::msg::ParameterDescriptor evaluation_naiv_pdesc;
  // evaluation_naiv_pdesc.name = "sensor_update.evaluation.naiv";
  // evaluation_naiv_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  // evaluation_naiv_pdesc.description = "Use naiv range evaluation";
  // node->declare_parameter<bool>(evaluation_naiv_pdesc.name, false, evaluation_naiv_pdesc);

  // rcl_interfaces::msg::ParameterDescriptor evaluation_likelihood_pdesc;
  // evaluation_likelihood_pdesc.name = "sensor_update.evaluation.likelihood";
  // evaluation_likelihood_pdesc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  // evaluation_likelihood_pdesc.description = "Use likelihood range evaluation";
  // node->declare_parameter<bool>(evaluation_likelihood_pdesc.name, true, evaluation_likelihood_pdesc);
}

} // namespace tsdf_localization