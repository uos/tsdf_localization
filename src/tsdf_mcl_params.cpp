

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
  max_range_pdesc.name = "max_range";
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

  rcl_interfaces::msg::ParameterDescriptor init_sigma_x_pdesc;
  init_sigma_x_pdesc.name = "init_sigma_x";
  init_sigma_x_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  init_sigma_x_pdesc.description = "Initial sigma X";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 10.0;
    range.step = 0.05;
    init_sigma_x_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(init_sigma_x_pdesc.name, 0.5, init_sigma_x_pdesc);

  rcl_interfaces::msg::ParameterDescriptor init_sigma_y_pdesc;
  init_sigma_y_pdesc.name = "init_sigma_y";
  init_sigma_y_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  init_sigma_y_pdesc.description = "Initial sigma Y";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 10.0;
    range.step = 0.05;
    init_sigma_y_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(init_sigma_y_pdesc.name, 0.5, init_sigma_y_pdesc);

  rcl_interfaces::msg::ParameterDescriptor init_sigma_z_pdesc;
  init_sigma_z_pdesc.name = "init_sigma_z";
  init_sigma_z_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  init_sigma_z_pdesc.description = "Initial sigma Z";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 10.0;
    range.step = 0.05;
    init_sigma_z_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(init_sigma_z_pdesc.name, 0.5, init_sigma_z_pdesc);

  rcl_interfaces::msg::ParameterDescriptor init_sigma_roll_pdesc;
  init_sigma_roll_pdesc.name = "init_sigma_roll";
  init_sigma_roll_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  init_sigma_roll_pdesc.description = "Initial sigma roll";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 2.0 * M_PI;
    range.step = 0.05;
    init_sigma_roll_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(init_sigma_roll_pdesc.name, 0.0, init_sigma_roll_pdesc);

  rcl_interfaces::msg::ParameterDescriptor init_sigma_pitch_pdesc;
  init_sigma_pitch_pdesc.name = "init_sigma_pitch";
  init_sigma_pitch_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  init_sigma_pitch_pdesc.description = "Initial sigma pitch";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 2.0 * M_PI;
    range.step = 0.05;
    init_sigma_pitch_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(init_sigma_pitch_pdesc.name, 0.0, init_sigma_pitch_pdesc);

  rcl_interfaces::msg::ParameterDescriptor init_sigma_yaw_pdesc;
  init_sigma_yaw_pdesc.name = "init_sigma_yaw";
  init_sigma_yaw_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  init_sigma_yaw_pdesc.description = "Initial sigma yaw";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 2.0 * M_PI;
    // range.step = 0.05;
    init_sigma_yaw_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(init_sigma_yaw_pdesc.name, 0.5, init_sigma_yaw_pdesc);

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

  rcl_interfaces::msg::ParameterDescriptor a_1_pdesc;
  a_1_pdesc.name = "motion_update.a_1";
  a_1_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  a_1_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_1_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_1_pdesc.name, 1.0, a_1_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_2_pdesc;
  a_2_pdesc.name = "motion_update.a_2";
  a_2_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_2_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_2_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_2_pdesc.name, 0.0, a_2_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_3_pdesc;
  a_3_pdesc.name = "motion_update.a_3";
  a_3_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_3_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_3_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_3_pdesc.name, 1.0, a_3_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_4_pdesc;
  a_4_pdesc.name = "motion_update.a_4";
  a_4_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_4_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_4_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_4_pdesc.name, 0.0, a_4_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_5_pdesc;
  a_5_pdesc.name = "motion_update.a_5";
  a_5_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_5_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_5_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_5_pdesc.name, 1.0, a_5_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_6_pdesc;
  a_6_pdesc.name = "motion_update.a_6";
  a_6_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_6_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_6_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_6_pdesc.name, 0.0, a_6_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_7_pdesc;
  a_7_pdesc.name = "motion_update.a_7";
  a_7_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_7_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_7_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_7_pdesc.name, 0.0, a_7_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_8_pdesc;
  a_8_pdesc.name = "motion_update.a_8";
  a_8_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_8_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_8_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_8_pdesc.name, 1.0, a_8_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_9_pdesc;
  a_9_pdesc.name = "motion_update.a_9";
  a_9_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_9_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_9_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_9_pdesc.name, 0.0, a_9_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_10_pdesc;
  a_10_pdesc.name = "motion_update.a_10";
  a_10_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_10_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_10_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_10_pdesc.name, 1.0, a_10_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_11_pdesc;
  a_11_pdesc.name = "motion_update.a_11";
  a_11_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_11_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_11_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_11_pdesc.name, 0.0, a_11_pdesc);

  rcl_interfaces::msg::ParameterDescriptor a_12_pdesc;
  a_12_pdesc.name = "motion_update.a_12";
  a_12_pdesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;  
  a_12_pdesc.description = "Parameter for the motion model";
  {
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value =  0.0;
    range.to_value   = 30.0;
    a_12_pdesc.floating_point_range.push_back(range);
  }
  node->declare_parameter<double>(a_12_pdesc.name, 1.0, a_12_pdesc);

  
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