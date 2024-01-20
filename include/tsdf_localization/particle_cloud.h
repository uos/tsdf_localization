/**
 * @file particle_cloud.h
 * @author Marc Eisoldt (meisoldt@uni-osnabrueck.de)
 * 
 * @brief Class of the particle cloud with an integrated motion model
 * 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 */

#ifndef PARTICLE_CLOUD_H
#define PARTICLE_CLOUD_H

#include <random>
#include <cmath>
#include <vector>
#include <array>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tsdf_localization/util/util.h>
#include <tsdf_localization/resampling/resampler.h>

#include <tsdf_localization/particle.h>

#include <tsdf_localization/util/imu_accumulator.h>

#include <tsdf_localization/cuda/cuda_evaluator.h>

namespace tsdf_localization
{

/// One Particle should consist of an two dimensional pose (x, y, rotation) and a weight
//typedef std::pair<FLOAT_T[6], FLOAT_T> Particle;

/**
 * @class ParticleCloud
 * 
 * @brief Represents the handling of the initialisation and resampling of particles
 */
class ParticleCloud
{
  /// Array of the current particles
  std::vector<Particle> m_particles;

  /// random generator
  std::shared_ptr<std::mt19937> m_generator_ptr;

  rclcpp::Time m_last_time;

  // Reference pose of the robot tto determine the pose change since the last sensor update
  std::array<FLOAT_T, 6> ref_pose;

  // Parameters for the uncertainty model of the motion update
  FLOAT_T a_1_ = 0.1;
  FLOAT_T a_2_ = 0.1;
  FLOAT_T a_3_ = 0.1;
  FLOAT_T a_4_ = 0.1;
  FLOAT_T a_5_ = 0.1;
  FLOAT_T a_6_ = 0.1;
  FLOAT_T a_7_ = 0.1;
  FLOAT_T a_8_ = 0.1;
  FLOAT_T a_9_ = 0.1;
  FLOAT_T a_10_ = 0.1;
  FLOAT_T a_11_ = 0.1;
  FLOAT_T a_12_ = 0.1;

  // bool ignore_motion_;

  void apply_model(std::normal_distribution<>& distribution_linear_x, std::normal_distribution<>& distribution_linear_y, std::normal_distribution<>& distribution_linear_z,
                   std::normal_distribution<>& distribution_roll, std::normal_distribution<>& distribution_pitch, std::normal_distribution<>& distribution_yaw, std::vector<Particle>& new_particles);

public:
  /**
   * @brief Only initializes the random generator
   */
  ParticleCloud();

  /**
   * @brief Initialize the particle cloud local with a normal distribution
   */
  ParticleCloud(const geometry_msgs::msg::Pose& center_pose, unsigned int number_particles, FLOAT_T init_sigma_x, FLOAT_T init_sigma_y, FLOAT_T init_sigma_z, FLOAT_T init_sigma_roll, FLOAT_T init_sigma_pitch, FLOAT_T init_sigma_yaw);

  /**
   * @brief Initialize the particle cloud local with a uniform distribution
   */
  ParticleCloud(unsigned int number_particles, const geometry_msgs::msg::Pose& center_pose, FLOAT_T dx, FLOAT_T dy, FLOAT_T dz, FLOAT_T droll = M_PI, FLOAT_T dpitch = M_PI, FLOAT_T dyaw = M_PI);

  /**
   * @brief Initialize the particle cloud global in the given map
   */
  ParticleCloud(unsigned int number_particles, const std::vector<CudaPoint>& free_map, const geometry_msgs::msg::Pose& center_pose, FLOAT_T droll = M_PI, FLOAT_T dpitch = M_PI, FLOAT_T dyaw = M_PI);

  /**
   * @brief Initializes the particles by sampling a normal distribution with a given pose as average 
   *
   * @param center_pose Center of the sampled normal distribution of two dimensional poses
   * @param number_particles The number of particles that should be sampled
   */
  void initialize(const geometry_msgs::msg::Pose& center_pose, unsigned int number_particles, FLOAT_T init_sigma_x, FLOAT_T init_sigma_y, FLOAT_T init_sigma_z, FLOAT_T init_sigma_roll, FLOAT_T init_sigma_pitch, FLOAT_T init_sigma_yaw);

  /**
   * @brief Initialize the particle cloud local with a uniform distribution
   */
  void initialize(unsigned int number_particles, const geometry_msgs::msg::Pose& center_pose, FLOAT_T dx, FLOAT_T dy, FLOAT_T dz, FLOAT_T droll = M_PI, FLOAT_T dpitch = M_PI, FLOAT_T dyaw = M_PI);

  /**
   * @brief Initialize the particle cloud global in the given map
   */
  void initialize(unsigned int number_particles, const std::vector<CudaPoint>& free_map, const geometry_msgs::msg::Pose& center_pose, FLOAT_T droll = M_PI, FLOAT_T dpitch = M_PI, FLOAT_T dyaw = M_PI);


  /**
   * @brief Performs a motion update on the particle cloud based on an odometry estimation
   * 
   * @param odom Containing information about the current action of the robot 
   */
  void motionUpdate(const nav_msgs::msg::Odometry& odom);

  /**
   * @brief Performs a motion update on the particle cloud based on IMU data
   * 
   * @param imu_date Containing information about the current action of the robot 
   */
  void motionUpdate(const ImuAccumulator::Data& imu_data);

  /**
   * @brief Performs a motion update on the particle cloud based on normal distributed noise 
   * 
   * @param lin_scale Noise parameter for the translastion
   * @param ang_scale Noise parameter for the rotation
   */
  void motionUpdate(FLOAT_T lin_scale, FLOAT_T ang_scale);

  /**
   * @brief Performs a motion update on the particle cloud based on IMU data normal distributed noise for the translation
   * 
   * @param lin_scale Noise parameter for the translastion
   * @param imu_data Containing information about the current action of the robot
   */
  void motionUpdate(FLOAT_T lin_scale, const ImuAccumulator::Data& imu_data);

  /**
   * @brief Performs a motion update on the particle cloud based on a delta pose
   * 
   * @param dx Translation in x-direction
   * @param dy Translation in y-direction
   * @param dz Translation in z-direction
   * @param roll Rotation around the x-axis
   * @param pitch Rotation around the y-axis
   * @param yaw  Rotation around the z-axis
   * @param lin_base_noise Base noise to be applied on the translation of the particles
   * @param ang_base_noise Base noise to be applied on the rotation of the particles
   */
  void motion_update(FLOAT_T dx, FLOAT_T dy, FLOAT_T dz, FLOAT_T roll, FLOAT_T pitch, FLOAT_T yaw, FLOAT_T lin_base_noise = 0, FLOAT_T ang_base_noise = 0);

  /**
   * @brief Set parameters of the motion update
   */
  void setMotionParameters(FLOAT_T a_1, FLOAT_T a_2, FLOAT_T a_3, FLOAT_T a_4, FLOAT_T a_5, FLOAT_T a_6, FLOAT_T a_7, FLOAT_T a_8, FLOAT_T a_9, FLOAT_T a_10, FLOAT_T a_11, FLOAT_T a_12)
  {
    a_1_ = a_1;
    a_2_ = a_2;
    a_3_ = a_3;
    a_4_ = a_4;
    a_5_ = a_5;
    a_6_ = a_6;
    a_7_ = a_7;
    a_8_ = a_8;
    a_9_ = a_9;
    a_10_ = a_10;
    a_11_ = a_11;
    a_12_ = a_12;
  }

  /**
   * @brief Access to single particles
   * 
   * @throw std::out_of_range
   */
  Particle& operator[](unsigned int index);

  /**
   * @brief Returns the number of particles
   */
  std::size_t size() const;

  std::vector<Particle>& particles()
  {
    return m_particles;
  }

  /**
   * @brief Return the reference pose to determine the pose change of the robot 
   */
  const std::array<FLOAT_T, 6>& refPose()
  {
    return ref_pose;
  }

  /**
   * @brief Reset the reference pose of the robot
   */
  void resetRef()
  {
    ref_pose[0] = 0;
    ref_pose[1] = 0;
    ref_pose[2] = 0;
    ref_pose[3] = 0;
    ref_pose[4] = 0;
    ref_pose[5] = 0;
  }

  /**
   * @brief Get the distance traveled by the robot since the last sensor update
   */
  FLOAT_T refDist()
  {
    return std::sqrt(ref_pose[0] * ref_pose[0] + ref_pose[1] * ref_pose[1] + ref_pose[2] * ref_pose[2]);
  }

  /**
   * @brief Get the rotation change since the last sensor update
   */
  FLOAT_T refAngle()
  {
    return std::fabs(ref_pose[5]);
  }

  /**
   * @brief Changes the number of particles
   */
  void resize(unsigned int number_of_particles);

  /**
   * @brief The particles were initialized?  
   */
  bool isInitialized() const;

  /**
   * @brief Gets a random particle based on the weights of the current ones
   */
  const Particle& getRandomParticle() const;

  /**
   * @brief Option to ignore the sensor information of the odometry or the IMU during the morion update and only apply noise to all particles
   * 
   * @param state Should the sensor information be ignored during the motion update?
   */
  // void setIgnoreMotion(bool state)
  // {
  //   ignore_motion_ = state;
  // }
};

} // namespace tsdf_localization

#endif