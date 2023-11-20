#include <tsdf_localization/particle_cloud.h>

#include <exception>
#include <iostream>

#include <tf2/LinearMath/Matrix3x3.h>

namespace tsdf_localization
{

ParticleCloud::ParticleCloud()
{
  std::random_device device{};
  m_generator_ptr.reset(new std::mt19937(device()));
}

ParticleCloud::ParticleCloud(const geometry_msgs::Pose& center_pose, unsigned int number_particles, FLOAT_T init_sigma_x, FLOAT_T init_sigma_y, FLOAT_T init_sigma_z, FLOAT_T init_sigma_roll, FLOAT_T init_sigma_pitch, FLOAT_T init_sigma_yaw) : ParticleCloud()
{
  initialize(center_pose, number_particles, init_sigma_x, init_sigma_y, init_sigma_z, init_sigma_roll, init_sigma_pitch, init_sigma_yaw);
}

ParticleCloud::ParticleCloud(unsigned int number_particles, const geometry_msgs::Pose& center_pose, FLOAT_T dx, FLOAT_T dy, FLOAT_T dz, FLOAT_T droll, FLOAT_T dpitch, FLOAT_T dyaw) : ParticleCloud()
{
  initialize(number_particles, center_pose, dx, dy, dz, droll, dpitch, dyaw);
}

ParticleCloud::ParticleCloud(unsigned int number_particles, const std::vector<CudaPoint>& free_map, const geometry_msgs::Pose& center_pose, FLOAT_T droll, FLOAT_T dpitch, FLOAT_T dyaw) : ParticleCloud()
{
  initialize(number_particles, free_map, center_pose, droll, dpitch, dyaw);
}

void ParticleCloud::initialize(const geometry_msgs::Pose& center_pose, unsigned int number_particles, FLOAT_T init_sigma_x, FLOAT_T init_sigma_y, FLOAT_T init_sigma_z, FLOAT_T init_sigma_roll, FLOAT_T init_sigma_pitch, FLOAT_T init_sigma_yaw)
{
  ref_pose.fill(0);

  // Choose initial particles in a normal distributed environment around the initial pose
  std::normal_distribution<> distribution_x{center_pose.position.x, init_sigma_x};
  std::normal_distribution<> distribution_y{center_pose.position.y, init_sigma_y};
  std::normal_distribution<> distribution_z{center_pose.position.z, init_sigma_z};
  std::normal_distribution<> distribution_roll{getRollFromQuaternion(center_pose.orientation), init_sigma_roll};
  std::normal_distribution<> distribution_pitch{getPitchFromQuaternion(center_pose.orientation), init_sigma_pitch};
  std::normal_distribution<> distribution_yaw{getYawFromQuaternion(center_pose.orientation), init_sigma_yaw};
  
  auto init_weight = 1.0 / number_particles;
  m_particles.resize(number_particles);

  //#pragma omp parallel for firstprivate(particle)
  for(auto index = 0u; index < number_particles; index++)
  { 
    m_particles[index].second = init_weight;
    m_particles[index].first[0] = distribution_x(*m_generator_ptr);
    m_particles[index].first[1] = distribution_y(*m_generator_ptr);
    m_particles[index].first[2] = distribution_z(*m_generator_ptr);
    m_particles[index].first[3] = distribution_roll(*m_generator_ptr);
    m_particles[index].first[4] = distribution_pitch(*m_generator_ptr);
    m_particles[index].first[5] = distribution_yaw(*m_generator_ptr);
  }

  m_last_time = ros::Time::now();
}

void ParticleCloud::initialize(unsigned int number_particles, const geometry_msgs::Pose& center_pose, FLOAT_T dx, FLOAT_T dy, FLOAT_T dz, FLOAT_T droll, FLOAT_T dpitch, FLOAT_T dyaw)
{
  ref_pose.fill(0);

  std::uniform_real_distribution<double> distribution_x{center_pose.position.x - dx, center_pose.position.x + dx};
  std::uniform_real_distribution<double> distribution_y{center_pose.position.y - dy, center_pose.position.y + dy};
  std::uniform_real_distribution<double> distribution_z{center_pose.position.z - dz, center_pose.position.z + dz};
 
  auto roll = getRollFromQuaternion(center_pose.orientation);
  auto pitch = getPitchFromQuaternion(center_pose.orientation);
  auto yaw = getYawFromQuaternion(center_pose.orientation);

  auto min_roll = roll - droll;
  auto min_pitch = pitch - dpitch;
  auto min_yaw = yaw - dyaw;

  auto max_roll = roll + droll;
  auto max_pitch = pitch + dpitch;
  auto max_yaw = yaw + dyaw;
 
  std::uniform_real_distribution<double> distribution_roll{min_roll, max_roll};
  std::uniform_real_distribution<double> distribution_pitch{min_pitch, max_pitch};
  std::uniform_real_distribution<double> distribution_yaw{min_yaw, max_yaw};

  auto init_weight = 1.0 / number_particles;
  m_particles.resize(number_particles);

  //#pragma omp parallel for firstprivate(particle)
  for(auto index = 0u; index < number_particles; index++)
  { 
    m_particles[index].second = init_weight;
    m_particles[index].first[0] = distribution_x(*m_generator_ptr);
    m_particles[index].first[1] = distribution_y(*m_generator_ptr);
    m_particles[index].first[2] = distribution_z(*m_generator_ptr);
    m_particles[index].first[3] = distribution_roll(*m_generator_ptr);
    m_particles[index].first[4] = distribution_pitch(*m_generator_ptr);
    m_particles[index].first[5] = distribution_yaw(*m_generator_ptr);
  }

  m_last_time = ros::Time::now();
}

void ParticleCloud::initialize(unsigned int number_particles, const std::vector<CudaPoint>& free_map, const geometry_msgs::Pose& center_pose, FLOAT_T droll, FLOAT_T dpitch, FLOAT_T dyaw)
{
  ref_pose.fill(0);

  std::uniform_int_distribution<size_t> distribution_index{0, free_map.size()};
  
  auto init_weight = 1.0 / number_particles;
  m_particles.resize(number_particles);

  auto roll = getRollFromQuaternion(center_pose.orientation);
  auto pitch = getPitchFromQuaternion(center_pose.orientation);
  auto yaw = getYawFromQuaternion(center_pose.orientation);

  auto min_roll = roll - droll;
  auto min_pitch = pitch - dpitch;
  auto min_yaw = yaw - dyaw;

  auto max_roll = roll + droll;
  auto max_pitch = pitch + dpitch;
  auto max_yaw = yaw + dyaw;
 
  std::uniform_real_distribution<double> distribution_roll{min_roll, max_roll};
  std::uniform_real_distribution<double> distribution_pitch{min_pitch, max_pitch};
  std::uniform_real_distribution<double> distribution_yaw{min_yaw, max_yaw};

  //#pragma omp parallel for firstprivate(particle)
  for(auto index = 0u; index < number_particles; index++)
  { 

    m_particles[index].second = init_weight;
    
    auto rand_index = distribution_index(*m_generator_ptr);

    m_particles[index].first[0] = free_map[rand_index].x;
    m_particles[index].first[1] = free_map[rand_index].y;
    m_particles[index].first[2] = free_map[rand_index].z - 0.5;
    m_particles[index].first[3] = distribution_roll(*m_generator_ptr);
    m_particles[index].first[4] = distribution_pitch(*m_generator_ptr);
    m_particles[index].first[5] = distribution_yaw(*m_generator_ptr);
  }

  m_last_time = ros::Time::now();
}

bool ParticleCloud::isInitialized() const
{
  return m_particles.size() != 0;
}

void ParticleCloud::motionUpdate(const nav_msgs::Odometry& odom)
{
  std::vector<Particle> new_particles(m_particles.size());

  // std::normal_distribution<> distribution_linear_x{0, 5 * sqrt(odom.twist.covariance[0])};
  // std::normal_distribution<> distribution_linear_y{0, 5 * sqrt(odom.twist.covariance[0])};
  // std::normal_distribution<> distribution_linear_z{0, 3 * sqrt(odom.twist.covariance[0])};
  
  // std::normal_distribution<> distribution_roll{0, 3 * sqrt(odom.twist.covariance[35])};
  // std::normal_distribution<> distribution_pitch{0, 3 * sqrt(odom.twist.covariance[35])};
  // std::normal_distribution<> distribution_yaw{0, 3 * sqrt(odom.twist.covariance[35])};


  auto linear_velocity = odom.twist.twist.linear.x;
  auto angular_velocity = odom.twist.twist.angular.z;

  FLOAT_T random_linear_velocity_x;
  FLOAT_T random_linear_velocity_y;
  FLOAT_T random_linear_velocity_z;
  FLOAT_T random_velocity_roll;
  FLOAT_T random_velocity_pitch;
  FLOAT_T random_velocity_yaw;

  ros::Time current_time = ros::Time::now();
  FLOAT_T time_diff = (current_time - m_last_time).toSec();
  m_last_time = current_time;

  ref_pose[0] = ref_pose[0] + linear_velocity * time_diff * cos(ref_pose[5] + (angular_velocity / 2 * time_diff));
  ref_pose[1] = ref_pose[1] + linear_velocity * time_diff * sin(ref_pose[5] + (angular_velocity / 2 * time_diff));
  ref_pose[5] = ref_pose[5] + angular_velocity * time_diff;

  auto ref_dist = std::sqrt(ref_pose[0] * ref_pose[0] + ref_pose[1] * ref_pose[1] + ref_pose[2] * ref_pose[2]);


  auto d = linear_velocity * time_diff;
  auto d_square = d * d;

  auto theta = angular_velocity * time_diff;
  auto theta_square = theta * theta;

  std::normal_distribution<> distribution_linear_x{d, a_1_ * d_square + a_2_ * theta_square};
  std::normal_distribution<> distribution_linear_y{0, a_3_ * d_square + a_4_ * theta_square};
  std::normal_distribution<> distribution_linear_z{0, a_5_ * d_square + a_6_ * theta_square};
  
  std::normal_distribution<> distribution_roll{0, a_7_ * d_square + a_8_ * theta_square};
  std::normal_distribution<> distribution_pitch{0, a_9_ * d_square + a_10_ * theta_square};
  std::normal_distribution<> distribution_yaw{theta, a_11_ * d_square + a_12_ * theta_square};

  for(auto index = 0u; index < m_particles.size(); index++)
  { 
    auto& last_particle = m_particles[index];

    auto& last_pose = last_particle.first;

    // random_linear_velocity_x = distribution_linear_x(*m_generator_ptr) + linear_velocity;
    // random_linear_velocity_y = distribution_linear_y(*m_generator_ptr);
    // random_linear_velocity_z = distribution_linear_z(*m_generator_ptr);

    // random_velocity_roll = distribution_roll(*m_generator_ptr);
    // random_velocity_pitch = distribution_pitch(*m_generator_ptr);
    // random_velocity_yaw = distribution_yaw(*m_generator_ptr) + angular_velocity;
  
    // auto dx = random_linear_velocity_x * time_diff;
    // auto dy = random_linear_velocity_y * time_diff;
    // auto dz = 0;//random_linear_velocity_z * time_diff;
    
    // auto roll = random_velocity_roll * time_diff;
    // auto pitch = random_velocity_pitch * time_diff;
    // auto yaw = random_velocity_yaw * time_diff;

    auto dx = distribution_linear_x(*m_generator_ptr);
    auto dy = distribution_linear_y(*m_generator_ptr);
    auto dz = distribution_linear_z(*m_generator_ptr);

    auto roll  = distribution_roll(*m_generator_ptr);
    auto pitch = distribution_pitch(*m_generator_ptr);
    auto yaw   = distribution_yaw(*m_generator_ptr);

    FLOAT_T sin_roll  = sin(roll);
    FLOAT_T cos_roll  = cos(roll);  
    FLOAT_T sin_pitch = sin(pitch);
    FLOAT_T cos_pitch = cos(pitch);
    FLOAT_T sin_yaw   = sin(yaw);
    FLOAT_T cos_yaw   = cos(yaw);

    FLOAT_T tf_odom[16];

    tf_odom[0] = cos_pitch * cos_yaw;
    tf_odom[4] = cos_pitch * sin_yaw;
    tf_odom[8] = -sin_pitch;
    tf_odom[3] = dx;
    
    tf_odom[1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
    tf_odom[5] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
    tf_odom[9] = sin_roll * cos_pitch;
    tf_odom[7] = dy;

    tf_odom[2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;
    tf_odom[6] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;
    tf_odom[10] = cos_roll * cos_pitch;
    tf_odom[11] = dz;



    auto alpha = last_pose[3];
    auto beta = last_pose[4];
    auto gamma = last_pose[5];


    FLOAT_T sin_alpha = sin(alpha);
    FLOAT_T cos_alpha = cos(alpha);  
    FLOAT_T sin_beta  = sin(beta);
    FLOAT_T cos_beta  = cos(beta);
    FLOAT_T sin_gamma = sin(gamma);
    FLOAT_T cos_gamma = cos(gamma);

    FLOAT_T tf_particle[16];

    tf_particle[0] = cos_beta * cos_gamma;
    tf_particle[4] = cos_beta * sin_gamma;
    tf_particle[8] = -sin_beta;
    // tf_particle[3] = particles[tid].first[0];
    tf_particle[3] = 0;
    
    tf_particle[1] = sin_alpha * sin_beta * cos_gamma - cos_alpha * sin_gamma;
    tf_particle[5] = sin_alpha * sin_beta * sin_gamma + cos_alpha * cos_gamma;
    tf_particle[9] = sin_alpha * cos_beta;
    // tf_particle[7] = particles[tid].first[1];
    tf_particle[7] = 0;

    tf_particle[2] = cos_alpha * sin_beta * cos_gamma + sin_alpha * sin_gamma;
    tf_particle[6] = cos_alpha * sin_beta * sin_gamma - sin_alpha * cos_gamma;
    tf_particle[10] = cos_alpha * cos_beta;
    // tf_particle[11] = particles[tid].first[2];
    tf_particle[11] = 0;

    

    FLOAT_T tf[16];

    tf[0] = tf_particle[0] * tf_odom[0] + tf_particle[1] * tf_odom[4] + tf_particle[2] * tf_odom[8];
    tf[1] = tf_particle[0] * tf_odom[1] + tf_particle[1] * tf_odom[5] + tf_particle[2] * tf_odom[9];
    tf[2] = tf_particle[0] * tf_odom[2] + tf_particle[1] * tf_odom[6] + tf_particle[2] * tf_odom[10];
    tf[3] = tf_particle[0] * tf_odom[3] + tf_particle[1] * tf_odom[7] + tf_particle[2] * tf_odom[11] + tf_particle[3];

    tf[4] = tf_particle[4] * tf_odom[0] + tf_particle[5] * tf_odom[4] + tf_particle[6] * tf_odom[8];
    tf[5] = tf_particle[4] * tf_odom[1] + tf_particle[5] * tf_odom[5] + tf_particle[6] * tf_odom[9];
    tf[6] = tf_particle[4] * tf_odom[2] + tf_particle[5] * tf_odom[6] + tf_particle[6] * tf_odom[10];
    tf[7] = tf_particle[4] * tf_odom[3] + tf_particle[5] * tf_odom[7] + tf_particle[6] * tf_odom[11] + tf_particle[7];

    tf[8] = tf_particle[8] * tf_odom[0] + tf_particle[9] * tf_odom[4] + tf_particle[10] * tf_odom[8];
    tf[9] = tf_particle[8] * tf_odom[1] + tf_particle[9] * tf_odom[5] + tf_particle[10] * tf_odom[9];
    tf[10] = tf_particle[8] * tf_odom[2] + tf_particle[9] * tf_odom[6] + tf_particle[10] * tf_odom[10];
    tf[11] = tf_particle[8] * tf_odom[3] + tf_particle[9] * tf_odom[7] + tf_particle[10] * tf_odom[11] + tf_particle[11];

    auto t_x = tf[3];
    auto t_y = tf[7];
    auto t_z = tf[11];

    FLOAT_T t_roll  = 0.0;  
    FLOAT_T t_pitch = 0.0;
    FLOAT_T t_yaw   = 0.0;
    getAngleFromMat(tf, t_roll, t_pitch, t_yaw);

    new_particles[index].first[0] = last_pose[0] + t_x;
    new_particles[index].first[1] = last_pose[1] + t_y;
    new_particles[index].first[2] = last_pose[2] + t_z;
    
    new_particles[index].second = last_particle.second;

    // new_particles[index].first[5] = last_pose[5] + t_yaw;//yaw;
    // new_particles[index].first[3] = last_pose[3] + t_roll;//roll; 
    // new_particles[index].first[4] = last_pose[4] + t_pitch;//pitch;
  
    new_particles[index].first[5] = t_yaw;//yaw;
    new_particles[index].first[3] = t_roll;//roll; 
    new_particles[index].first[4] = t_pitch;//pitch;
  }

  m_particles = new_particles;
}

void ParticleCloud::motionUpdate(const ImuAccumulator::Data& imu_data)
{
  std::vector<Particle> new_particles(m_particles.size());

  auto linear_velocity = imu_data.linear_vel; // dom.twist.twist.linear.x;
  auto angular_velocity = imu_data.angular_yaw; // odom.twist.twist.angular.z;

  FLOAT_T random_linear_velocity_x;
  FLOAT_T random_linear_velocity_y;
  FLOAT_T random_linear_velocity_z;
  FLOAT_T random_velocity_roll;
  FLOAT_T random_velocity_pitch;
  FLOAT_T random_velocity_yaw;

  ros::Time current_time = ros::Time::now();
  FLOAT_T time_diff = (current_time - m_last_time).toSec();
  m_last_time = current_time;

  ref_pose[0] = ref_pose[0] + linear_velocity * time_diff * cos(ref_pose[5] + (angular_velocity / 2 * time_diff));
  ref_pose[1] = ref_pose[1] + linear_velocity * time_diff * sin(ref_pose[5] + (angular_velocity / 2 * time_diff));
  ref_pose[5] = ref_pose[5] + angular_velocity * time_diff;

  auto ref_dist = std::sqrt(ref_pose[0] * ref_pose[0] + ref_pose[1] * ref_pose[1] + ref_pose[2] * ref_pose[2]);


  auto d = linear_velocity * time_diff;
  auto d_square = d * d;

  auto theta = angular_velocity * time_diff;
  auto theta_square = theta * theta;

  std::normal_distribution<> distribution_linear_x{d, a_1_ * d_square + a_2_ * theta_square};
  std::normal_distribution<> distribution_linear_y{0, a_3_ * d_square + a_4_ * theta_square};
  std::normal_distribution<> distribution_linear_z{0, a_5_ * d_square + a_6_ * theta_square};
  
  std::normal_distribution<> distribution_roll{0, a_7_ * d_square + a_8_ * theta_square};
  std::normal_distribution<> distribution_pitch{0, a_9_ * d_square + a_10_ * theta_square};
  std::normal_distribution<> distribution_yaw{theta, a_11_ * d_square + a_12_ * theta_square};

  apply_model(distribution_linear_x, distribution_linear_y, distribution_linear_z,
              distribution_roll, distribution_pitch, distribution_yaw, new_particles);

  m_particles = new_particles;
}

void ParticleCloud::motionUpdate(FLOAT_T lin_scale, FLOAT_T ang_scale)
{
  std::vector<Particle> new_particles(m_particles.size());

  FLOAT_T random_linear_velocity_x;
  FLOAT_T random_linear_velocity_y;
  FLOAT_T random_linear_velocity_z;
  FLOAT_T random_velocity_roll;
  FLOAT_T random_velocity_pitch;
  FLOAT_T random_velocity_yaw;

  ros::Time current_time = ros::Time::now();
  FLOAT_T time_diff = (current_time - m_last_time).toSec();
  m_last_time = current_time;

  auto d = lin_scale * time_diff;
  auto d_square = d * d;

  auto theta = ang_scale * time_diff;
  auto theta_square = theta * theta;

  std::normal_distribution<> distribution_linear_x{0, a_1_ * d_square + a_2_ * theta_square};
  std::normal_distribution<> distribution_linear_y{0, a_3_ * d_square + a_4_ * theta_square};
  std::normal_distribution<> distribution_linear_z{0, a_5_ * d_square + a_6_ * theta_square};
  
  std::normal_distribution<> distribution_roll{0, a_7_ * d_square + a_8_ * theta_square};
  std::normal_distribution<> distribution_pitch{0, a_9_ * d_square + a_10_ * theta_square};
  std::normal_distribution<> distribution_yaw{0, a_11_ * d_square + a_12_ * theta_square};

  apply_model(distribution_linear_x, distribution_linear_y, distribution_linear_z,
              distribution_roll, distribution_pitch, distribution_yaw, new_particles);

  m_particles = new_particles;
}

void ParticleCloud::motionUpdate(FLOAT_T lin_scale, const ImuAccumulator::Data& imu_data)
{
  std::vector<Particle> new_particles(m_particles.size());

  FLOAT_T random_linear_velocity_x;
  FLOAT_T random_linear_velocity_y;
  FLOAT_T random_linear_velocity_z;
  FLOAT_T random_velocity_roll;
  FLOAT_T random_velocity_pitch;
  FLOAT_T random_velocity_yaw;

  ros::Time current_time = ros::Time::now();
  FLOAT_T time_diff = (current_time - m_last_time).toSec();
  m_last_time = current_time;

  auto d = lin_scale * time_diff;
  auto d_square = d * d;

  auto roll = imu_data.delta_roll;
  auto roll_square = roll * roll;

  auto pitch = imu_data.delta_pitch;
  auto pitch_square = pitch * pitch;

  auto theta = imu_data.delta_yaw;
  auto theta_square = theta * theta;

  std::normal_distribution<> distribution_linear_x{0, a_1_ * d_square + a_2_ * theta_square};
  std::normal_distribution<> distribution_linear_y{0, a_3_ * d_square + a_4_ * theta_square};
  std::normal_distribution<> distribution_linear_z{0, a_5_ * d_square + a_6_ * theta_square};

  std::normal_distribution<> distribution_roll{roll, a_8_ * roll_square};
  std::normal_distribution<> distribution_pitch{pitch, a_10_ * pitch_square};
  std::normal_distribution<> distribution_yaw{theta, a_12_ * theta_square};

  apply_model(distribution_linear_x, distribution_linear_y, distribution_linear_z,
              distribution_roll, distribution_pitch, distribution_yaw, new_particles);

  m_particles = new_particles;
}

void ParticleCloud::motion_update(FLOAT_T dx, FLOAT_T dy, FLOAT_T dz, FLOAT_T roll, FLOAT_T pitch, FLOAT_T yaw, FLOAT_T lin_base_noise, FLOAT_T ang_base_noise)
{
  std::vector<Particle> new_particles(m_particles.size());

  FLOAT_T random_linear_velocity_x;
  FLOAT_T random_linear_velocity_y;
  FLOAT_T random_linear_velocity_z;
  FLOAT_T random_velocity_roll;
  FLOAT_T random_velocity_pitch;
  FLOAT_T random_velocity_yaw;

  ros::Time current_time = ros::Time::now();
  FLOAT_T time_diff = (current_time - m_last_time).toSec();
  m_last_time = current_time;

  ref_pose[0] = ref_pose[0] + dx * cos(ref_pose[5] + yaw);
  ref_pose[1] = ref_pose[1] + dx * sin(ref_pose[5] + yaw);
  ref_pose[5] = ref_pose[5] + yaw;

  // auto ref_dist = std::sqrt(ref_pose[0] * ref_pose[0] + ref_pose[1] * ref_pose[1] + ref_pose[2] * ref_pose[2]);

  auto dx_square = dx * dx;
  auto dy_square = dy * dy;
  auto dz_square = dz * dz;

  auto roll_square = roll * roll;
  auto pitch_square = pitch * pitch;
  auto yaw_square = yaw * yaw;

  std::normal_distribution<> distribution_linear_x{dx, a_1_ * dx_square + a_2_ * yaw_square + lin_base_noise};
  std::normal_distribution<> distribution_linear_y{dy, a_3_ * dy_square + a_4_ * yaw_square + lin_base_noise};
  std::normal_distribution<> distribution_linear_z{dz, a_5_ * dz_square + a_6_ * yaw_square + lin_base_noise};

  std::normal_distribution<> distribution_roll{roll, a_7_ * dx_square + a_8_ * roll_square + ang_base_noise};
  std::normal_distribution<> distribution_pitch{pitch, a_9_ * dx_square + a_10_ * pitch_square + ang_base_noise};
  std::normal_distribution<> distribution_yaw{yaw, a_11_ * dx_square + a_12_ * yaw_square + ang_base_noise};

  apply_model(distribution_linear_x, distribution_linear_y, distribution_linear_z,
              distribution_roll, distribution_pitch, distribution_yaw, new_particles);

  m_particles = new_particles;
}

void ParticleCloud::apply_model(std::normal_distribution<>& distribution_linear_x, std::normal_distribution<>& distribution_linear_y, std::normal_distribution<>& distribution_linear_z,
                   std::normal_distribution<>& distribution_roll, std::normal_distribution<>& distribution_pitch, std::normal_distribution<>& distribution_yaw, std::vector<Particle>& new_particles)
{
  for(auto index = 0u; index < m_particles.size(); index++)
  { 
    auto& last_particle = m_particles[index];

    auto& last_pose = last_particle.first;

    auto dx = distribution_linear_x(*m_generator_ptr);
    auto dy = distribution_linear_y(*m_generator_ptr);
    auto dz = distribution_linear_z(*m_generator_ptr);

    auto roll  = distribution_roll(*m_generator_ptr);
    auto pitch = distribution_pitch(*m_generator_ptr);
    auto yaw   = distribution_yaw(*m_generator_ptr);

    FLOAT_T sin_roll  = sin(roll);
    FLOAT_T cos_roll  = cos(roll);  
    FLOAT_T sin_pitch = sin(pitch);
    FLOAT_T cos_pitch = cos(pitch);
    FLOAT_T sin_yaw   = sin(yaw);
    FLOAT_T cos_yaw   = cos(yaw);

    FLOAT_T tf_odom[16];

    tf_odom[0] = cos_pitch * cos_yaw;
    tf_odom[4] = cos_pitch * sin_yaw;
    tf_odom[8] = -sin_pitch;
    tf_odom[3] = dx;
    
    tf_odom[1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
    tf_odom[5] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
    tf_odom[9] = sin_roll * cos_pitch;
    tf_odom[7] = dy;

    tf_odom[2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;
    tf_odom[6] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;
    tf_odom[10] = cos_roll * cos_pitch;
    tf_odom[11] = dz;



    auto alpha = last_pose[3];
    auto beta = last_pose[4];
    auto gamma = last_pose[5];


    FLOAT_T sin_alpha = sin(alpha);
    FLOAT_T cos_alpha = cos(alpha);  
    FLOAT_T sin_beta  = sin(beta);
    FLOAT_T cos_beta  = cos(beta);
    FLOAT_T sin_gamma = sin(gamma);
    FLOAT_T cos_gamma = cos(gamma);

    FLOAT_T tf_particle[16];

    tf_particle[0] = cos_beta * cos_gamma;
    tf_particle[4] = cos_beta * sin_gamma;
    tf_particle[8] = -sin_beta;
    // tf_particle[3] = particles[tid].first[0];
    tf_particle[3] = 0;
    
    tf_particle[1] = sin_alpha * sin_beta * cos_gamma - cos_alpha * sin_gamma;
    tf_particle[5] = sin_alpha * sin_beta * sin_gamma + cos_alpha * cos_gamma;
    tf_particle[9] = sin_alpha * cos_beta;
    // tf_particle[7] = particles[tid].first[1];
    tf_particle[7] = 0;

    tf_particle[2] = cos_alpha * sin_beta * cos_gamma + sin_alpha * sin_gamma;
    tf_particle[6] = cos_alpha * sin_beta * sin_gamma - sin_alpha * cos_gamma;
    tf_particle[10] = cos_alpha * cos_beta;
    // tf_particle[11] = particles[tid].first[2];
    tf_particle[11] = 0;

    

    FLOAT_T tf[16];

    tf[0] = tf_particle[0] * tf_odom[0] + tf_particle[1] * tf_odom[4] + tf_particle[2] * tf_odom[8];
    tf[1] = tf_particle[0] * tf_odom[1] + tf_particle[1] * tf_odom[5] + tf_particle[2] * tf_odom[9];
    tf[2] = tf_particle[0] * tf_odom[2] + tf_particle[1] * tf_odom[6] + tf_particle[2] * tf_odom[10];
    tf[3] = tf_particle[0] * tf_odom[3] + tf_particle[1] * tf_odom[7] + tf_particle[2] * tf_odom[11] + tf_particle[3];

    tf[4] = tf_particle[4] * tf_odom[0] + tf_particle[5] * tf_odom[4] + tf_particle[6] * tf_odom[8];
    tf[5] = tf_particle[4] * tf_odom[1] + tf_particle[5] * tf_odom[5] + tf_particle[6] * tf_odom[9];
    tf[6] = tf_particle[4] * tf_odom[2] + tf_particle[5] * tf_odom[6] + tf_particle[6] * tf_odom[10];
    tf[7] = tf_particle[4] * tf_odom[3] + tf_particle[5] * tf_odom[7] + tf_particle[6] * tf_odom[11] + tf_particle[7];

    tf[8] = tf_particle[8] * tf_odom[0] + tf_particle[9] * tf_odom[4] + tf_particle[10] * tf_odom[8];
    tf[9] = tf_particle[8] * tf_odom[1] + tf_particle[9] * tf_odom[5] + tf_particle[10] * tf_odom[9];
    tf[10] = tf_particle[8] * tf_odom[2] + tf_particle[9] * tf_odom[6] + tf_particle[10] * tf_odom[10];
    tf[11] = tf_particle[8] * tf_odom[3] + tf_particle[9] * tf_odom[7] + tf_particle[10] * tf_odom[11] + tf_particle[11];

    auto t_x = tf[3];
    auto t_y = tf[7];
    auto t_z = tf[11];

    FLOAT_T t_roll  = 0.0;  
    FLOAT_T t_pitch = 0.0;
    FLOAT_T t_yaw   = 0.0;
    getAngleFromMat(tf, t_roll, t_pitch, t_yaw);

    new_particles[index].first[0] = last_pose[0] + t_x;
    new_particles[index].first[1] = last_pose[1] + t_y;
    new_particles[index].first[2] = last_pose[2] + t_z;
    
    new_particles[index].second = last_particle.second;

    // new_particles[index].first[5] = last_pose[5] + t_yaw;//yaw;
    // new_particles[index].first[3] = last_pose[3] + t_roll;//roll; 
    // new_particles[index].first[4] = last_pose[4] + t_pitch;//pitch;
  
    new_particles[index].first[5] = t_yaw;//yaw;
    new_particles[index].first[3] = t_roll;//roll; 
    new_particles[index].first[4] = t_pitch;//pitch;
  }

}

Particle& ParticleCloud::operator[](unsigned int index)
{
  if(index >= size())
  {
    throw std::out_of_range("Index exceeds number of particles");
  }

  return m_particles[index];
}

std::size_t ParticleCloud::size() const
{
  return m_particles.size();
}

void ParticleCloud::resize(unsigned int number_of_particles)
{
  m_particles.resize(number_of_particles);
}

const Particle& ParticleCloud::getRandomParticle() const
{
  // Initializing an uniform distribution for choosing a new particle
  static std::uniform_real_distribution<> uniform_distribution(0.0, 1.0);

  FLOAT_T random_value = uniform_distribution(*m_generator_ptr);
  FLOAT_T weight_sum = 0.0;
  FLOAT_T current_weight;


  for(auto index = 0u; index < m_particles.size(); index++)
  { 
    current_weight = m_particles[index].second;
    weight_sum += current_weight;

    if(random_value <= weight_sum)
    {
      return m_particles[index];
    }
  }

  return m_particles[m_particles.size() - 1];
}

} // namespace tsdf_localization