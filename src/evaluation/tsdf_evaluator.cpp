#include <tsdf_localization/evaluation/tsdf_evaluator.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tsdf_localization/evaluation/model/evaluation_model.h>

#include <tsdf_localization/evaluation/model/likelihood_evaluation.h>

#include <tsdf_localization/util/constant.h>

#include <omp.h>

namespace tsdf_localization
{

static int avg_reuse = 0;
static int reuse_count = 0;
static double invalid_rate = 0;

FLOAT_T TSDFEvaluator::evaluatePose(FLOAT_T* pose, const std::vector<CudaPoint> cloud, EvaluationModel& model) const
{
    geometry_msgs::msg::Point point;

    FLOAT_T eval_sum = 0.0;

    if (map_ptr_->trackReuse())
    {
      map_ptr_->resetInvalidCount();
    }

    for (const auto& point : cloud)
    {
        auto x = point.x;
        auto y = point.y;
        auto z = point.z;

        auto transformed_x = pose[0] * x + pose[1] * y + pose[2] * z + pose[3];
        auto transformed_y = pose[4] * x + pose[5] * y + pose[6] * z + pose[7];
        auto transformed_z = pose[8] * x + pose[9] * y + pose[10] * z + pose[11];

        auto value = map_ptr_->getEntry(transformed_x, transformed_y, transformed_z);
        
        if (map_ptr_->trackReuse() && map_ptr_->isNewTrack())
        {
          avg_reuse += map_ptr_->getLastTrack();
          ++reuse_count;
        }

        auto square_dist = x*x + y*y + z*z;

        if (square_dist < max_range_squared_)
        {
          value = a_hit_ * value + a_range_* inv_max_range_;
        }
        else
        {
          value = a_hit_ * value + a_max_;
        }

        eval_sum += value;
    }

    if (map_ptr_->trackReuse())
    {
      invalid_rate += (map_ptr_->getInvalidCount() / static_cast<double>(cloud.size()));
    }

    return eval_sum;
}

geometry_msgs::msg::PoseWithCovariance TSDFEvaluator::evaluate(std::vector<Particle>& particles, const std::vector<CudaPoint>& points, FLOAT_T tf_matrix[16], bool use_cuda)
{
  if (use_cuda)
  {
    return cuda_evaluator_->evaluate(particles, points, tf_matrix);
  }

  FLOAT_T weight_sum = 0.0;

  std::array<geometry_msgs::msg::Pose, OMP_THREADS> laser_pose;
  std::array<tf2::Transform, OMP_THREADS> tf_pose;

  int threads = OMP_THREADS;
  
  if (map_ptr_->trackReuse())
  {
    threads = 1;
  }

  #pragma omp parallel for num_threads(threads) schedule(dynamic) reduction(+:weight_sum)
  for(auto index = 0u; index < particles.size(); index++)
  {
    // Build a pose for every particle
    
    FLOAT_T tf_particle[16];

    FLOAT_T alpha = particles[index].first[3];
    FLOAT_T beta =  particles[index].first[4];
    FLOAT_T gamma = particles[index].first[5];

    FLOAT_T sin_alpha = sin(alpha);
    FLOAT_T cos_alpha = cos(alpha);  
    FLOAT_T sin_beta  = sin(beta);
    FLOAT_T cos_beta  = cos(beta);
    FLOAT_T sin_gamma = sin(gamma);
    FLOAT_T cos_gamma = cos(gamma);

    tf_particle[0] = cos_beta * cos_gamma;
    tf_particle[4] = cos_beta * sin_gamma;
    tf_particle[8] = -sin_beta;
    tf_particle[3] = particles[index].first[0];
    
    tf_particle[1] = sin_alpha * sin_beta * cos_gamma - cos_alpha * sin_gamma;
    tf_particle[5] = sin_alpha * sin_beta * sin_gamma + cos_alpha * cos_gamma;
    tf_particle[9] = sin_alpha * cos_beta;
    tf_particle[7] = particles[index].first[1];
    
    tf_particle[2] = cos_alpha * sin_beta * cos_gamma + sin_alpha * sin_gamma;
    tf_particle[6] = cos_alpha * sin_beta * sin_gamma - sin_alpha * cos_gamma;
    tf_particle[10] = cos_alpha * cos_beta;
    tf_particle[11] = particles[index].first[2];

    FLOAT_T tf_laser[16];

    tf_laser[0] = tf_particle[0] * tf_matrix[0] + tf_particle[1] * tf_matrix[4] + tf_particle[2] * tf_matrix[8];
    tf_laser[1] = tf_particle[0] * tf_matrix[1] + tf_particle[1] * tf_matrix[5] + tf_particle[2] * tf_matrix[9];
    tf_laser[2] = tf_particle[0] * tf_matrix[2] + tf_particle[1] * tf_matrix[6] + tf_particle[2] * tf_matrix[10];
    tf_laser[3] = tf_particle[0] * tf_matrix[3] + tf_particle[1] * tf_matrix[7] + tf_particle[2] * tf_matrix[11] + tf_particle[3];

    tf_laser[4] = tf_particle[4] * tf_matrix[0] + tf_particle[5] * tf_matrix[4] + tf_particle[6] * tf_matrix[8];
    tf_laser[5] = tf_particle[4] * tf_matrix[1] + tf_particle[5] * tf_matrix[5] + tf_particle[6] * tf_matrix[9];
    tf_laser[6] = tf_particle[4] * tf_matrix[2] + tf_particle[5] * tf_matrix[6] + tf_particle[6] * tf_matrix[10];
    tf_laser[7] = tf_particle[4] * tf_matrix[3] + tf_particle[5] * tf_matrix[7] + tf_particle[6] * tf_matrix[11] + tf_particle[7];

    tf_laser[8] = tf_particle[8] * tf_matrix[0] + tf_particle[9] * tf_matrix[4] + tf_particle[10] * tf_matrix[8];
    tf_laser[9] = tf_particle[8] * tf_matrix[1] + tf_particle[9] * tf_matrix[5] + tf_particle[10] * tf_matrix[9];
    tf_laser[10] = tf_particle[8] * tf_matrix[2] + tf_particle[9] * tf_matrix[6] + tf_particle[10] * tf_matrix[10];
    tf_laser[11] = tf_particle[8] * tf_matrix[3] + tf_particle[9] * tf_matrix[7] + tf_particle[10] * tf_matrix[11] + tf_particle[11];

    tf_laser[15] = 1;

    // Simulate a scan for every considered pose and calculate how well it matches the actual one

    LikelihoodEvaluation eval(10000);

    FLOAT_T weight = evaluatePose(tf_laser, points, eval);

    particles[index].second = weight;
    weight_sum += weight;
  }

  if(weight_sum == 0.0)
  {
    throw std::runtime_error("No particle is valid!");
  }

  // Normalize weights and estimate current robot pose

  Particle average_particle;
  geometry_msgs::msg::PoseWithCovariance average_pose;

  FLOAT_T variance_x = 0.0;
  FLOAT_T variance_y = 0.0;
  FLOAT_T variance_z = 0.0;

  FLOAT_T variance_roll = 0.0;
  FLOAT_T variance_pitch = 0.0;
  FLOAT_T variance_yaw = 0.0;

  average_particle.first[0] = 0.0;
  average_particle.first[1] = 0.0;
  average_particle.first[2] = 0.0;
  average_particle.first[3] = 0.0;
  average_particle.first[4] = 0.0;
  average_particle.first[5] = 0.0;

  FLOAT_T sum_sin_roll = 0;
  FLOAT_T sum_cos_roll = 0;
  
  FLOAT_T sum_sin_pitch = 0;
  FLOAT_T sum_cos_pitch = 0;

  FLOAT_T sum_sin_yaw = 0;
  FLOAT_T sum_cos_yaw = 0;

  auto& avg_x = average_particle.first[0];
  auto& avg_y = average_particle.first[1];
  auto& avg_z = average_particle.first[2];


  #pragma omp parallel for num_threads(threads) schedule(dynamic) reduction(+:avg_x) reduction(+:avg_y) reduction(+:avg_z) \
                                                                  reduction(+:sum_sin_roll) reduction(+:sum_sin_pitch) reduction(+:sum_sin_yaw) \
                                                                  reduction(+:sum_cos_roll) reduction(+:sum_cos_pitch) reduction(+:sum_cos_yaw)
  for(auto index = 0u; index < particles.size(); index++)
  {
    particles[index].second /= weight_sum;

    average_particle.first[0] += particles[index].first[0] * particles[index].second;
    average_particle.first[1] += particles[index].first[1] * particles[index].second;
    average_particle.first[2] += particles[index].first[2] * particles[index].second;
    
    sum_sin_roll += sin(particles[index].first[3]) * particles[index].second;
    sum_cos_roll += cos(particles[index].first[3]) * particles[index].second;

    sum_sin_pitch += sin(particles[index].first[4]) * particles[index].second;
    sum_cos_pitch += cos(particles[index].first[4]) * particles[index].second;

    sum_sin_yaw += sin(particles[index].first[5]) * particles[index].second;
    sum_cos_yaw += cos(particles[index].first[5]) * particles[index].second;
  }
  
  average_particle.first[3] = atan2(sum_sin_roll, sum_cos_roll);
  average_particle.first[4] = atan2(sum_sin_pitch, sum_cos_pitch);
  average_particle.first[5] = atan2(sum_sin_yaw, sum_cos_yaw);

  average_pose.pose.position.x = average_particle.first[0];
  average_pose.pose.position.y = average_particle.first[1];
  average_pose.pose.position.z = average_particle.first[2];

  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(average_particle.first[3] , average_particle.first[4], average_particle.first[5]);
  tf2::convert(tf_quaternion, average_pose.pose.orientation);

  average_pose.covariance.data()[0] = variance_x;
  average_pose.covariance.data()[7] = variance_y;
  average_pose.covariance.data()[14] = variance_z;
  average_pose.covariance.data()[21] = variance_roll;
  average_pose.covariance.data()[28] = variance_pitch;
  average_pose.covariance.data()[35] = variance_yaw;

  if (map_ptr_->trackReuse())
  {
    std::cout << "Average coarse grid reuse during point cloud iteration: " << avg_reuse / static_cast<double>(reuse_count) << std::endl;
    std::cout << "Average invalid rate: " << invalid_rate / particles.size() << std::endl;
  }

  return average_pose;
}

geometry_msgs::msg::PoseWithCovariance TSDFEvaluator::evaluateParticles(ParticleCloud& particle_cloud, const sensor_msgs::msg::PointCloud2& real_cloud, const std::string& robot_frame, const std::string& scan_frame, bool use_cuda, bool ignore_tf)
{
  FLOAT_T tf_matrix[16];

  if (!ignore_tf)
  {
    static tf2::BufferCore tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    geometry_msgs::msg::TransformStamped scan_to_base; 

    try
    {
      // TODO: Pass 0.5 secs
      //scan_to_base = tf_buffer.lookupTransform(robot_frame, scan_frame, real_cloud.header.stamp);
      scan_to_base = tf_buffer.lookupTransform(robot_frame, scan_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException& e)
    {
      std::cerr << "Couldn't transform robot frame to scan frame: " << e.what() << std::endl;
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
  }
  else
  {
    tf_matrix[0] = 1;
    tf_matrix[1] = 0;
    tf_matrix[2] = 0;
    tf_matrix[3] = 0;
    
    tf_matrix[4] = 0;
    tf_matrix[5] = 1;
    tf_matrix[6] = 0;
    tf_matrix[7] = 0;
    
    tf_matrix[8] = 0;
    tf_matrix[9] = 0;
    tf_matrix[10] = 1;
    tf_matrix[11] = 0;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(real_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<short> iter_ring(real_cloud, "ring");

  std::unordered_set<SortClass, hash> point_set;

  int index = 0;

  for (; iter_x != iter_x.end(); ++iter_x)
  {
      FLOAT_T x = iter_x[0];
      FLOAT_T y = iter_x[1];
      FLOAT_T z = iter_x[2];

      FLOAT_T dist = std::sqrt(x*x + y*y + z*z);
      
      if (dist < 1.0)
      {
          continue;
      }

      CudaPoint center = {static_cast<float>(std::floor(iter_x[0] / map_res_) * map_res_ + map_res_half_), 
                          static_cast<float>(std::floor(iter_x[1] / map_res_) * map_res_ + map_res_half_), 
                          static_cast<float>(std::floor(iter_x[2] / map_res_) * map_res_ + map_res_half_)};
      
      //point_set.insert(std::make_pair(iter_ring[0], center));
      point_set.insert(SortClass(iter_ring[0], index, center, std::make_shared<CudaPoint>(iter_x[0], iter_x[1], iter_x[2])));

      ++iter_ring;
      ++index;
  }

  std::vector<SortClass> reduced_points;
  reduced_points.resize(point_set.size());
  std::copy(point_set.begin(), point_set.end(), reduced_points.begin());

  std::vector<CudaPoint> ordered_points;
  ordered_points.reserve(reduced_points.size());

  std::vector<std::vector<std::pair<int, CudaPoint>>> sort_points(64);

  struct
  {
      bool operator()(std::pair<int, CudaPoint>& a, std::pair<int, CudaPoint>& b)
      {
          return a.first < b.first;
      }
  } customComp;


  for (auto& curr_ring : sort_points)
  {
      curr_ring.reserve(1500);
  }

  for (const auto& point : reduced_points)
  {
      sort_points[point.ring_].push_back(std::make_pair(point.index_, *point.original_));
  }

  for (auto& curr_ring : sort_points)
  {
      std::sort(curr_ring.begin(), curr_ring.end(), customComp);
  }

  for (auto index = 0u; index < sort_points.size(); ++index)
  {
      auto& curr_ring = sort_points[index];

      for (auto& point : curr_ring)
      {
          ordered_points.push_back(point.second);
      }
  }

return evaluate(particle_cloud.particles(), ordered_points, tf_matrix, use_cuda);
}

} // namespace tsdf_localization