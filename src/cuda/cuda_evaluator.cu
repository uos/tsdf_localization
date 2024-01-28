#include <tsdf_localization/cuda/cuda_evaluator.h>
#include <stdio.h>

#include <tsdf_localization/cuda/cuda_util.h>

// #include <sensor_msgs/point_cloud2_iterator.hpp>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tsdf_localization/util/runtime_evaluator.h>
#include <map>

#include <tsdf_localization/cuda/cuda_data.h>
#include <tsdf_localization/cuda/cuda_eval_particles.h>
#include <tsdf_localization/cuda/cuda_sum.h>

namespace tsdf_localization 
{

CudaEvaluator::CudaEvaluator(CudaSubVoxelMap<FLOAT_T, FLOAT_T>& map, bool per_point, FLOAT_T a_hit, FLOAT_T a_range, FLOAT_T a_max, FLOAT_T max_range) : 
d_map_(nullptr), per_point_(per_point), d_grid_occ_(nullptr), d_data_(nullptr), d_particles_(nullptr), d_particles_ordered_(nullptr), particles_reserved_(0), d_points_(nullptr), d_points_ordered_(nullptr), points_reserved_(0), d_transform_(nullptr), d_new_weights_(nullptr), d_point_weights_(nullptr), point_weights_size_(0),
p_x_{nullptr}, p_y_{nullptr}, p_z_{nullptr}, sin_a_{nullptr}, cos_a_{nullptr}, sin_b_{nullptr}, cos_b_{nullptr}, sin_c_{nullptr}, cos_c_{nullptr}/*, dev_random_{nullptr}*/,
a_hit_{a_hit}, a_range_(a_range), a_max_(a_max), 
max_range_(max_range), inv_max_range_(1.0 / max_range), max_range_squared_(max_range * max_range)
{
  try
  {
    cudaMemcpyToSymbol(const_map_coef_, &(map.coef()), sizeof(CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef));
    cudaMemcpyToSymbol(const_a_hit_, &a_hit_, sizeof(FLOAT_T));
    cudaMemcpyToSymbol(const_a_range_, &a_range_, sizeof(FLOAT_T));
    cudaMemcpyToSymbol(const_a_max_, &a_max_, sizeof(FLOAT_T));

    cudaMemcpyToSymbol(const_max_range_, &max_range_, sizeof(FLOAT_T));
    cudaMemcpyToSymbol(const_max_range_squared_, &max_range_squared_, sizeof(FLOAT_T));
    cudaMemcpyToSymbol(const_inv_max_range_, &inv_max_range_, sizeof(FLOAT_T));


    cudaCheck();

    cudaMalloc((OCC_T**) &(g_grid_occ_), map.gridOccBytes());
    cudaMemcpy(g_grid_occ_, map.rawGridOcc(), map.gridOccBytes(), cudaMemcpyHostToDevice);

    cudaMalloc((FLOAT_T**) &(g_data_), map.dataBytes());
    cudaMemcpy(g_data_, map.rawData(), map.dataBytes(), cudaMemcpyHostToDevice);
    
    cudaCheck();

    cudaMalloc((FLOAT_T**) &(d_transform_), TRANSFORM_BYTES);

    cudaCheck();
  }
  catch (std::runtime_error&)
  {
    throw std::runtime_error("Error while creating the CUDA context for the map!");
  }

  cudaFuncSetCacheConfig(cudaEvaluateParticlesOrdered, cudaFuncCachePreferL1);
}

struct lower_than_key
{
    inline bool operator()(const Particle& particle1, const Particle& particle2) 
    {
        return particle1.first[0] < particle2.first[0];
    }
};

inline FLOAT_T particle_dist(const Particle& particle1, const Particle& particle2)
{
    auto dx = particle1.first[0] - particle2.first[0];
    auto dy = particle1.first[1] - particle2.first[1];
    auto dz = particle1.first[2] - particle2.first[2];

    return sqrt((dx * dx + dy * dy + dz * dz));
} 

// geometry_msgs::msg::PoseWithCovariance CudaEvaluator::evaluate(std::vector<Particle>& particles, const sensor_msgs::msg::PointCloud2& real_cloud, FLOAT_T tf_matrix[16])
// {
//   sensor_msgs::PointCloud2ConstIterator<float> iter_x(real_cloud, "x");
//   sensor_msgs::PointCloud2ConstIterator<int> iter_ring(real_cloud, "ring");

//   std::vector<CudaPoint> points;
//   points.reserve(real_cloud.width);
  
//   std::multimap<int, CudaPoint> map;

//   for (; iter_x != iter_x.end(); ++iter_x)
//   {
//       map.insert(std::pair<int, CudaPoint>(iter_ring[0], {iter_x[0], iter_x[1], iter_x[2]}));

//       ++iter_ring;
//   }

//   for (const auto& entry : map)
//   {
//     points.push_back(entry.second);
//   }

//   std::unordered_set<CudaPoint, hash> point_set;

//   for (const auto& point : points)
//   {
//       CudaPoint center = {static_cast<float>(std::floor(point.x / 0.064) * 0.064 + 0.032), 
//                           static_cast<float>(std::floor(point.y / 0.064) * 0.064 + 0.032), 
//                           static_cast<float>(std::floor(point.z / 0.064) * 0.064 + 0.032)};
      
//       point_set.insert(center);
//   }

//   std::vector<CudaPoint> reduced_points;
//   reduced_points.resize(point_set.size());
//   std::copy(point_set.begin(), point_set.end(), reduced_points.begin());

//   return evaluate(particles, reduced_points, tf_matrix);
// }

Particle CudaEvaluator::evaluate(std::vector<Particle>& particles, const std::vector<CudaPoint>& points, FLOAT_T tf_matrix[16])
{
  static auto& eval = RuntimeEvaluator::get_instance();

  if (points.size() == 0)
  {
    return Particle();
  }

  eval.start("init_kernel");

  try
  {
    auto particle_bytes = sizeof(Particle) * particles.size();
    
    std::vector<FLOAT_T> particles_ordered(6 * particles.size());
    auto particle_ordered_bytes = sizeof(FLOAT_T) * particles_ordered.size();

    if(particles_reserved_ < particles.size())
    {
      if (d_particles_ != nullptr)
      {
        cudaFree(d_particles_);
        cudaFree(d_particles_ordered_);

        cudaFree(p_x_);
        cudaFree(p_y_);
        cudaFree(p_z_);

        cudaFree(sin_a_);
        cudaFree(cos_a_);

        cudaFree(sin_b_);
        cudaFree(cos_b_);

        cudaFree(sin_c_);
        cudaFree(cos_c_);

        if (!per_point_)
        {
          cudaFree(d_new_weights_);
        }

        d_particles_ = nullptr;
        d_particles_ordered_ = nullptr;
        d_new_weights_ = nullptr;

        p_x_ = nullptr;
        p_y_ = nullptr;
        p_z_ = nullptr;

        sin_a_ = nullptr;
        cos_a_ = nullptr;

        sin_b_ = nullptr;
        cos_b_ = nullptr;

        sin_c_ = nullptr;
        cos_c_ = nullptr;
      }

      cudaCheck();

      cudaMalloc((Particle**) &d_particles_, particle_bytes);
      cudaMalloc((FLOAT_T**) &d_particles_ordered_, particle_ordered_bytes);

      cudaMalloc((FLOAT_T**) &p_x_, sizeof(FLOAT_T) * particles.size());
      cudaMalloc((FLOAT_T**) &p_y_, sizeof(FLOAT_T) * particles.size());
      cudaMalloc((FLOAT_T**) &p_z_, sizeof(FLOAT_T) * particles.size());

      cudaMalloc((FLOAT_T**) &sin_a_, sizeof(FLOAT_T) * particles.size());
      cudaMalloc((FLOAT_T**) &cos_a_, sizeof(FLOAT_T) * particles.size());

      cudaMalloc((FLOAT_T**) &sin_b_, sizeof(FLOAT_T) * particles.size());
      cudaMalloc((FLOAT_T**) &cos_b_, sizeof(FLOAT_T) * particles.size());

      cudaMalloc((FLOAT_T**) &sin_c_, sizeof(FLOAT_T) * particles.size());
      cudaMalloc((FLOAT_T**) &cos_c_, sizeof(FLOAT_T) * particles.size());
      
      cudaCheck();

      if (!per_point_)
      {
        cudaMalloc((FLOAT_T**) &d_new_weights_, sizeof(FLOAT_T) * particles.size());
      }

      cudaCheck();

      new_weights_.resize(particles.size());

      particles_reserved_ = particles.size();
    }

    for (auto index = 0u; index < particles.size(); ++index)
    {
      const auto& particle = particles[index];

      particles_ordered[                       index] = particle.first[0];
      particles_ordered[    particles.size() + index] = particle.first[1];
      particles_ordered[2 * particles.size() + index] = particle.first[2];
      particles_ordered[3 * particles.size() + index] = particle.first[3];
      particles_ordered[4 * particles.size() + index] = particle.first[4];
      particles_ordered[5 * particles.size() + index] = particle.first[5];
    }

    cudaMemcpy(d_particles_, particles.data(), particle_bytes, cudaMemcpyHostToDevice);
    cudaMemcpy(d_particles_ordered_, particles_ordered.data(), particle_ordered_bytes, cudaMemcpyHostToDevice);
    cudaCheck();
  }
  catch (std::runtime_error&)
  {
    throw std::runtime_error("Error while preparing the particles for thhe CUDA evaluation kernel!");
  }

  try
  {
    auto point_bytes = sizeof(CudaPoint) * points.size();

    std::vector<FLOAT_T> h_points_ordered(points.size() * 3);
    auto points_ordered_bytes = h_points_ordered.size() * sizeof(FLOAT_T);

    if(points_reserved_ < points.size())
    {
      if (per_point_)
      {
        if (d_points_ordered_ != nullptr)
        {
          cudaFree(d_points_ordered_);
        }

        cudaMalloc((FLOAT_T**) &d_points_ordered_, points_ordered_bytes);
      }
      //else
      {
        if (d_points_ != nullptr)
        {
          cudaFree(d_points_);
        }

        cudaMalloc((CudaPoint**) &d_points_, point_bytes);
      }

      points_reserved_ = points.size();
    }

    cudaMemcpy(d_points_, points.data(), point_bytes, cudaMemcpyHostToDevice);
    cudaCheck();

    if (per_point_)
    {
      for (auto index = 0u; index < points.size(); ++index)
      {
        const auto& point = points[index];

        h_points_ordered[                    index] = point.x;
        h_points_ordered[    points.size() + index] = point.y;
        h_points_ordered[2 * points.size() + index] = point.z;
      }

      cudaMemcpy(d_points_ordered_, h_points_ordered.data(), points_ordered_bytes, cudaMemcpyHostToDevice);
      cudaCheck();
    }
  }
  catch (std::runtime_error&)
  {
    throw std::runtime_error("Error while preparing the cloud points for the CUDA evaluation kernel!");
  }

  try
  {
    cudaMemcpy(d_transform_, tf_matrix, TRANSFORM_BYTES, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(const_tf_matrix, tf_matrix, TRANSFORM_BYTES);
    cudaCheck();
  }
  catch (std::runtime_error&)
  {
    throw std::runtime_error("Error while preparing the cloud points for the CUDA evaluation kernel!");
  }

  eval.stop("init_kernel");

  eval.start("exec_kernel", true);

  int blocksize = 64;

  try
  {

    if (per_point_)
    {
      auto num_data_points = particles.size() * points.size();
      auto num_data_points_bytes = num_data_points * sizeof(FLOAT_T);

      dim3 block(blocksize, 1);
      dim3 grid((num_data_points + block.x - 1) / block.x, 1);

      if (point_weights_size_ < num_data_points_bytes)
      {
        if (d_point_weights_ != nullptr)
        {
          cudaFree(d_point_weights_);
          d_point_weights_ = nullptr;
          cudaCheck();
        }

        cudaMalloc((FLOAT_T**) &d_point_weights_, num_data_points_bytes);
        cudaCheck();
        point_weights_size_ = num_data_points_bytes;
      }

      cudaEvaluatePointsOrdered<<<grid, block>>>(g_grid_occ_, g_data_, g_map_coef_, d_points_ordered_, points.size(), d_particles_ordered_, particles.size(), d_transform_, d_point_weights_);
      cudaDeviceSynchronize();
      cudaCheck();

      if (d_new_weights_ != nullptr)
      {
        cudaFree(d_new_weights_);
        d_new_weights_ = nullptr;
        cudaCheck();
      }

      d_new_weights_ = sumBatched(d_point_weights_, num_data_points, points.size());
      cudaCheck();

      //cudaFree(d_point_weights);
    }
    else
    {
      dim3 block(blocksize, 1);
      dim3 grid((particles.size() + block.x - 1) / block.x, 1);

      cudaEvaluateParticlesOrdered<<<grid, block>>>(g_grid_occ_, g_data_, g_map_coef_, d_points_, points.size(), d_particles_ordered_, particles.size(), d_transform_, d_new_weights_);
    }
    
    cudaDeviceSynchronize();
    cudaCheck();
  }
  catch (std::runtime_error&)
  {
    throw std::runtime_error("Error during the execution of the evaluation kernel occured!");
  }

  eval.stop("exec_kernel");

  eval.start("weight_update");
  
  FLOAT_T weight_sum = weightSum(d_new_weights_, particles.size());

  if(weight_sum == 0.0)
  {
    throw std::runtime_error("No particle is valid!");
  }

  Particle average_particle;
  // geometry_msgs::msg::PoseWithCovariance average_pose;

  // FLOAT_T variance_x = 0.0;
  // FLOAT_T variance_y = 0.0;
  // FLOAT_T variance_z = 0.0;

  // FLOAT_T variance_roll = 0.0;
  // FLOAT_T variance_pitch = 0.0;
  // FLOAT_T variance_yaw = 0.0;

  dim3 block(blocksize, 1);
  dim3 grid((particles.size() + block.x - 1) / block.x, 1);
  weight_particles<<<grid, block>>>(d_particles_ordered_, particles.size(), d_new_weights_, weight_sum, p_x_, p_y_, p_z_, sin_a_, cos_a_,  sin_b_, cos_b_,  sin_c_, cos_c_);
  cudaDeviceSynchronize();

  average_particle.first[0] = weightSum(p_x_, particles.size());
  average_particle.first[1] = weightSum(p_y_, particles.size());
  average_particle.first[2] = weightSum(p_z_, particles.size());
  average_particle.first[3] = atan2(weightSum(sin_a_, particles.size()), weightSum(cos_a_, particles.size()));
  average_particle.first[4] = atan2(weightSum(sin_b_, particles.size()), weightSum(cos_b_, particles.size()));
  average_particle.first[5] = atan2(weightSum(sin_c_, particles.size()), weightSum(cos_c_, particles.size()));


  try
  {
    cudaMemcpy(new_weights_.data(), d_new_weights_, sizeof(FLOAT_T) * particles.size(), cudaMemcpyDeviceToHost);
    cudaCheck();
  }
  catch (std::runtime_error&)
  {
    throw std::runtime_error("Error occured while copying data back from the gpu!");
  }

  for(auto index = 0u; index < particles.size(); index++)
  {
    particles[index].second = new_weights_[index];
  }

  // average_pose.pose.position.x = average_particle.first[0];
  // average_pose.pose.position.y = average_particle.first[1];
  // average_pose.pose.position.z = average_particle.first[2];

  // tf2::Quaternion tf_quaternion;
  // tf_quaternion.setRPY(average_particle.first[3] , average_particle.first[4], average_particle.first[5]);
  // tf2::convert(tf_quaternion, average_pose.pose.orientation);

  // average_pose.covariance.data()[0] = variance_x;
  // average_pose.covariance.data()[7] = variance_y;
  // average_pose.covariance.data()[14] = variance_z;
  // average_pose.covariance.data()[21] = variance_roll;
  // average_pose.covariance.data()[28] = variance_pitch;
  // average_pose.covariance.data()[35] = variance_yaw;

  eval.stop("weight_update");

  return average_particle;
}

CudaEvaluator::~CudaEvaluator()
{
  if (p_x_ != nullptr)
  {
    cudaFree(p_x_);
    cudaCheck();
  }

  if (p_y_ != nullptr)
  {
    cudaFree(p_y_);
    cudaCheck();
  }

  if (p_z_ != nullptr)
  {
    cudaFree(p_z_);
    cudaCheck();
  }

  if (d_grid_occ_ != nullptr)
  {
    cudaFree(d_grid_occ_);
    cudaCheck();
  }

  if (d_data_ != nullptr)
  {
    cudaFree(d_data_);
    cudaCheck();
  }

  if (d_map_ != nullptr)
  {
    cudaFree(d_map_);
    cudaCheck();
  }

  if (d_particles_ != nullptr)
  {
    cudaFree(d_particles_);
    cudaCheck();
  }

  if (d_particles_ordered_ != nullptr)
  {
    cudaFree(d_particles_ordered_);
    cudaCheck();
  }

  if (d_points_ != nullptr)
  {
    cudaFree(d_points_);
    cudaCheck();
  }

  if (d_points_ordered_ != nullptr)
  {
    cudaFree(d_points_ordered_);
    cudaCheck();
  }

  if (d_transform_ != nullptr)
  {
    cudaFree(d_transform_);
    cudaCheck();
  }

  if (d_new_weights_ != nullptr)
  {
    cudaFree(d_new_weights_);
    cudaCheck();
  }

  if (g_grid_occ_ != nullptr)
  {
    cudaFree(g_grid_occ_);
    cudaCheck();
  }

  if (g_data_ != nullptr)
  {
    cudaFree(g_data_);
    cudaCheck();
  }

  if (g_map_coef_ != nullptr)
  {
    cudaFree(g_map_coef_);
    cudaCheck();
  }

  if (d_tex_grid_ != nullptr)
  {
    cudaFree(d_tex_grid_);
    cudaCheck();
  }

  if (d_point_weights_ != nullptr)
  {
    cudaFree(d_point_weights_);
    cudaCheck();
  }

  if (sin_a_ != nullptr)
  {
    cudaFree(sin_a_);
    cudaCheck();
  }

  if (cos_a_ != nullptr)
  {
    cudaFree(cos_a_);
    cudaCheck();
  }

  if (sin_b_ != nullptr)
  {
    cudaFree(sin_b_);
    cudaCheck();
  }

  if (cos_b_ != nullptr)
  {
    cudaFree(cos_b_);
    cudaCheck();
  }

  if (sin_c_ != nullptr)
  {
    cudaFree(sin_c_);
    cudaCheck();
  }

  if (cos_c_ != nullptr)
  {
    cudaFree(cos_c_);
    cudaCheck();
  }
}

} // namespace tsdf_localization
