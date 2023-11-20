#ifndef CUDA_EVAL_PARTICLES_H
#define CUDA_EVAL_PARTICLES_H

#include <tsdf_localization/cuda/cuda_data.h>

#include <curand_kernel.h>
#include <curand.h>

namespace tsdf_localization
{

__device__ inline unsigned int getIndex(const OCC_T* __restrict__ grid_occ, const FLOAT_T* __restrict__ data, const CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef* __restrict__ coef, FLOAT_T x, FLOAT_T y, FLOAT_T z)
{
  unsigned int grid_x = x - const_map_coef_.min_x_;
  unsigned int grid_y = y - const_map_coef_.min_y_;
  unsigned int grid_z = z - const_map_coef_.min_z_;
  
  grid_x /= const_map_coef_.resolution_;
  grid_y /= const_map_coef_.resolution_;
  grid_z /= const_map_coef_.resolution_;
  
  if (grid_x >= const_map_coef_.dim_x_ || grid_y >= const_map_coef_.dim_y_ || grid_z >= const_map_coef_.dim_z_)
  {
      return const_map_coef_.data_size_;
  }

  auto x_offset = x - const_map_coef_.min_x_;
  auto y_offset = y - const_map_coef_.min_y_;
  auto z_offset = z - const_map_coef_.min_z_;

  unsigned int up_x, up_y, up_z;


  up_x = (x_offset / sub_voxel_size);
  up_y = (y_offset / sub_voxel_size);
  up_z = (z_offset / sub_voxel_size);
  
  FLOAT_T sub_pos_x, sub_pos_y, sub_pos_z;

  sub_pos_x = x_offset - up_x * sub_voxel_size;
  sub_pos_y = y_offset - up_y * sub_voxel_size;
  sub_pos_z = z_offset - up_z * sub_voxel_size;
  
  unsigned int up_index = up_x + up_y * const_map_coef_.up_dim_x_ + up_z * const_map_coef_.up_dim_2_;

  if (up_index >= const_map_coef_.grid_occ_size_)
  {
      return const_map_coef_.data_size_;   
  }

  //auto sub_index = up_index; //grid_occ[up_index];
  auto sub_index = grid_occ[up_index];

  if (sub_index < 0)
  {
      return const_map_coef_.data_size_;
      //throw std::invalid_argument("Upper voxel entry does not exist!");
  }

  unsigned int sub_x, sub_y, sub_z;

  sub_x = sub_pos_x / const_map_coef_.resolution_;
  sub_y = sub_pos_y / const_map_coef_.resolution_;
  sub_z = sub_pos_z / const_map_coef_.resolution_;
  
  return sub_index + sub_x + sub_y * const_map_coef_.sub_dim_ + sub_z * const_map_coef_.sub_dim_2_ ;
}

__device__ inline FLOAT_T getEntry(const OCC_T* __restrict__ grid_occ, const FLOAT_T* __restrict__ data, const CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef* __restrict__ coef, FLOAT_T x, FLOAT_T y, FLOAT_T z)
{
  auto index = getIndex(grid_occ, data, coef, x, y, z);

  if (index < const_map_coef_.data_size_)
  {
      //return index; //__ldg(&data[index]);
      return data[index];
  }
  else
  {
      return const_map_coef_.init_value_;
  }
}

__device__ inline unsigned int getIndex(const OCC_T* __restrict__ grid_occ, const FLOAT_T* __restrict__ data, const CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef* __restrict__ coef, FLOAT_T x, FLOAT_T y, FLOAT_T z, OCC_T& last_grid, OCC_T& last_offset)
{
  unsigned int grid_x = x - const_map_coef_.min_x_;
  unsigned int grid_y = y - const_map_coef_.min_y_;
  unsigned int grid_z = z - const_map_coef_.min_z_;
  
  grid_x /= const_map_coef_.resolution_;
  grid_y /= const_map_coef_.resolution_;
  grid_z /= const_map_coef_.resolution_;
  
  if (grid_x >= const_map_coef_.dim_x_ || grid_y >= const_map_coef_.dim_y_ || grid_z >= const_map_coef_.dim_z_)
  {
      return const_map_coef_.data_size_;
  }

  auto x_offset = x - const_map_coef_.min_x_;
  auto y_offset = y - const_map_coef_.min_y_;
  auto z_offset = z - const_map_coef_.min_z_;

  unsigned int up_x, up_y, up_z;


  up_x = (x_offset / sub_voxel_size);
  up_y = (y_offset / sub_voxel_size);
  up_z = (z_offset / sub_voxel_size);
  
  FLOAT_T sub_pos_x, sub_pos_y, sub_pos_z;

  sub_pos_x = x_offset - up_x * sub_voxel_size;
  sub_pos_y = y_offset - up_y * sub_voxel_size;
  sub_pos_z = z_offset - up_z * sub_voxel_size;
  
  unsigned int up_index = up_x + up_y * const_map_coef_.up_dim_x_ + up_z * const_map_coef_.up_dim_2_;

  if (up_index >= const_map_coef_.grid_occ_size_)
  {
      return const_map_coef_.data_size_;   
  }

  OCC_T sub_index;

  if (last_grid == up_index)
  {
    sub_index = last_offset;
  }
  else
  {
    sub_index = grid_occ[up_index];
    last_grid = up_index;
    last_offset = sub_index;
  }

  if (sub_index < 0)
  {
      return const_map_coef_.data_size_;
      //throw std::invalid_argument("Upper voxel entry does not exist!");
  }

  unsigned int sub_x, sub_y, sub_z;

  sub_x = sub_pos_x / const_map_coef_.resolution_;
  sub_y = sub_pos_y / const_map_coef_.resolution_;
  sub_z = sub_pos_z / const_map_coef_.resolution_;
  
  return sub_index + sub_x + sub_y * const_map_coef_.sub_dim_ + sub_z * const_map_coef_.sub_dim_2_ ;
}

__device__ inline FLOAT_T getEntry(const OCC_T* __restrict__ grid_occ, const FLOAT_T* __restrict__ data, const CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef* __restrict__ coef, FLOAT_T x, FLOAT_T y, FLOAT_T z, OCC_T& last_grid, OCC_T& last_offset)
{
  auto index = getIndex(grid_occ, data, coef, x, y, z, last_grid, last_offset);

  if (index < const_map_coef_.data_size_)
  {
      //return index; //__ldg(&data[index]);
      return data[index];
  }
  else
  {
      return const_map_coef_.init_value_;
  }
}

//__device__ FLOAT_T cudaEvaluatePose(FLOAT_T* transform, CudaPoint* points, unsigned int num_points, CudaSubVoxelMap<FLOAT_T, FLOAT_T, 1>* map)
__device__ inline FLOAT_T cudaEvaluatePose(const OCC_T* __restrict__ grid_occ, const FLOAT_T* __restrict__ data, const CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef* __restrict__ coef, const FLOAT_T* __restrict__ transform, const CudaPoint* __restrict__ points, unsigned int num_points)
{
  FLOAT_T eval_sum = 0.0f;

  //auto tid = blockIdx.x * blockDim.x + threadIdx.x;

  OCC_T last_grid = -1;
  OCC_T last_offset = -1;

  for (auto point_index = 0u; point_index < num_points; ++point_index)
  { 
    auto x = points[point_index].x;
    auto y = points[point_index].y;
    auto z = points[point_index].z;

    auto t_x = transform[0] * x + transform[1] * y + transform[2] * z + transform[3];
    auto t_y = transform[4] * x + transform[5] * y + transform[6] * z + transform[7];
    auto t_z = transform[8] * x + transform[9] * y + transform[10] * z + transform[11];

    //FLOAT_T value = map->getEntry(t_x, t_y, t_z);
    
    FLOAT_T value = getEntry(grid_occ, data, coef, t_x, t_y, t_z, last_grid, last_offset);

    //FLOAT_T value = t_x + t_y + t_z;

    //FLOAT_T eval = 0.5f * expf(-value * value / 0.08f);

    //eval_sum += eval * eval;

    //FLOAT_T eval = expf(-(value * value) / SIGMAR_QUAD / 2) / (sqrtf(2 * SIGMAR_QUAD * M_PI));

    //eval_sum += (eval * eval * eval);

    auto square_dist = x*x + y*y + z*z;

    if (square_dist < const_max_range_squared_)
    {
      value = const_a_hit_ * value + const_a_range_ * INV_MAX_RANGE;
    }
    else
    {
      value = const_a_hit_ * value + const_a_max_;
    }

    eval_sum += value;
  }

  return eval_sum;
}

//__global__  void cudaEvaluateParticles(CudaSubVoxelMap<FLOAT_T, FLOAT_T, 1>* map, CudaPoint* points, size_t points_size, Particle* particles, size_t particles_size, FLOAT_T* tf_matrix, FLOAT_T* new_weights)
__global__  void cudaEvaluateParticles(const OCC_T* __restrict__ grid_occ, const FLOAT_T* __restrict__ data, const CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef* __restrict__ coef, const CudaPoint* __restrict__ points, size_t points_size, const Particle* __restrict__ particles, size_t particles_size, const FLOAT_T* __restrict__ tf_matrix, FLOAT_T* new_weights)
{
  auto tid = blockIdx.x * blockDim.x + threadIdx.x;


  FLOAT_T tf_particle[16];

  FLOAT_T alpha = particles[tid].first[3];
  FLOAT_T beta =  particles[tid].first[4];
  FLOAT_T gamma = particles[tid].first[5];

  FLOAT_T sin_alpha = sinf(alpha);
  FLOAT_T cos_alpha = cosf(alpha);  
  FLOAT_T sin_beta  = sinf(beta);
  FLOAT_T cos_beta  = cosf(beta);
  FLOAT_T sin_gamma = sinf(gamma);
  FLOAT_T cos_gamma = cosf(gamma);

  tf_particle[0] = cos_beta * cos_gamma;
  tf_particle[4] = cos_beta * sin_gamma;
  tf_particle[8] = -sin_beta;
  tf_particle[3] = particles[tid].first[0];
  
  tf_particle[1] = sin_alpha * sin_beta * cos_gamma - cos_alpha * sin_gamma;
  tf_particle[5] = sin_alpha * sin_beta * sin_gamma + cos_alpha * cos_gamma;
  tf_particle[9] = sin_alpha * cos_beta;
  tf_particle[7] = particles[tid].first[1];
  
  tf_particle[2] = cos_alpha * sin_beta * cos_gamma + sin_alpha * sin_gamma;
  tf_particle[6] = cos_alpha * sin_beta * sin_gamma - sin_alpha * cos_gamma;
  tf_particle[10] = cos_alpha * cos_beta;
  tf_particle[11] = particles[tid].first[2];

  FLOAT_T tf_laser[16];

  tf_laser[0] = tf_particle[0] * const_tf_matrix[0] + tf_particle[1] * const_tf_matrix[4] + tf_particle[2] * const_tf_matrix[8];
  tf_laser[1] = tf_particle[0] * const_tf_matrix[1] + tf_particle[1] * const_tf_matrix[5] + tf_particle[2] * const_tf_matrix[9];
  tf_laser[2] = tf_particle[0] * const_tf_matrix[2] + tf_particle[1] * const_tf_matrix[6] + tf_particle[2] * const_tf_matrix[10];
  tf_laser[3] = tf_particle[0] * const_tf_matrix[3] + tf_particle[1] * const_tf_matrix[7] + tf_particle[2] * const_tf_matrix[11] + tf_particle[3];

  tf_laser[4] = tf_particle[4] * const_tf_matrix[0] + tf_particle[5] * const_tf_matrix[4] + tf_particle[6] * const_tf_matrix[8];
  tf_laser[5] = tf_particle[4] * const_tf_matrix[1] + tf_particle[5] * const_tf_matrix[5] + tf_particle[6] * const_tf_matrix[9];
  tf_laser[6] = tf_particle[4] * const_tf_matrix[2] + tf_particle[5] * const_tf_matrix[6] + tf_particle[6] * const_tf_matrix[10];
  tf_laser[7] = tf_particle[4] * const_tf_matrix[3] + tf_particle[5] * const_tf_matrix[7] + tf_particle[6] * const_tf_matrix[11] + tf_particle[7];

  tf_laser[8] = tf_particle[8] * const_tf_matrix[0] + tf_particle[9] * const_tf_matrix[4] + tf_particle[10] * const_tf_matrix[8];
  tf_laser[9] = tf_particle[8] * const_tf_matrix[1] + tf_particle[9] * const_tf_matrix[5] + tf_particle[10] * const_tf_matrix[9];
  tf_laser[10] = tf_particle[8] * const_tf_matrix[2] + tf_particle[9] * const_tf_matrix[6] + tf_particle[10] * const_tf_matrix[10];
  tf_laser[11] = tf_particle[8] * const_tf_matrix[3] + tf_particle[9] * const_tf_matrix[7] + tf_particle[10] * const_tf_matrix[11] + tf_particle[11];

  tf_laser[15] = 1;

  new_weights[tid] = cudaEvaluatePose(grid_occ, data, coef, tf_laser, points, points_size);
}

__global__  void cudaEvaluateParticlesOrdered(const OCC_T* __restrict__ grid_occ, const FLOAT_T* __restrict__ data, const CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef* __restrict__ coef, const CudaPoint* __restrict__ points, size_t points_size, const FLOAT_T* __restrict__ particles, size_t particles_size, const FLOAT_T* __restrict__ tf_matrix, FLOAT_T* new_weights)
{
  auto tidx = blockIdx.x * blockDim.x + threadIdx.x;

  FLOAT_T tf_particle[16];

  // FLOAT_T alpha = particles[tid].first[3];
  // FLOAT_T beta =  particles[tid].first[4];
  // FLOAT_T gamma = particles[tid].first[5];

  FLOAT_T x     = particles[                     tidx];
  FLOAT_T y     = particles[    particles_size + tidx];
  FLOAT_T z     = particles[2 * particles_size + tidx];
  FLOAT_T alpha = particles[3 * particles_size + tidx];
  FLOAT_T beta  = particles[4 * particles_size + tidx];
  FLOAT_T gamma = particles[5 * particles_size + tidx];

  FLOAT_T sin_alpha = sinf(alpha);
  FLOAT_T cos_alpha = cosf(alpha);  
  FLOAT_T sin_beta  = sinf(beta);
  FLOAT_T cos_beta  = cosf(beta);
  FLOAT_T sin_gamma = sinf(gamma);
  FLOAT_T cos_gamma = cosf(gamma);

  tf_particle[0] = cos_beta * cos_gamma;
  tf_particle[4] = cos_beta * sin_gamma;
  tf_particle[8] = -sin_beta;
  tf_particle[3] = x;
  
  tf_particle[1] = sin_alpha * sin_beta * cos_gamma - cos_alpha * sin_gamma;
  tf_particle[5] = sin_alpha * sin_beta * sin_gamma + cos_alpha * cos_gamma;
  tf_particle[9] = sin_alpha * cos_beta;
  tf_particle[7] = y;
  
  tf_particle[2] = cos_alpha * sin_beta * cos_gamma + sin_alpha * sin_gamma;
  tf_particle[6] = cos_alpha * sin_beta * sin_gamma - sin_alpha * cos_gamma;
  tf_particle[10] = cos_alpha * cos_beta;
  tf_particle[11] = z;


  FLOAT_T tf_laser[16];

  tf_laser[0] = tf_particle[0] * const_tf_matrix[0] + tf_particle[1] * const_tf_matrix[4] + tf_particle[2] * const_tf_matrix[8];
  tf_laser[1] = tf_particle[0] * const_tf_matrix[1] + tf_particle[1] * const_tf_matrix[5] + tf_particle[2] * const_tf_matrix[9];
  tf_laser[2] = tf_particle[0] * const_tf_matrix[2] + tf_particle[1] * const_tf_matrix[6] + tf_particle[2] * const_tf_matrix[10];
  tf_laser[3] = tf_particle[0] * const_tf_matrix[3] + tf_particle[1] * const_tf_matrix[7] + tf_particle[2] * const_tf_matrix[11] + tf_particle[3];

  tf_laser[4] = tf_particle[4] * const_tf_matrix[0] + tf_particle[5] * const_tf_matrix[4] + tf_particle[6] * const_tf_matrix[8];
  tf_laser[5] = tf_particle[4] * const_tf_matrix[1] + tf_particle[5] * const_tf_matrix[5] + tf_particle[6] * const_tf_matrix[9];
  tf_laser[6] = tf_particle[4] * const_tf_matrix[2] + tf_particle[5] * const_tf_matrix[6] + tf_particle[6] * const_tf_matrix[10];
  tf_laser[7] = tf_particle[4] * const_tf_matrix[3] + tf_particle[5] * const_tf_matrix[7] + tf_particle[6] * const_tf_matrix[11] + tf_particle[7];

  tf_laser[8] = tf_particle[8] * const_tf_matrix[0] + tf_particle[9] * const_tf_matrix[4] + tf_particle[10] * const_tf_matrix[8];
  tf_laser[9] = tf_particle[8] * const_tf_matrix[1] + tf_particle[9] * const_tf_matrix[5] + tf_particle[10] * const_tf_matrix[9];
  tf_laser[10] = tf_particle[8] * const_tf_matrix[2] + tf_particle[9] * const_tf_matrix[6] + tf_particle[10] * const_tf_matrix[10];
  tf_laser[11] = tf_particle[8] * const_tf_matrix[3] + tf_particle[9] * const_tf_matrix[7] + tf_particle[10] * const_tf_matrix[11] + tf_particle[11];

  tf_laser[15] = 1;

  new_weights[tidx] = cudaEvaluatePose(grid_occ, data, coef, tf_laser, points, points_size);
}

__global__  void cudaEvaluatePoints(const OCC_T* __restrict__ grid_occ, const FLOAT_T* __restrict__ data, const CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef* __restrict__ coef, const CudaPoint* __restrict__ points, size_t points_size, const FLOAT_T* __restrict__ particles, size_t particles_size, const FLOAT_T* __restrict__ tf_matrix, FLOAT_T* new_weights)
{
  auto tidx = blockIdx.x * blockDim.x + threadIdx.x;

  auto particle_index = tidx / points_size;
  auto point_index = tidx % points_size;

  FLOAT_T tf_particle[16];

  // FLOAT_T alpha = particles[tid].first[3];
  // FLOAT_T beta =  particles[tid].first[4];
  // FLOAT_T gamma = particles[tid].first[5];

  FLOAT_T x     = particles[                     particle_index];
  FLOAT_T y     = particles[    particles_size + particle_index];
  FLOAT_T z     = particles[2 * particles_size + particle_index];
  FLOAT_T alpha = particles[3 * particles_size + particle_index];
  FLOAT_T beta  = particles[4 * particles_size + particle_index];
  FLOAT_T gamma = particles[5 * particles_size + particle_index];

  auto p_x = points[point_index].x;
  auto p_y = points[point_index].y;
  auto p_z = points[point_index].z;

  FLOAT_T sin_alpha = sinf(alpha);
  FLOAT_T cos_alpha = cosf(alpha);  
  FLOAT_T sin_beta  = sinf(beta);
  FLOAT_T cos_beta  = cosf(beta);
  FLOAT_T sin_gamma = sinf(gamma);
  FLOAT_T cos_gamma = cosf(gamma);

  tf_particle[0] = cos_beta * cos_gamma;
  tf_particle[4] = cos_beta * sin_gamma;
  tf_particle[8] = -sin_beta;
  tf_particle[3] = x;
  
  tf_particle[1] = sin_alpha * sin_beta * cos_gamma - cos_alpha * sin_gamma;
  tf_particle[5] = sin_alpha * sin_beta * sin_gamma + cos_alpha * cos_gamma;
  tf_particle[9] = sin_alpha * cos_beta;
  tf_particle[7] = y;
  
  tf_particle[2] = cos_alpha * sin_beta * cos_gamma + sin_alpha * sin_gamma;
  tf_particle[6] = cos_alpha * sin_beta * sin_gamma - sin_alpha * cos_gamma;
  tf_particle[10] = cos_alpha * cos_beta;
  tf_particle[11] = z;

  FLOAT_T tf_laser[16];

  tf_laser[0] = tf_particle[0] * const_tf_matrix[0] + tf_particle[1] * const_tf_matrix[4] + tf_particle[2] * const_tf_matrix[8];
  tf_laser[1] = tf_particle[0] * const_tf_matrix[1] + tf_particle[1] * const_tf_matrix[5] + tf_particle[2] * const_tf_matrix[9];
  tf_laser[2] = tf_particle[0] * const_tf_matrix[2] + tf_particle[1] * const_tf_matrix[6] + tf_particle[2] * const_tf_matrix[10];
  tf_laser[3] = tf_particle[0] * const_tf_matrix[3] + tf_particle[1] * const_tf_matrix[7] + tf_particle[2] * const_tf_matrix[11] + tf_particle[3];

  tf_laser[4] = tf_particle[4] * const_tf_matrix[0] + tf_particle[5] * const_tf_matrix[4] + tf_particle[6] * const_tf_matrix[8];
  tf_laser[5] = tf_particle[4] * const_tf_matrix[1] + tf_particle[5] * const_tf_matrix[5] + tf_particle[6] * const_tf_matrix[9];
  tf_laser[6] = tf_particle[4] * const_tf_matrix[2] + tf_particle[5] * const_tf_matrix[6] + tf_particle[6] * const_tf_matrix[10];
  tf_laser[7] = tf_particle[4] * const_tf_matrix[3] + tf_particle[5] * const_tf_matrix[7] + tf_particle[6] * const_tf_matrix[11] + tf_particle[7];

  tf_laser[8] = tf_particle[8] * const_tf_matrix[0] + tf_particle[9] * const_tf_matrix[4] + tf_particle[10] * const_tf_matrix[8];
  tf_laser[9] = tf_particle[8] * const_tf_matrix[1] + tf_particle[9] * const_tf_matrix[5] + tf_particle[10] * const_tf_matrix[9];
  tf_laser[10] = tf_particle[8] * const_tf_matrix[2] + tf_particle[9] * const_tf_matrix[6] + tf_particle[10] * const_tf_matrix[10];
  tf_laser[11] = tf_particle[8] * const_tf_matrix[3] + tf_particle[9] * const_tf_matrix[7] + tf_particle[10] * const_tf_matrix[11] + tf_particle[11];

  tf_laser[15] = 1;

  //FLOAT_T eval_sum = 1.0f;

  //for (auto point_index = 0u; point_index < points_size; ++point_index)
  //{ 

    auto t_x = tf_laser[0] * p_x + tf_laser[1] * p_y + tf_laser[2] * p_z + tf_laser[3];
    auto t_y = tf_laser[4] * p_x + tf_laser[5] * p_y + tf_laser[6] * p_z + tf_laser[7];
    auto t_z = tf_laser[8] * p_x + tf_laser[9] * p_y + tf_laser[10] * p_z + tf_laser[11];

    //FLOAT_T value = map->getEntry(t_x, t_y, t_z);
    
    FLOAT_T value = getEntry(grid_occ, data, coef, t_x, t_y, t_z);

    FLOAT_T eval = value;
  //}

  //new_weights[tidx] = eval * eval;
  atomicAdd(&(new_weights[particle_index]), eval);
}

__global__  void cudaEvaluatePointsOrdered(const OCC_T* __restrict__ grid_occ, const FLOAT_T* __restrict__ data, const CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef* __restrict__ coef, const FLOAT_T* __restrict__ points, size_t points_size, const FLOAT_T* __restrict__ particles, size_t particles_size, const FLOAT_T* __restrict__ tf_matrix, FLOAT_T* new_weights)
{
  auto tidx = blockIdx.x * blockDim.x + threadIdx.x;

  auto particle_index = tidx / points_size;
  auto point_index = tidx % points_size;

  FLOAT_T tf_particle[16];

  // FLOAT_T alpha = particles[tid].first[3];
  // FLOAT_T beta =  particles[tid].first[4];
  // FLOAT_T gamma = particles[tid].first[5];

  FLOAT_T x     = particles[                     particle_index];
  FLOAT_T y     = particles[    particles_size + particle_index];
  FLOAT_T z     = particles[2 * particles_size + particle_index];
  FLOAT_T alpha = particles[3 * particles_size + particle_index];
  FLOAT_T beta  = particles[4 * particles_size + particle_index];
  FLOAT_T gamma = particles[5 * particles_size + particle_index];

  auto p_x = points[                  point_index];
  auto p_y = points[    points_size + point_index];
  auto p_z = points[2 * points_size + point_index];

  FLOAT_T sin_alpha = sinf(alpha);
  FLOAT_T cos_alpha = cosf(alpha);  
  FLOAT_T sin_beta  = sinf(beta);
  FLOAT_T cos_beta  = cosf(beta);
  FLOAT_T sin_gamma = sinf(gamma);
  FLOAT_T cos_gamma = cosf(gamma);

  tf_particle[0] = cos_beta * cos_gamma;
  tf_particle[4] = cos_beta * sin_gamma;
  tf_particle[8] = -sin_beta;
  tf_particle[3] = x;
  
  tf_particle[1] = sin_alpha * sin_beta * cos_gamma - cos_alpha * sin_gamma;
  tf_particle[5] = sin_alpha * sin_beta * sin_gamma + cos_alpha * cos_gamma;
  tf_particle[9] = sin_alpha * cos_beta;
  tf_particle[7] = y;
  
  tf_particle[2] = cos_alpha * sin_beta * cos_gamma + sin_alpha * sin_gamma;
  tf_particle[6] = cos_alpha * sin_beta * sin_gamma - sin_alpha * cos_gamma;
  tf_particle[10] = cos_alpha * cos_beta;
  tf_particle[11] = z;

  FLOAT_T tf_laser[16];

  tf_laser[0] = tf_particle[0] * const_tf_matrix[0] + tf_particle[1] * const_tf_matrix[4] + tf_particle[2] * const_tf_matrix[8];
  tf_laser[1] = tf_particle[0] * const_tf_matrix[1] + tf_particle[1] * const_tf_matrix[5] + tf_particle[2] * const_tf_matrix[9];
  tf_laser[2] = tf_particle[0] * const_tf_matrix[2] + tf_particle[1] * const_tf_matrix[6] + tf_particle[2] * const_tf_matrix[10];
  tf_laser[3] = tf_particle[0] * const_tf_matrix[3] + tf_particle[1] * const_tf_matrix[7] + tf_particle[2] * const_tf_matrix[11] + tf_particle[3];

  tf_laser[4] = tf_particle[4] * const_tf_matrix[0] + tf_particle[5] * const_tf_matrix[4] + tf_particle[6] * const_tf_matrix[8];
  tf_laser[5] = tf_particle[4] * const_tf_matrix[1] + tf_particle[5] * const_tf_matrix[5] + tf_particle[6] * const_tf_matrix[9];
  tf_laser[6] = tf_particle[4] * const_tf_matrix[2] + tf_particle[5] * const_tf_matrix[6] + tf_particle[6] * const_tf_matrix[10];
  tf_laser[7] = tf_particle[4] * const_tf_matrix[3] + tf_particle[5] * const_tf_matrix[7] + tf_particle[6] * const_tf_matrix[11] + tf_particle[7];

  tf_laser[8] = tf_particle[8] * const_tf_matrix[0] + tf_particle[9] * const_tf_matrix[4] + tf_particle[10] * const_tf_matrix[8];
  tf_laser[9] = tf_particle[8] * const_tf_matrix[1] + tf_particle[9] * const_tf_matrix[5] + tf_particle[10] * const_tf_matrix[9];
  tf_laser[10] = tf_particle[8] * const_tf_matrix[2] + tf_particle[9] * const_tf_matrix[6] + tf_particle[10] * const_tf_matrix[10];
  tf_laser[11] = tf_particle[8] * const_tf_matrix[3] + tf_particle[9] * const_tf_matrix[7] + tf_particle[10] * const_tf_matrix[11] + tf_particle[11];

  tf_laser[15] = 1;

  //FLOAT_T eval_sum = 1.0f;

  //for (auto point_index = 0u; point_index < points_size; ++point_index)
  //{ 
  

    auto t_x = tf_laser[0] * p_x + tf_laser[1] * p_y + tf_laser[2] * p_z + tf_laser[3];
    auto t_y = tf_laser[4] * p_x + tf_laser[5] * p_y + tf_laser[6] * p_z + tf_laser[7];
    auto t_z = tf_laser[8] * p_x + tf_laser[9] * p_y + tf_laser[10] * p_z + tf_laser[11];

    //FLOAT_T value = map->getEntry(t_x, t_y, t_z);
    
    FLOAT_T value = getEntry(grid_occ, data, coef, t_x, t_y, t_z);

    auto square_dist = x*x + y*y + z*z;

    if (square_dist < const_max_range_squared_)
    {
      value = const_a_hit_ * value + const_a_range_ * INV_MAX_RANGE;
    }
    else
    {
      value = const_a_hit_ * value + const_a_max_;
    }

    FLOAT_T eval = value;
  //}

  //new_weights[tidx] = eval * eval;
  //new_weights[particle_index] += eval * eval;


  //atomicAdd(&(new_weights[particle_index]), eval);
  new_weights[tidx] = eval;
}

__global__ void weight_particles(const FLOAT_T* particles, size_t particles_size, FLOAT_T* weights, FLOAT_T weight_sum, FLOAT_T* p_x, FLOAT_T* p_y, FLOAT_T* p_z, FLOAT_T* p_a_sin, FLOAT_T* p_a_cos, 
                                                                                                                        FLOAT_T* p_b_sin, FLOAT_T* p_b_cos, 
                                                                                                                        FLOAT_T* p_c_sin, FLOAT_T* p_c_cos)
{
  auto tidx = blockIdx.x * blockDim.x + threadIdx.x;
  
  FLOAT_T x = particles[                     tidx];
  FLOAT_T y = particles[    particles_size + tidx];
  FLOAT_T z = particles[2 * particles_size + tidx];
  FLOAT_T a = particles[3 * particles_size + tidx];
  FLOAT_T b = particles[4 * particles_size + tidx];
  FLOAT_T c = particles[5 * particles_size + tidx];

  FLOAT_T weight = weights[tidx] / weight_sum;

  FLOAT_T sin_a = sinf(a);
  FLOAT_T cos_a = cosf(a);  
  FLOAT_T sin_b  = sinf(b);
  FLOAT_T cos_b  = cosf(b);
  FLOAT_T sin_c = sinf(c);
  FLOAT_T cos_c = cosf(c);

  p_x[tidx] = weight * x;
  p_y[tidx] = weight * y;
  p_z[tidx] = weight * z;

  p_a_sin[tidx] = weight * sin_a;
  p_a_cos[tidx] = weight * cos_a;
  
  p_b_sin[tidx] = weight * sin_b;
  p_b_cos[tidx] = weight * cos_b;

  p_c_sin[tidx] = weight * sin_c;
  p_c_cos[tidx] = weight * cos_c;

  weights[tidx] = weight;
}

} // namespace tsdf_localization

#endif