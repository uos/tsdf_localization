#ifndef CUDA_EVALUATOR_H
#define CUDA_EVALUATOR_H

#include <tsdf_localization/map/device_map.h>
#include <tsdf_localization/cuda/cuda_sub_voxel_map.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <memory>
#include <tuple>
#include <vector>
#include <tsdf_localization/util/util.h>
#include <stdexcept>

#include <tsdf_localization/particle.h>

#include <unordered_set>

namespace tsdf_localization
{

struct CudaPoint
{
  FLOAT_T x;
  FLOAT_T y;
  FLOAT_T z;

  CudaPoint(FLOAT_T x = 0, FLOAT_T y = 0, FLOAT_T z = 0) : x(x), y(y), z(z) {}

  bool operator==(const CudaPoint& other) const
  {
    return (this->x == other.x && this->y == other.y && this->z == other.z);
  }

  bool operator!=(const CudaPoint& other) const
  {
    return !(*this == other);
  }
};

struct SortClass
{
  SortClass()
  {

  }

  SortClass(int ring, int index, const CudaPoint& id_point, const std::shared_ptr<CudaPoint>& original = nullptr) : ring_(ring), index_(index), point_(id_point), original_(original)
  {

  }

  bool operator==(const SortClass& other) const
  {
    return ring_ == other.ring_ && point_ == other.point_;
  }

  int ring_;
  int index_;
  CudaPoint point_;
  std::shared_ptr<CudaPoint> original_;
};

struct hash
{
  std::size_t operator()(const CudaPoint& p) const noexcept
  {
      long long v = ((long long)(p.x * 1000) << 32) ^ ((long long)(p.y * 1000) << 16) ^ (long long)(p.z * 1000);
      return std::hash<long long>()(v);
  }

  std::size_t operator()(const std::pair<int, CudaPoint>& p) const noexcept
  {
      long long v = ((long long)(p.second.x * 1000) << 32) ^ ((long long)(p.second.y * 1000) << 16) ^ (long long)(p.second.z * 1000);
      return std::hash<long long>()(v);
  }

  std::size_t operator()(const std::pair<int, std::pair<int, CudaPoint>>& p) const noexcept
  {
      long long v = ((long long)(p.second.second.x * 1000) << 32) ^ ((long long)(p.second.second.y * 1000) << 16) ^ (long long)(p.second.second.z * 1000);
      return std::hash<long long>()(v);
  }

  std::size_t operator()(const SortClass& p) const noexcept
  {
      long long v = ((long long)(p.point_.x * 1000) << 32) ^ ((long long)(p.point_.y * 1000) << 16) ^ (long long)(p.point_.z * 1000);
      return std::hash<long long>()(v);
  }

};

constexpr size_t TRANSFORM_BYTES = sizeof(FLOAT_T) * 16;

class Evaluator
{
public:
  virtual geometry_msgs::msg::PoseWithCovariance evaluate(std::vector<Particle>& particles, const sensor_msgs::msg::PointCloud2& real_cloud, FLOAT_T tf_matrix[16])
  {
    throw std::runtime_error("CUDA acceleration is not supported. Please install CUDA!");
  }

  virtual geometry_msgs::msg::PoseWithCovariance evaluate(std::vector<Particle>& particles, const std::vector<CudaPoint>& points, FLOAT_T tf_matrix[16])
  {
    throw std::runtime_error("CUDA acceleration is not supported. Please install CUDA!");
  }

};

class CudaEvaluator : public Evaluator
{
public:
    CudaEvaluator(CudaSubVoxelMap<FLOAT_T, FLOAT_T>& map, bool per_point = false, FLOAT_T a_hit = 0.9, FLOAT_T a_range = 0.1, FLOAT_T a_max = 0.0, FLOAT_T max_range = 100.0);

    CudaEvaluator(const CudaEvaluator&) = delete;
    CudaEvaluator& operator=(const CudaEvaluator&) = delete;

    virtual geometry_msgs::msg::PoseWithCovariance evaluate(std::vector<Particle>& particles, const sensor_msgs::msg::PointCloud2& real_cloud, FLOAT_T tf_matrix[16]);

    virtual geometry_msgs::msg::PoseWithCovariance evaluate(std::vector<Particle>& particles, const std::vector<CudaPoint>& points, FLOAT_T tf_matrix[16]);

    virtual ~CudaEvaluator();

private:
    CudaSubVoxelMap<FLOAT_T, FLOAT_T>* d_map_;
    bool per_point_;
    
    OCC_T* d_grid_occ_;

    FLOAT_T* d_data_;

    Particle* d_particles_;
    FLOAT_T* d_particles_ordered_;
    size_t particles_reserved_;

    CudaPoint* d_points_;
    FLOAT_T* d_points_ordered_;
    size_t points_reserved_;

    FLOAT_T* d_transform_;

    std::vector<FLOAT_T> new_weights_;
    FLOAT_T* d_new_weights_;

    FLOAT_T* d_point_weights_;
    size_t point_weights_size_;

    FLOAT_T* p_x_;
    FLOAT_T* p_y_;
    FLOAT_T* p_z_;

    FLOAT_T* sin_a_;
    FLOAT_T* cos_a_;

    FLOAT_T* sin_b_;
    FLOAT_T* cos_b_;

    FLOAT_T* sin_c_;
    FLOAT_T* cos_c_;

    FLOAT_T a_hit_;
    FLOAT_T a_range_;
    FLOAT_T a_max_;

    FLOAT_T max_range_;
    FLOAT_T inv_max_range_;
    FLOAT_T max_range_squared_;

    //curandState* dev_random_;
};


} // namespace tsdf_localization

#endif