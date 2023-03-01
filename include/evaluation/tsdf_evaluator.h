/**
 * @file tsdf_evaluator.h
 * @author Marc Eiosldt (meisoldt@uni-osnabrueck.de)
 * 
 * @brief Class of a particle evaluator that works with a TSDF map and can be accelerated by using CUDA
 * 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 */

#ifndef TSDF_EVALUATOR_H
#define TSDF_EVALUATOR_H

#include <map/device_map.h>
#include <cuda/cuda_sub_voxel_map.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <evaluation/model/evaluation_model.h>
#include <particle_cloud.h>

#include <memory>

#include <cuda/cuda_evaluator.h>

#include <util/util.h>

namespace mcl
{

// constexpr double map_res = 0.256; // 0.256;
// constexpr double map_res_half = map_res / 2;

/**
 * @brief Implementation of the sensor update with a given TSDF map 
 */
class TSDFEvaluator
{
    // Pointer to the TSDF map
    std::shared_ptr<CudaSubVoxelMap<FLOAT_T, FLOAT_T>> map_ptr_;
    // Pointer to the GPU accelerated implementation of the sensor update
    std::shared_ptr<Evaluator> cuda_evaluator_;

    // Parameters for the sensor update
    FLOAT_T a_hit_;
    FLOAT_T a_range_;
    FLOAT_T a_max_;
    FLOAT_T max_range_;
    FLOAT_T inv_max_range_;
    FLOAT_T max_range_squared_;
    FLOAT_T map_res_;
    FLOAT_T map_res_half_;

protected:
    /**
     * @brief Evaluation of given pose (particle) with the measured laser scan in the TSDF map 
     */
    virtual FLOAT_T evaluatePose(FLOAT_T* pose, const std::vector<CudaPoint> cloud, EvaluationModel& model) const;

public:
    /**
     * @brief Initialization of the TSDF evaluator for the sensor update
     * 
     * @param map_ptr Pointer to the TSDF map that should be used in the sensor update
     * @param per_point Should the the per point implementation be used in the sensor update?
     * @param a_hit Weighting for the normal distribution of the scan to map difference in the sensor update
     * @param a_range Weighting for a random occuring scna point in the receiced laser ascan
     * @param a_max  Weighting for scan point occuring at the maximum range of the laser scanner
     * @param max_range Maximum range of the laser scanner
     */
    TSDFEvaluator(const std::shared_ptr<CudaSubVoxelMap<FLOAT_T, FLOAT_T>>& map_ptr, bool per_point = false, FLOAT_T a_hit = 0.9, FLOAT_T a_range = 0.1, FLOAT_T a_max = 0.0, FLOAT_T max_range = 100.0, FLOAT_T reduction_cell_size = 0.064) : 
    map_ptr_(map_ptr), 
    a_hit_{a_hit}, a_range_(a_range), a_max_(a_max), 
    max_range_(max_range), inv_max_range_(1.0 / max_range), max_range_squared_(max_range * max_range),
    map_res_(reduction_cell_size), map_res_half_(reduction_cell_size / 2)
    {
        cuda_evaluator_.reset(new CudaEvaluator(*map_ptr, per_point, a_hit_, a_range_, a_max_, max_range_));
        //cuda_evaluator_.reset(new Evaluator());
    }

    /**
     * @brief Destroy the TSDFEvaluator object 
     */
    ~TSDFEvaluator()
    {
        cuda_evaluator_.reset();
    }

    /**
     * @brief Evaluate a given particle cloud in the provided TSDF map of the environment based on a measured laser scan
     * 
     * @param particle_cloud Particle cloud taht should be evaluation in the sensor update
     * @param real_cloud  Measured laser scan of the environment
     * @param robot_frame TF frame of the robot
     * @param scan_frame  TF frame of the laser scanner
     * @param use_cuda Should the gpu acceleration be used?
     * @param ignore_tf Should the transformation between the origin of the robot and the laser scanner be ignored during the sensor update?
     * 
     * @return geometry_msgs::PoseWithCovariance Pose estimation of the robot determined by the sensor update with uncertainty
     */
    virtual geometry_msgs::PoseWithCovariance evaluateParticles(ParticleCloud& particle_cloud, const sensor_msgs::PointCloud2& real_cloud, const std::string& robot_frame = "base_footprint", const std::string& scan_frame = "scanner", bool use_cuda = false, bool ignore_tf = false);
    
    /**
     * @brief Evaluate a given particle cloud in the provided TSDF map of the environment based on a measured laser scan
     * 
     * @param particles Particle cloud taht should be evaluation in the sensor update
     * @param points Measured laser scan of the environment
     * @param tf_matrix Tranformation between the reference system of the laser scanner and the robot
     * @param use_cuda Should the gpu acceleration be used?
     * 
     * @return geometry_msgs::PoseWithCovariance Pose estimation of the robot determined by the sensor update with uncertainty
     */
    virtual geometry_msgs::PoseWithCovariance evaluate(std::vector<Particle>& particles, const std::vector<CudaPoint>& points, FLOAT_T tf_matrix[16], bool use_cuda = false);
};

} // namespace mcl

#endif