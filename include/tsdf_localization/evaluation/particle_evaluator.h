/**
 * @file particle_cloud.h
 * @author Marc Eiosldt (meisoldt@uni-osnabrueck.de)
 * 
 * @brief Interface of a class to evaluate a particle cloud in the Monte Carlo Localization
 * 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 */

#ifndef PARTICLE_EVALUATOR_H
#define PARTICLE_EVALUATOR_H

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/tf.h>

#include <tsdf_localization/particle_cloud.h>
#include <tsdf_localization/evaluation/model/evaluation_model.h>

#include <tsdf_localization/util/util.h>

namespace tsdf_localization
{

/**
 * @brief Interface to represent classes that can evaluate particle clouds in the Monte Carlo Localization
 * 
 */
class ParticleEvaluator
{
protected:
    virtual FLOAT_T evaluatePose(geometry_msgs::Pose& pose, const sensor_msgs::LaserScan& scan, EvaluationModel& model) const = 0;

public:
    virtual geometry_msgs::PoseWithCovariance evaluateParticles(ParticleCloud& particle_cloud, const sensor_msgs::LaserScan& real_scan, EvaluationModel& model, const std::string& robot_frame = "base_footprint", const std::string& scan_frame = "scanner") const = 0;
};

} // namespace tsdf_localization

#endif