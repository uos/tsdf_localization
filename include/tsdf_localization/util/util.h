#ifndef UTILS_H
#define UTILS_H

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
//#include <tf/tf.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace tsdf_localization
{

using FLOAT_T = float;

constexpr FLOAT_T MAX_RANGE = 100.0;
constexpr FLOAT_T INV_MAX_RANGE = 1.0 / MAX_RANGE;

constexpr FLOAT_T A_HIT = 0.9;
constexpr FLOAT_T A_RAND = 0.1;
constexpr FLOAT_T A_MAX = 0.0;

/**
 * @brief Extract the yaw (two dimensional rotation) from a given quaternion
 * 
 * @param quaternion Quaternion where the angle should be extracted
 * 
 * @return The yaw angle from the given quaternion 
 */
FLOAT_T getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion);

/**
 * @brief Extract the roll from a given quaternion
 * 
 * @param quaternion Quaternion where the angle should be extracted
 * 
 * @return The roll angle from the given quaternion 
 */
FLOAT_T getRollFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion);

/**
 * @brief Extract the pitch from a given quaternion
 * 
 * @param quaternion Quaternion where the angle should be extracted
 * 
 * @return The pitch angle from the given quaternion 
 */
FLOAT_T getPitchFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion);

/**
 * @brief Transform a given point based into a coordinate system represented by a pose
 * 
 * @param point Point which should be transformed
 * @param transform Representation of the target coordinate system
 * 
 * @return transformed point
 */
geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, const geometry_msgs::msg::Pose& transform);

/**
 * @brief Builds a tf transform from a map representation to a global frame  
 * 
 * @param map_meta Information about the map representation
 * 
 * @return Transformation from the map representation to the global coordinate system 
 */
geometry_msgs::msg::TransformStamped getMapToWorldTF(const nav_msgs::msg::MapMetaData& map_meta);

/**
 * @brief Builds a tf transform from a global frame to a map representation   
 * 
 * @param map_meta Information about the map representation
 * 
 * @return Transformation from the global coordinate system to the map representation 
 */
geometry_msgs::msg::TransformStamped getWorldToMapTF(const nav_msgs::msg::MapMetaData& map_meta);

void getAngleFromMat(const FLOAT_T mat[16], FLOAT_T& roll,  FLOAT_T& pitch,  FLOAT_T& yaw);

} // namespace tsdf_localization

#endif