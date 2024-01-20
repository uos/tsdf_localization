#include <tsdf_localization/util/util.h>

#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace tsdf_localization
{

FLOAT_T getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
  double roll, pitch, yaw;
  tf2::Quaternion tf_quaternion;

  tf2::convert(quaternion, tf_quaternion);
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

  return yaw;
}

FLOAT_T getRollFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
  double roll, pitch, yaw;
  tf2::Quaternion tf_quaternion;

  tf2::convert(quaternion, tf_quaternion);
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

  return roll;
}

FLOAT_T getPitchFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
  double roll, pitch, yaw;
  tf2::Quaternion tf_quaternion;

  tf2::convert(quaternion, tf_quaternion);
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

  return pitch;
}

geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, const geometry_msgs::msg::Pose& origin)
{
  geometry_msgs::msg::Point result;
  geometry_msgs::msg::TransformStamped stamped_transform;
  stamped_transform.transform.translation.x = origin.position.x;
  stamped_transform.transform.translation.y = origin.position.y;
  stamped_transform.transform.translation.z = origin.position.z;
  stamped_transform.transform.rotation = origin.orientation;

  tf2::doTransform(point, result, stamped_transform);

  return result;
}


geometry_msgs::msg::TransformStamped getMapToWorldTF(const nav_msgs::msg::MapMetaData& map_meta)
{
  geometry_msgs::msg::TransformStamped map_to_world;
  map_to_world.transform.translation.x = map_meta.origin.position.x;
  map_to_world.transform.translation.y = map_meta.origin.position.y;
  map_to_world.transform.translation.z = map_meta.origin.position.z;
  map_to_world.transform.rotation = map_meta.origin.orientation;

  return map_to_world;
}

geometry_msgs::msg::TransformStamped getWorldToMapTF(const nav_msgs::msg::MapMetaData& map_meta)
{
  geometry_msgs::msg::TransformStamped map_to_world = getMapToWorldTF(map_meta);
  geometry_msgs::msg::TransformStamped world_to_map;
  tf2::Transform tf_tmp;
  tf2::convert(map_to_world.transform, tf_tmp);
  tf2::convert(tf_tmp.inverse(), world_to_map.transform);
  
  return world_to_map;
}

void getAngleFromMat(const FLOAT_T mat[16], FLOAT_T& roll,  FLOAT_T& pitch,  FLOAT_T& yaw)
{
    if (fabs(mat[8]) >= 1)
    {
        yaw = 0;

        auto delta = atan2(mat[9], mat[10]);

        if (mat[8] < 0)
        {
            pitch = M_PI / 2.0;
            roll  = delta;
        }
        else
        {
            pitch = -M_PI / 2.0;
            roll  = delta;
        }
    }
    else
    {
        pitch = -asin(mat[8]);
        roll  = atan2(mat[9] / cos(pitch), mat[10] / cos(pitch));
        yaw   = atan2(mat[4] / cos(pitch), mat[0] / cos(pitch));
    }
}

} // namespace tsdf_localization