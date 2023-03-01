/**
 * @file snap_vis.cpp
 * @author Marc Eisoldt (meisoldt@uni-osnabrueck.de)
 * 
 * @brief Node to visualize a MCL snapshot that consist of the current scan, particle cloud, initial pose and estimated transformation of the robot
 * 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/publisher.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/filesystem.hpp>

#include <util/mcl_file.h>

using namespace mcl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "snap_vis_node");

    ros::NodeHandle nh("~");

    if (argc != 2)
    {
        std::cout << "usage: " << argv[0] << " <mcl-file>" << std::endl;
        return 0;
    }

    std::string mcl_name(argv[1]);
    
    if (!boost::filesystem::exists(mcl_name))
    {
        std::cout << "mcl file \"" << mcl_name << "\" does not exist" << std::endl;
        return 1;
    }

    std::cout << "Read mcl data from file..." << std::endl;

    MCLFile file(mcl_name);

    std::vector<CudaPoint> points;
    std::vector<int> rings;
    std::vector<Particle> particles;
    std::array<FLOAT_T, 16> tf_matrix;

    FLOAT_T x, y, z, q_1, q_2, q_3, q_4;

    file.read(points, rings, particles, tf_matrix, x, y, z, q_1, q_2, q_3, q_4);

    std::unordered_set<CudaPoint, hash> point_set;

    for (const auto& point : points)
    {
        CudaPoint center = {static_cast<float>(std::floor(point.x / 0.064) * 0.064 + 0.032), 
                            static_cast<float>(std::floor(point.y / 0.064) * 0.064 + 0.032), 
                            static_cast<float>(std::floor(point.z / 0.064) * 0.064 + 0.032)};
        
        point_set.insert(center);
    }

    std::vector<CudaPoint> reduced_points;
    reduced_points.resize(point_set.size());
    std::copy(point_set.begin(), point_set.end(), reduced_points.begin());


    std::cout << "Convert data to ROS..." << std::endl;

    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = "map";
    cloud.points.resize(reduced_points.size());

    for (auto index = 0u; index < reduced_points.size(); ++index)
    {
        cloud.points[index].x = reduced_points[index].x;
        cloud.points[index].y = reduced_points[index].y;
        cloud.points[index].z = reduced_points[index].z;
    }

    cloud.points.resize(6000);

    geometry_msgs::Pose pose;
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.poses.resize(particles.size());

    for (auto index = 0u; index < particles.size(); ++index)
    {
        pose.position.x = particles[index].first[0];
        pose.position.y = particles[index].first[1];
        pose.position.z = particles[index].first[2];
        
        tf2::Quaternion tf_quaternion;
        tf_quaternion.setRPY(particles[index].first[3], particles[index].first[4], particles[index].first[5]);

        tf2::convert(tf_quaternion, pose.orientation);

        pose_array.poses[index] = pose;
    }

    geometry_msgs::PoseStamped init_pose_stamped_;
    init_pose_stamped_.header.frame_id = "map";

    init_pose_stamped_.pose.position.x = x;
    init_pose_stamped_.pose.position.y = y;
    init_pose_stamped_.pose.position.z = z;
    init_pose_stamped_.pose.orientation.w = q_1;
    init_pose_stamped_.pose.orientation.x = q_2;
    init_pose_stamped_.pose.orientation.y = q_3;
    init_pose_stamped_.pose.orientation.z = q_4;

    geometry_msgs::PoseStamped tf_pose_stamped_;
    tf_pose_stamped_.header.frame_id = "map";

    tf_pose_stamped_.pose.position.x = tf_matrix[3];
    tf_pose_stamped_.pose.position.y = tf_matrix[7];
    tf_pose_stamped_.pose.position.z = tf_matrix[11];

    tf2::Matrix3x3 rot_mat;
    rot_mat[0][0] = tf_matrix[0];
    rot_mat[0][1] = tf_matrix[1];
    rot_mat[0][2] = tf_matrix[2];
    rot_mat[1][0] = tf_matrix[4];
    rot_mat[1][1] = tf_matrix[5];
    rot_mat[1][2] = tf_matrix[6];
    rot_mat[2][0] = tf_matrix[8];
    rot_mat[2][1] = tf_matrix[9];
    rot_mat[2][2] = tf_matrix[10];

    tf2::Quaternion tf_quaternion;
    rot_mat.getRotation(tf_quaternion);
    tf2::convert(tf_quaternion, tf_pose_stamped_.pose.orientation);

    std::cout << "Publish data..." << std::endl;

    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("read_cloud", 1);
    ros::Publisher particle_pub = nh.advertise<geometry_msgs::PoseArray>("read_particles", 1);
    ros::Publisher tf_pub = nh.advertise<geometry_msgs::PoseStamped>("read_tf", 1);
    ros::Publisher init_pub = nh.advertise<geometry_msgs::PoseStamped>("read_init", 1);

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        cloud.header.stamp = ros::Time::now();
        cloud_pub.publish(cloud);

        pose_array.header.stamp = ros::Time::now();
        particle_pub.publish(pose_array);

        tf_pose_stamped_.header.stamp = ros::Time::now();
        tf_pub.publish(tf_pose_stamped_);

        init_pose_stamped_.header.stamp = ros::Time::now();
        init_pub.publish(init_pose_stamped_);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}