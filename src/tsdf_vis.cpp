/**
 * @file tsdf_vis.cpp
 * @author Marc Eisoldt (meisoldt@uni-osnabrueck.de)
 * @author Alexander Mock (amock@uos.de)
 * 
 * @brief Node that loads a given TSDF map in HDF5 file format and publishes it as markers that can be visualized in RViz
 * 
 * @version 0.2
 * @date 2025-04-01
 * 
 * @copyright Copyright (c) 2025
 */

#include "ros/ros.h"
#include <iostream>
#include <highfive/H5File.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>

#include <vector>

#include <tsdf_localization/util/tsdf.h>
#include <tsdf_localization/map/grid_map.h>
#include <pwd.h>

#include <tsdf_localization/map/sub_voxel_map.h>
#include <tsdf_localization/map/map_util.h>

using namespace tsdf_localization;

std::string map_file_name = "ros_ws/tsdf_maps/sim_map.h5";



void octoCallback(const visualization_msgs::MarkerArray::ConstPtr& marker_array)
{
    std::cout << "map received!" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsdf_vis");
    ros::NodeHandle nh;
    
    if (argc != 2)
    {
        std::cout << "usage: " << argv[0] << " <h5-map-file>" << std::endl;
        return 0;
    }

    //struct passwd *pw = getpwuid(getuid());
    map_file_name = argv[1]; //std::string(pw->pw_dir) + "/" + map_file_name;

    HighFive::File f(map_file_name, HighFive::File::ReadOnly);
    HighFive::Group g = f.getGroup("/map");

    std::cout << "Read map..." << std::endl;

    size_t read_entries = 0;

    std::vector<geometry_msgs::Point> points;
    points.reserve(4'000'000);
    std::vector<std_msgs::ColorRGBA> colors;
    colors.reserve(4'000'000);

    std::vector<geometry_msgs::Point> free_points;
    points.reserve(4'000'000);
    std::vector<std_msgs::ColorRGBA> free_colors;
    colors.reserve(4'000'000);

    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.b = 0;

    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("tsdf_map", 1, true);
    ros::Publisher free_pub = nh.advertise<visualization_msgs::Marker>("free_map", 1, true);
    ros::Publisher occ_pub = nh.advertise<visualization_msgs::Marker>("occ_map", 1, true);

    ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>("/occupied_cells_vis_array", 1, octoCallback);

    std::vector<CudaPoint> free_map;
    using MapT = SubVoxelMap<FLOAT_T, FLOAT_T, 1>;
    auto map = createTSDFMap<MapT, FLOAT_T, FLOAT_T>(map_file_name, free_map);
    
    // Fill the grid with the valid TSDF values of the map
    for (auto tag : g.listObjectNames())
    {
        // Get the chunk data
        HighFive::DataSet d = g.getDataSet(tag);
        std::vector<TSDFValue::RawType> chunk_data;
        d.read(chunk_data);
        // Get the chunk position
        std::vector<int> chunk_pos;
        std::string delimiter = "_";
        size_t pos = 0;
        std::string token;
        while ((pos = tag.find(delimiter)) != std::string::npos)
        {
            token = tag.substr(0, pos);
            chunk_pos.push_back(std::stoi(token));
            tag.erase(0, pos + delimiter.length());
        }
        chunk_pos.push_back(std::stoi(tag));

        for (int i = 0; i < CHUNK_SIZE; i++)
        {
            for (int j = 0; j < CHUNK_SIZE; j++)
            {
                for (int k = 0; k < CHUNK_SIZE; k++)
                {
                    auto entry = TSDFValue(chunk_data[CHUNK_SIZE * CHUNK_SIZE * i + CHUNK_SIZE * j + k]);

                    auto tsdf_value = (float)(entry.value());
                    auto weight = entry.weight();

                    int x = CHUNK_SIZE * chunk_pos[0] + i;
                    int y = CHUNK_SIZE * chunk_pos[1] + j;
                    int z = CHUNK_SIZE * chunk_pos[2] + k;

                    // Only touched cells are considered
                    if (weight != 0 && std::abs(tsdf_value) < truncation)
                    {
                        point.x = static_cast<double>(x) * MAP_RESOLUTION * 0.001;
                        point.y = static_cast<double>(y) * MAP_RESOLUTION * 0.001;
                        point.z = static_cast<double>(z) * MAP_RESOLUTION * 0.001;

                        if (tsdf_value >= 0)
                        {
                            color.r = tsdf_value / truncation;
                            color.g = 0;
                            color.b = 0;
                        }
                        else
                        {
                            color.r = 0;
                            color.b = 0;
                            color.g = -tsdf_value / truncation;
                        }


                        points.push_back(point);
                        colors.push_back(color);

                        // DO SOMETHING!
                        ++read_entries;
                    }

                    // Only touched cells are considered
                    if (weight != 0 && std::abs(tsdf_value) >= truncation)
                    {
                        point.x = static_cast<double>(x) * MAP_RESOLUTION * 0.001;
                        point.y = static_cast<double>(y) * MAP_RESOLUTION * 0.001;
                        point.z = static_cast<double>(z) * MAP_RESOLUTION * 0.001;

                        color.b = tsdf_value / truncation;
                        color.g = 0;
                        color.r = 0;

                        free_points.push_back(point);
                        free_colors.push_back(color);
                    }
                }
            }
        }
    }

    // std::cout << read_entries << " entries read" << std::endl;
    std::cout << "End reading procedure!" << std::endl;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.ns = "map";
    marker.id = 0;
    marker.pose.position.x = 0.0; // TSDF chunk offsets could be easily applied here
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.8 * MAP_RESOLUTION * 0.001;
    marker.points = std::move(points);
    marker.colors = std::move(colors);

    visualization_msgs::Marker free_marker;
    free_marker.header.frame_id = "map";
    free_marker.type = visualization_msgs::Marker::CUBE_LIST;
    free_marker.action = visualization_msgs::Marker::MODIFY;
    free_marker.ns = "map";
    free_marker.id = 0;
    free_marker.pose.position.x = 0.0; // TSDF chunk offsets could be easily applied here
    free_marker.pose.position.y = 0.0;
    free_marker.pose.position.z = 0.0;
    free_marker.pose.orientation.x = 0.0;
    free_marker.pose.orientation.y = 0.0;
    free_marker.pose.orientation.z = 0.0;
    free_marker.pose.orientation.w = 1.0;
    free_marker.scale.x = free_marker.scale.y = free_marker.scale.z = 0.8 * MAP_RESOLUTION * 0.001;
    free_marker.points = std::move(free_points);
    free_marker.colors = std::move(free_colors);

    marker.header.stamp = ros::Time::now(); 
    free_marker.header.stamp = ros::Time::now(); 

    pub.publish(marker);
    free_pub.publish(free_marker);

    points.clear();
    points.reserve(map->gridOccSize());

    colors.clear();
    colors.reserve(map->gridOccSize());

    //color.a = 0.1;

    size_t index = 0;

    for (auto occ_x = 0u; occ_x < map->getUpDimX(); ++occ_x)
    {
        for (auto occ_y = 0u; occ_y < map->getUpDimY(); ++occ_y)
        {
            for (auto occ_z = 0u; occ_z < map->getUpDimZ(); ++occ_z)
            {
                auto index = occ_x + occ_y * map->getUpDimX() + occ_z * map->getUpDimX() * map->getUpDimY();
                auto entry = map->rawGridOcc()[index];

                point.x = occ_x + map->getMinX() + 0.5;
                point.y = occ_y + map->getMinY() + 0.5;
                point.z = occ_z + map->getMinZ() + 0.5;

                if (entry >= 0)
                {
                    color.r = 1.0;
                    color.g = 0.0;
                    color.b = 1.0;   
                    color.a = 0.3;
                }
                else
                {
                    color.r = 0.0;
                    color.g = 1.0;
                    color.b = 1.0;
                    color.a = 0.01;
                }

                points.push_back(point);
                colors.push_back(color);
            
                ++index;
            }
        }   
    }

    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.ns = "map";
    marker.id = 0;
    marker.pose.position.x = 0.0; // TSDF chunk offsets could be easily applied here
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0; //0.8 * MAP_RESOLUTION * 0.001;
    marker.points = std::move(points);
    marker.colors = std::move(colors);

    occ_pub.publish(marker);

    marker.header.stamp = ros::Time::now(); 

    ros::Rate rate(10);
    while (ros::ok())
    {      
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}