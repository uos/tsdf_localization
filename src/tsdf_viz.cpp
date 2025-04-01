
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <highfive/H5File.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tsdf_localization/util/tsdf.h>
#include <tsdf_localization/map/grid_map.h>

#include <vector>
#include <thread>
#include <atomic>

#include <tsdf_localization/map/sub_voxel_map.h>
#include <tsdf_localization/map/map_util.h>

namespace tsdf_localization
{

class TSDFVizNode : public rclcpp::Node
{
public:

  

  TSDFVizNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("tsdf_viz", options)
  {
    // parameters
    {
      rcl_interfaces::msg::ParameterDescriptor map_file_pdesc; 
      map_file_pdesc.name = "map_file";
      map_file_pdesc.type = rclcpp::ParameterType::PARAMETER_STRING;  
      map_file_pdesc.description = "The path to a tsdf map (h5 format)";
      map_file_name_ = this->declare_parameter<std::string>(map_file_pdesc.name, "", map_file_pdesc);
  
      rcl_interfaces::msg::ParameterDescriptor map_frame_pdesc; 
      map_frame_pdesc.name = "map_frame";
      map_frame_pdesc.type = rclcpp::ParameterType::PARAMETER_STRING;  
      map_frame_pdesc.description = "Map frame name";
      map_frame_ = this->declare_parameter<std::string>(map_frame_pdesc.name, "map", map_frame_pdesc);
    }

    if(map_file_name_ == "")
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Please provide a map!");
      throw std::runtime_error("ERROR: Please provide a map!");
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading map from '" << map_file_name_ << "'");

    pub_tsdf_ = this->create_publisher<visualization_msgs::msg::Marker>("tsdf_map", 1);
    pub_free_ = this->create_publisher<visualization_msgs::msg::Marker>("free_map", 1);
    pub_occ_ = this->create_publisher<visualization_msgs::msg::Marker>("occ_map", 1);

    loadMap(map_file_name_);

    // init latched publishers
    pub_tsdf_ = this->create_publisher<visualization_msgs::msg::Marker>("tsdf_map", 1);
    pub_free_ = this->create_publisher<visualization_msgs::msg::Marker>("free_map", 1);
    pub_occ_  = this->create_publisher<visualization_msgs::msg::Marker>("occ_map",  1);
    
    n_sub_tsdf_ = 0;
    n_sub_free_ = 0;
    n_sub_occ_ = 0;

    thread_pub_tsdf_ = std::thread([this](){
      while(rclcpp::ok() && !stop_pub_threads_) 
      {
        const size_t n_sub = pub_tsdf_->get_subscription_count();
        if(n_sub > n_sub_tsdf_)
        {
          marker_tsdf_.header.stamp = this->now();
          pub_tsdf_->publish(marker_tsdf_);
          RCLCPP_INFO_STREAM(this->get_logger(), "Published TSDF marker");
        }
        n_sub_tsdf_ = n_sub;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

    thread_pub_free_ = std::thread([this](){
      while(rclcpp::ok() && !stop_pub_threads_) 
      {
        const size_t n_sub = pub_free_->get_subscription_count();
        if(n_sub > n_sub_free_)
        {
          marker_free_.header.stamp = this->now();
          pub_free_->publish(marker_free_);
          RCLCPP_INFO_STREAM(this->get_logger(), "Published free marker");
        }
        n_sub_free_ = n_sub;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });

    thread_pub_occ_ = std::thread([this](){
      while(rclcpp::ok() && !stop_pub_threads_) 
      {
        const size_t n_sub = pub_occ_->get_subscription_count();
        if(n_sub > n_sub_occ_)
        {
          marker_occ_.header.stamp = this->now();
          pub_occ_->publish(marker_occ_);
          RCLCPP_INFO_STREAM(this->get_logger(), "Published occupied marker");
        }
        n_sub_occ_ = n_sub;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
  }

  ~TSDFVizNode()
  {
    stop_pub_threads_ = true;
    if(thread_pub_tsdf_.joinable()) 
    {
      thread_pub_tsdf_.join();
    }

    if(thread_pub_free_.joinable())
    {
      thread_pub_free_.join();
    }

    if(thread_pub_occ_.joinable())
    {
      thread_pub_occ_.join();
    }
  }

  void loadMap(std::string filename)
  {
    HighFive::File f(map_file_name_, HighFive::File::ReadOnly);
    HighFive::Group g = f.getGroup("/map");

    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(4'000'000);
    std::vector<std_msgs::msg::ColorRGBA> colors;
    colors.reserve(4'000'000);

    std::vector<geometry_msgs::msg::Point> free_points;
    points.reserve(4'000'000);
    std::vector<std_msgs::msg::ColorRGBA> free_colors;
    colors.reserve(4'000'000);

    geometry_msgs::msg::Point point;
    std_msgs::msg::ColorRGBA color;
    color.a = 1;
    color.b = 0;

    std::vector<CudaPoint> free_map;
    using MapT = SubVoxelMap<FLOAT_T, FLOAT_T, 1>;
    std::shared_ptr<MapT> map = createTSDFMap<MapT, FLOAT_T, FLOAT_T>(map_file_name_, free_map);
  
    size_t read_entries = 0;

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

    marker_tsdf_.header.frame_id = map_frame_;
    marker_tsdf_.header.stamp = this->now();
    marker_tsdf_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_tsdf_.action = visualization_msgs::msg::Marker::MODIFY;
    marker_tsdf_.ns = "map";
    marker_tsdf_.id = 0;
    marker_tsdf_.pose.position.x = 0.0; // TSDF chunk offsets could be easily applied here
    marker_tsdf_.pose.position.y = 0.0;
    marker_tsdf_.pose.position.z = 0.0;
    marker_tsdf_.pose.orientation.x = 0.0;
    marker_tsdf_.pose.orientation.y = 0.0;
    marker_tsdf_.pose.orientation.z = 0.0;
    marker_tsdf_.pose.orientation.w = 1.0;
    marker_tsdf_.scale.x = marker_tsdf_.scale.y = marker_tsdf_.scale.z = 0.8 * MAP_RESOLUTION * 0.001;
    marker_tsdf_.points = std::move(points);
    marker_tsdf_.colors = std::move(colors);

    marker_free_.header.frame_id = map_frame_;
    marker_free_.header.stamp = this->now();
    marker_free_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_free_.action = visualization_msgs::msg::Marker::MODIFY;
    marker_free_.ns = "map";
    marker_free_.id = 0;
    marker_free_.pose.position.x = 0.0; // TSDF chunk offsets could be easily applied here
    marker_free_.pose.position.y = 0.0;
    marker_free_.pose.position.z = 0.0;
    marker_free_.pose.orientation.x = 0.0;
    marker_free_.pose.orientation.y = 0.0;
    marker_free_.pose.orientation.z = 0.0;
    marker_free_.pose.orientation.w = 1.0;
    marker_free_.scale.x = marker_free_.scale.y = marker_free_.scale.z = 0.8 * MAP_RESOLUTION * 0.001;
    marker_free_.points = std::move(free_points);
    marker_free_.colors = std::move(free_colors);

    // old: publish

    points.clear();
    points.reserve(map->gridOccSize());

    colors.clear();
    colors.reserve(map->gridOccSize());

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

    marker_occ_.header.frame_id = map_frame_;
    marker_occ_.header.stamp = this->now(); 
    marker_occ_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_occ_.action = visualization_msgs::msg::Marker::MODIFY;
    marker_occ_.ns = "map";
    marker_occ_.id = 0;
    marker_occ_.pose.position.x = 0.0; // TSDF chunk offsets could be easily applied here
    marker_occ_.pose.position.y = 0.0;
    marker_occ_.pose.position.z = 0.0;
    marker_occ_.pose.orientation.x = 0.0;
    marker_occ_.pose.orientation.y = 0.0;
    marker_occ_.pose.orientation.z = 0.0;
    marker_occ_.pose.orientation.w = 1.0;
    marker_occ_.scale.x = marker_occ_.scale.y = marker_occ_.scale.z = 1.0; //0.8 * MAP_RESOLUTION * 0.001;
    marker_occ_.points = std::move(points);
    marker_occ_.colors = std::move(colors);

    // old publish
  }

private:
  std::string map_file_name_;
  std::string map_frame_;

  visualization_msgs::msg::Marker marker_tsdf_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr 
      pub_tsdf_;
  std::thread thread_pub_tsdf_;
  size_t n_sub_tsdf_;
  
  visualization_msgs::msg::Marker marker_free_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr 
      pub_free_;
  std::thread thread_pub_free_;
  size_t n_sub_free_;

  visualization_msgs::msg::Marker marker_occ_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr 
      pub_occ_;
  std::thread thread_pub_occ_;
  size_t n_sub_occ_;

  std::atomic<bool> stop_pub_threads_ = false;
};

} // namespace tsdf_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tsdf_localization::TSDFVizNode)
