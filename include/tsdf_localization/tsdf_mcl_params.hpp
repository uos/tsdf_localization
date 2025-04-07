#ifndef TSDF_MCL_PARAMS_HPP
#define TSDF_MCL_PARAMS_HPP

// #include <rclcpp/rclcpp.hpp>

namespace rclcpp
{
class Node;
} // namespace rclcpp

namespace tsdf_localization
{

void declareMCLParams(rclcpp::Node* node);

} // namespace tsdf_localization

#endif // TSDF_MCL_PARAMS_HPP