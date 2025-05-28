// Move to target pkg when this issue is resolved https://github.com/ANYbotics/grid_map/issues/382
#pragma once

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <std_srvs/srv/set_bool.hpp>

class TraversabilityEstimationNode : public rclcpp::Node
{
public:
  TraversabilityEstimationNode();

  void init();

private:
  void initCommunication();
  void computeTraversability(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  void callbackTurnOffObstacles(std_srvs::srv::SetBool::Request::ConstSharedPtr req,
                                std_srvs::srv::SetBool::Response::SharedPtr res);
  void publishOccupancyGrid(const grid_map::GridMap& gridMap);

  bool mAddObstacles;
  double mTraversabilityThreshold, mSlopeScaling, mSlopeSearchRadius;

  std::string mTraversabilityMapTopic;
  std::string mOccupancyMapTopic;
  std::string mElevationMapTopic;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr mPubTraversability;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mPubOccupancy;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr mSubElevation;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mSetMapModeServer;
};