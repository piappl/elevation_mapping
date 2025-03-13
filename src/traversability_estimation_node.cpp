// Move to target pkg when this issue is resolved https://github.com/ANYbotics/grid_map/issues/382
#include "elevation_mapping/traversability_estimation_node.hpp"

TraversabilityEstimationNode::TraversabilityEstimationNode() : Node("traversability_estimation_node")
{
}

void TraversabilityEstimationNode::init()
{
  // Declare parameters
  this->declare_parameter<double>("traversability_threshold", 0.1);
  this->declare_parameter<std::string>("traversability_map_topic", "traversability_map");
  this->declare_parameter<std::string>("occupancy_map_topic", "local_map");
  this->declare_parameter<std::string>("elevation_map_topic", "elevation_map");

  // Get parameters
  this->get_parameter("traversability_threshold", mTraversabilityThreshold);
  this->get_parameter("traversability_map_topic", mTraversabilityMapTopic);
  this->get_parameter("occupancy_map_topic", mOccupancyMapTopic);
  this->get_parameter("elevation_map_topic", mElevationMapTopic);

  initCommunication();
  RCLCPP_INFO(this->get_logger(), "Traversability Node Initialized.");
}

void TraversabilityEstimationNode::initCommunication()
{
  // Publishers
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                                .reliability(rclcpp::ReliabilityPolicy::Reliable)
                                .durability(rclcpp::DurabilityPolicy::TransientLocal);
  mPubTraversability = this->create_publisher<grid_map_msgs::msg::GridMap>(mTraversabilityMapTopic, 10);
  mPubOccupancy = this->create_publisher<nav_msgs::msg::OccupancyGrid>(mOccupancyMapTopic, qos_profile);

  // Subscriber
  mSubElevation = this->create_subscription<grid_map_msgs::msg::GridMap>(
      mElevationMapTopic, rclcpp::SensorDataQoS(),
      std::bind(&TraversabilityEstimationNode::computeTraversability, this, std::placeholders::_1));
}

void TraversabilityEstimationNode::computeTraversability(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  // Convert ROS message to GridMap
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  if (!map.exists("elevation"))
  {
    RCLCPP_WARN(this->get_logger(), "No 'elevation' layer found in the received grid map.");
    return;
  }

  // Create a new traversability layer
  // map.add("traversability", 1.0);  // Default to 1 (fully traversable)
  map.add("traversability");

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    grid_map::Position position;
    map.getPosition(*it, position);

    // Get elevation at current position
    float elevation;
    // if (!map.isValid(position, "elevation"))
    //   continue;
    elevation = map.atPosition("elevation", position);

    // Compute height variation in a small neighborhood
    double max_slope = 0.0;
    for (grid_map::CircleIterator sub_it(map, position, 0.4); !sub_it.isPastEnd(); ++sub_it)
    {
      if ((*it).isApprox(*sub_it))
        continue;

      float neighbor_elevation;
      if (map.isValid(*sub_it, "elevation"))
      {
        neighbor_elevation = map.at("elevation", *sub_it);
        double slope = std::abs(neighbor_elevation - elevation);
        max_slope = std::max(max_slope, slope);
      }
    }

    // Define traversability based on slope
    double traversability = std::max(0.0, 1.0 - (max_slope / 0.2));
    map.at("traversability", *it) = static_cast<float>(traversability);
  }

  // Publish traversability map
  std::unique_ptr<grid_map_msgs::msg::GridMap> traversabilityMsg;
  traversabilityMsg = grid_map::GridMapRosConverter::toMessage(map);
  mPubTraversability->publish(std::move(traversabilityMsg));

  // Convert to OccupancyGrid and publish
  publishOccupancyGrid(map);
}

void TraversabilityEstimationNode::publishOccupancyGrid(const grid_map::GridMap& gridMap)
{
  // std::cout << "publishOccupancyGrid 1" << std::endl;
  // nav_msgs::msg::OccupancyGrid message;
  // grid_map::GridMapRosConverter::toOccupancyGrid(map, "traversability", 1.0, 0.0, message);
  // // Convert GridMap to OccupancyGrid
  // nav_msgs::msg::OccupancyGrid occupancy_msg;
  // occupancy_msg.header.stamp = this->get_clock()->now();
  // occupancy_msg.header.frame_id = "map";  // Change if needed
  // occupancy_msg.info.resolution = map.getResolution();
  // occupancy_msg.info.width = map.getSize()(0);
  // occupancy_msg.info.height = map.getSize()(1);

  // grid_map::Position origin;
  // map.getPosition(grid_map::Index(0, 0), origin);
  // occupancy_msg.info.origin.position.x = origin.x();
  // occupancy_msg.info.origin.position.y = origin.y();
  // occupancy_msg.info.origin.position.z = 0.0;
  // occupancy_msg.info.origin.orientation.w = 1.0;

  // // Convert traversability to occupancy grid format
  // occupancy_msg.data.resize(occupancy_msg.info.width * occupancy_msg.info.height, -1);

  // for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
  // {
  //   grid_map::Index index = *it;
  //   float traversability = map.at("traversability", index);

  //   int grid_index = index(1) * occupancy_msg.info.width + index(0);
  //   if (traversability > mTraversabilityThreshold)
  //     occupancy_msg.data[grid_index] = 1;  // Free space
  //   else
  //     occupancy_msg.data[grid_index] = 100;  // Obstacle
  // }
  // std::cout << "publishOccupancyGrid 2" << std::endl;

  nav_msgs::msg::OccupancyGrid occupancyGrid;
  occupancyGrid.header.frame_id = gridMap.getFrameId();
  occupancyGrid.header.stamp = rclcpp::Time(gridMap.getTimestamp());
  // Same as header stamp as we do not load the map.
  occupancyGrid.info.map_load_time = occupancyGrid.header.stamp;
  occupancyGrid.info.resolution = gridMap.getResolution();
  occupancyGrid.info.width = gridMap.getSize()(0);
  occupancyGrid.info.height = gridMap.getSize()(1);
  grid_map::Position position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
  occupancyGrid.info.origin.position.x = position.x();
  occupancyGrid.info.origin.position.y = position.y();
  occupancyGrid.info.origin.position.z = 0.0;
  occupancyGrid.info.origin.orientation.x = 0.0;
  occupancyGrid.info.origin.orientation.y = 0.0;
  occupancyGrid.info.origin.orientation.z = 0.0;
  occupancyGrid.info.origin.orientation.w = 1.0;
  size_t nCells = gridMap.getSize().prod();
  occupancyGrid.data.resize(nCells);
  std::fill(occupancyGrid.data.begin(), occupancyGrid.data.end(), -1);

  // Occupancy probabilities are in the range [0,100]. Unknown is 100.
  const float cellMin = 0;
  const float cellMax = 100;
  const float cellRange = cellMax - cellMin;

  float dataMin = 1.0;
  float dataMax = 0.0;
  std::string layer = "traversability";
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
  {
    float initValue = gridMap.at(layer, *iterator);
    float value = (gridMap.at(layer, *iterator) - dataMin) / (dataMax - dataMin);
    if (std::isnan(initValue))
    {
      value = 100;
    }
    else if (value < mTraversabilityThreshold)
    {
      value = 100;
    }
    else
    {
      value = cellMin + std::min(std::max(0.0f, value), 1.0f) * cellRange;
    }
    size_t index = grid_map::getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(), false);
    // Reverse cell order because of different conventions between occupancy grid and grid map.
    occupancyGrid.data[nCells - index - 1] = value;
  }
  mPubOccupancy->publish(occupancyGrid);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TraversabilityEstimationNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
