/*
 * ElevationMap.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Kindr
#include <kindr/Core>

// Boost
#include <boost/thread/recursive_mutex.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// Elevation Mapping
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/postprocessing/PostprocessorPool.hpp"

namespace elevation_mapping
{

/*!
 * Elevation map stored as grid map handling elevation height, variance, color etc.
 */
class ElevationMap
{
public:
  /*!
   * Constructor.
   */
  explicit ElevationMap(std::shared_ptr<rclcpp::Node> nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ElevationMap();

  /*!
   * Set the geometry of the elevation map. Clears all the data.
   * @param length the side lengths in x, and y-direction of the elevation map [m].
   * @param resolution the cell size in [m/cell].
   * @param position the 2d position of the elevation map in the elevation map frame [m].
   * @return true if successful.
   */
  void setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position);

  /*!
   * Add new measurements to the elevation map.
   * @param pointCloud the point cloud data.
   * @param pointCloudVariances the corresponding variances of the point cloud data.
   * @param timeStamp the time of the input point cloud.
   * @param transformationSensorToMap
   * @return true if successful.
   */
  bool add(const PointCloudType::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances, const rclcpp::Time& timeStamp,
           const Eigen::Affine3d& transformationSensorToMap);

  /*!
   * Update the elevation map with variance update data.
   * @param varianceUpdate the variance update in vertical direction.
   * @param horizontalVarianceUpdateX the variance update in horizontal x-direction.
   * @param horizontalVarianceUpdateY the variance update in horizontal y-direction.
   * @param horizontalVarianceUpdateXY the correlated variance update in horizontal xy-direction.
   * @param time the time of the update.
   * @return true if successful.
   */
  bool update(const grid_map::Matrix& varianceUpdate, const grid_map::Matrix& horizontalVarianceUpdateX,
              const grid_map::Matrix& horizontalVarianceUpdateY, const grid_map::Matrix& horizontalVarianceUpdateXY,
              const rclcpp::Time& time);

  /*!
   * Triggers the fusion of the entire elevation map.
   * @return true if successful.
   */
  bool fuseAll();

  /*!
   * Fuses the elevation map for a certain rectangular area.
   * @param position the center position of the area to fuse.
   * @param length the sides lengths of the area to fuse.
   * @return true if successful.
   */
  bool fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length);

  /*!
   * Clears all data of the elevation map (data and time).
   * @return true if successful.
   */
  bool clear();

  /*!
   * Removes parts of the map based on visibility criterion with ray tracing.
   * @param transformationSensorToMap
   * @param updatedTime
   */
  void visibilityCleanup(const rclcpp::Time& updatedTime);

  /*!
   * Move the grid map w.r.t. to the grid map frame.
   * @param position the new location of the elevation map in the map frame.
   */
  void move(const Eigen::Vector2d& position);

  /*!
   * Publishes the (latest) raw elevation map. Optionally, if a postprocessing pipeline was configured,
   * the map is postprocessed before publishing.
   * @return true if successful.
   */
  bool postprocessAndPublishRawElevationMap();

  /*!
   * Publishes the fused elevation map. Takes the latest available fused elevation
   * map, does not trigger the fusion process.
   * @return true if successful.
   */
  bool publishFusedElevationMap();

  /*!
   * Publishes the (latest) visibility cleanup map.
   * @return true if successful.
   */
  bool publishVisibilityCleanupMap();

  /*!
   * Gets a reference to the raw grid map.
   * @return the raw grid map.
   */
  grid_map::GridMap& getRawGridMap();

  /*!
   * Sets a raw grid map.
   * @param map The input raw grid map to set.
   */
  void setRawGridMap(const grid_map::GridMap& map);

  /*!
   * Gets a reference to the fused grid map.
   * @return the fused grid map.
   */
  grid_map::GridMap& getFusedGridMap();

  /*!
   * Sets a fused grid map.
   * @param map The input fused grid map to set.
   */
  void setFusedGridMap(const grid_map::GridMap& map);

  /*!
   * Gets the time of last map update.
   * @return time of the last map update.
   */
  rclcpp::Time getTimeOfLastUpdate();

  /*!
   * Gets the time of last map fusion.
   * @return time of the last map fusion.
   */
  rclcpp::Time getTimeOfLastFusion();

  /*!
   * Get the pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
   * @return pose of the elevation map frame w.r.t. the parent frame of the robot.
   */
  const kindr::HomTransformQuatD& getPose();

  /*!
   * Gets the position of a raw data point (x, y of cell position & height of cell value) in
   * the parent frame of the robot.
   * @param index the index of the requested cell.
   * @param position the position of the data point in the parent frame of the robot.
   * @return true if successful, false if no valid data available.
   */
  bool getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::Position3D& position);

  /*!
   * Gets the fused data mutex.
   * @return reference to the fused data mutex.
   */
  boost::recursive_mutex& getFusedDataMutex();

  /*!
   * Gets the raw data mutex.
   * @return reference to the raw data mutex.
   */
  boost::recursive_mutex& getRawDataMutex();

  /*!
   * Set the frame id.
   * @param frameId the frame id.
   */
  void setFrameId(const std::string& frameId);

  /*!
   * Get the frame id.
   * @return the frameId.
   */
  const std::string& getFrameId();

  /*!
   * Set the timestamp of the raw and fused elevation map.
   * @param timestmap to set.
   */
  void setTimestamp(rclcpp::Time timestamp);

  /*!
   * If the raw elevation map has subscribers.
   * @return true if number of subscribers bigger then 0.
   */
  bool hasRawMapSubscribers() const;

  /*!
   * If the fused elevation map has subscribers.
   * @return true if number of subscribers bigger then 0.
   */
  bool hasFusedMapSubscribers() const;

  /*!
   * Callback method for the updates of the underlying map.
   * Updates the internal underlying map.
   * @param underlyingMap the underlying map.
   */
  void underlyingMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr underlyingMap);

  /*!
   * Method to set the height value around the center of the robot, can be used for initialization.
   * @param initPosition Position to calculate inner rectangle.
   * @param mapHeight The height that gets set uniformly.
   * @param lengthInXSubmap Length of the submap in X direction.
   * @param lengthInYSubmap Length of the submap in Y direction.
   * @param margin Extra margin that gets added to the submap boundaries.
   */
  void setRawSubmapHeight(const grid_map::Position& initPosition, float mapHeight, double lengthInXSubmap,
                          double lengthInYSubmap, double margin);

  friend class ElevationMapping;

private:
  /*!
   * Fuses a region of the map.
   * @param topLeftIndex the top left index of the region.
   * @param size the size (in number of cells) of the region.
   * @return true if successful.
   */
  bool fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size);

  /*!
   * Cleans the elevation map data to stay within the specified bounds.
   * @return true if successful.
   */
  bool clean();

  /*!
   * Resets the fused map data.
   * @return true if successful.
   */
  void resetFusedData();

  /*!
   * Cumulative distribution function.
   * @param x the argument value.
   * @param mean the mean of the distribution.
   * @param standardDeviation the standardDeviation of the distribution.
   * @return the function value.
   */
  float cumulativeDistributionFunction(float x, float mean, float standardDeviation);

  //! ROS nodehandle.
  std::shared_ptr<rclcpp::Node> nodeHandle_;

  //! Raw elevation map as grid map.
  grid_map::GridMap rawMap_;

  //! Fused elevation map as grid map.
  grid_map::GridMap fusedMap_;

  //! Visibility cleanup debug map.
  grid_map::GridMap visibilityCleanupMap_;

  //! Underlying map, used for ground truth maps, multi-robot mapping etc.
  grid_map::GridMap underlyingMap_;

  //! Thread Pool to handle raw map postprocessing filter pipelines.
  PostprocessorPool postprocessorPool_;

  //! True if underlying map has been set, false otherwise.
  bool hasUnderlyingMap_;

  //! Pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
  kindr::HomTransformQuatD pose_;

  //! ROS publishers. Publishing of the raw elevation map is handled by the postprocessing pool.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr elevationMapFusedPublisher_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr visibilityCleanupMapPublisher_;

  //! Mutex lock for fused map.
  boost::recursive_mutex fusedMapMutex_;

  //! Mutex lock for raw map.
  boost::recursive_mutex rawMapMutex_;

  //! Mutex lock for visibility cleanup map.
  boost::recursive_mutex visibilityCleanupMapMutex_;

  //! Underlying map subscriber.
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr underlyingMapSubscriber_;

  //! Initial ros time
  rclcpp::Time initialTime_;

  //! Parameters. Are set through the ElevationMapping class.
  double minVariance_;
  double maxVariance_;
  double mahalanobisDistanceThreshold_;
  double multiHeightNoise_;
  double minHorizontalVariance_;
  double maxHorizontalVariance_;
  std::string underlyingMapTopic_;
  bool enableVisibilityCleanup_;
  bool enableContinuousCleanup_;
  double visibilityCleanupDuration_;
  double scanningDuration_;
};

}  // namespace elevation_mapping
