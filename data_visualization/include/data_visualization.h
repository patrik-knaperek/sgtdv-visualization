/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <vector>

/* ROS */
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>

/* SGT */
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/ConeWithCovStampedArr.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>
#include <sgtdv_msgs/Trajectory.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/Control.h>
#include "../../../SGT_Utils.h"

class DataVisualization
{
public:
  DataVisualization(ros::NodeHandle& handle);
  ~DataVisualization() = default;
  
  /* Callbacks */
  void cameraCallback(const sgtdv_msgs::ConeStampedArr::ConstPtr &msg);
  void lidarCallback(const sgtdv_msgs::Point2DStampedArr::ConstPtr &msg);
  void fusionCallback(const sgtdv_msgs::ConeWithCovStampedArr::ConstPtr &msg);
  void poseCallback(const sgtdv_msgs::CarPose::ConstPtr& msg);
  void mapCallback(const sgtdv_msgs::ConeArr::ConstPtr& msg);
  void trajectoryCallback(const sgtdv_msgs::Trajectory::ConstPtr& msg);
  void commandCallback(const sgtdv_msgs::Control::ConstPtr& msg);

  /* Constants */
  static constexpr double THROTTLE_MARKER_BASE[2] = {-1.0, 1.5};
  static constexpr double THROTTLE_GAIN = 1 / 50.0;
  static constexpr double STEER_MARKER_BASE[2] = {2.0, 0.0};
  static constexpr double STEER_GAIN = 2;

private:
  void initCameraMarker(void);
  void initLidarMarker(void);
  void initFusionMarker(void);
  void initPoseMarker(void);
  void initMapMarker(void);
  void initTrajectoryMarker(void);
  void initCommandMarkers(const ros::NodeHandle& handle);
  void initFOV(const ros::NodeHandle& handle);    
  void deleteMarkers(visualization_msgs::MarkerArray& marker_array, const ros::Publisher& publisher) const;
  std_msgs::ColorRGBA setConeColor(const uint8_t color_msg);

  ros::Publisher camera_publisher_;
  ros::Publisher lidar_publisher_;
  ros::Publisher fusion_publisher_;
  ros::Publisher pose_publisher_;
  ros::Publisher map_publisher_;
  ros::Publisher trajectory_pos_publisher_;
  ros::Publisher trajectory_vel_publisher_;
  ros::Publisher command_publisher_;
  ros::Publisher camera_fov_publisher_;
  ros::Publisher lidar_fov_publisher_;

  ros::Subscriber camera_subscriber_;
  ros::Subscriber lidar_subscriber_;
  ros::Subscriber fusion_subscriber_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber map_subscriber_;
  ros::Subscriber trajectory_subscriber_;
  ros::Subscriber command_subscriber_;

  visualization_msgs::Marker camera_marker_, lidar_marker_, fusion_marker_, steering_marker_, throtle_marker_,
                              pose_marker_, map_marker_;
  visualization_msgs::MarkerArray command_markers_;
  geometry_msgs::PolygonStamped camera_fov_marker_, lidar_fov_marker_;
  nav_msgs::Path trajectory_pos_marker_, trajectory_vel_marker_;
};
